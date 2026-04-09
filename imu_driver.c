/**
 * sensors/imu_driver.c  — IMU Driver
 *
 * Supports:
 *   BMI088  (industrial MEMS, SPI)           — NAV_FEAT_MEMS_IMU
 *   KVH 1775 / ADIS16507  (FOG/tactical)     — NAV_FEAT_FOG_IMU
 *
 * All output normalised to ImuMeas regardless of hardware.
 * Runs in Core 0 ISR at NAV_IMU_RATE_HZ.
 */

#include "sensor_hal.h"
#include "../comms/comms.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

/* ─── BMI088 REGISTER MAP ───────────────────────────────── */
#define BMI_GYRO_ID_REG     0x00
#define BMI_GYRO_RATE_X_L   0x02
#define BMI_GYRO_RANGE      0x0F
#define BMI_GYRO_BW         0x10
#define BMI_GYRO_INT_CTRL   0x15
#define BMI_GYRO_INT3_4_IO  0x18
#define BMI_GYRO_SOFTRESET  0x14
#define BMI_ACCEL_ID_REG    0x00
#define BMI_ACCEL_DATA      0x12
#define BMI_ACCEL_CONF      0x40
#define BMI_ACCEL_RANGE     0x41
#define BMI_ACCEL_PWR_CTRL  0x7D
#define BMI_ACCEL_SOFTRESET 0x7E
#define BMI_ACCEL_TEMP_MSB  0x22

/* Scale factors */
#define BMI_GYRO_SCALE   (1.0f / 16.384f * (float)M_PI / 180.0f)  /* ±2000°/s */
#define BMI_ACCEL_SCALE  (1.0f / 1365.33f * NAV_GRAVITY_MSS)       /* ±24 g    */

/* ─── SEQLOCK SHARED IMU BUFFER ─────────────────────────── */
typedef struct {
    volatile uint32_t seq;
    ImuMeas           meas;
} __attribute__((aligned(4))) ImuShared;

static ImuShared s_imu_shared;

/* ─── LOW-LEVEL BMI088 INIT ──────────────────────────────── */
static bool bmi088_init_gyro(void) {
    /* CS deselect guard + soft reset */
    gpio_put(PIN_CS_GYRO, 1);
    sleep_us(2);
    spi_write_reg(PIN_CS_GYRO, BMI_GYRO_SOFTRESET, 0xB6);
    sleep_ms(30);

    uint8_t id = spi_read_reg(PIN_CS_GYRO, BMI_GYRO_ID_REG);
    if (id != 0x0F) return false;

    spi_write_reg(PIN_CS_GYRO, BMI_GYRO_RANGE,     0x00); /* ±2000 °/s  */
    spi_write_reg(PIN_CS_GYRO, BMI_GYRO_BW,        0x81); /* 2000 Hz    */
    spi_write_reg(PIN_CS_GYRO, BMI_GYRO_INT_CTRL,  0x80); /* DRDY enable*/
    spi_write_reg(PIN_CS_GYRO, BMI_GYRO_INT3_4_IO, 0x01); /* INT3 push-pull */
    return true;
}

static bool bmi088_init_accel(void) {
    spi_write_reg(PIN_CS_ACCEL, BMI_ACCEL_SOFTRESET, 0xB6);
    sleep_ms(50);
    /* Dummy read to force SPI mode */
    (void)spi_read_reg(PIN_CS_ACCEL, BMI_ACCEL_ID_REG);
    uint8_t id = spi_read_reg(PIN_CS_ACCEL, BMI_ACCEL_ID_REG);
    if (id != 0x1E) return false;

    spi_write_reg(PIN_CS_ACCEL, BMI_ACCEL_CONF,     0xAC); /* 1600Hz, OSR4  */
    spi_write_reg(PIN_CS_ACCEL, BMI_ACCEL_RANGE,    0x03); /* ±24 g         */
    spi_write_reg(PIN_CS_ACCEL, BMI_ACCEL_PWR_CTRL, 0x04); /* power on      */
    sleep_ms(5);
    return true;
}

/* ─── TEMPERATURE-COMPENSATED BIAS CORRECTION ───────────── */
static float bmi088_read_temp(void) {
    uint8_t buf[2];
    spi_read_burst(PIN_CS_ACCEL, BMI_ACCEL_TEMP_MSB, buf, 2);
    int16_t raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;
    return raw * 0.125f + 23.0f;
}

/* Gyro bias vs temperature model (coefficients filled at calibration) */
static struct {
    float b0[3];   /* bias at T_ref */
    float tc[3];   /* temp coefficient (rad/s/°C) */
    float T_ref;
} s_gyro_cal = {
    .b0    = {0.0f, 0.0f, 0.0f},
    .tc    = {0.0f, 0.0f, 0.0f},
    .T_ref = 25.0f
};

static void apply_gyro_temp_comp(Vec3 *g, float T) {
    float dT = T - s_gyro_cal.T_ref;
    g->x -= s_gyro_cal.b0[0] + s_gyro_cal.tc[0] * dT;
    g->y -= s_gyro_cal.b0[1] + s_gyro_cal.tc[1] * dT;
    g->z -= s_gyro_cal.b0[2] + s_gyro_cal.tc[2] * dT;
}

/* ─── 1 kHz ISR (Core 0 hardware alarm) ─────────────────── */
static bool s_imu_alarm_callback(repeating_timer_t *rt) {
    (void)rt;
    uint8_t buf[6];
    ImuMeas m;
    m.ts_us = time_us_64();
    m.valid = true;

    /* --- Gyro burst read 0x02–0x07 --- */
    spi_read_burst(PIN_CS_GYRO, BMI_GYRO_RATE_X_L, buf, 6);
    int16_t raw_gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_gz = (int16_t)((buf[5] << 8) | buf[4]);

    /* --- Accel burst read 0x12–0x17 --- */
    spi_read_burst(PIN_CS_ACCEL, BMI_ACCEL_DATA, buf, 6);
    int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);

    m.gyro_rps.x  = raw_gx * BMI_GYRO_SCALE;
    m.gyro_rps.y  = raw_gy * BMI_GYRO_SCALE;
    m.gyro_rps.z  = raw_gz * BMI_GYRO_SCALE;
    m.accel_mss.x = raw_ax * BMI_ACCEL_SCALE;
    m.accel_mss.y = raw_ay * BMI_ACCEL_SCALE;
    m.accel_mss.z = raw_az * BMI_ACCEL_SCALE;

    /* Temperature & compensation every 20 ms (every 20th call) */
    static uint8_t temp_div = 0;
    if (++temp_div >= 20) {
        m.temp_c = bmi088_read_temp();
        temp_div = 0;
        apply_gyro_temp_comp(&m.gyro_rps, m.temp_c);
    }

    /* Outlier rejection — spike filter */
    float amag = sqrtf(m.accel_mss.x*m.accel_mss.x +
                        m.accel_mss.y*m.accel_mss.y +
                        m.accel_mss.z*m.accel_mss.z);
    if (amag > NAV_IMU_OUTLIER_G * NAV_GRAVITY_MSS) {
        m.valid = false;   /* signal EKF to skip this sample */
    }

    /* Seqlock write (Core 1 reads via imu_meas_get) */
    __dmb();
    s_imu_shared.seq++;
    __dmb();
    s_imu_shared.meas = m;
    __dmb();
    s_imu_shared.seq++;
    __dmb();

    return true; /* keep timer running */
}

/* ─── FOG / RS-422 PATH (KVH 1775) ──────────────────────── */
#if NAV_FEAT_FOG_IMU
#define KVH_FRAME_LEN  36

static uint8_t  s_fog_frame[KVH_FRAME_LEN];
static uint8_t  s_fog_idx  = 0;
static bool     s_fog_sync = false;

static float be32f(const uint8_t *p) {
    union { uint8_t b[4]; float f; } u;
    u.b[3]=p[0]; u.b[2]=p[1]; u.b[1]=p[2]; u.b[0]=p[3];
    return u.f;
}

static uint16_t crc16_ccitt(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/* Called from UART1 ISR or polling in Core 1 */
void imu_fog_feed(uint8_t byte) {
    if (!s_fog_sync) {
        if (byte == 0x01) { s_fog_frame[0] = 0x01; s_fog_sync = true; s_fog_idx = 1; }
        return;
    }
    s_fog_frame[s_fog_idx++] = byte;
    if (s_fog_idx == 1 && byte != 0xFE) { s_fog_sync = false; return; }
    if (s_fog_idx < KVH_FRAME_LEN) return;

    s_fog_sync = false;
    uint16_t rx_crc = (uint16_t)((s_fog_frame[34] << 8) | s_fog_frame[35]);
    if (crc16_ccitt(s_fog_frame, 34) != rx_crc) return;

    ImuMeas m;
    m.ts_us = time_us_64();
    m.valid = true;
    /* KVH outputs delta-angle (rad) and delta-velocity (m/s) at 1000 Hz */
    /* Convert to rate by dividing by dt=0.001s */
    float dt_inv = 1000.0f;
    m.gyro_rps.x  = be32f(s_fog_frame +  3) * dt_inv;
    m.gyro_rps.y  = be32f(s_fog_frame +  7) * dt_inv;
    m.gyro_rps.z  = be32f(s_fog_frame + 11) * dt_inv;
    m.accel_mss.x = be32f(s_fog_frame + 15) * dt_inv;
    m.accel_mss.y = be32f(s_fog_frame + 19) * dt_inv;
    m.accel_mss.z = be32f(s_fog_frame + 23) * dt_inv;
    m.temp_c      = be32f(s_fog_frame + 27);

    __dmb();
    s_imu_shared.seq++;
    __dmb();
    s_imu_shared.meas = m;
    __dmb();
    s_imu_shared.seq++;
    __dmb();
}
#endif /* NAV_FEAT_FOG_IMU */

/* ─── PUBLIC API ─────────────────────────────────────────── */

bool imu_init(ImuGrade *out_grade) {
#if NAV_FEAT_FOG_IMU
    /* FOG: just configure the UART; frame sync handled in feed() */
    uart_set_baudrate(COMMS_UART_LIDAR_FOG, COMMS_BAUD_FOG);
    if (out_grade) *out_grade = IMU_GRADE_NAVIGATION;
    return true;
#else
    bool ok = bmi088_init_gyro() && bmi088_init_accel();
    if (out_grade) *out_grade = IMU_GRADE_INDUSTRIAL;
    return ok;
#endif
}

/* Start 1 kHz hardware repeating timer on Core 0 */
static repeating_timer_t s_imu_timer;
void imu_start_timer(void) {
#if !NAV_FEAT_FOG_IMU
    add_repeating_timer_us(-1000, s_imu_alarm_callback, NULL, &s_imu_timer);
#endif
}

/* Lock-free read (Core 1 / EKF thread) */
bool imu_meas_get(ImuMeas *out) {
    uint32_t s1, s2;
    do {
        s1 = s_imu_shared.seq;
        __dmb();
        *out = s_imu_shared.meas;
        __dmb();
        s2 = s_imu_shared.seq;
    } while ((s1 & 1u) || s1 != s2);
    return out->valid;
}

/* Run static IMU calibration (stationary, called before arming) */
void imu_calibrate_static(uint32_t samples) {
    double sum_gx=0, sum_gy=0, sum_gz=0;
    ImuMeas m;
    for (uint32_t i = 0; i < samples; i++) {
        sleep_ms(1);
        if (imu_meas_get(&m) && m.valid) {
            sum_gx += m.gyro_rps.x;
            sum_gy += m.gyro_rps.y;
            sum_gz += m.gyro_rps.z;
        }
    }
    s_gyro_cal.b0[0] = (float)(sum_gx / samples);
    s_gyro_cal.b0[1] = (float)(sum_gy / samples);
    s_gyro_cal.b0[2] = (float)(sum_gz / samples);
}
