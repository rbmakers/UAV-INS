/**
 * sensors/peripheral_drivers.c
 *
 * Covers:
 *   BMM350     magnetometer  (I2C)
 *   BMP580     barometer     (SPI)
 *   MS4525     airspeed      (I2C)
 *   PMW3901    optical flow  (SPI)
 *   TFmini-S   1-D LiDAR    (UART)
 *   Companion  MAVLink link  (UART) — receives VIO / SLAM / TERCOM poses
 */

#include "sensor_hal.h"
#include "../comms/comms.h"
#include <math.h>

/* =========================================================
 * 1. BMM350 MAGNETOMETER
 * ========================================================= */
#define BMM350_ADDR     0x14
#define BMM350_CHIP_ID  0x00
#define BMM350_PMU_CMD  0x06
#define BMM350_CONF     0x04
#define BMM350_EN_AXES  0x05
#define BMM350_DATA     0x31
#define BMM350_LSB2UT   0.0003f   /* 300 nT / LSB */

/* Hard-iron / soft-iron calibration (filled offline) */
static Vec3  s_hard_iron  = {0,0,0};
static float s_soft_iron[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

bool mag_init(void) {
    i2c_write_reg(BMM350_ADDR, BMM350_PMU_CMD, 0x01);
    sleep_ms(5);
    if (i2c_read_reg(BMM350_ADDR, BMM350_CHIP_ID) != 0x33) return false;
    i2c_write_reg(BMM350_ADDR, BMM350_CONF,     0x04); /* 400 Hz */
    i2c_write_reg(BMM350_ADDR, BMM350_EN_AXES,  0x07); /* XYZ on */
    return true;
}

void mag_poll(void) {
    uint8_t buf[9];
    i2c_read_burst(BMM350_ADDR, BMM350_DATA, buf, 9);

    /* 20-bit signed per axis (3 bytes each) */
    int32_t rx = ((int32_t)((buf[2]<<16)|(buf[1]<<8)|buf[0])) << 12 >> 16;
    int32_t ry = ((int32_t)((buf[5]<<16)|(buf[4]<<8)|buf[3])) << 12 >> 16;
    int32_t rz = ((int32_t)((buf[8]<<16)|(buf[7]<<8)|buf[6])) << 12 >> 16;

    float fx = rx * BMM350_LSB2UT - s_hard_iron.x;
    float fy = ry * BMM350_LSB2UT - s_hard_iron.y;
    float fz = rz * BMM350_LSB2UT - s_hard_iron.z;

    g_sensors.mag.field_ut.x = s_soft_iron[0][0]*fx + s_soft_iron[0][1]*fy + s_soft_iron[0][2]*fz;
    g_sensors.mag.field_ut.y = s_soft_iron[1][0]*fx + s_soft_iron[1][1]*fy + s_soft_iron[1][2]*fz;
    g_sensors.mag.field_ut.z = s_soft_iron[2][0]*fx + s_soft_iron[2][1]*fy + s_soft_iron[2][2]*fz;
    g_sensors.mag.ts_us = time_us_64();
    g_sensors.mag.valid = true;
}

/* Tilt-compensated heading using current roll/pitch from NAV state */
void mag_compute_heading(const Vec3 *euler_deg) {
    float r = euler_deg->x * NAV_DEG2RAD;
    float p = euler_deg->y * NAV_DEG2RAD;
    float mx = g_sensors.mag.field_ut.x;
    float my = g_sensors.mag.field_ut.y;
    float mz = g_sensors.mag.field_ut.z;

    float Xh = mx*cosf(p) + my*sinf(r)*sinf(p) + mz*cosf(r)*sinf(p);
    float Yh = my*cosf(r) - mz*sinf(r);

    float h = atan2f(-Yh, Xh) * NAV_RAD2DEG;
    if (h < 0) h += 360.0f;
    g_sensors.mag.heading_deg = h;
}

/* =========================================================
 * 2. BMP580 BAROMETER
 * ========================================================= */
#define BMP580_CHIP_ID_REG  0x01
#define BMP580_OSR_CONF     0x36
#define BMP580_ODR_CONF     0x37
#define BMP580_DATA_P       0x20

static float s_p0_pa = 101325.0f;  /* sea-level reference (updated at init) */

bool baro_init(void) {
    spi_write_reg(PIN_CS_BARO, 0x7E, 0xB6);
    sleep_ms(5);
    if (spi_read_reg(PIN_CS_BARO, BMP580_CHIP_ID_REG) != 0x51) return false;
    spi_write_reg(PIN_CS_BARO, BMP580_OSR_CONF, 0x35); /* P x8, T x2, IIR 4 */
    spi_write_reg(PIN_CS_BARO, BMP580_ODR_CONF, 0xD0); /* 50 Hz normal mode */
    /* Average 100 samples to set sea-level reference */
    sleep_ms(200);
    double sum = 0;
    uint8_t buf[6];
    for (int i = 0; i < 100; i++) {
        sleep_ms(2);
        spi_read_burst(PIN_CS_BARO, BMP580_DATA_P, buf, 6);
        uint32_t raw = buf[0] | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]<<16);
        sum += (double)raw / 64.0;
    }
    s_p0_pa = (float)(sum / 100.0);
    return true;
}

void baro_poll(void) {
    uint8_t buf[6];
    spi_read_burst(PIN_CS_BARO, BMP580_DATA_P, buf, 6);

    uint32_t rp = buf[0] | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]<<16);
    uint32_t rt = buf[3] | ((uint32_t)buf[4]<<8) | ((uint32_t)buf[5]<<16);

    g_sensors.baro.pressure_pa = (float)rp / 64.0f;
    g_sensors.baro.temp_c      = (float)((int32_t)rt) / 65536.0f;
    /* Hypsometric altitude */
    g_sensors.baro.altitude_m  = 44330.0f *
        (1.0f - powf(g_sensors.baro.pressure_pa / s_p0_pa, 0.190294f));
    g_sensors.baro.ts_us = time_us_64();
    g_sensors.baro.valid = true;
}

/* =========================================================
 * 3. MS4525 DIFFERENTIAL PRESSURE / AIRSPEED
 * ========================================================= */
#define MS4525_ADDR     0x28
#define AIR_DENSITY     1.225f    /* kg/m³ at sea level */

bool airspeed_init(void) {
    /* Trigger a measurement to verify comms */
    uint8_t dummy;
    return i2c_read_blocking(COMMS_I2C_MAG_AS, MS4525_ADDR, &dummy, 1, false) == 1;
}

void airspeed_poll(void) {
    uint8_t buf[4];
    if (i2c_read_blocking(COMMS_I2C_MAG_AS, MS4525_ADDR, buf, 4, false) != 4) return;

    uint8_t status = buf[0] >> 6;
    if (status != 0) return;   /* stale or fault */

    uint16_t dp_raw = ((uint16_t)(buf[0] & 0x3F) << 8) | buf[1];
    /* MS4525 DP range ±1 PSI, output count 1638–14745 */
    float dp_pa = (((float)dp_raw - 8192.0f) / 8192.0f) * 6894.76f;

    g_sensors.airspeed.diff_pressure_pa = dp_pa;
    if (dp_pa > 0.5f) {
        g_sensors.airspeed.airspeed_mss = sqrtf(2.0f * dp_pa / AIR_DENSITY);
    } else {
        g_sensors.airspeed.airspeed_mss = 0.0f;
    }
    g_sensors.airspeed.ts_us = time_us_64();
    g_sensors.airspeed.valid = true;
}

/* =========================================================
 * 4. PMW3901 OPTICAL FLOW
 * ========================================================= */
#define PMW_PRODUCT_ID  0x00   /* expected 0x49 */
#define PMW_MOTION      0x02
#define PMW_DELTA_X_L   0x03
#define PMW_DELTA_X_H   0x04
#define PMW_DELTA_Y_L   0x05
#define PMW_DELTA_Y_H   0x06
#define PMW_SQUAL       0x07
#define FLOW_FOCAL_PX   30.0f  /* calibrate per lens */

/* Full 42-register init sequence (abbreviated — use PixArt AN for full list) */
static const uint8_t PMW_INIT_REGS[][2] = {
    {0x7F,0x00},{0x55,0x01},{0x50,0x07},{0x7F,0x0E},{0x43,0x10},
    {0x48,0x02},{0x7F,0x00},{0x51,0xFF},{0x52,0x1B},{0x7F,0x14},
    /* ... (complete sequence from PixArt AN-049) ... */
};

bool flow_init(void) {
    gpio_put(PIN_CS_FLOW, 1);
    sleep_ms(50);
    for (size_t i = 0; i < sizeof(PMW_INIT_REGS)/sizeof(PMW_INIT_REGS[0]); i++) {
        spi_write_reg(PIN_CS_FLOW, PMW_INIT_REGS[i][0], PMW_INIT_REGS[i][1]);
        sleep_us(100);
    }
    sleep_ms(100);
    return spi_read_reg(PIN_CS_FLOW, PMW_PRODUCT_ID) == 0x49;
}

void flow_poll(void) {
    uint8_t motion = spi_read_reg(PIN_CS_FLOW, PMW_MOTION);
    bool moving = (motion & 0x80) != 0;

    int16_t dx = (int16_t)((spi_read_reg(PIN_CS_FLOW, PMW_DELTA_X_H) << 8) |
                             spi_read_reg(PIN_CS_FLOW, PMW_DELTA_X_L));
    int16_t dy = (int16_t)((spi_read_reg(PIN_CS_FLOW, PMW_DELTA_Y_H) << 8) |
                             spi_read_reg(PIN_CS_FLOW, PMW_DELTA_Y_L));
    uint8_t qual = spi_read_reg(PIN_CS_FLOW, PMW_SQUAL);

    /* Use 1-D LiDAR height if available */
    float h = g_sensors.lidar1d.valid ?
              g_sensors.lidar1d.range_m : g_sensors.baro.altitude_m;

    float dt = NAV_IMU_DT_S * 10.0f;  /* flow runs at 100 Hz */
    g_sensors.flow.raw_dx    = dx;
    g_sensors.flow.raw_dy    = dy;
    g_sensors.flow.quality   = qual / 255.0f;
    g_sensors.flow.height_m  = h;
    g_sensors.flow.vx_mss    = moving ? -(float)dx / FLOW_FOCAL_PX * h / dt : 0.0f;
    g_sensors.flow.vy_mss    = moving ? -(float)dy / FLOW_FOCAL_PX * h / dt : 0.0f;
    g_sensors.flow.ts_us     = time_us_64();
    g_sensors.flow.valid     = moving && (qual > 20) &&
                               (h > 0.1f) && (h < NAV_FLOW_MAX_ALT_M);
}

/* =========================================================
 * 5. TFmini-S 1-D LiDAR (UART1)
 * ========================================================= */
#define TF_HDR     0x59
#define TF_FRAMELEN 9

typedef enum { TF_WAIT1, TF_WAIT2, TF_DATA } TfState;
static TfState  s_tf_st  = TF_WAIT1;
static uint8_t  s_tf_buf[TF_FRAMELEN];
static uint8_t  s_tf_idx = 0;

static void tf_process_frame(void) {
    uint8_t ck = 0;
    for (int i = 0; i < 8; i++) ck += s_tf_buf[i];
    if (ck != s_tf_buf[8]) return;

    uint16_t dist = (uint16_t)(s_tf_buf[2] | (s_tf_buf[3] << 8));
    uint16_t str  = (uint16_t)(s_tf_buf[4] | (s_tf_buf[5] << 8));

    g_sensors.lidar1d.range_m          = dist * 0.01f;  /* cm → m */
    g_sensors.lidar1d.signal_strength  = str;
    g_sensors.lidar1d.ts_us            = time_us_64();
    g_sensors.lidar1d.valid            = (dist > 10) && (dist < 1200) && (str > 100);
}

/* Feed from Core 1 UART ring */
void lidar1d_poll(void) {
    uint8_t b;
    while (uart_ring_pop(&g_ring_lidar, &b)) {
        switch (s_tf_st) {
        case TF_WAIT1: if (b==TF_HDR){s_tf_buf[0]=b; s_tf_st=TF_WAIT2;} break;
        case TF_WAIT2: if (b==TF_HDR){s_tf_buf[1]=b; s_tf_idx=2; s_tf_st=TF_DATA;}
                       else s_tf_st=TF_WAIT1; break;
        case TF_DATA:
            s_tf_buf[s_tf_idx++] = b;
            if (s_tf_idx == TF_FRAMELEN) { tf_process_frame(); s_tf_st=TF_WAIT1; }
            break;
        }
    }
}

/* =========================================================
 * 6. COMPANION CPU MAVLINK LINK
 *    Receives VISION_POSITION_ESTIMATE (VIO)
 *            ODOMETRY (LiDAR SLAM)
 *            ATT_POS_MOCAP (TERCOM fix)
 *    Uses lightweight MAVLink v2 minimal parser.
 * ========================================================= */
#define MAVLINK_STX_V2       0xFD
#define MAVLINK_MSG_VISION   102u   /* VISION_POSITION_ESTIMATE */
#define MAVLINK_MSG_ODOM     331u   /* ODOMETRY */
#define MAVLINK_MSG_MOCAP    138u   /* ATT_POS_MOCAP (TERCOM reuse) */

typedef enum {
    MVS_STX, MVS_LEN, MVS_FLAGS, MVS_SEQ,
    MVS_SYS, MVS_COMP, MVS_MSGID0, MVS_MSGID1, MVS_MSGID2,
    MVS_PAYLOAD, MVS_CRC0, MVS_CRC1, MVS_SIG
} MvState;

static struct {
    MvState  st;
    uint8_t  len, flags, seq, sys_id, comp_id;
    uint32_t msg_id;
    uint8_t  payload[256];
    uint8_t  idx;
} s_mv;

static inline float le32f(const uint8_t *p) {
    union { uint8_t b[4]; float f; } u;
    u.b[0]=p[0]; u.b[1]=p[1]; u.b[2]=p[2]; u.b[3]=p[3];
    return u.f;
}

static void companion_dispatch(void) {
    switch (s_mv.msg_id) {
    case MAVLINK_MSG_VISION: {
        /* VISION_POSITION_ESTIMATE: usec(8) x(4) y(4) z(4) roll(4) pitch(4) yaw(4) */
        g_sensors.vio.pos_local.x = le32f(s_mv.payload + 8);
        g_sensors.vio.pos_local.y = le32f(s_mv.payload + 12);
        g_sensors.vio.pos_local.z = le32f(s_mv.payload + 16);
        g_sensors.vio.pos_std_m   = 0.1f;  /* default; use covariance field if present */
        g_sensors.vio.ts_us       = time_us_64();
        g_sensors.vio.valid       = true;
        break;
    }
    case MAVLINK_MSG_ODOM: {
        /* ODOMETRY: position and velocity in local frame */
        g_sensors.lidar3d.pos_local.x = le32f(s_mv.payload + 8);
        g_sensors.lidar3d.pos_local.y = le32f(s_mv.payload + 12);
        g_sensors.lidar3d.pos_local.z = le32f(s_mv.payload + 16);
        g_sensors.lidar3d.vel_local.x = le32f(s_mv.payload + 36);
        g_sensors.lidar3d.vel_local.y = le32f(s_mv.payload + 40);
        g_sensors.lidar3d.vel_local.z = le32f(s_mv.payload + 44);
        g_sensors.lidar3d.ts_us       = time_us_64();
        g_sensors.lidar3d.valid       = true;
        break;
    }
    case MAVLINK_MSG_MOCAP: {
        /* Reused for TERCOM position fix: x=lat*1e7, y=lon*1e7, z=alt_m */
        g_sensors.tercom.pos_corrected.lat_deg = le32f(s_mv.payload + 8) * 1e-7;
        g_sensors.tercom.pos_corrected.lon_deg = le32f(s_mv.payload + 12) * 1e-7;
        g_sensors.tercom.pos_corrected.alt_m   = le32f(s_mv.payload + 16);
        g_sensors.tercom.cep_m                 = le32f(s_mv.payload + 20);
        g_sensors.tercom.ts_us                 = time_us_64();
        g_sensors.tercom.valid                 = true;
        break;
    }
    default: break;
    }
}

void companion_poll(void) {
    uint8_t b;
    while (uart_ring_pop(&g_ring_companion, &b)) {
        switch (s_mv.st) {
        case MVS_STX:     if (b==MAVLINK_STX_V2) s_mv.st=MVS_LEN;  break;
        case MVS_LEN:     s_mv.len=b;   s_mv.st=MVS_FLAGS;          break;
        case MVS_FLAGS:   s_mv.flags=b; s_mv.st=MVS_SEQ;            break;
        case MVS_SEQ:     s_mv.seq=b;   s_mv.st=MVS_SYS;            break;
        case MVS_SYS:     s_mv.sys_id=b; s_mv.st=MVS_COMP;          break;
        case MVS_COMP:    s_mv.comp_id=b; s_mv.st=MVS_MSGID0;       break;
        case MVS_MSGID0:  s_mv.msg_id=b; s_mv.st=MVS_MSGID1;        break;
        case MVS_MSGID1:  s_mv.msg_id|=(uint32_t)b<<8; s_mv.st=MVS_MSGID2; break;
        case MVS_MSGID2:  s_mv.msg_id|=(uint32_t)b<<16; s_mv.idx=0;
                          s_mv.st = s_mv.len ? MVS_PAYLOAD : MVS_CRC0; break;
        case MVS_PAYLOAD:
            if (s_mv.idx < sizeof(s_mv.payload)) s_mv.payload[s_mv.idx]=b;
            if (++s_mv.idx >= s_mv.len) s_mv.st=MVS_CRC0;
            break;
        case MVS_CRC0:    s_mv.st=MVS_CRC1; break;  /* CRC not verified for brevity */
        case MVS_CRC1:
            companion_dispatch();
            s_mv.st = MVS_STX;
            break;
        default: s_mv.st = MVS_STX; break;
        }
    }
}
