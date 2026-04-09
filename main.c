/**
 * main.c — Navigation System Entry Point
 *
 * RP2350 Dual-Core Partition:
 *
 *  Core 0 — DETERMINISTIC REAL-TIME (1 kHz hard loop)
 *  ─────────────────────────────────────────────────
 *  • BMI088 ISR via hardware repeating timer
 *  • EKF prediction step (nav_fsm_predict)
 *  • Motor / servo output (PWM/DSHOT)  ← left to flight controller layer
 *
 *  Core 1 — SENSOR SERVICE & ESTIMATION UPDATE (~100 Hz)
 *  ─────────────────────────────────────────────────────
 *  • GNSS UART ring-buffer drain + parser
 *  • LiDAR / FOG UART ring-buffer drain + parser
 *  • Companion CPU MAVLink drain (VIO / SLAM / TERCOM)
 *  • BMM350 magnetometer poll (400 Hz)
 *  • BMP580 barometer poll (50 Hz)
 *  • MS4525 airspeed poll (100 Hz)
 *  • PMW3901 optical flow poll (100 Hz)
 *  • nav_fsm_update() — level selection + EKF measurement updates
 *  • Telemetry output
 *
 *  UART ISR — fills ring buffers for GNSS, LiDAR, Companion
 */

#include "nav_system.h"
#include "comms/comms.h"
#include "sensors/sensor_hal.h"
#include "sensors/imu_driver.h"
#include "sensors/gnss_driver.h"
#include "sensors/peripheral_drivers.h"
#include "ekf/ekf.h"
#include "fsm/nav_fsm.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

/* ─── GLOBAL INSTANCES ───────────────────────────────────── */
NavContext    g_nav;
SensorBundle  g_sensors;
EkfState      g_ekf;
UartRing      g_ring_gnss;
UartRing      g_ring_lidar;
UartRing      g_ring_companion;

/* ─── UART INTERRUPT HANDLERS (fill ring buffers) ────────── */
static void uart0_irq_handler(void) {   /* GNSS */
    while (uart_is_readable(COMMS_UART_GNSS))
        uart_ring_push(&g_ring_gnss, uart_getc(COMMS_UART_GNSS));
}

static void uart1_irq_handler(void) {   /* LiDAR / FOG */
    while (uart_is_readable(COMMS_UART_LIDAR_FOG)) {
        uint8_t b = uart_getc(COMMS_UART_LIDAR_FOG);
        uart_ring_push(&g_ring_lidar, b);
#if NAV_FEAT_FOG_IMU
        imu_fog_feed(b);               /* also route to FOG parser */
#endif
    }
}

static void uart2_irq_handler(void) {   /* Companion CPU */
    while (uart_is_readable(COMMS_UART_COMPANION))
        uart_ring_push(&g_ring_companion, uart_getc(COMMS_UART_COMPANION));
}

static void uarts_irq_init(void) {
    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
    uart_set_irq_enables(COMMS_UART_GNSS, true, false);
    irq_set_enabled(UART0_IRQ, true);

    irq_set_exclusive_handler(UART1_IRQ, uart1_irq_handler);
    uart_set_irq_enables(COMMS_UART_LIDAR_FOG, true, false);
    irq_set_enabled(UART1_IRQ, true);

    irq_set_exclusive_handler(UART2_IRQ, uart2_irq_handler);
    uart_set_irq_enables(COMMS_UART_COMPANION, true, false);
    irq_set_enabled(UART2_IRQ, true);
}

/* ─── INITIAL ALIGNMENT ──────────────────────────────────── */
/**
 * Coarse alignment while stationary on ground:
 *   - gravity vector  → roll & pitch
 *   - magnetometer    → yaw
 *   - average 1000 IMU samples (1 s) for bias estimation
 */
static void nav_coarse_align(void) {
    printf("[NAV] Coarse alignment started...\n");

    /* 1. Wait for first GNSS fix (up to 60 s) */
    uint64_t t0 = time_us_64();
    while (!g_sensors.gnss.valid && (time_us_64() - t0) < 60000000ULL) {
        gnss_poll();
        sleep_ms(100);
    }

    /* 2. Collect 1 s of IMU data for static bias estimate */
    imu_calibrate_static(1000);

    /* 3. Average accelerometer to find gravity direction */
    double sum_ax=0, sum_ay=0, sum_az=0;
    ImuMeas m;
    for (int i = 0; i < 200; i++) {
        sleep_ms(5);
        if (imu_meas_get(&m)) {
            sum_ax += m.accel_mss.x;
            sum_ay += m.accel_mss.y;
            sum_az += m.accel_mss.z;
        }
    }
    float ax = (float)(sum_ax/200);
    float ay = (float)(sum_ay/200);
    float az = (float)(sum_az/200);

    /* Roll and pitch from gravity */
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    /* Yaw from magnetometer (tilt-compensated) */
    mag_poll();
    Vec3 e = { roll * NAV_RAD2DEG, pitch * NAV_RAD2DEG, 0 };
    mag_compute_heading(&e);
    float yaw = g_sensors.mag.heading_deg * NAV_DEG2RAD;

    /* Build initial attitude quaternion from Euler angles */
    float cr=cosf(roll/2),  sr=sinf(roll/2);
    float cp=cosf(pitch/2), sp=sinf(pitch/2);
    float cy=cosf(yaw/2),   sy=sinf(yaw/2);
    Quat q_init = {
        .w = cr*cp*cy + sr*sp*sy,
        .x = sr*cp*cy - cr*sp*sy,
        .y = cr*sp*cy + sr*cp*sy,
        .z = cr*cp*sy - sr*sp*cy
    };

    /* Set initial nav state */
    GeoPos pos_init = g_sensors.gnss.valid ? g_sensors.gnss.pos
                                           : (GeoPos){0,0,0};
    Vec3 vel_init = {0,0,0};
    ekf_set_initial_state(&pos_init, &vel_init, &q_init);
    g_nav.initialized = true;

    printf("[NAV] Alignment complete: roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
           roll*NAV_RAD2DEG, pitch*NAV_RAD2DEG, yaw*NAV_RAD2DEG);
}

/* ─── CORE 1 ENTRY ───────────────────────────────────────── */
static void core1_main(void) {
    /* Rate divisors relative to 1 kHz base (counts in units of 1 ms) */
    uint32_t tick   = 0;
    uint32_t t_last = time_us_32();

    while (true) {
        /* Wait for next 1 ms slot */
        while ((time_us_32() - t_last) < 1000u) tight_loop_contents();
        t_last += 1000u;
        tick++;

        /* 1 kHz: drain all UART rings */
        gnss_poll();
        lidar1d_poll();
        companion_poll();

        /* 2.5 Hz effective — airspeed (every 4th tick = 250 Hz → /4 = 62.5 Hz ok) */
        if (tick % 10 == 0) airspeed_poll();

        /* 100 Hz: optical flow + magnetometer */
        if (tick % 10 == 0) {
            flow_poll();
            mag_poll();
            if (g_nav.state.valid)
                mag_compute_heading(&g_nav.state.euler_deg);
        }

        /* 50 Hz: barometer */
        if (tick % 20 == 0) baro_poll();

        /* 100 Hz: navigation FSM update (health + EKF measurement updates) */
        if (tick % 10 == 0) nav_fsm_update();

        /* 1 Hz: telemetry printout */
        if (tick % 1000 == 0) {
            NavState s;
            if (nav_fsm_get_state(&s)) {
                printf("[NAV lvl=%s] lat=%.6f lon=%.6f alt=%.1fm "
                       "vN=%.2f vE=%.2f vD=%.2f "
                       "roll=%.1f pitch=%.1f yaw=%.1f\n",
                       nav_level_name(nav_fsm_get_level()),
                       s.pos.lat_deg, s.pos.lon_deg, s.pos.alt_m,
                       s.vel_ned.x, s.vel_ned.y, s.vel_ned.z,
                       s.euler_deg.x, s.euler_deg.y, s.euler_deg.z);
            }
        }
    }
}

/* ─── CORE 0 ENTRY ───────────────────────────────────────── */
int main(void) {
    stdio_init_all();
    sleep_ms(500);   /* allow USB CDC to enumerate */

    printf("[NAV] General Navigation System v%d.%d.%d\n",
           NAV_VERSION_MAJOR, NAV_VERSION_MINOR, NAV_VERSION_PATCH);

    /* 1. Hardware initialisation */
    comms_init_all();
    uarts_irq_init();

    /* 2. Sensor initialisation */
    ImuGrade grade;
    if (!imu_init(&grade)) {
        printf("[FATAL] IMU init failed\n"); while(1);
    }
    printf("[NAV] IMU grade: %d\n", (int)grade);

    ekf_init(grade);
    nav_fsm_init();

    gnss_init();
    bool mag_ok   = mag_init();
    bool baro_ok  = baro_init();
    bool flow_ok  = flow_init();
    bool air_ok   = airspeed_init();

    printf("[NAV] Sensors: mag=%d baro=%d flow=%d airspeed=%d\n",
           mag_ok, baro_ok, flow_ok, air_ok);

    /* 3. Static alignment (blocks until complete) */
    nav_coarse_align();

    /* 4. Launch Core 1 sensor/estimation loop */
    multicore_launch_core1(core1_main);

    /* 5. Core 0: start 1 kHz IMU timer + tight EKF predict loop */
    imu_start_timer();

    while (true) {
        ImuMeas m;
        if (imu_meas_get(&m)) {
            nav_fsm_predict(&m);
        }
        /* Yield to allow timer callback to fire */
        tight_loop_contents();
    }
    return 0;
}
