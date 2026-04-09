/**
 * fsm/nav_fsm.c — Navigation Fallback State Machine
 *
 * This is the TOP-LEVEL controller.  Every NAV_IMU_RATE_HZ tick it:
 *   1. Assesses sensor health
 *   2. Selects the best available navigation level
 *   3. Routes measurements to the EKF
 *   4. Triggers level transitions with hysteresis
 *   5. Logs transitions
 *
 * ┌───────────────────────────────────────────────────────────────┐
 * │  LEVEL 0  GNSS + INS     ←→  LEVEL 1  INS + Mag/Baro/AS      │
 * │       ↓ GNSS denied                ↓ drift excessive         │
 * │  LEVEL 2  Visual (VIO/Flow)  ←→   LEVEL 3  LiDAR SLAM        │
 * │       ↓ featureless                ↓ no SLAM                 │
 * │  LEVEL 4  TERCOM               LEVEL 5  Target Acq           │
 * │       ↓ all else fails                                        │
 * │  LEVEL 6  Pure Dead Reckoning   →  LEVEL FAULT               │
 * └───────────────────────────────────────────────────────────────┘
 */

#include "nav_fsm.h"
#include "../ekf/ekf.h"
#include "../sensors/sensor_hal.h"
#include <stdio.h>

NavContext g_nav;

/* ─── TRANSITION LOG ─────────────────────────────────────── */
static void fsm_log_transition(NavLevel from, NavLevel to, const char *reason) {
    (void)from;
    /* In production: log to telemetry / black box */
    printf("[NAV] %s → %s  (%s)\n",
           nav_level_name(from), nav_level_name(to), reason);
}

/* ─── SENSOR HEALTH ASSESSMENT ───────────────────────────── */
static void fsm_assess_health(float now_s) {
    SensorHealth h = 0;

    /* IMU: always check first — system is dead without it */
    if (g_sensors.imu.valid &&
        (now_s - (float)(g_sensors.imu.ts_us * 1e-6f)) < 0.005f)
        h |= SENS_HEALTH_IMU_OK;

    /* GNSS */
    if (g_sensors.gnss.valid &&
        (now_s - g_nav.gnss_last_fix_s) < (NAV_GNSS_TIMEOUT_MS * 1e-3f))
        h |= SENS_HEALTH_GNSS_OK;

    /* Update last GNSS fix time */
    if (g_sensors.gnss.valid) g_nav.gnss_last_fix_s = now_s;

    /* Magnetometer */
    if (g_sensors.mag.valid) h |= SENS_HEALTH_MAG_OK;

    /* Barometer — extra sanity vs INS altitude */
    if (g_sensors.baro.valid) {
        float baro_err = fabsf(g_sensors.baro.altitude_m - g_nav.state.pos.alt_m);
        if (baro_err < NAV_BARO_MAX_ERROR_M) h |= SENS_HEALTH_BARO_OK;
    }

    /* Airspeed */
    if (g_sensors.airspeed.valid) h |= SENS_HEALTH_AIRSPEED_OK;

    /* Optical Flow */
    if (g_sensors.flow.valid &&
        g_nav.state.pos.alt_m < NAV_FLOW_MAX_ALT_M)
        h |= SENS_HEALTH_FLOW_OK;

    /* 1-D LiDAR */
    if (g_sensors.lidar1d.valid) h |= SENS_HEALTH_LIDAR1D_OK;

    /* 3-D LiDAR SLAM (via companion) */
    float lidar3d_age = now_s - (float)(g_sensors.lidar3d.ts_us * 1e-6f);
    if (g_sensors.lidar3d.valid && lidar3d_age < 0.2f)
        h |= SENS_HEALTH_LIDAR3D_OK;

    /* VIO (via companion) */
    float vio_age = now_s - (float)(g_sensors.vio.ts_us * 1e-6f);
    if (g_sensors.vio.valid && vio_age < 0.2f)
        h |= SENS_HEALTH_VIO_OK;

    /* TERCOM */
    float tercom_age = now_s - (float)(g_sensors.tercom.ts_us * 1e-6f);
    if (g_sensors.tercom.valid && tercom_age < NAV_TERCOM_INTERVAL_S)
        h |= SENS_HEALTH_TERCOM_OK;

    g_nav.health = h;
}

/* ─── LEVEL SELECTION (hysteresis: must see condition for 2 s) ── */
#define HYSTERESIS_S    2.0f

static float s_level_candidate_time = 0;
static NavLevel s_level_candidate    = NAV_LEVEL_0_GNSS_INS;

static NavLevel fsm_select_level(void) {
    NavLevel best;
    SensorHealth h = g_nav.health;

    /* Priority: highest confidence source first */
    if (h & SENS_HEALTH_GNSS_OK)
        best = NAV_LEVEL_0_GNSS_INS;
    else if ((h & SENS_HEALTH_MAG_OK) && (h & SENS_HEALTH_BARO_OK))
        best = NAV_LEVEL_1_INS_SENSORS;
    else if ((h & SENS_HEALTH_VIO_OK) || (h & SENS_HEALTH_FLOW_OK))
        best = NAV_LEVEL_2_VISUAL;
    else if (h & SENS_HEALTH_LIDAR3D_OK)
        best = NAV_LEVEL_3_LIDAR_SLAM;
    else if (h & SENS_HEALTH_TERCOM_OK)
        best = NAV_LEVEL_4_TERCOM;
    else if (g_sensors.target.locked)
        best = NAV_LEVEL_5_TARGET;
    else
        best = NAV_LEVEL_6_DEAD_RECK;

    /* Never upgrade to a lower-confidence level instantly */
    if (best > g_nav.level) return best;   /* immediate downgrade */

    /* Upgrade requires hysteresis */
    float now_s = (float)(time_us_64() * 1e-6f);
    if (best < g_nav.level) {
        if (best != s_level_candidate) {
            s_level_candidate      = best;
            s_level_candidate_time = now_s;
        } else if ((now_s - s_level_candidate_time) >= HYSTERESIS_S) {
            return best;
        }
    }
    return g_nav.level;   /* hold current */
}

/* ─── LEVEL-SPECIFIC EKF UPDATE ROUTING ─────────────────── */
static void fsm_route_measurements(NavLevel lvl, float now_s) {
    switch (lvl) {

    case NAV_LEVEL_0_GNSS_INS:
        if (g_sensors.gnss.valid)
            ekf_update_gnss(&g_sensors.gnss);
        /* Always use baro for altitude even with GNSS */
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        if (g_nav.health & SENS_HEALTH_MAG_OK)
            ekf_update_mag_hdg(g_sensors.mag.heading_deg);
        break;

    case NAV_LEVEL_1_INS_SENSORS:
        /* GNSS gone — rely on mag heading + baro altitude + airspeed velocity */
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        if (g_nav.health & SENS_HEALTH_MAG_OK)
            ekf_update_mag_hdg(g_sensors.mag.heading_deg);
        /* Airspeed: inject as horizontal velocity magnitude (with heading) */
        if (g_nav.health & SENS_HEALTH_AIRSPEED_OK) {
            float hdg = g_sensors.mag.heading_deg * NAV_DEG2RAD;
            float as  = g_sensors.airspeed.airspeed_mss;
            /* Synthesise FlowMeas for velocity update */
            FlowMeas fm = {
                .vx_mss  = as * cosf(hdg),
                .vy_mss  = as * sinf(hdg),
                .quality = 0.7f, .valid = true
            };
            ekf_update_flow(&fm);
        }
        break;

    case NAV_LEVEL_2_VISUAL:
        if (g_nav.health & SENS_HEALTH_VIO_OK)
            ekf_update_vio(&g_sensors.vio);
        if (g_nav.health & SENS_HEALTH_FLOW_OK)
            ekf_update_flow(&g_sensors.flow);
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        if (g_nav.health & SENS_HEALTH_MAG_OK)
            ekf_update_mag_hdg(g_sensors.mag.heading_deg);
        break;

    case NAV_LEVEL_3_LIDAR_SLAM:
        if (g_nav.health & SENS_HEALTH_LIDAR3D_OK)
            ekf_update_lidar3d(&g_sensors.lidar3d);
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        if (g_nav.health & SENS_HEALTH_MAG_OK)
            ekf_update_mag_hdg(g_sensors.mag.heading_deg);
        break;

    case NAV_LEVEL_4_TERCOM:
        /* Periodic TERCOM position reset */
        if (g_nav.health & SENS_HEALTH_TERCOM_OK)
            ekf_update_tercom(&g_sensors.tercom);
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        if (g_nav.health & SENS_HEALTH_MAG_OK)
            ekf_update_mag_hdg(g_sensors.mag.heading_deg);
        break;

    case NAV_LEVEL_5_TARGET:
        /* Target acquisition: maintain INS + baro; guidance takes over */
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        if (g_nav.health & SENS_HEALTH_MAG_OK)
            ekf_update_mag_hdg(g_sensors.mag.heading_deg);
        /* Note: terminal guidance uses g_sensors.target directly, not EKF pos */
        break;

    case NAV_LEVEL_6_DEAD_RECK:
        /* Pure INS — only baro altitude anchor if available */
        if (g_nav.health & SENS_HEALTH_BARO_OK)
            ekf_update_baro(&g_sensors.baro);
        g_nav.dr_elapsed_s += NAV_IMU_DT_S;
        if (g_nav.dr_elapsed_s > NAV_DEAD_RECKONING_MAX_S) {
            snprintf(g_nav.fault_msg, sizeof(g_nav.fault_msg),
                     "Dead reckoning exceeded %.0f s — position unreliable",
                     NAV_DEAD_RECKONING_MAX_S);
            /* Trigger FAULT in calling loop if needed */
        }
        break;

    case NAV_LEVEL_FAULT:
        break;
    }

    (void)now_s;
}

/* ─── PUBLIC API ─────────────────────────────────────────── */

void nav_fsm_init(void) {
    memset(&g_nav, 0, sizeof(g_nav));
    g_nav.level = NAV_LEVEL_6_DEAD_RECK;  /* start conservative */
}

/* Called every IMU tick (Core 0, 1 kHz) — EKF predict */
void nav_fsm_predict(const ImuMeas *imu) {
    ekf_predict(imu, NAV_IMU_DT_S);
}

/* Called from Core 1 sensor task (~100 Hz) — health + updates */
void nav_fsm_update(void) {
    float now_s = (float)(time_us_64() * 1e-6f);

    /* 1. Health check */
    fsm_assess_health(now_s);

    /* 2. Abort if IMU failed */
    if (!(g_nav.health & SENS_HEALTH_IMU_OK)) {
        if (g_nav.level != NAV_LEVEL_FAULT) {
            fsm_log_transition(g_nav.level, NAV_LEVEL_FAULT, "IMU failure");
            g_nav.level = NAV_LEVEL_FAULT;
        }
        return;
    }

    /* 3. Level selection with hysteresis */
    NavLevel new_level = fsm_select_level();
    if (new_level != g_nav.level) {
        static const char *reasons[] = {
            "GNSS restored",     "GNSS lost, using sensors",
            "Using visual nav",  "Using LiDAR SLAM",
            "Using TERCOM",      "Target locked",
            "All aiding lost",   "System fault"
        };
        fsm_log_transition(g_nav.level, new_level,
                            reasons[new_level < 8 ? new_level : 7]);
        if (new_level == NAV_LEVEL_6_DEAD_RECK)
            g_nav.dr_elapsed_s = 0;
        g_nav.level = new_level;
    }

    /* 4. Route measurements to EKF */
    fsm_route_measurements(g_nav.level, now_s);

    /* 5. Pull updated estimate */
    ekf_get_nav_state(&g_nav.state);
}

/* Retrieve current navigation output */
bool nav_fsm_get_state(NavState *out) {
    if (!g_nav.initialized || g_nav.level == NAV_LEVEL_FAULT) return false;
    *out = g_nav.state;
    return out->valid;
}

NavLevel nav_fsm_get_level(void) { return g_nav.level; }
SensorHealth nav_fsm_get_health(void) { return g_nav.health; }
