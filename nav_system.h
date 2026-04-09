#pragma once
/**
 * nav_system.h — General Navigation System
 * Covers: several km to several hundred km range
 * Target: RP2350 (primary MCU) + companion CPU (VIO / LiDAR SLAM)
 *
 * Architecture:  TOP-DOWN FALLBACK
 *   Level 0 — GNSS + INS tight-coupled EKF           (nominal, all ranges)
 *   Level 1 — INS + Mag + Baro + Airspeed            (GNSS lost, < 30 min)
 *   Level 2 — INS + Optical Flow / VIO               (GPS-denied, low-alt)
 *   Level 3 — INS + LiDAR SLAM                       (feature-rich terrain)
 *   Level 4 — INS + TERCOM terrain matching          (long-range over land)
 *   Level 5 — INS + EO/IR target acquisition         (terminal phase)
 *   Level 6 — Pure INS dead reckoning                (last resort)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* =========================================================
 * VERSION & BUILD
 * ========================================================= */
#define NAV_VERSION_MAJOR   1
#define NAV_VERSION_MINOR   0
#define NAV_VERSION_PATCH   0

/* =========================================================
 * COMPILE-TIME FEATURE FLAGS
 * Set to 1 to enable, 0 to disable each sensor
 * ========================================================= */
#define NAV_FEAT_GNSS           1   /* u-blox M9/M10 UBX */
#define NAV_FEAT_MEMS_IMU       1   /* BMI088 (primary) */
#define NAV_FEAT_FOG_IMU        0   /* KVH 1775 / ADIS16507 (optional upgrade) */
#define NAV_FEAT_MAGNETOMETER   1   /* BMM350 */
#define NAV_FEAT_BAROMETER      1   /* BMP580 */
#define NAV_FEAT_AIRSPEED       1   /* MS4525 pitot (fixed-wing) */
#define NAV_FEAT_OPTICAL_FLOW   1   /* PMW3901 */
#define NAV_FEAT_LIDAR_1D       1   /* TFmini-S rangefinder */
#define NAV_FEAT_LIDAR_3D       0   /* Livox/Ouster (companion CPU via MAVLink) */
#define NAV_FEAT_VIO            0   /* Visual-Inertial Odometry (companion CPU) */
#define NAV_FEAT_TERCOM         0   /* Terrain Contour Matching (long-range) */
#define NAV_FEAT_TARGET_ACQ     0   /* EO/IR terminal guidance (companion CPU) */

/* =========================================================
 * TIMING CONSTANTS
 * ========================================================= */
#define NAV_IMU_RATE_HZ         1000u
#define NAV_EKF_PREDICT_HZ      1000u
#define NAV_EKF_UPDATE_HZ_GNSS  10u
#define NAV_EKF_UPDATE_HZ_BARO  50u
#define NAV_EKF_UPDATE_HZ_MAG   100u
#define NAV_EKF_UPDATE_HZ_FLOW  100u
#define NAV_IMU_DT_S            (1.0f / NAV_IMU_RATE_HZ)

/* =========================================================
 * PHYSICAL CONSTANTS
 * ========================================================= */
#define NAV_GRAVITY_MSS         9.80665f
#define NAV_EARTH_RADIUS_M      6371000.0f
#define NAV_DEG2RAD             (float)(M_PI / 180.0)
#define NAV_RAD2DEG             (float)(180.0 / M_PI)

/* =========================================================
 * FALLBACK THRESHOLDS
 * ========================================================= */
#define NAV_GNSS_TIMEOUT_MS         2000u   /* declare GNSS lost after 2 s */
#define NAV_GNSS_MIN_SATS           4u      /* minimum satellites for 3D fix */
#define NAV_GNSS_MAX_HACC_M         50.0f   /* max acceptable horiz accuracy */
#define NAV_FLOW_MAX_ALT_M          30.0f   /* optical flow valid below this alt */
#define NAV_BARO_MAX_ERROR_M        200.0f  /* baro sanity check vs INS */
#define NAV_IMU_OUTLIER_G           10.0f   /* accel spike rejection threshold */
#define NAV_TERCOM_INTERVAL_S       30.0f   /* TERCOM position reset interval */
#define NAV_DEAD_RECKONING_MAX_S    300.0f  /* max pure INS time before warning */

/* =========================================================
 * 3D VECTOR / QUATERNION TYPES
 * ========================================================= */
typedef struct { float x, y, z; }          Vec3;
typedef struct { float w, x, y, z; }       Quat;
typedef struct { float m[3][3]; }           Mat3;

/* Geographic position */
typedef struct {
    double  lat_deg;    /* WGS-84 latitude  (degrees) */
    double  lon_deg;    /* WGS-84 longitude (degrees) */
    float   alt_m;      /* altitude above MSL (metres) */
} GeoPos;

/* Full navigation state (truth estimate) */
typedef struct {
    GeoPos  pos;            /* WGS-84 position */
    Vec3    vel_ned;        /* velocity in NED frame (m/s) */
    Quat    att;            /* attitude quaternion body→NED */
    Vec3    euler_deg;      /* roll/pitch/yaw (degrees, display only) */
    Vec3    accel_bias;     /* estimated accelerometer bias (m/s²) */
    Vec3    gyro_bias;      /* estimated gyroscope bias (rad/s) */
    float   timestamp_s;    /* system time at this estimate */
    bool    valid;
} NavState;

/* =========================================================
 * NAVIGATION FALLBACK LEVEL (state machine)
 * ========================================================= */
typedef enum {
    NAV_LEVEL_0_GNSS_INS    = 0,  /* Full GNSS + INS fusion */
    NAV_LEVEL_1_INS_SENSORS = 1,  /* INS + Mag + Baro + Airspeed */
    NAV_LEVEL_2_VISUAL      = 2,  /* INS + Optical Flow / VIO */
    NAV_LEVEL_3_LIDAR_SLAM  = 3,  /* INS + LiDAR SLAM */
    NAV_LEVEL_4_TERCOM      = 4,  /* INS + Terrain Matching */
    NAV_LEVEL_5_TARGET      = 5,  /* INS + Target Acquisition */
    NAV_LEVEL_6_DEAD_RECK   = 6,  /* Pure INS dead reckoning */
    NAV_LEVEL_FAULT         = 7,  /* Navigation fault — land/abort */
} NavLevel;

static inline const char *nav_level_name(NavLevel lvl) {
    static const char *names[] = {
        "GNSS+INS", "INS+Sensors", "Visual", "LiDAR-SLAM",
        "TERCOM", "TargetAcq", "DeadReck", "FAULT"
    };
    return (lvl <= NAV_LEVEL_FAULT) ? names[lvl] : "UNKNOWN";
}

/* =========================================================
 * SENSOR HEALTH FLAGS (bitfield)
 * ========================================================= */
typedef uint32_t SensorHealth;
#define SENS_HEALTH_IMU_OK      (1u << 0)
#define SENS_HEALTH_GNSS_OK     (1u << 1)
#define SENS_HEALTH_MAG_OK      (1u << 2)
#define SENS_HEALTH_BARO_OK     (1u << 3)
#define SENS_HEALTH_FLOW_OK     (1u << 4)
#define SENS_HEALTH_LIDAR1D_OK  (1u << 5)
#define SENS_HEALTH_LIDAR3D_OK  (1u << 6)
#define SENS_HEALTH_VIO_OK      (1u << 7)
#define SENS_HEALTH_TERCOM_OK   (1u << 8)
#define SENS_HEALTH_AIRSPEED_OK (1u << 9)
#define SENS_HEALTH_ALL_CORE    (SENS_HEALTH_IMU_OK | SENS_HEALTH_BARO_OK | SENS_HEALTH_MAG_OK)

/* =========================================================
 * SYSTEM CONTEXT (global singleton)
 * ========================================================= */
typedef struct {
    NavState    state;          /* current best navigation estimate */
    NavLevel    level;          /* active fallback level */
    SensorHealth health;        /* bitmask of healthy sensors */
    float       gnss_last_fix_s;/* timestamp of last valid GNSS fix */
    float       dr_elapsed_s;   /* seconds of pure dead reckoning */
    bool        initialized;
    char        fault_msg[64];
} NavContext;

extern NavContext g_nav;        /* defined in nav_fsm.c */
