/**
 * ekf/ekf.c — 15-State Error-State EKF Implementation
 */

#include "ekf.h"
#include <string.h>
#include <math.h>

EkfState g_ekf;

/* ─── SMALL HELPERS ──────────────────────────────────────── */
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 3×3 skew-symmetric matrix from vector */
static void skew3(const float v[3], float out[3][3]) {
    out[0][0]= 0;     out[0][1]=-v[2]; out[0][2]= v[1];
    out[1][0]= v[2];  out[1][1]= 0;    out[1][2]=-v[0];
    out[2][0]=-v[1];  out[2][1]= v[0]; out[2][2]= 0;
}

/* Rotation matrix from quaternion (body → NED) */
static void quat_to_dcm(const Quat *q, float R[3][3]) {
    float w=q->w, x=q->x, y=q->y, z=q->z;
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z); R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x); R[2][2]=1-2*(x*x+y*y);
}

/* Quaternion multiply q = a ⊗ b, then normalise */
static void qmul(const Quat *a, const Quat *b, Quat *q) {
    q->w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    q->x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    q->y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    q->z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;
    float n = sqrtf(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
    if (n > 1e-8f) { q->w/=n; q->x/=n; q->y/=n; q->z/=n; }
}

/* P = F*P*Fᵀ + Q (in-place, N=15) */
static void ekf_cov_predict(float P[EKF_N][EKF_N],
                              float F[EKF_N][EKF_N],
                              float q_diag[EKF_N]) {
    float FP[EKF_N][EKF_N] = {0};
    float FPFt[EKF_N][EKF_N] = {0};

    for (int i=0;i<EKF_N;i++)
        for (int j=0;j<EKF_N;j++)
            for (int k=0;k<EKF_N;k++)
                FP[i][j] += F[i][k]*P[k][j];

    for (int i=0;i<EKF_N;i++)
        for (int j=0;j<EKF_N;j++)
            for (int k=0;k<EKF_N;k++)
                FPFt[i][j] += FP[i][k]*F[j][k];  /* F[j][k] = Fᵀ[k][j] */

    for (int i=0;i<EKF_N;i++) {
        for (int j=0;j<EKF_N;j++)
            P[i][j] = FPFt[i][j];
        P[i][i] += q_diag[i];
    }
}

/* Generic scalar measurement update: H is 1×N row vector */
static float ekf_scalar_update(EkfState *e, const float H[EKF_N],
                                 float z, float R) {
    /* S = H*P*Hᵀ + R */
    float PH[EKF_N];
    for (int i=0;i<EKF_N;i++) {
        PH[i] = 0;
        for (int j=0;j<EKF_N;j++) PH[i] += e->P[i][j]*H[j];
    }
    float S = R;
    for (int j=0;j<EKF_N;j++) S += H[j]*PH[j];
    if (fabsf(S) < 1e-12f) return 0;

    /* K = PH / S */
    float K[EKF_N];
    for (int i=0;i<EKF_N;i++) K[i] = PH[i] / S;

    /* x += K*z */
    for (int i=0;i<EKF_N;i++) e->x[i] += K[i]*z;

    /* P = (I - K*H)*P  (Joseph form for numerical stability) */
    float IKH[EKF_N][EKF_N];
    for (int i=0;i<EKF_N;i++)
        for (int j=0;j<EKF_N;j++)
            IKH[i][j] = (i==j ? 1.0f : 0.0f) - K[i]*H[j];
    float newP[EKF_N][EKF_N] = {0};
    for (int i=0;i<EKF_N;i++)
        for (int j=0;j<EKF_N;j++)
            for (int k=0;k<EKF_N;k++)
                newP[i][j] += IKH[i][k]*e->P[k][j];
    memcpy(e->P, newP, sizeof(e->P));

    return fabsf(z);
}

/* Inject error state into nominal state, then reset x to zero */
static void ekf_inject_reset(EkfState *e) {
    const float DEG = 1.0 / 111320.0;  /* 1 m in degrees lat/lon (approx) */

    /* Position correction */
    e->pos.lat_deg += e->x[0] * DEG;
    e->pos.lon_deg += e->x[1] * DEG / cosf((float)(e->pos.lat_deg * M_PI/180.0));
    e->pos.alt_m   -= e->x[2];

    /* Velocity correction */
    e->vel_ned.x += e->x[3];
    e->vel_ned.y += e->x[4];
    e->vel_ned.z += e->x[5];

    /* Attitude correction (small angle rotation → quaternion) */
    float dψx = e->x[6]*0.5f, dψy = e->x[7]*0.5f, dψz = e->x[8]*0.5f;
    Quat dq = { 1.0f, dψx, dψy, dψz };
    float n = sqrtf(1+dψx*dψx+dψy*dψy+dψz*dψz);
    dq.w /= n; dq.x /= n; dq.y /= n; dq.z /= n;
    Quat q_new;
    qmul(&e->att, &dq, &q_new);
    e->att = q_new;

    memset(e->x, 0, sizeof(e->x));  /* reset error state */
}

/* ─── INITIALISATION ─────────────────────────────────────── */
void ekf_init(ImuGrade grade) {
    memset(&g_ekf, 0, sizeof(g_ekf));

    /* Tune noise parameters per IMU grade */
    switch (grade) {
    case IMU_GRADE_CONSUMER:
        g_ekf.Q_gyro  = 1e-3f;  g_ekf.Q_accel = 1e-1f;
        g_ekf.Q_bg    = 1e-6f;  g_ekf.Q_ba    = 1e-4f;
        break;
    case IMU_GRADE_INDUSTRIAL:  /* BMI088 */
        g_ekf.Q_gyro  = 1e-5f;  g_ekf.Q_accel = 5e-3f;
        g_ekf.Q_bg    = 1e-8f;  g_ekf.Q_ba    = 1e-6f;
        break;
    case IMU_GRADE_TACTICAL:    /* ADIS16507 */
        g_ekf.Q_gyro  = 1e-6f;  g_ekf.Q_accel = 1e-3f;
        g_ekf.Q_bg    = 1e-10f; g_ekf.Q_ba    = 1e-7f;
        break;
    case IMU_GRADE_NAVIGATION:  /* FOG */
    case IMU_GRADE_STRATEGIC:
        g_ekf.Q_gyro  = 1e-8f;  g_ekf.Q_accel = 1e-4f;
        g_ekf.Q_bg    = 1e-12f; g_ekf.Q_ba    = 1e-9f;
        break;
    }

    g_ekf.R_gnss_pos    = 5.0f*5.0f;
    g_ekf.R_gnss_vel    = 0.3f*0.3f;
    g_ekf.R_baro        = 2.0f*2.0f;
    g_ekf.R_mag_hdg     = (5.0f * NAV_DEG2RAD) * (5.0f * NAV_DEG2RAD);
    g_ekf.R_flow_vel    = 0.5f*0.5f;
    g_ekf.R_lidar3d_pos = 0.1f*0.1f;
    g_ekf.R_vio_pos     = 0.3f*0.3f;

    /* Initial covariance */
    for (int i=0;i<EKF_N;i++) g_ekf.P[i][i] = 1.0f;
    g_ekf.P[0][0]=g_ekf.P[1][1]=g_ekf.P[2][2] = 100.0f;  /* 10 m pos uncertainty */
    g_ekf.P[3][3]=g_ekf.P[4][4]=g_ekf.P[5][5] = 1.0f;
    g_ekf.P[6][6]=g_ekf.P[7][7]=g_ekf.P[8][8] = (10.0f*NAV_DEG2RAD)*(10.0f*NAV_DEG2RAD);
}

void ekf_set_initial_state(const GeoPos *pos, const Vec3 *vel, const Quat *att) {
    g_ekf.pos = *pos;
    g_ekf.vel_ned = *vel;
    g_ekf.att = *att;
    g_ekf.initialized = true;
}

/* ─── PREDICTION ─────────────────────────────────────────── */
void ekf_predict(const ImuMeas *imu, float dt) {
    if (!g_ekf.initialized || !imu->valid) return;

    /* Remove estimated biases */
    float gx = imu->gyro_rps.x  - g_ekf.x[9];
    float gy = imu->gyro_rps.y  - g_ekf.x[10];
    float gz = imu->gyro_rps.z  - g_ekf.x[11];
    float ax = imu->accel_mss.x - g_ekf.x[12];
    float ay = imu->accel_mss.y - g_ekf.x[13];
    float az = imu->accel_mss.z - g_ekf.x[14];

    /* ── Nominal state propagation ── */
    /* 1. Attitude integration (quaternion, first-order) */
    float w_norm = sqrtf(gx*gx + gy*gy + gz*gz);
    if (w_norm > 1e-8f) {
        float s = sinf(w_norm*dt*0.5f) / w_norm;
        Quat dq = { cosf(w_norm*dt*0.5f), gx*s, gy*s, gz*s };
        Quat q_new; qmul(&g_ekf.att, &dq, &q_new); g_ekf.att = q_new;
    }

    /* 2. Rotate specific force to NED */
    float R[3][3]; quat_to_dcm(&g_ekf.att, R);
    float fn = R[0][0]*ax + R[0][1]*ay + R[0][2]*az;
    float fe = R[1][0]*ax + R[1][1]*ay + R[1][2]*az;
    float fd = R[2][0]*ax + R[2][1]*ay + R[2][2]*az;

    /* 3. Velocity integration (subtract gravity) */
    g_ekf.vel_ned.x += (fn) * dt;
    g_ekf.vel_ned.y += (fe) * dt;
    g_ekf.vel_ned.z += (fd + NAV_GRAVITY_MSS) * dt;

    /* 4. Position integration */
    const float DEG = 1.0 / 111320.0;
    g_ekf.pos.lat_deg += g_ekf.vel_ned.x * dt * DEG;
    g_ekf.pos.lon_deg += g_ekf.vel_ned.y * dt * DEG /
                          cosf((float)(g_ekf.pos.lat_deg * M_PI / 180.0));
    g_ekf.pos.alt_m   -= g_ekf.vel_ned.z * dt;

    /* ── Error state covariance propagation (sparse F matrix) ── */
    /* F is identity with key off-diagonal terms:
       F[0:3, 3:6]  = I*dt           (pos += vel*dt)
       F[3:6, 6:9]  = -R * [a×] * dt (vel coupling with attitude error)
       F[3:6,12:15] = -R * dt         (vel coupling with accel bias)
       F[6:9, 9:12] = -I * dt         (att coupling with gyro bias)
    */
    float F[EKF_N][EKF_N];
    memset(F, 0, sizeof(F));
    for (int i=0;i<EKF_N;i++) F[i][i] = 1.0f;

    /* δp += δv*dt */
    F[0][3]=dt; F[1][4]=dt; F[2][5]=dt;

    /* δv coupling: skew(a_ned) * attitude error */
    float a_ned[3] = {fn - NAV_GRAVITY_MSS*R[0][2],
                       fe - NAV_GRAVITY_MSS*R[1][2],
                       fd};
    float SA[3][3]; skew3(a_ned, SA);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            F[3+i][6+j] = -SA[i][j]*dt;

    /* δv coupling with accel bias: -R*dt */
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            F[3+i][12+j] = -R[i][j]*dt;

    /* δψ coupling with gyro bias: -I*dt */
    F[6][9]=-dt; F[7][10]=-dt; F[8][11]=-dt;

    /* Build process noise diagonal */
    float q_diag[EKF_N] = {0};
    for (int i=3;i<6;i++)  q_diag[i] = g_ekf.Q_accel * dt;
    for (int i=6;i<9;i++)  q_diag[i] = g_ekf.Q_gyro  * dt;
    for (int i=9;i<12;i++) q_diag[i] = g_ekf.Q_bg    * dt;
    for (int i=12;i<15;i++)q_diag[i] = g_ekf.Q_ba    * dt;

    ekf_cov_predict(g_ekf.P, F, q_diag);
}

/* ─── MEASUREMENT UPDATES ────────────────────────────────── */

float ekf_update_gnss(const GnssMeas *m) {
    if (!m->valid) return 0;
    float innov = 0;

    /* Adaptive noise: scale R by reported accuracy */
    float r_pos = fmaxf(m->h_acc_m * m->h_acc_m, g_ekf.R_gnss_pos);
    float r_vel = fmaxf(m->vel_acc_mss * m->vel_acc_mss, g_ekf.R_gnss_vel);

    /* Position residuals (in metres, NED) */
    float dp_n = (float)((m->pos.lat_deg - g_ekf.pos.lat_deg) * 111320.0);
    float dp_e = (float)((m->pos.lon_deg - g_ekf.pos.lon_deg) * 111320.0 *
                          cos(g_ekf.pos.lat_deg * M_PI/180.0));
    float dp_d = -(m->pos.alt_m - g_ekf.pos.alt_m);

    float H_pN[EKF_N]={0}; H_pN[0]=1; innov += ekf_scalar_update(&g_ekf, H_pN, dp_n, r_pos);
    float H_pE[EKF_N]={0}; H_pE[1]=1; innov += ekf_scalar_update(&g_ekf, H_pE, dp_e, r_pos);
    float H_pD[EKF_N]={0}; H_pD[2]=1; innov += ekf_scalar_update(&g_ekf, H_pD, dp_d, r_pos*4);

    /* Velocity residuals */
    float H_vN[EKF_N]={0}; H_vN[3]=1; innov += ekf_scalar_update(&g_ekf, H_vN, m->vel_ned.x - g_ekf.vel_ned.x, r_vel);
    float H_vE[EKF_N]={0}; H_vE[4]=1; innov += ekf_scalar_update(&g_ekf, H_vE, m->vel_ned.y - g_ekf.vel_ned.y, r_vel);
    float H_vD[EKF_N]={0}; H_vD[5]=1; innov += ekf_scalar_update(&g_ekf, H_vD, m->vel_ned.z - g_ekf.vel_ned.z, r_vel);

    ekf_inject_reset(&g_ekf);
    return innov;
}

float ekf_update_baro(const BaroMeas *m) {
    if (!m->valid) return 0;
    float H[EKF_N] = {0};
    H[2] = -1.0f;   /* δp_down = -(δalt) */
    float dh = m->altitude_m - g_ekf.pos.alt_m;
    /* Sanity gate: ±200 m */
    if (fabsf(dh) > NAV_BARO_MAX_ERROR_M) return 0;
    float innov = ekf_scalar_update(&g_ekf, H, dh, g_ekf.R_baro);
    ekf_inject_reset(&g_ekf);
    return innov;
}

float ekf_update_mag_hdg(float hdg_deg) {
    /* Extract predicted yaw from attitude quaternion */
    float w=g_ekf.att.w, x=g_ekf.att.x, y=g_ekf.att.y, z=g_ekf.att.z;
    float pred_yaw = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z));

    float meas_yaw = hdg_deg * NAV_DEG2RAD;
    float dz = meas_yaw - pred_yaw;
    /* Wrap to ±π */
    while (dz >  (float)M_PI) dz -= 2*(float)M_PI;
    while (dz < -(float)M_PI) dz += 2*(float)M_PI;

    float H[EKF_N] = {0};
    H[8] = 1.0f;    /* yaw component of attitude error */
    float innov = ekf_scalar_update(&g_ekf, H, dz, g_ekf.R_mag_hdg);
    ekf_inject_reset(&g_ekf);
    return innov;
}

float ekf_update_flow(const FlowMeas *m) {
    if (!m->valid || m->quality < 0.3f) return 0;
    float H_vx[EKF_N]={0}; H_vx[3]=1;
    float H_vy[EKF_N]={0}; H_vy[4]=1;
    float r = g_ekf.R_flow_vel / fmaxf(m->quality, 0.1f);
    float innov = ekf_scalar_update(&g_ekf, H_vx, m->vx_mss - g_ekf.vel_ned.x, r);
           innov+= ekf_scalar_update(&g_ekf, H_vy, m->vy_mss - g_ekf.vel_ned.y, r);
    ekf_inject_reset(&g_ekf);
    return innov;
}

float ekf_update_lidar3d(const Lidar3DPose *m) {
    if (!m->valid) return 0;
    /* LiDAR SLAM gives position in a local frame; use as relative correction */
    float r = fmaxf(m->pos_std_m * m->pos_std_m, g_ekf.R_lidar3d_pos);
    float H0[EKF_N]={0}; H0[0]=1; float i0=ekf_scalar_update(&g_ekf,H0,m->pos_local.x,r);
    float H1[EKF_N]={0}; H1[1]=1; float i1=ekf_scalar_update(&g_ekf,H1,m->pos_local.y,r);
    float H2[EKF_N]={0}; H2[2]=1; float i2=ekf_scalar_update(&g_ekf,H2,m->pos_local.z,r);
    ekf_inject_reset(&g_ekf);
    return i0+i1+i2;
}

float ekf_update_vio(const VioPose *m) {
    if (!m->valid) return 0;
    float r = fmaxf(m->pos_std_m * m->pos_std_m, g_ekf.R_vio_pos);
    float H0[EKF_N]={0}; H0[0]=1; float i0=ekf_scalar_update(&g_ekf,H0,m->pos_local.x,r);
    float H1[EKF_N]={0}; H1[1]=1; float i1=ekf_scalar_update(&g_ekf,H1,m->pos_local.y,r);
    float H2[EKF_N]={0}; H2[2]=1; float i2=ekf_scalar_update(&g_ekf,H2,m->pos_local.z,r);
    ekf_inject_reset(&g_ekf);
    return i0+i1+i2;
}

float ekf_update_tercom(const TercomFix *m) {
    if (!m->valid) return 0;
    float r = fmaxf(m->cep_m * m->cep_m, 100.0f);
    float dp_n=(float)((m->pos_corrected.lat_deg-g_ekf.pos.lat_deg)*111320.0);
    float dp_e=(float)((m->pos_corrected.lon_deg-g_ekf.pos.lon_deg)*111320.0*
                        cos(g_ekf.pos.lat_deg*M_PI/180.0));
    float H0[EKF_N]={0}; H0[0]=1; float i0=ekf_scalar_update(&g_ekf,H0,dp_n,r);
    float H1[EKF_N]={0}; H1[1]=1; float i1=ekf_scalar_update(&g_ekf,H1,dp_e,r);
    ekf_inject_reset(&g_ekf);
    return i0+i1;
}

/* ─── OUTPUT ─────────────────────────────────────────────── */
void ekf_get_nav_state(NavState *out) {
    out->pos     = g_ekf.pos;
    out->vel_ned = g_ekf.vel_ned;
    out->att     = g_ekf.att;

    /* Euler angles from quaternion */
    float w=g_ekf.att.w, x=g_ekf.att.x, y=g_ekf.att.y, z=g_ekf.att.z;
    out->euler_deg.x = atan2f(2*(w*x+y*z), 1-2*(x*x+y*y)) * NAV_RAD2DEG;
    out->euler_deg.y = asinf(clampf(2*(w*y-z*x), -1, 1))  * NAV_RAD2DEG;
    out->euler_deg.z = atan2f(2*(w*z+x*y), 1-2*(y*y+z*z)) * NAV_RAD2DEG;

    out->gyro_bias.x  = g_ekf.x[9];
    out->gyro_bias.y  = g_ekf.x[10];
    out->gyro_bias.z  = g_ekf.x[11];
    out->accel_bias.x = g_ekf.x[12];
    out->accel_bias.y = g_ekf.x[13];
    out->accel_bias.z = g_ekf.x[14];
    out->valid        = g_ekf.initialized;
}
