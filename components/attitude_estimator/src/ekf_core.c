/**
 * @file ekf_core.c
 * @brief Extended Kalman Filter core implementation
 *
 * 6-state EKF for attitude estimation as per research.md decision.
 * State: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */

#include "ekf_types.h"
#include "matrix_math.h"
#include "fc_types.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 * Helper Functions
 * ========================================================================== */

/**
 * @brief Normalize angle to [-PI, PI]
 */
static float normalize_angle(float angle)
{
    while (angle > FC_PI) angle -= FC_2PI;
    while (angle < -FC_PI) angle += FC_2PI;
    return angle;
}

/**
 * @brief Clamp value to range
 */
static float clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ==========================================================================
 * EKF Initialization
 * ========================================================================== */

int ekf_init(ekf_state_t *ekf, const ekf_config_t *config)
{
    if (ekf == NULL) {
        return -1;
    }

    memset(ekf, 0, sizeof(ekf_state_t));

    /* Apply configuration */
    if (config != NULL) {
        ekf->config = *config;
    } else {
        ekf_config_t default_config = EKF_CONFIG_DEFAULT();
        ekf->config = default_config;
    }

    /* Initialize state to zero (level attitude, no bias) */
    vec6_zero(ekf->x);

    /* Initialize covariance P */
    mat6_zero(ekf->P);
    ekf->P[EKF_IDX_ROLL][EKF_IDX_ROLL] = ekf->config.p0_attitude;
    ekf->P[EKF_IDX_PITCH][EKF_IDX_PITCH] = ekf->config.p0_attitude;
    ekf->P[EKF_IDX_YAW][EKF_IDX_YAW] = ekf->config.p0_attitude;
    ekf->P[EKF_IDX_BIAS_X][EKF_IDX_BIAS_X] = ekf->config.p0_bias;
    ekf->P[EKF_IDX_BIAS_Y][EKF_IDX_BIAS_Y] = ekf->config.p0_bias;
    ekf->P[EKF_IDX_BIAS_Z][EKF_IDX_BIAS_Z] = ekf->config.p0_bias;

    /* Initialize process noise Q */
    mat6_zero(ekf->Q);
    ekf->Q[EKF_IDX_ROLL][EKF_IDX_ROLL] = ekf->config.q_roll;
    ekf->Q[EKF_IDX_PITCH][EKF_IDX_PITCH] = ekf->config.q_pitch;
    ekf->Q[EKF_IDX_YAW][EKF_IDX_YAW] = ekf->config.q_yaw;
    ekf->Q[EKF_IDX_BIAS_X][EKF_IDX_BIAS_X] = ekf->config.q_bias_x;
    ekf->Q[EKF_IDX_BIAS_Y][EKF_IDX_BIAS_Y] = ekf->config.q_bias_y;
    ekf->Q[EKF_IDX_BIAS_Z][EKF_IDX_BIAS_Z] = ekf->config.q_bias_z;

    /* Initialize accelerometer measurement noise R_accel */
    memset(ekf->R_accel, 0, sizeof(ekf->R_accel));
    ekf->R_accel[0][0] = ekf->config.r_accel_x;
    ekf->R_accel[1][1] = ekf->config.r_accel_y;
    ekf->R_accel[2][2] = ekf->config.r_accel_z;

    /* Initialize GPS heading measurement noise */
    ekf->R_gps = ekf->config.r_gps_heading;

    ekf->initialized = true;
    ekf->valid = true;
    ekf->last_update_us = 0;
    ekf->update_count = 0;

    return 0;
}

int ekf_reset(ekf_state_t *ekf)
{
    if (ekf == NULL) {
        return -1;
    }

    /* Keep configuration, reset state */
    ekf_config_t saved_config = ekf->config;
    return ekf_init(ekf, &saved_config);
}

/* ==========================================================================
 * EKF Prediction Step (T026c, T026d)
 *
 * State transition model:
 *   roll_new = roll + (gyro_x - bias_x) * dt
 *   pitch_new = pitch + (gyro_y - bias_y) * dt
 *   yaw_new = yaw + (gyro_z - bias_z) * dt
 *   bias_new = bias (random walk)
 * ========================================================================== */

int ekf_predict(ekf_state_t *ekf, float gyro_x, float gyro_y, float gyro_z, float dt)
{
    if (ekf == NULL || !ekf->initialized) {
        return -1;
    }

    /* Bias-corrected angular rates */
    float wx = gyro_x - ekf->x[EKF_IDX_BIAS_X];
    float wy = gyro_y - ekf->x[EKF_IDX_BIAS_Y];
    float wz = gyro_z - ekf->x[EKF_IDX_BIAS_Z];

    /* State prediction (Euler integration) */
    float roll = ekf->x[EKF_IDX_ROLL];
    float pitch = ekf->x[EKF_IDX_PITCH];

    /* Compute attitude derivatives (simplified, small angle approximation) */
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float tan_pitch = tanf(pitch);
    float cos_pitch = cosf(pitch);

    /* Prevent division by zero near gimbal lock */
    if (fabsf(cos_pitch) < 0.01f) {
        cos_pitch = 0.01f;
    }

    /* Euler angle rates from body rates */
    float roll_dot = wx + sin_roll * tan_pitch * wy + cos_roll * tan_pitch * wz;
    float pitch_dot = cos_roll * wy - sin_roll * wz;
    float yaw_dot = (sin_roll / cos_pitch) * wy + (cos_roll / cos_pitch) * wz;

    /* Update state */
    ekf->x[EKF_IDX_ROLL] = normalize_angle(ekf->x[EKF_IDX_ROLL] + roll_dot * dt);
    ekf->x[EKF_IDX_PITCH] = clamp(ekf->x[EKF_IDX_PITCH] + pitch_dot * dt, -FC_PI/2 + 0.01f, FC_PI/2 - 0.01f);
    ekf->x[EKF_IDX_YAW] = normalize_angle(ekf->x[EKF_IDX_YAW] + yaw_dot * dt);
    /* Biases unchanged in prediction (random walk) */

    /* Jacobian F = df/dx (state transition matrix) */
    mat6x6_t F;
    mat6_identity(F);

    /* Partial derivatives (simplified) */
    F[EKF_IDX_ROLL][EKF_IDX_PITCH] = (cos_roll * tan_pitch * wy - sin_roll * tan_pitch * wz) * dt;
    F[EKF_IDX_ROLL][EKF_IDX_BIAS_X] = -dt;

    F[EKF_IDX_PITCH][EKF_IDX_ROLL] = (-sin_roll * wy - cos_roll * wz) * dt;
    F[EKF_IDX_PITCH][EKF_IDX_BIAS_Y] = -cos_roll * dt;
    F[EKF_IDX_PITCH][EKF_IDX_BIAS_Z] = sin_roll * dt;

    F[EKF_IDX_YAW][EKF_IDX_ROLL] = ((cos_roll / cos_pitch) * wy - (sin_roll / cos_pitch) * wz) * dt;
    F[EKF_IDX_YAW][EKF_IDX_PITCH] = ((sin_roll * sin_roll / (cos_pitch * cos_pitch)) * wy +
                                     (cos_roll * sin_roll / (cos_pitch * cos_pitch)) * wz) * dt;
    F[EKF_IDX_YAW][EKF_IDX_BIAS_Y] = -(sin_roll / cos_pitch) * dt;
    F[EKF_IDX_YAW][EKF_IDX_BIAS_Z] = -(cos_roll / cos_pitch) * dt;

    /* Scale Q by dt */
    mat6x6_t Q_scaled;
    mat6_scale(Q_scaled, ekf->Q, dt);

    /* Covariance prediction: P = F * P * F^T + Q */
    mat6_covariance_predict(ekf->P, ekf->P, F, Q_scaled);

    ekf->update_count++;
    return 0;
}

/* ==========================================================================
 * EKF Accelerometer Update (T026e)
 *
 * Measurement model: Gravity direction in body frame
 *   h(x) = R(roll, pitch) * [0, 0, g]^T
 * ========================================================================== */

int ekf_update_accel(ekf_state_t *ekf, float accel_x, float accel_y, float accel_z)
{
    if (ekf == NULL || !ekf->initialized) {
        return -1;
    }

    /* Normalize acceleration vector */
    float accel_mag = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (accel_mag < 0.5f * GRAVITY_MS2 || accel_mag > 1.5f * GRAVITY_MS2) {
        /* Acceleration too far from 1g, skip update (dynamic motion) */
        return 0;
    }

    float ax = accel_x / accel_mag * GRAVITY_MS2;
    float ay = accel_y / accel_mag * GRAVITY_MS2;
    float az = accel_z / accel_mag * GRAVITY_MS2;

    /* Current state */
    float roll = ekf->x[EKF_IDX_ROLL];
    float pitch = ekf->x[EKF_IDX_PITCH];

    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);

    /* Expected acceleration (gravity in body frame) */
    float hx = -GRAVITY_MS2 * sin_pitch;
    float hy = GRAVITY_MS2 * cos_pitch * sin_roll;
    float hz = GRAVITY_MS2 * cos_pitch * cos_roll;

    /* Innovation (measurement residual) */
    float y[3] = {
        ax - hx,
        ay - hy,
        az - hz
    };

    /* Measurement Jacobian H (3x6) */
    float H[3][MAT_DIM] = {0};
    H[0][EKF_IDX_ROLL] = 0.0f;
    H[0][EKF_IDX_PITCH] = -GRAVITY_MS2 * cos_pitch;

    H[1][EKF_IDX_ROLL] = GRAVITY_MS2 * cos_pitch * cos_roll;
    H[1][EKF_IDX_PITCH] = -GRAVITY_MS2 * sin_pitch * sin_roll;

    H[2][EKF_IDX_ROLL] = -GRAVITY_MS2 * cos_pitch * sin_roll;
    H[2][EKF_IDX_PITCH] = -GRAVITY_MS2 * sin_pitch * cos_roll;

    /* S = H * P * H^T + R (3x3) */
    float S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < MAT_DIM; k++) {
                for (int l = 0; l < MAT_DIM; l++) {
                    S[i][j] += H[i][k] * ekf->P[k][l] * H[j][l];
                }
            }
            if (i == j) {
                S[i][j] += ekf->R_accel[i][i];
            }
        }
    }

    /* Invert S (3x3) - simplified for 3x3 */
    float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
              - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
              + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    if (fabsf(det) < 1e-10f) {
        return -1; /* Singular */
    }

    float S_inv[3][3];
    float inv_det = 1.0f / det;
    S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * inv_det;
    S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * inv_det;
    S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv_det;
    S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * inv_det;
    S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv_det;
    S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * inv_det;
    S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * inv_det;
    S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * inv_det;
    S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv_det;

    /* Kalman gain K = P * H^T * S^-1 (6x3) */
    float K[MAT_DIM][3] = {0};
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < MAT_DIM; k++) {
                for (int l = 0; l < 3; l++) {
                    K[i][j] += ekf->P[i][k] * H[l][k] * S_inv[l][j];
                }
            }
        }
    }

    /* State update: x = x + K * y */
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->x[i] += K[i][j] * y[j];
        }
    }

    /* Normalize angles */
    ekf->x[EKF_IDX_ROLL] = normalize_angle(ekf->x[EKF_IDX_ROLL]);
    ekf->x[EKF_IDX_PITCH] = clamp(ekf->x[EKF_IDX_PITCH], -FC_PI/2 + 0.01f, FC_PI/2 - 0.01f);
    ekf->x[EKF_IDX_YAW] = normalize_angle(ekf->x[EKF_IDX_YAW]);

    /* Covariance update: P = (I - KH) * P (simplified form) */
    mat6x6_t I_KH;
    mat6_identity(I_KH);
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            for (int k = 0; k < 3; k++) {
                I_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }

    mat6x6_t P_new;
    mat6_mult(P_new, I_KH, ekf->P);
    mat6_copy(ekf->P, P_new);
    mat6_force_symmetric(ekf->P);

    return 0;
}

/* ==========================================================================
 * EKF GPS Heading Update (T026f)
 * ========================================================================== */

int ekf_update_gps_heading(ekf_state_t *ekf, float heading_rad)
{
    if (ekf == NULL || !ekf->initialized) {
        return -1;
    }

    /* Innovation */
    float y = normalize_angle(heading_rad - ekf->x[EKF_IDX_YAW]);

    /* Measurement Jacobian H (1x6) */
    float H[MAT_DIM] = {0};
    H[EKF_IDX_YAW] = 1.0f;

    /* S = H * P * H^T + R (scalar) */
    float S = ekf->P[EKF_IDX_YAW][EKF_IDX_YAW] + ekf->R_gps;

    if (fabsf(S) < 1e-10f) {
        return -1;
    }

    /* Kalman gain K = P * H^T / S (6x1) */
    float K[MAT_DIM];
    for (int i = 0; i < MAT_DIM; i++) {
        K[i] = ekf->P[i][EKF_IDX_YAW] / S;
    }

    /* State update */
    for (int i = 0; i < MAT_DIM; i++) {
        ekf->x[i] += K[i] * y;
    }

    /* Normalize yaw */
    ekf->x[EKF_IDX_YAW] = normalize_angle(ekf->x[EKF_IDX_YAW]);

    /* Joseph form covariance update */
    mat6_joseph_update(ekf->P, ekf->P, K, H, &ekf->R_gps, 1);

    return 0;
}

/* ==========================================================================
 * Utility Functions
 * ========================================================================== */

float ekf_get_covariance_trace(const ekf_state_t *ekf)
{
    if (ekf == NULL) {
        return -1.0f;
    }
    return mat6_trace(ekf->P);
}

bool ekf_is_valid(const ekf_state_t *ekf)
{
    if (ekf == NULL || !ekf->initialized) {
        return false;
    }

    /* Check state vector for NaN */
    for (int i = 0; i < MAT_DIM; i++) {
        if (isnan(ekf->x[i]) || isinf(ekf->x[i])) {
            return false;
        }
    }

    /* Check covariance matrix */
    if (!mat6_is_valid(ekf->P)) {
        return false;
    }

    /* Check positive definiteness */
    if (!mat6_is_positive_definite(ekf->P)) {
        return false;
    }

    /* Check for divergence (covariance too large) */
    float trace = mat6_trace(ekf->P);
    if (trace > 100.0f) {
        return false;
    }

    return true;
}
