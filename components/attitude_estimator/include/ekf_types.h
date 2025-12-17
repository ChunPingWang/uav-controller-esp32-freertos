/**
 * @file ekf_types.h
 * @brief Extended Kalman Filter types and constants
 *
 * Defines EKF state vector, matrices, and configuration.
 * Based on research.md decision for 6-dimensional state.
 */

#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * EKF Dimensions
 *
 * State vector: x = [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]^T
 * ========================================================================== */

#define EKF_STATE_DIM       6       /**< State vector dimension */
#define EKF_ACCEL_MEAS_DIM  3       /**< Accelerometer measurement dimension */
#define EKF_GPS_MEAS_DIM    1       /**< GPS heading measurement dimension */

/* State vector indices */
#define EKF_IDX_ROLL        0
#define EKF_IDX_PITCH       1
#define EKF_IDX_YAW         2
#define EKF_IDX_BIAS_X      3
#define EKF_IDX_BIAS_Y      4
#define EKF_IDX_BIAS_Z      5

/* ==========================================================================
 * EKF Configuration
 * ========================================================================== */

/**
 * @brief EKF tuning parameters
 */
typedef struct {
    /* Process noise covariance (Q) diagonal elements */
    float q_roll;           /**< Roll process noise */
    float q_pitch;          /**< Pitch process noise */
    float q_yaw;            /**< Yaw process noise */
    float q_bias_x;         /**< X gyro bias process noise */
    float q_bias_y;         /**< Y gyro bias process noise */
    float q_bias_z;         /**< Z gyro bias process noise */

    /* Measurement noise covariance (R) elements */
    float r_accel_x;        /**< Accel X measurement noise */
    float r_accel_y;        /**< Accel Y measurement noise */
    float r_accel_z;        /**< Accel Z measurement noise */
    float r_gps_heading;    /**< GPS heading measurement noise */

    /* Initial covariance (P0) diagonal */
    float p0_attitude;      /**< Initial attitude uncertainty (rad^2) */
    float p0_bias;          /**< Initial bias uncertainty (rad/s)^2 */

    /* GPS heading constraints */
    float gps_min_speed_ms; /**< Minimum speed for GPS heading update */
} ekf_config_t;

/**
 * @brief Default EKF configuration
 *
 * Tuned for typical fixed-wing UAV IMU characteristics.
 */
#define EKF_CONFIG_DEFAULT() {              \
    .q_roll = 0.001f,                       \
    .q_pitch = 0.001f,                      \
    .q_yaw = 0.001f,                        \
    .q_bias_x = 0.0001f,                    \
    .q_bias_y = 0.0001f,                    \
    .q_bias_z = 0.0001f,                    \
    .r_accel_x = 0.1f,                      \
    .r_accel_y = 0.1f,                      \
    .r_accel_z = 0.1f,                      \
    .r_gps_heading = 0.05f,                 \
    .p0_attitude = 1.0f,                    \
    .p0_bias = 0.01f,                       \
    .gps_min_speed_ms = 5.0f                \
}

/* ==========================================================================
 * EKF State Structure
 * ========================================================================== */

/**
 * @brief EKF state and covariance
 *
 * All matrices are statically allocated (Constitution III).
 */
typedef struct {
    /* State vector: [roll, pitch, yaw, bias_x, bias_y, bias_z] */
    float x[EKF_STATE_DIM];

    /* State covariance matrix (6x6, symmetric) */
    float P[EKF_STATE_DIM][EKF_STATE_DIM];

    /* Process noise covariance (6x6, diagonal) */
    float Q[EKF_STATE_DIM][EKF_STATE_DIM];

    /* Accelerometer measurement noise (3x3, diagonal) */
    float R_accel[EKF_ACCEL_MEAS_DIM][EKF_ACCEL_MEAS_DIM];

    /* GPS heading measurement noise (1x1) */
    float R_gps;

    /* Configuration */
    ekf_config_t config;

    /* Status */
    bool initialized;
    bool valid;
    uint64_t last_update_us;
    uint32_t update_count;
    uint32_t last_compute_us;   /**< Last computation time */
} ekf_state_t;

/* ==========================================================================
 * EKF Core Functions
 * ========================================================================== */

/**
 * @brief Initialize EKF state
 *
 * Sets initial state and covariance matrices.
 *
 * @param ekf Pointer to EKF state structure
 * @param config Pointer to configuration (NULL for defaults)
 * @return 0 on success, -1 on error
 */
int ekf_init(ekf_state_t *ekf, const ekf_config_t *config);

/**
 * @brief Reset EKF to initial state
 *
 * @param ekf Pointer to EKF state
 * @return 0 on success
 */
int ekf_reset(ekf_state_t *ekf);

/**
 * @brief EKF prediction step
 *
 * Updates state using gyroscope angular rates.
 * x_k = f(x_{k-1}, u_k)
 * P_k = F * P_{k-1} * F^T + Q
 *
 * @param ekf Pointer to EKF state
 * @param gyro_x Angular rate X (rad/s)
 * @param gyro_y Angular rate Y (rad/s)
 * @param gyro_z Angular rate Z (rad/s)
 * @param dt Time step (seconds)
 * @return 0 on success
 */
int ekf_predict(ekf_state_t *ekf, float gyro_x, float gyro_y, float gyro_z, float dt);

/**
 * @brief EKF accelerometer measurement update
 *
 * Updates state using gravity direction from accelerometer.
 *
 * @param ekf Pointer to EKF state
 * @param accel_x Acceleration X (m/s^2)
 * @param accel_y Acceleration Y (m/s^2)
 * @param accel_z Acceleration Z (m/s^2)
 * @return 0 on success
 */
int ekf_update_accel(ekf_state_t *ekf, float accel_x, float accel_y, float accel_z);

/**
 * @brief EKF GPS heading measurement update
 *
 * Updates yaw state using GPS track-over-ground.
 *
 * @param ekf Pointer to EKF state
 * @param heading_rad GPS heading in radians
 * @return 0 on success
 */
int ekf_update_gps_heading(ekf_state_t *ekf, float heading_rad);

/**
 * @brief Get state covariance trace
 *
 * @param ekf Pointer to EKF state
 * @return Trace of P matrix
 */
float ekf_get_covariance_trace(const ekf_state_t *ekf);

/**
 * @brief Check if EKF state is valid
 *
 * Checks for NaN, divergence, and reasonable covariance.
 *
 * @param ekf Pointer to EKF state
 * @return true if state is valid
 */
bool ekf_is_valid(const ekf_state_t *ekf);

#ifdef __cplusplus
}
#endif

#endif /* EKF_TYPES_H */
