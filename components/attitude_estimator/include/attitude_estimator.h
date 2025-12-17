/**
 * @file attitude_estimator.h
 * @brief Attitude Estimator Public API
 *
 * Estimates aircraft attitude using Extended Kalman Filter (EKF).
 * Requirements: FR-005 (attitude), FR-006 (10ms), FR-007 (<2° accuracy)
 */

#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "esp_err.h"
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/**
 * @brief Attitude estimator configuration
 */
typedef struct {
    float process_noise_gyro;       /**< Gyroscope process noise variance */
    float process_noise_bias;       /**< Gyro bias process noise variance */
    float measurement_noise_accel;  /**< Accelerometer measurement noise */
    float measurement_noise_gps;    /**< GPS heading measurement noise */
    float initial_covariance;       /**< Initial state covariance */
    float gps_heading_min_speed;    /**< Min speed for GPS heading (m/s) */
} attitude_config_t;

/**
 * @brief Default attitude estimator configuration
 */
#define ATTITUDE_CONFIG_DEFAULT() {         \
    .process_noise_gyro = 0.001f,           \
    .process_noise_bias = 0.0001f,          \
    .measurement_noise_accel = 0.1f,        \
    .measurement_noise_gps = 0.05f,         \
    .initial_covariance = 1.0f,             \
    .gps_heading_min_speed = 5.0f           \
}

/* ==========================================================================
 * Initialization
 * ========================================================================== */

/**
 * @brief Initialize attitude estimator
 *
 * Initializes EKF with default state and covariance.
 *
 * @param config Pointer to configuration (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_init(const attitude_config_t *config);

/**
 * @brief Deinitialize attitude estimator
 *
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_deinit(void);

/**
 * @brief Reset estimator to initial state
 *
 * Resets EKF state and covariance to initial values.
 *
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_reset(void);

/**
 * @brief Check if estimator is ready
 *
 * @return true if initialized and state is valid
 */
bool attitude_estimator_is_ready(void);

/* ==========================================================================
 * EKF Update Functions (FR-005, FR-006)
 * ========================================================================== */

/**
 * @brief Predict step using gyroscope data
 *
 * Performs EKF prediction using gyroscope angular rates.
 * Should be called at 100Hz with each IMU reading.
 *
 * @param gyro_x Angular rate around X axis (rad/s)
 * @param gyro_y Angular rate around Y axis (rad/s)
 * @param gyro_z Angular rate around Z axis (rad/s)
 * @param dt Time step (seconds)
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_predict(float gyro_x, float gyro_y, float gyro_z, float dt);

/**
 * @brief Update step using accelerometer data
 *
 * Performs EKF measurement update using gravity direction.
 * Should be called at 100Hz with each IMU reading.
 *
 * @param accel_x Acceleration X (m/s^2)
 * @param accel_y Acceleration Y (m/s^2)
 * @param accel_z Acceleration Z (m/s^2)
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_update_accel(float accel_x, float accel_y, float accel_z);

/**
 * @brief Update step using GPS heading
 *
 * Performs EKF measurement update for yaw correction.
 * Only valid when speed > configured minimum.
 *
 * @param heading_deg GPS heading in degrees (0-360)
 * @param speed_ms Ground speed in m/s
 * @return ESP_OK on success, FC_ERR_INVALID_PARAM if speed too low
 */
esp_err_t attitude_estimator_update_gps_heading(float heading_deg, float speed_ms);

/**
 * @brief Process complete IMU reading
 *
 * Convenience function that runs predict and accel update.
 *
 * @param sensor_data Pointer to sensor data
 * @param dt Time step (seconds)
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_process_imu(const sensor_data_t *sensor_data, float dt);

/* ==========================================================================
 * State Access (FR-007: <2° accuracy)
 * ========================================================================== */

/**
 * @brief Get current attitude state
 *
 * @param[out] state Pointer to store attitude state
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_get_state(attitude_state_t *state);

/**
 * @brief Get roll angle
 *
 * @return Roll angle in radians (-PI to PI)
 */
float attitude_estimator_get_roll(void);

/**
 * @brief Get pitch angle
 *
 * @return Pitch angle in radians (-PI/2 to PI/2)
 */
float attitude_estimator_get_pitch(void);

/**
 * @brief Get yaw angle
 *
 * @return Yaw/heading angle in radians (0 to 2*PI)
 */
float attitude_estimator_get_yaw(void);

/**
 * @brief Get gyroscope bias estimates
 *
 * @param[out] bias_x X-axis gyro bias (rad/s)
 * @param[out] bias_y Y-axis gyro bias (rad/s)
 * @param[out] bias_z Z-axis gyro bias (rad/s)
 * @return ESP_OK on success
 */
esp_err_t attitude_estimator_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z);

/**
 * @brief Check if attitude estimate is valid
 *
 * Returns false if EKF has diverged or covariance is unreasonable.
 *
 * @return true if estimate is trustworthy
 */
bool attitude_estimator_is_valid(void);

/* ==========================================================================
 * Diagnostics
 * ========================================================================== */

/**
 * @brief Get EKF covariance trace
 *
 * Useful for monitoring filter convergence.
 *
 * @return Trace of covariance matrix
 */
float attitude_estimator_get_covariance_trace(void);

/**
 * @brief Get last update computation time
 *
 * @return Computation time in microseconds
 */
uint32_t attitude_estimator_get_compute_time_us(void);

/* ==========================================================================
 * Task Interface
 * ========================================================================== */

/**
 * @brief Attitude compute task function
 *
 * FreeRTOS task that receives IMU data and runs EKF.
 * Priority: configMAX_PRIORITIES-2
 * Stack: 8192 bytes (for matrix operations)
 *
 * @param pvParameters Task parameters (unused)
 */
void vTask_Attitude_Compute(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_ESTIMATOR_H */
