/**
 * @file imu_sensor.h
 * @brief IMU Sensor Public API
 *
 * Six-axis IMU (accelerometer + gyroscope) interface.
 * Requirements: FR-001 (100Hz), FR-002 (filter), FR-003 (calibration), FR-004 (quality)
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "esp_err.h"
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/**
 * @brief IMU configuration structure
 */
typedef struct {
    int i2c_port;               /**< I2C port number */
    int sda_pin;                /**< SDA GPIO pin */
    int scl_pin;                /**< SCL GPIO pin */
    uint32_t i2c_freq_hz;       /**< I2C clock frequency */
    uint16_t sample_rate_hz;    /**< Target sample rate (default: 100Hz) */
    bool enable_filter;         /**< Enable digital low-pass filter */
    float filter_cutoff_hz;     /**< Filter cutoff frequency */
} imu_config_t;

/**
 * @brief Default IMU configuration
 */
#define IMU_CONFIG_DEFAULT() {          \
    .i2c_port = 0,                      \
    .sda_pin = 21,                      \
    .scl_pin = 22,                      \
    .i2c_freq_hz = 400000,              \
    .sample_rate_hz = 100,              \
    .enable_filter = true,              \
    .filter_cutoff_hz = 20.0f           \
}

/* ==========================================================================
 * Initialization
 * ========================================================================== */

/**
 * @brief Initialize IMU sensor
 *
 * Configures I2C/SPI interface and initializes the sensor chip.
 *
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_sensor_init(const imu_config_t *config);

/**
 * @brief Deinitialize IMU sensor
 *
 * Releases resources and stops sampling.
 *
 * @return ESP_OK on success
 */
esp_err_t imu_sensor_deinit(void);

/**
 * @brief Check if IMU is initialized and responding
 *
 * @return true if IMU is ready
 */
bool imu_sensor_is_ready(void);

/* ==========================================================================
 * Data Reading (FR-001)
 * ========================================================================== */

/**
 * @brief Read raw sensor data
 *
 * Reads accelerometer and gyroscope data without filtering.
 *
 * @param[out] data Pointer to store sensor data
 * @return ESP_OK on success, FC_ERR_IMU_* on error
 */
esp_err_t imu_sensor_read_raw(sensor_data_t *data);

/**
 * @brief Read filtered sensor data (FR-002)
 *
 * Reads sensor data with digital low-pass filter applied.
 *
 * @param[out] data Pointer to store filtered sensor data
 * @return ESP_OK on success
 */
esp_err_t imu_sensor_read_filtered(sensor_data_t *data);

/**
 * @brief Get data quality (FR-004)
 *
 * @return Quality value: 0=invalid, 1=degraded, 2=good
 */
uint8_t imu_sensor_get_quality(void);

/* ==========================================================================
 * Calibration (FR-003)
 * ========================================================================== */

/**
 * @brief Start calibration procedure
 *
 * Device must be stationary and level during calibration.
 *
 * @param duration_ms Calibration duration in milliseconds
 * @return ESP_OK on success
 */
esp_err_t imu_sensor_start_calibration(uint32_t duration_ms);

/**
 * @brief Check if calibration is in progress
 *
 * @return true if calibration is running
 */
bool imu_sensor_is_calibrating(void);

/**
 * @brief Get calibration data
 *
 * @param[out] calib Pointer to store calibration data
 * @return ESP_OK if calibration data is valid
 */
esp_err_t imu_sensor_get_calibration(imu_calibration_t *calib);

/**
 * @brief Apply calibration data
 *
 * @param[in] calib Calibration data to apply
 * @return ESP_OK on success
 */
esp_err_t imu_sensor_set_calibration(const imu_calibration_t *calib);

/**
 * @brief Check if sensor is calibrated
 *
 * @return true if calibration data is loaded
 */
bool imu_sensor_is_calibrated(void);

/* ==========================================================================
 * Self-Test
 * ========================================================================== */

/**
 * @brief Run IMU self-test
 *
 * Executes built-in self-test registers.
 *
 * @return ESP_OK if self-test passed, FC_ERR_IMU_SELFTEST on failure
 */
esp_err_t imu_sensor_selftest(void);

/* ==========================================================================
 * Task Interface
 * ========================================================================== */

/**
 * @brief IMU read task function
 *
 * FreeRTOS task that reads IMU at configured rate and sends to queue.
 * Priority: configMAX_PRIORITIES-1 (highest real-time)
 *
 * @param pvParameters Task parameters (unused)
 */
void vTask_IMU_Read(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SENSOR_H */
