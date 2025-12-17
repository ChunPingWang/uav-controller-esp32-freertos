/**
 * @file imu_sensor.c
 * @brief IMU sensor driver implementation
 *
 * Supports common 6-axis IMUs (MPU6050, ICM-20602, etc.)
 * Requirements: FR-001 (100Hz), FR-004 (quality detection)
 */

#include "imu_sensor.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "driver/i2c.h"
#include "esp_timer.h"

#include <string.h>
#include <math.h>

static const char *TAG = TAG_IMU;

/* IMU state */
static imu_config_t s_config;
static imu_calibration_t s_calibration;
static bool s_initialized = false;
static bool s_calibrating = false;
static uint8_t s_quality = 0;

/* Low-pass filter state (declared in imu_filter.c) */
extern void imu_filter_init(float cutoff_hz, float sample_rate_hz);
extern void imu_filter_apply(sensor_data_t *data);

/* ==========================================================================
 * I2C Communication (placeholder - hardware specific)
 * ========================================================================== */

static esp_err_t imu_write_reg(uint8_t reg, uint8_t value)
{
    /* TODO: Implement actual I2C write */
    (void)reg;
    (void)value;
    return ESP_OK;
}

static esp_err_t imu_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    /* TODO: Implement actual I2C read */
    (void)reg;
    memset(data, 0, len);
    return ESP_OK;
}

/* ==========================================================================
 * Initialization
 * ========================================================================== */

esp_err_t imu_sensor_init(const imu_config_t *config)
{
    if (s_initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    if (config == NULL) {
        imu_config_t default_cfg = IMU_CONFIG_DEFAULT();
        s_config = default_cfg;
    } else {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing IMU on I2C%d (SDA=%d, SCL=%d)",
             s_config.i2c_port, s_config.sda_pin, s_config.scl_pin);

    /* Initialize I2C */
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = s_config.sda_pin,
        .scl_io_num = s_config.scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = s_config.i2c_freq_hz,
    };

    esp_err_t ret = i2c_param_config(s_config.i2c_port, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(s_config.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* TODO: Initialize specific IMU chip (WHO_AM_I check, config registers) */
    /* This is a placeholder - actual implementation depends on IMU model */

    /* Initialize filter if enabled */
    if (s_config.enable_filter) {
        imu_filter_init(s_config.filter_cutoff_hz, (float)s_config.sample_rate_hz);
    }

    /* Initialize calibration to defaults (no calibration) */
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_calibration.accel_scale_x = 1.0f;
    s_calibration.accel_scale_y = 1.0f;
    s_calibration.accel_scale_z = 1.0f;
    s_calibration.valid = false;

    s_quality = 2; /* Assume good until proven otherwise */
    s_initialized = true;

    ESP_LOGI(TAG, "IMU initialized at %d Hz", s_config.sample_rate_hz);
    return ESP_OK;
}

esp_err_t imu_sensor_deinit(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    i2c_driver_delete(s_config.i2c_port);
    s_initialized = false;

    ESP_LOGI(TAG, "IMU deinitialized");
    return ESP_OK;
}

bool imu_sensor_is_ready(void)
{
    return s_initialized && !s_calibrating;
}

/* ==========================================================================
 * Data Reading
 * ========================================================================== */

esp_err_t imu_sensor_read_raw(sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    /* TODO: Read actual sensor registers
     * This is a placeholder returning simulated data
     */

    uint8_t raw[14]; /* 6 accel + 2 temp + 6 gyro */
    esp_err_t ret = imu_read_regs(0x3B, raw, sizeof(raw));
    if (ret != ESP_OK) {
        s_quality = 0;
        return FC_ERR_IMU_COMM;
    }

    /* Convert raw bytes to float (placeholder - depends on IMU model)
     * Typical MPU6050: accel ±2g = 16384 LSB/g, gyro ±250°/s = 131 LSB/°/s
     */
    const float accel_scale = GRAVITY_MS2 / 16384.0f;
    const float gyro_scale = DEG_TO_RAD(1.0f) / 131.0f;

    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    data->accel_x = (float)ax_raw * accel_scale;
    data->accel_y = (float)ay_raw * accel_scale;
    data->accel_z = (float)az_raw * accel_scale;
    data->gyro_x = (float)gx_raw * gyro_scale;
    data->gyro_y = (float)gy_raw * gyro_scale;
    data->gyro_z = (float)gz_raw * gyro_scale;

    /* Apply calibration if available */
    if (s_calibration.valid) {
        data->accel_x = (data->accel_x - s_calibration.accel_offset_x) * s_calibration.accel_scale_x;
        data->accel_y = (data->accel_y - s_calibration.accel_offset_y) * s_calibration.accel_scale_y;
        data->accel_z = (data->accel_z - s_calibration.accel_offset_z) * s_calibration.accel_scale_z;
        data->gyro_x -= s_calibration.gyro_offset_x;
        data->gyro_y -= s_calibration.gyro_offset_y;
        data->gyro_z -= s_calibration.gyro_offset_z;
    }

    data->timestamp_us = esp_timer_get_time();
    data->quality = s_quality;

    return ESP_OK;
}

esp_err_t imu_sensor_read_filtered(sensor_data_t *data)
{
    esp_err_t ret = imu_sensor_read_raw(data);
    if (ret != ESP_OK) {
        return ret;
    }

    if (s_config.enable_filter) {
        imu_filter_apply(data);
    }

    return ESP_OK;
}

uint8_t imu_sensor_get_quality(void)
{
    return s_quality;
}

/* ==========================================================================
 * Calibration (FR-003) - Implemented in imu_calibration.c
 * ========================================================================== */

esp_err_t imu_sensor_get_calibration(imu_calibration_t *calib)
{
    if (calib == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *calib = s_calibration;
    return ESP_OK;
}

esp_err_t imu_sensor_set_calibration(const imu_calibration_t *calib)
{
    if (calib == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_calibration = *calib;
    ESP_LOGI(TAG, "Calibration applied");
    return ESP_OK;
}

bool imu_sensor_is_calibrated(void)
{
    return s_calibration.valid;
}

/* ==========================================================================
 * Self-Test
 * ========================================================================== */

esp_err_t imu_sensor_selftest(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    ESP_LOGI(TAG, "Running IMU self-test...");

    /* TODO: Implement actual self-test using IMU registers */

    /* Placeholder: Check communication */
    uint8_t who_am_i;
    esp_err_t ret = imu_read_regs(0x75, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Self-test failed: communication error");
        return FC_ERR_IMU_SELFTEST;
    }

    ESP_LOGI(TAG, "Self-test passed");
    return ESP_OK;
}
