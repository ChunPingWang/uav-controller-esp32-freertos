/**
 * @file imu_calibration.c
 * @brief IMU calibration implementation (FR-003)
 */

#include "imu_sensor.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <math.h>

static const char *TAG = TAG_IMU;

/* Calibration state */
static bool s_calibrating = false;
static uint32_t s_calib_duration_ms = 0;
static uint64_t s_calib_start_us = 0;
static uint32_t s_calib_samples = 0;

/* Accumulator for averaging */
static float s_accel_sum[3] = {0};
static float s_gyro_sum[3] = {0};

esp_err_t imu_sensor_start_calibration(uint32_t duration_ms)
{
    if (s_calibrating) {
        return FC_ERR_BUSY;
    }

    ESP_LOGI(TAG, "Starting calibration for %lu ms", (unsigned long)duration_ms);
    ESP_LOGW(TAG, "Keep device STATIONARY and LEVEL!");

    s_calibrating = true;
    s_calib_duration_ms = duration_ms;
    s_calib_start_us = esp_timer_get_time();
    s_calib_samples = 0;

    /* Reset accumulators */
    for (int i = 0; i < 3; i++) {
        s_accel_sum[i] = 0.0f;
        s_gyro_sum[i] = 0.0f;
    }

    return ESP_OK;
}

bool imu_sensor_is_calibrating(void)
{
    if (!s_calibrating) {
        return false;
    }

    /* Check if duration elapsed */
    uint64_t elapsed_us = esp_timer_get_time() - s_calib_start_us;
    if (elapsed_us >= s_calib_duration_ms * 1000ULL) {
        /* Calibration complete */
        if (s_calib_samples > 10) {
            /* Calculate averages */
            float inv_n = 1.0f / (float)s_calib_samples;

            imu_calibration_t calib = {0};

            /* Gyro offsets (should be near zero when stationary) */
            calib.gyro_offset_x = s_gyro_sum[0] * inv_n;
            calib.gyro_offset_y = s_gyro_sum[1] * inv_n;
            calib.gyro_offset_z = s_gyro_sum[2] * inv_n;

            /* Accel offsets (should read [0, 0, g] when level)
             * offset = measured - expected
             */
            calib.accel_offset_x = s_accel_sum[0] * inv_n;
            calib.accel_offset_y = s_accel_sum[1] * inv_n;
            calib.accel_offset_z = s_accel_sum[2] * inv_n - GRAVITY_MS2;

            /* Scale factors default to 1.0 (would need more complex cal for scale) */
            calib.accel_scale_x = 1.0f;
            calib.accel_scale_y = 1.0f;
            calib.accel_scale_z = 1.0f;

            calib.calibration_date = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            calib.valid = true;

            /* Apply calibration */
            imu_sensor_set_calibration(&calib);

            ESP_LOGI(TAG, "Calibration complete (%lu samples)", (unsigned long)s_calib_samples);
            ESP_LOGI(TAG, "Gyro offset: [%.4f, %.4f, %.4f] rad/s",
                     calib.gyro_offset_x, calib.gyro_offset_y, calib.gyro_offset_z);
            ESP_LOGI(TAG, "Accel offset: [%.4f, %.4f, %.4f] m/s^2",
                     calib.accel_offset_x, calib.accel_offset_y, calib.accel_offset_z);
        } else {
            ESP_LOGE(TAG, "Calibration failed: insufficient samples");
        }

        s_calibrating = false;
    }

    return s_calibrating;
}

/* Called from IMU read to accumulate samples during calibration */
void imu_calibration_accumulate(const sensor_data_t *data)
{
    if (!s_calibrating || data == NULL) {
        return;
    }

    s_accel_sum[0] += data->accel_x;
    s_accel_sum[1] += data->accel_y;
    s_accel_sum[2] += data->accel_z;

    s_gyro_sum[0] += data->gyro_x;
    s_gyro_sum[1] += data->gyro_y;
    s_gyro_sum[2] += data->gyro_z;

    s_calib_samples++;
}
