/**
 * @file attitude_estimator.c
 * @brief Attitude estimator wrapper integrating EKF
 */

#include "attitude_estimator.h"
#include "ekf_types.h"
#include "matrix_math.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_ATT;

/* Static EKF state (Constitution III) */
static ekf_state_t s_ekf;
static bool s_initialized = false;

/* ==========================================================================
 * Initialization
 * ========================================================================== */

esp_err_t attitude_estimator_init(const attitude_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return FC_ERR_ALREADY_INIT;
    }

    ekf_config_t ekf_cfg = EKF_CONFIG_DEFAULT();

    if (config != NULL) {
        ekf_cfg.q_roll = config->process_noise_gyro;
        ekf_cfg.q_pitch = config->process_noise_gyro;
        ekf_cfg.q_yaw = config->process_noise_gyro;
        ekf_cfg.q_bias_x = config->process_noise_bias;
        ekf_cfg.q_bias_y = config->process_noise_bias;
        ekf_cfg.q_bias_z = config->process_noise_bias;
        ekf_cfg.r_accel_x = config->measurement_noise_accel;
        ekf_cfg.r_accel_y = config->measurement_noise_accel;
        ekf_cfg.r_accel_z = config->measurement_noise_accel;
        ekf_cfg.r_gps_heading = config->measurement_noise_gps;
        ekf_cfg.p0_attitude = config->initial_covariance;
        ekf_cfg.gps_min_speed_ms = config->gps_heading_min_speed;
    }

    if (ekf_init(&s_ekf, &ekf_cfg) != 0) {
        ESP_LOGE(TAG, "EKF initialization failed");
        return FC_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Attitude estimator initialized");

    return ESP_OK;
}

esp_err_t attitude_estimator_deinit(void)
{
    s_initialized = false;
    memset(&s_ekf, 0, sizeof(s_ekf));
    ESP_LOGI(TAG, "Attitude estimator deinitialized");
    return ESP_OK;
}

esp_err_t attitude_estimator_reset(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (ekf_reset(&s_ekf) != 0) {
        return FC_FAIL;
    }

    ESP_LOGI(TAG, "Attitude estimator reset");
    return ESP_OK;
}

bool attitude_estimator_is_ready(void)
{
    return s_initialized && s_ekf.valid;
}

/* ==========================================================================
 * EKF Update Functions
 * ========================================================================== */

esp_err_t attitude_estimator_predict(float gyro_x, float gyro_y, float gyro_z, float dt)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    int64_t start = esp_timer_get_time();

    if (ekf_predict(&s_ekf, gyro_x, gyro_y, gyro_z, dt) != 0) {
        s_ekf.valid = false;
        return FC_ERR_EKF_DIVERGED;
    }

    s_ekf.last_compute_us = (uint32_t)(esp_timer_get_time() - start);
    return ESP_OK;
}

esp_err_t attitude_estimator_update_accel(float accel_x, float accel_y, float accel_z)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    int64_t start = esp_timer_get_time();

    if (ekf_update_accel(&s_ekf, accel_x, accel_y, accel_z) != 0) {
        /* Update failed but not critical */
        ESP_LOGD(TAG, "Accel update skipped");
    }

    s_ekf.last_compute_us += (uint32_t)(esp_timer_get_time() - start);

    /* Validate state */
    s_ekf.valid = ekf_is_valid(&s_ekf);

    return ESP_OK;
}

esp_err_t attitude_estimator_update_gps_heading(float heading_deg, float speed_ms)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (speed_ms < s_ekf.config.gps_min_speed_ms) {
        return FC_ERR_INVALID_PARAM;
    }

    float heading_rad = DEG_TO_RAD(heading_deg);

    if (ekf_update_gps_heading(&s_ekf, heading_rad) != 0) {
        ESP_LOGW(TAG, "GPS heading update failed");
        return FC_FAIL;
    }

    return ESP_OK;
}

esp_err_t attitude_estimator_process_imu(const sensor_data_t *sensor_data, float dt)
{
    if (sensor_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    /* Predict using gyroscope */
    ret = attitude_estimator_predict(
        sensor_data->gyro_x,
        sensor_data->gyro_y,
        sensor_data->gyro_z,
        dt);

    if (ret != ESP_OK) {
        return ret;
    }

    /* Update using accelerometer */
    ret = attitude_estimator_update_accel(
        sensor_data->accel_x,
        sensor_data->accel_y,
        sensor_data->accel_z);

    return ret;
}

/* ==========================================================================
 * State Access
 * ========================================================================== */

esp_err_t attitude_estimator_get_state(attitude_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    state->roll = s_ekf.x[EKF_IDX_ROLL];
    state->pitch = s_ekf.x[EKF_IDX_PITCH];
    state->yaw = s_ekf.x[EKF_IDX_YAW];

    /* Rate estimates (bias-corrected) - would need last gyro reading */
    state->roll_rate = 0.0f;
    state->pitch_rate = 0.0f;
    state->yaw_rate = 0.0f;

    state->timestamp_us = s_ekf.last_update_us;
    state->valid = s_ekf.valid;

    return ESP_OK;
}

float attitude_estimator_get_roll(void)
{
    return s_initialized ? s_ekf.x[EKF_IDX_ROLL] : 0.0f;
}

float attitude_estimator_get_pitch(void)
{
    return s_initialized ? s_ekf.x[EKF_IDX_PITCH] : 0.0f;
}

float attitude_estimator_get_yaw(void)
{
    return s_initialized ? s_ekf.x[EKF_IDX_YAW] : 0.0f;
}

esp_err_t attitude_estimator_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (bias_x) *bias_x = s_ekf.x[EKF_IDX_BIAS_X];
    if (bias_y) *bias_y = s_ekf.x[EKF_IDX_BIAS_Y];
    if (bias_z) *bias_z = s_ekf.x[EKF_IDX_BIAS_Z];

    return ESP_OK;
}

bool attitude_estimator_is_valid(void)
{
    return s_initialized && s_ekf.valid;
}

/* ==========================================================================
 * Diagnostics
 * ========================================================================== */

float attitude_estimator_get_covariance_trace(void)
{
    return s_initialized ? ekf_get_covariance_trace(&s_ekf) : -1.0f;
}

uint32_t attitude_estimator_get_compute_time_us(void)
{
    return s_initialized ? s_ekf.last_compute_us : 0;
}
