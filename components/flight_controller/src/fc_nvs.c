/**
 * @file fc_nvs.c
 * @brief NVS storage wrapper for calibration data and configuration
 *
 * Provides persistent storage for IMU calibration and home point (FR-027).
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = TAG_FC;

/* NVS namespace */
#define NVS_NAMESPACE       "fc_config"

/* NVS keys */
#define NVS_KEY_IMU_CALIB   "imu_calib"
#define NVS_KEY_HOME_POINT  "home_point"
#define NVS_KEY_TELEM_RATE  "telem_rate"

static bool s_nvs_initialized = false;

/* ==========================================================================
 * Initialization
 * ========================================================================== */

esp_err_t fc_nvs_init(void)
{
    if (s_nvs_initialized) {
        return ESP_OK;
    }

    /* NVS flash should already be initialized in main.c */
    s_nvs_initialized = true;
    ESP_LOGI(TAG, "NVS wrapper initialized");

    return ESP_OK;
}

/* ==========================================================================
 * IMU Calibration Storage
 * ========================================================================== */

esp_err_t fc_nvs_save_imu_calibration(const imu_calibration_t *calib)
{
    if (calib == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Take mutex for thread safety */
    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        xSemaphoreGive(g_mutexes.nvs_access);
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_IMU_CALIB, calib, sizeof(imu_calibration_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "IMU calibration saved");
    } else {
        ESP_LOGE(TAG, "Failed to save IMU calibration: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t fc_nvs_load_imu_calibration(imu_calibration_t *calib)
{
    if (calib == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_mutexes.nvs_access);
        /* Not found is expected on first boot */
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "No IMU calibration found in NVS");
            memset(calib, 0, sizeof(imu_calibration_t));
            calib->valid = false;
            return FC_ERR_NO_DATA;
        }
        return ret;
    }

    size_t size = sizeof(imu_calibration_t);
    ret = nvs_get_blob(handle, NVS_KEY_IMU_CALIB, calib, &size);

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    if (ret == ESP_OK && size == sizeof(imu_calibration_t)) {
        ESP_LOGI(TAG, "IMU calibration loaded");
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        memset(calib, 0, sizeof(imu_calibration_t));
        calib->valid = false;
        ret = FC_ERR_NO_DATA;
    }

    return ret;
}

/* ==========================================================================
 * Home Point Storage (FR-027)
 * ========================================================================== */

esp_err_t fc_nvs_save_home_point(const home_point_t *home)
{
    if (home == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_mutexes.nvs_access);
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_HOME_POINT, home, sizeof(home_point_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Home point saved: %.6f, %.6f",
                 home->latitude, home->longitude);
    }

    return ret;
}

esp_err_t fc_nvs_load_home_point(home_point_t *home)
{
    if (home == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_mutexes.nvs_access);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            memset(home, 0, sizeof(home_point_t));
            home->is_set = false;
            return FC_ERR_NO_DATA;
        }
        return ret;
    }

    size_t size = sizeof(home_point_t);
    ret = nvs_get_blob(handle, NVS_KEY_HOME_POINT, home, &size);

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        memset(home, 0, sizeof(home_point_t));
        home->is_set = false;
        ret = FC_ERR_NO_DATA;
    }

    return ret;
}

/* ==========================================================================
 * Telemetry Rate Storage
 * ========================================================================== */

esp_err_t fc_nvs_save_telemetry_rate(uint8_t rate_hz)
{
    if (rate_hz < 1 || rate_hz > 50) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_mutexes.nvs_access);
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_TELEM_RATE, rate_hz);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    return ret;
}

esp_err_t fc_nvs_load_telemetry_rate(uint8_t *rate_hz)
{
    if (rate_hz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_mutexes.nvs_access);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            *rate_hz = 20; /* Default 20Hz */
            return FC_ERR_NO_DATA;
        }
        return ret;
    }

    ret = nvs_get_u8(handle, NVS_KEY_TELEM_RATE, rate_hz);

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        *rate_hz = 20;
        ret = FC_ERR_NO_DATA;
    }

    return ret;
}

/* ==========================================================================
 * Utility Functions
 * ========================================================================== */

esp_err_t fc_nvs_erase_all(void)
{
    if (xSemaphoreTake(g_mutexes.nvs_access, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_mutexes.nvs_access);
        return ret;
    }

    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    xSemaphoreGive(g_mutexes.nvs_access);

    ESP_LOGW(TAG, "All NVS data erased");
    return ret;
}
