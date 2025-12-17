/**
 * @file fc_log.c
 * @brief Logging implementation for Fixed-Wing Flight Controller
 */

#include "fc_log.h"
#include "fc_types.h"
#include <string.h>

static bool s_debug_enabled = false;

esp_err_t fc_log_init(void)
{
    /* Set default log levels */
    esp_log_level_set(TAG_FC, ESP_LOG_INFO);
    esp_log_level_set(TAG_IMU, ESP_LOG_INFO);
    esp_log_level_set(TAG_ATT, ESP_LOG_INFO);
    esp_log_level_set(TAG_GPS, ESP_LOG_INFO);
    esp_log_level_set(TAG_MOTOR, ESP_LOG_INFO);
    esp_log_level_set(TAG_TELEM, ESP_LOG_INFO);
    esp_log_level_set(TAG_FAILSAFE, ESP_LOG_WARN);

    ESP_LOGI(TAG_FC, "Logging initialized");
    return ESP_OK;
}

void fc_log_set_level(const char *tag, esp_log_level_t level)
{
    if (tag != NULL) {
        esp_log_level_set(tag, level);
    }
}

void fc_log_set_debug(bool enable)
{
    s_debug_enabled = enable;

    esp_log_level_t level = enable ? ESP_LOG_DEBUG : ESP_LOG_INFO;

    esp_log_level_set(TAG_FC, level);
    esp_log_level_set(TAG_IMU, level);
    esp_log_level_set(TAG_ATT, level);
    esp_log_level_set(TAG_GPS, level);
    esp_log_level_set(TAG_MOTOR, level);
    esp_log_level_set(TAG_TELEM, level);

    ESP_LOGI(TAG_FC, "Debug logging %s", enable ? "enabled" : "disabled");
}
