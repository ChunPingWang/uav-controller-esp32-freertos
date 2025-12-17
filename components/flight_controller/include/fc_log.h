/**
 * @file fc_log.h
 * @brief Logging wrapper for Fixed-Wing Flight Controller
 *
 * Provides logging macros and utilities following Constitution VI.
 * Wraps ESP-IDF ESP_LOG macros with flight controller context.
 */

#ifndef FC_LOG_H
#define FC_LOG_H

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Module Tags
 * ========================================================================== */

#define TAG_FC          "FC"
#define TAG_IMU         "IMU"
#define TAG_ATT         "ATT"
#define TAG_GPS         "GPS"
#define TAG_MOTOR       "MOTOR"
#define TAG_TELEM       "TELEM"
#define TAG_FAILSAFE    "FAILSAFE"

/* ==========================================================================
 * Log Level Control
 * ========================================================================== */

/**
 * @brief Initialize logging subsystem
 *
 * Sets default log levels for all modules.
 *
 * @return ESP_OK on success
 */
esp_err_t fc_log_init(void);

/**
 * @brief Set log level for a specific tag
 *
 * @param tag Module tag
 * @param level Log level (ESP_LOG_NONE to ESP_LOG_VERBOSE)
 */
void fc_log_set_level(const char *tag, esp_log_level_t level);

/**
 * @brief Enable/disable all debug logging
 *
 * @param enable true to enable verbose logging
 */
void fc_log_set_debug(bool enable);

/* ==========================================================================
 * Enhanced Logging Macros
 * ========================================================================== */

/**
 * @brief Log with timestamp (microseconds)
 */
#define FC_LOG_TIMED(level, tag, fmt, ...) \
    ESP_LOG_LEVEL(level, tag, "[%llu] " fmt, \
        (unsigned long long)esp_timer_get_time(), ##__VA_ARGS__)

/**
 * @brief Log sensor data
 */
#define FC_LOG_SENSOR(tag, name, x, y, z) \
    ESP_LOGD(tag, "%s: x=%.4f y=%.4f z=%.4f", name, (x), (y), (z))

/**
 * @brief Log attitude data
 */
#define FC_LOG_ATTITUDE(tag, roll, pitch, yaw) \
    ESP_LOGD(tag, "Attitude: R=%.2f P=%.2f Y=%.2f deg", \
        RAD_TO_DEG(roll), RAD_TO_DEG(pitch), RAD_TO_DEG(yaw))

/**
 * @brief Log GPS position
 */
#define FC_LOG_GPS(tag, lat, lon, alt) \
    ESP_LOGI(tag, "GPS: %.6f, %.6f, %.1fm", (lat), (lon), (alt))

/**
 * @brief Log error with function name
 */
#define FC_LOG_ERR_FUNC(tag, err) \
    ESP_LOGE(tag, "%s failed: %s", __func__, esp_err_to_name(err))

/* ==========================================================================
 * Critical Event Logging
 * ========================================================================== */

/**
 * @brief Log failsafe activation (always logged)
 */
#define FC_LOG_FAILSAFE(reason) \
    ESP_LOGW(TAG_FAILSAFE, "FAILSAFE ACTIVATED: %s", (reason))

/**
 * @brief Log mode change
 */
#define FC_LOG_MODE_CHANGE(from, to) \
    ESP_LOGI(TAG_FC, "Mode: %s -> %s", (from), (to))

/**
 * @brief Log arm/disarm
 */
#define FC_LOG_ARM(armed) \
    ESP_LOGI(TAG_FC, "Motors %s", (armed) ? "ARMED" : "DISARMED")

/* ==========================================================================
 * Performance Logging
 * ========================================================================== */

/**
 * @brief Start timing measurement
 */
#define FC_PERF_START(var) \
    int64_t var = esp_timer_get_time()

/**
 * @brief End timing and log if exceeds threshold
 */
#define FC_PERF_END_CHECK(var, tag, name, threshold_us) do { \
    int64_t __elapsed = esp_timer_get_time() - (var);        \
    if (__elapsed > (threshold_us)) {                         \
        ESP_LOGW(tag, "%s took %lld us (threshold: %d us)",   \
            (name), (long long)__elapsed, (threshold_us));    \
    }                                                         \
} while(0)

/**
 * @brief Log stack high water mark
 */
#define FC_LOG_STACK(tag, task_handle) do {                             \
    UBaseType_t __hwm = uxTaskGetStackHighWaterMark(task_handle);       \
    ESP_LOGI(tag, "Stack HWM: %u words (%u bytes)",                     \
        (unsigned)__hwm, (unsigned)(__hwm * sizeof(StackType_t)));      \
} while(0)

#ifdef __cplusplus
}
#endif

#endif /* FC_LOG_H */
