/**
 * @file fc_failsafe.c
 * @brief Failsafe logic implementation
 *
 * Handles:
 * - FR-025: RTL on communication loss
 * - FR-026: 3-second comm loss timeout
 * - FR-027: Loiter on GPS loss
 * - Low battery RTL trigger
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_FAILSAFE;

/* Failsafe state */
typedef struct {
    bool initialized;
    bool failsafe_active;
    uint8_t failsafe_reason;

    /* Tracking */
    uint64_t last_telemetry_us;
    uint64_t last_gps_us;
    uint32_t comm_loss_count;
    uint32_t gps_loss_count;

    /* Thresholds */
    uint32_t comm_loss_timeout_ms;
    uint32_t gps_loss_timeout_ms;
    float low_battery_threshold;
} failsafe_state_t;

/* Failsafe reasons */
#define FAILSAFE_NONE           0
#define FAILSAFE_COMM_LOSS      1
#define FAILSAFE_GPS_LOSS       2
#define FAILSAFE_LOW_BATTERY    3
#define FAILSAFE_SENSOR_FAIL    4
#define FAILSAFE_MANUAL         5

static failsafe_state_t s_state = {0};

/* External functions - declared in other components */
extern bool telemetry_is_link_lost(void);
extern bool gps_driver_is_ready(void);
extern float fc_battery_get_percent(void);
extern esp_err_t navigation_trigger_rtl(void);
extern esp_err_t navigation_enter_loiter(float radius_m);

esp_err_t fc_failsafe_init(void)
{
    if (s_state.initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    ESP_LOGI(TAG, "Initializing failsafe module");

    memset(&s_state, 0, sizeof(s_state));

    s_state.comm_loss_timeout_ms = CONFIG_FAILSAFE_COMM_LOSS_MS;
    s_state.gps_loss_timeout_ms = 5000; /* 5 seconds */
    s_state.low_battery_threshold = (float)CONFIG_FAILSAFE_LOW_BATTERY_PCT;

    s_state.last_telemetry_us = esp_timer_get_time();
    s_state.last_gps_us = esp_timer_get_time();

    s_state.initialized = true;

    ESP_LOGI(TAG, "Failsafe initialized: comm_timeout=%lu ms, low_batt=%.0f%%",
             (unsigned long)s_state.comm_loss_timeout_ms,
             s_state.low_battery_threshold);

    return ESP_OK;
}

esp_err_t fc_failsafe_check(void)
{
    if (!s_state.initialized) {
        return FC_ERR_NOT_INIT;
    }

    uint64_t now = esp_timer_get_time();
    bool should_trigger_rtl = false;
    bool should_trigger_loiter = false;
    const char *reason = NULL;
    uint8_t failsafe_type = FAILSAFE_NONE;

    /* ============ Check communication loss (FR-025, FR-026) ============ */
    if (telemetry_is_link_lost()) {
        s_state.comm_loss_count++;

        if (!s_state.failsafe_active || s_state.failsafe_reason != FAILSAFE_COMM_LOSS) {
            ESP_LOGW(TAG, "Communication loss detected");
            should_trigger_rtl = true;
            reason = "Communication loss";
            failsafe_type = FAILSAFE_COMM_LOSS;
        }
    } else {
        s_state.last_telemetry_us = now;
        s_state.comm_loss_count = 0;
    }

    /* ============ Check GPS loss (FR-027) ============ */
    if (!gps_driver_is_ready()) {
        uint64_t gps_age_us = now - s_state.last_gps_us;

        if (gps_age_us > (s_state.gps_loss_timeout_ms * 1000ULL)) {
            s_state.gps_loss_count++;

            if (!s_state.failsafe_active || s_state.failsafe_reason != FAILSAFE_GPS_LOSS) {
                ESP_LOGW(TAG, "GPS loss detected");
                should_trigger_loiter = true;
                reason = "GPS signal loss";
                failsafe_type = FAILSAFE_GPS_LOSS;
            }
        }
    } else {
        s_state.last_gps_us = now;
        s_state.gps_loss_count = 0;
    }

    /* ============ Check low battery ============ */
    float battery_pct = fc_battery_get_percent();
    if (battery_pct < s_state.low_battery_threshold) {
        if (!s_state.failsafe_active || s_state.failsafe_reason != FAILSAFE_LOW_BATTERY) {
            ESP_LOGW(TAG, "Low battery: %.1f%% (threshold: %.1f%%)",
                     battery_pct, s_state.low_battery_threshold);
            should_trigger_rtl = true;
            reason = "Low battery";
            failsafe_type = FAILSAFE_LOW_BATTERY;
        }
    }

    /* ============ Trigger appropriate failsafe ============ */

    /* RTL has priority over loiter */
    if (should_trigger_rtl) {
        fc_failsafe_trigger_rtl(reason);
        s_state.failsafe_active = true;
        s_state.failsafe_reason = failsafe_type;
    } else if (should_trigger_loiter && !s_state.failsafe_active) {
        fc_failsafe_trigger_loiter(reason);
        s_state.failsafe_active = true;
        s_state.failsafe_reason = failsafe_type;
    }

    return ESP_OK;
}

esp_err_t fc_failsafe_trigger_rtl(const char *reason)
{
    ESP_LOGW(TAG, "FAILSAFE RTL: %s", reason ? reason : "unknown");
    FC_LOG_FAILSAFE(reason ? reason : "RTL triggered");

    /* Notify system of failsafe */
    fc_notify_failsafe(true);
    fc_event_set(EVT_FAILSAFE_TRIGGERED);

    /* Trigger RTL in navigation */
    esp_err_t ret = navigation_trigger_rtl();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger RTL: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t fc_failsafe_trigger_loiter(const char *reason)
{
    ESP_LOGW(TAG, "FAILSAFE LOITER: %s", reason ? reason : "unknown");
    FC_LOG_FAILSAFE(reason ? reason : "Loiter triggered");

    /* Notify system of failsafe */
    fc_notify_failsafe(true);
    fc_event_set(EVT_FAILSAFE_TRIGGERED);

    /* Enter loiter mode */
    esp_err_t ret = navigation_enter_loiter(100.0f); /* 100m radius */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter loiter: %s", esp_err_to_name(ret));
        /* Fall back to RTL */
        return fc_failsafe_trigger_rtl("Loiter failed, falling back to RTL");
    }

    return ret;
}

esp_err_t fc_failsafe_clear(void)
{
    if (s_state.failsafe_active) {
        ESP_LOGI(TAG, "Failsafe cleared");
        s_state.failsafe_active = false;
        s_state.failsafe_reason = FAILSAFE_NONE;
        fc_notify_failsafe(false);
    }

    return ESP_OK;
}

bool fc_failsafe_is_active(void)
{
    return s_state.failsafe_active;
}

uint8_t fc_failsafe_get_reason(void)
{
    return s_state.failsafe_reason;
}

void fc_failsafe_update_gps_time(void)
{
    s_state.last_gps_us = esp_timer_get_time();
}

void fc_failsafe_update_telemetry_time(void)
{
    s_state.last_telemetry_us = esp_timer_get_time();
}
