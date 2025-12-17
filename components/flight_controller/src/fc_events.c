/**
 * @file fc_events.c
 * @brief System event management
 *
 * Provides event notification functions for cross-task communication.
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"

static const char *TAG = TAG_FC;

/* ==========================================================================
 * Event Notification Functions
 * ========================================================================== */

void fc_event_set(EventBits_t bits)
{
    if (g_system_events != NULL) {
        xEventGroupSetBits(g_system_events, bits);
    }
}

void fc_event_set_from_isr(EventBits_t bits, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (g_system_events != NULL) {
        xEventGroupSetBitsFromISR(g_system_events, bits, pxHigherPriorityTaskWoken);
    }
}

void fc_event_clear(EventBits_t bits)
{
    if (g_system_events != NULL) {
        xEventGroupClearBits(g_system_events, bits);
    }
}

EventBits_t fc_event_wait(EventBits_t bits, bool wait_all, bool auto_clear, uint32_t timeout_ms)
{
    if (g_system_events == NULL) {
        return 0;
    }

    return xEventGroupWaitBits(g_system_events, bits,
                               auto_clear ? pdTRUE : pdFALSE,
                               wait_all ? pdTRUE : pdFALSE,
                               pdMS_TO_TICKS(timeout_ms));
}

EventBits_t fc_event_get(void)
{
    if (g_system_events == NULL) {
        return 0;
    }

    return xEventGroupGetBits(g_system_events);
}

/* ==========================================================================
 * High-Level Event Handlers
 * ========================================================================== */

void fc_notify_imu_ready(void)
{
    fc_event_set(EVT_IMU_DATA_READY);
}

void fc_notify_gps_fix(bool has_fix)
{
    if (has_fix) {
        fc_event_clear(EVT_GPS_FIX_LOST);
        fc_event_set(EVT_GPS_FIX_ACQUIRED);
    } else {
        fc_event_clear(EVT_GPS_FIX_ACQUIRED);
        fc_event_set(EVT_GPS_FIX_LOST);
    }
}

void fc_notify_comm_timeout(void)
{
    ESP_LOGW(TAG, "Communication timeout event");
    fc_event_set(EVT_COMM_TIMEOUT);
}

void fc_notify_low_battery(void)
{
    ESP_LOGW(TAG, "Low battery event");
    fc_event_set(EVT_LOW_BATTERY);
}

void fc_notify_failsafe(bool active)
{
    if (active) {
        ESP_LOGW(TAG, "Failsafe activated");
        fc_event_set(EVT_FAILSAFE_ACTIVE);
    } else {
        ESP_LOGI(TAG, "Failsafe cleared");
        fc_event_clear(EVT_FAILSAFE_ACTIVE);
    }
}

void fc_notify_armed(bool armed)
{
    if (armed) {
        fc_event_clear(EVT_MOTOR_DISARMED);
        fc_event_set(EVT_MOTOR_ARMED);
        ESP_LOGI(TAG, "Motors ARMED");
    } else {
        fc_event_clear(EVT_MOTOR_ARMED);
        fc_event_set(EVT_MOTOR_DISARMED);
        ESP_LOGI(TAG, "Motors DISARMED");
    }
}

void fc_notify_home_set(void)
{
    ESP_LOGI(TAG, "Home point set");
    fc_event_set(EVT_HOME_SET);
}

void fc_notify_selftest(bool passed)
{
    if (passed) {
        fc_event_set(EVT_SELFTEST_PASS);
        ESP_LOGI(TAG, "Self-test PASSED");
    } else {
        fc_event_set(EVT_SELFTEST_FAIL);
        ESP_LOGE(TAG, "Self-test FAILED");
    }
}

/* ==========================================================================
 * Event Status Queries
 * ========================================================================== */

bool fc_is_failsafe_active(void)
{
    return (fc_event_get() & EVT_FAILSAFE_ACTIVE) != 0;
}

bool fc_is_armed(void)
{
    return (fc_event_get() & EVT_MOTOR_ARMED) != 0;
}

bool fc_has_gps_fix(void)
{
    return (fc_event_get() & EVT_GPS_FIX_ACQUIRED) != 0;
}

bool fc_is_home_set(void)
{
    return (fc_event_get() & EVT_HOME_SET) != 0;
}
