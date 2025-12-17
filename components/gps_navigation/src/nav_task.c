/**
 * @file nav_task.c
 * @brief Navigation compute FreeRTOS task
 *
 * Task: vTask_Navigation_Compute
 * Priority: configMAX_PRIORITIES-3
 * Stack: 4096 bytes
 * Period: 100ms (10Hz)
 */

#include "gps_navigation.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

static const char *TAG = TAG_NAV;

/* Communication loss detection (FR-026) */
static uint64_t s_last_telemetry_us = 0;

void vTask_Navigation_Compute(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Navigation compute task started");

    /* Initialize navigation controller */
    esp_err_t ret = navigation_controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize navigation: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    gps_position_t gps_pos;
    attitude_state_t attitude;
    control_command_t command = {0};
    nav_state_t nav_state;

    uint32_t update_count = 0;

    /* Initialize last telemetry time */
    s_last_telemetry_us = esp_timer_get_time();

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        bool got_gps = false;
        bool got_attitude = false;

        /* Get GPS data */
        if (xQueueReceive(g_queues.gps_to_nav, &gps_pos, 0) == pdTRUE) {
            got_gps = true;
        }

        /* Get attitude data */
        if (fc_get_attitude_state(&attitude, 0) == ESP_OK) {
            got_attitude = true;
        }

        /* Check for telemetry timeout (communication loss) - FR-026 */
        uint64_t now = esp_timer_get_time();
        telemetry_packet_t telem;
        if (xQueueReceive(g_queues.telemetry_rx, &telem, 0) == pdTRUE) {
            s_last_telemetry_us = now;

            /* Process telemetry commands */
            if (telem.msg_type == 0x01) { /* Mode change command */
                nav_mode_t new_mode = (nav_mode_t)telem.payload[0];
                navigation_set_mode(new_mode);
            }
        }

        /* Check for communication loss */
        if ((now - s_last_telemetry_us) > (COMM_LOSS_TIMEOUT_MS * 1000ULL)) {
            if (!navigation_is_rtl_active()) {
                ESP_LOGW(TAG, "Communication loss detected, triggering RTL");
                navigation_trigger_rtl();
            }
        }

        /* Update navigation */
        if (got_gps && got_attitude) {
            /* Clear command */
            memset(&command, 0, sizeof(command));
            command.armed = true; /* Assume armed if navigating */

            ret = navigation_update(&gps_pos, &attitude, &command);

            if (ret == ESP_OK) {
                /* Get navigation state for logging */
                navigation_get_state(&nav_state);

                /* Send command to motor controller */
                if (xQueueSend(g_queues.cmd_to_motor, &command, 0) != pdTRUE) {
                    ESP_LOGD(TAG, "Nav->Motor queue full");
                }

                update_count++;

                /* Periodic logging */
                if ((update_count % 50) == 0) { /* Every 5 seconds at 10Hz */
                    const char *mode_names[] = {
                        "MANUAL", "STABILIZE", "LOITER", "WAYPOINT", "RTL"
                    };
                    ESP_LOGI(TAG, "Nav: mode=%s, dist=%.0fm, hdg=%.0f°",
                             mode_names[nav_state.mode],
                             nav_state.distance_to_target_m,
                             RAD_TO_DEG(nav_state.target_heading_rad));
                }
            }
        }

        /* Stack check every 60 seconds */
        if ((update_count % 600) == 0 && update_count > 0) {
            FC_LOG_STACK(TAG, NULL);
        }

        /* Maintain 10Hz rate */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Update telemetry timestamp (called from telemetry task)
 */
void navigation_update_telemetry_time(void)
{
    s_last_telemetry_us = esp_timer_get_time();
}
