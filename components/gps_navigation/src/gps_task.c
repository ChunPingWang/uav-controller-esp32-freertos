/**
 * @file gps_task.c
 * @brief GPS read FreeRTOS task
 *
 * Task: vTask_GPS_Read
 * Priority: configMAX_PRIORITIES-3
 * Stack: 4096 bytes
 * Period: 100ms (10Hz) - FR-011
 */

#include "gps_navigation.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

static const char *TAG = TAG_GPS;

void vTask_GPS_Read(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "GPS read task started");

    /* Initialize GPS driver */
    gps_config_t config = GPS_CONFIG_DEFAULT();
    esp_err_t ret = gps_driver_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPS: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    gps_position_t position;
    uint32_t valid_count = 0;
    uint32_t invalid_count = 0;
    bool home_set = false;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* Process incoming GPS data */
        gps_driver_process();

        /* Get current position */
        ret = gps_driver_get_position(&position);

        if (ret == ESP_OK && position.valid) {
            valid_count++;

            /* Auto-set home on first valid fix (FR-012) */
            if (!home_set && !navigation_home_is_set()) {
                if (position.satellites >= GPS_MIN_SATELLITES) {
                    navigation_set_home(&position);
                    home_set = true;
                    ESP_LOGI(TAG, "Home auto-set at first GPS lock");
                }
            }

            /* Send to attitude estimator for heading updates */
            if (position.speed_ms >= GPS_MIN_SPEED_MS) {
                if (xQueueSend(g_queues.gps_to_attitude, &position, 0) != pdTRUE) {
                    ESP_LOGD(TAG, "GPS->Attitude queue full");
                }
            }

            /* Send to navigation task */
            if (xQueueSend(g_queues.gps_to_nav, &position, 0) != pdTRUE) {
                ESP_LOGD(TAG, "GPS->Nav queue full");
            }

            /* Set event flag */
            fc_event_set(EVT_GPS_DATA_READY);

            /* Log every 10 seconds */
            if ((valid_count % 100) == 0) {
                ESP_LOGI(TAG, "GPS: %.6f, %.6f, %.1fm, %d sats, HDOP=%.1f, %.1f m/s",
                         position.latitude_deg, position.longitude_deg,
                         position.altitude_m, position.satellites,
                         position.hdop, position.speed_ms);
            }

        } else {
            invalid_count++;

            if ((invalid_count % 50) == 0) {
                ESP_LOGW(TAG, "GPS: no valid fix (sats=%d, hdop=%.1f)",
                         gps_driver_get_satellites(), gps_driver_get_hdop());
            }
        }

        /* Stack check every 60 seconds */
        if (((valid_count + invalid_count) % 600) == 0) {
            FC_LOG_STACK(TAG, NULL);
        }

        /* Maintain 10Hz rate (100ms period) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}
