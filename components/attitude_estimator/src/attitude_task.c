/**
 * @file attitude_task.c
 * @brief Attitude computation FreeRTOS task
 *
 * Task: vTask_Attitude_Compute
 * Priority: configMAX_PRIORITIES-2
 * Stack: 8192 bytes
 * Period: 10ms (100Hz)
 */

#include "attitude_estimator.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

static const char *TAG = TAG_ATT;

/* Performance monitoring */
static uint32_t s_max_compute_us = 0;
static uint32_t s_update_count = 0;

void vTask_Attitude_Compute(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Attitude compute task started");

    /* Initialize attitude estimator */
    attitude_config_t config = ATTITUDE_CONFIG_DEFAULT();
    esp_err_t ret = attitude_estimator_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize attitude estimator");
        vTaskDelete(NULL);
        return;
    }

    sensor_data_t imu_data;
    uint64_t last_timestamp = 0;
    const float dt_nominal = 0.01f; /* 10ms */

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* Wait for IMU data from queue */
        if (xQueueReceive(g_queues.imu_to_attitude, &imu_data, pdMS_TO_TICKS(20)) == pdTRUE) {

            /* Calculate actual dt */
            float dt = dt_nominal;
            if (last_timestamp > 0 && imu_data.timestamp_us > last_timestamp) {
                dt = (imu_data.timestamp_us - last_timestamp) * 1e-6f;
                /* Clamp dt to reasonable range */
                if (dt < 0.001f) dt = 0.001f;
                if (dt > 0.1f) dt = 0.1f;
            }
            last_timestamp = imu_data.timestamp_us;

            /* Performance timing */
            FC_PERF_START(compute_start);

            /* Process IMU data through EKF */
            ret = attitude_estimator_process_imu(&imu_data, dt);

            FC_PERF_END_CHECK(compute_start, TAG, "EKF update", 500);

            /* Track max compute time */
            uint32_t compute_us = attitude_estimator_get_compute_time_us();
            if (compute_us > s_max_compute_us) {
                s_max_compute_us = compute_us;
            }

            if (ret == ESP_OK) {
                /* Update shared attitude state */
                attitude_state_t state;
                attitude_estimator_get_state(&state);
                state.timestamp_us = esp_timer_get_time();

                fc_set_attitude_state(&state, 10);

                /* Set event flag */
                fc_event_set(EVT_IMU_DATA_READY);
            }

            s_update_count++;

            /* Log periodically */
            if ((s_update_count % 1000) == 0) {
                ESP_LOGI(TAG, "EKF updates: %lu, max compute: %lu us, cov trace: %.4f",
                         (unsigned long)s_update_count,
                         (unsigned long)s_max_compute_us,
                         attitude_estimator_get_covariance_trace());

                FC_LOG_STACK(TAG, NULL);
            }
        }

        /* Check for GPS heading update */
        gps_position_t gps_data;
        if (xQueueReceive(g_queues.gps_to_attitude, &gps_data, 0) == pdTRUE) {
            if (gps_data.valid && gps_data.speed_ms >= 5.0f) {
                attitude_estimator_update_gps_heading(gps_data.heading_deg, gps_data.speed_ms);
                ESP_LOGD(TAG, "GPS heading update: %.1f deg @ %.1f m/s",
                         gps_data.heading_deg, gps_data.speed_ms);
            }
        }

        /* Maintain 100Hz rate */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
