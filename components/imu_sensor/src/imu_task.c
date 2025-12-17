/**
 * @file imu_task.c
 * @brief IMU read FreeRTOS task
 *
 * Task: vTask_IMU_Read
 * Priority: configMAX_PRIORITIES-1 (highest real-time)
 * Stack: 4096 bytes
 * Period: 10ms (100Hz) - FR-001
 */

#include "imu_sensor.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

static const char *TAG = TAG_IMU;

/* Declared in imu_calibration.c */
extern void imu_calibration_accumulate(const sensor_data_t *data);

void vTask_IMU_Read(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "IMU read task started");

    /* Initialize IMU sensor */
    imu_config_t config = IMU_CONFIG_DEFAULT();
    esp_err_t ret = imu_sensor_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IMU: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    /* Run self-test */
    ret = imu_sensor_selftest();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU self-test failed, continuing anyway");
    }

    sensor_data_t data;
    uint32_t read_count = 0;
    uint32_t error_count = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* Read filtered IMU data */
        ret = imu_sensor_read_filtered(&data);

        if (ret == ESP_OK) {
            /* Check for calibration mode */
            if (imu_sensor_is_calibrating()) {
                imu_calibration_accumulate(&data);
            }

            /* Send to attitude estimator */
            if (xQueueSend(g_queues.imu_to_attitude, &data, 0) != pdTRUE) {
                /* Queue full - this shouldn't happen at 100Hz */
                ESP_LOGW(TAG, "IMU queue full");
            }

            read_count++;

            /* Debug logging every second */
            if ((read_count % 100) == 0) {
                ESP_LOGD(TAG, "IMU: ax=%.2f ay=%.2f az=%.2f gx=%.3f gy=%.3f gz=%.3f q=%d",
                         data.accel_x, data.accel_y, data.accel_z,
                         data.gyro_x, data.gyro_y, data.gyro_z,
                         data.quality);
            }

            /* Stack usage check every 10 seconds */
            if ((read_count % 1000) == 0) {
                FC_LOG_STACK(TAG, NULL);
            }

        } else {
            error_count++;
            if (error_count > 10) {
                ESP_LOGE(TAG, "Too many IMU read errors");
                /* Continue anyway - let failsafe handle it */
            }
        }

        /* Maintain 100Hz rate (10ms period) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
