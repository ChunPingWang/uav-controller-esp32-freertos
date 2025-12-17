/**
 * @file motor_task.c
 * @brief Motor control FreeRTOS task
 *
 * Task: vTask_Motor_Control
 * Priority: configMAX_PRIORITIES-2
 * Stack: 4096 bytes
 * Period: 20ms (50Hz)
 *
 * Receives control commands, processes through PID, outputs to servos.
 */

#include "motor_controller.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

static const char *TAG = TAG_MTR;

/* Performance monitoring */
static uint32_t s_update_count = 0;
static uint32_t s_max_latency_us = 0;

void vTask_Motor_Control(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Motor control task started");

    /* Initialize motor controller */
    motor_config_t config = MOTOR_CONFIG_DEFAULT();
    esp_err_t ret = motor_controller_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor controller: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    /* Start in safe state */
    motor_controller_set_safe();

    control_command_t command;
    control_output_t output;
    attitude_state_t attitude;
    uint64_t last_attitude_time = 0;

    const float dt = 0.02f; /* 50Hz = 20ms */

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* Check for new control command from telemetry/navigation */
        if (xQueueReceive(g_queues.cmd_to_motor, &command, 0) == pdTRUE) {
            /* Update armed state */
            motor_controller_set_armed(command.armed);
        }

        /* Get latest attitude */
        if (fc_get_attitude_state(&attitude, 5) == ESP_OK) {
            uint64_t now = esp_timer_get_time();

            /* Check attitude freshness */
            if (attitude.timestamp_us > last_attitude_time) {
                /* Calculate actual dt if possible */
                float actual_dt = dt;
                if (last_attitude_time > 0) {
                    actual_dt = (attitude.timestamp_us - last_attitude_time) * 1e-6f;
                    if (actual_dt < 0.001f) actual_dt = 0.001f;
                    if (actual_dt > 0.1f) actual_dt = 0.1f;
                }
                last_attitude_time = attitude.timestamp_us;

                /* Track latency from IMU read to motor output */
                uint32_t latency_us = (uint32_t)(now - attitude.timestamp_us);
                if (latency_us > s_max_latency_us) {
                    s_max_latency_us = latency_us;
                }

                /* Run control loop */
                if (attitude.valid) {
                    ret = motor_controller_update(&attitude, &command, &output, actual_dt);
                    if (ret == ESP_OK) {
                        motor_controller_apply_outputs(&output);
                    }
                }
            }
        } else {
            /* No attitude data - use failsafe outputs */
            if (motor_controller_is_armed()) {
                ESP_LOGW(TAG, "No attitude data, setting safe outputs");
                motor_controller_set_safe();
            }
        }

        s_update_count++;

        /* Periodic logging */
        if ((s_update_count % 500) == 0) { /* Every 10 seconds at 50Hz */
            ESP_LOGI(TAG, "Motor: updates=%lu, max_latency=%lu us, armed=%d",
                     (unsigned long)s_update_count,
                     (unsigned long)s_max_latency_us,
                     motor_controller_is_armed());

            FC_LOG_STACK(TAG, NULL);

            /* Reset max latency */
            s_max_latency_us = 0;
        }

        /* Maintain 50Hz rate */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}
