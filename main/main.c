/**
 * @file main.c
 * @brief Fixed-Wing Aircraft Controller - Application Entry Point
 *
 * This is the main application entry point for the ESP32-based flight controller.
 * Initializes all subsystems and creates FreeRTOS tasks.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "app_config.h"
#include "fc_types.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"

/* Component headers */
#include "imu_sensor.h"
#include "attitude_estimator.h"
#include "motor_controller.h"
#include "gps_navigation.h"
#include "xbee_telemetry.h"

static const char *TAG = "main";

/* External task functions */
extern void vTask_IMU_Read(void *pvParameters);
extern void vTask_Attitude_Compute(void *pvParameters);
extern void vTask_Motor_Control(void *pvParameters);
extern void vTask_GPS_Read(void *pvParameters);
extern void vTask_Navigation_Compute(void *pvParameters);
extern void vTask_Telemetry(void *pvParameters);
extern void vTask_Monitor(void *pvParameters);

/**
 * @brief Initialize all IPC mechanisms (queues, mutexes, events)
 */
static esp_err_t init_ipc(void)
{
    ESP_LOGI(TAG, "Initializing IPC mechanisms");

    esp_err_t ret = fc_ipc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IPC initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "IPC mechanisms initialized");
    return ESP_OK;
}

/**
 * @brief Create all FreeRTOS tasks
 */
static esp_err_t create_tasks(void)
{
    ESP_LOGI(TAG, "Creating FreeRTOS tasks");

    /* Task creation uses static allocation per Constitution III */

    /* Task 1: IMU Read - Highest priority (100Hz) */
    g_tasks.imu_task = xTaskCreateStatic(
        vTask_IMU_Read,
        "IMU_Read",
        TASK_STACK_SIZE_IMU / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_IMU,
        g_task_stacks.imu_stack,
        &g_task_tcbs.imu_tcb
    );
    if (g_tasks.imu_task == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_FAIL;
    }

    /* Task 2: Attitude Compute (100Hz) */
    g_tasks.attitude_task = xTaskCreateStatic(
        vTask_Attitude_Compute,
        "Attitude",
        TASK_STACK_SIZE_ATTITUDE / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_ATTITUDE,
        g_task_stacks.attitude_stack,
        &g_task_tcbs.attitude_tcb
    );
    if (g_tasks.attitude_task == NULL) {
        ESP_LOGE(TAG, "Failed to create Attitude task");
        return ESP_FAIL;
    }

    /* Task 3: Motor Control (50Hz) */
    g_tasks.motor_task = xTaskCreateStatic(
        vTask_Motor_Control,
        "Motor",
        TASK_STACK_SIZE_MOTOR / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_MOTOR,
        g_task_stacks.motor_stack,
        &g_task_tcbs.motor_tcb
    );
    if (g_tasks.motor_task == NULL) {
        ESP_LOGE(TAG, "Failed to create Motor task");
        return ESP_FAIL;
    }

    /* Task 4: GPS Read (10Hz) */
    g_tasks.gps_task = xTaskCreateStatic(
        vTask_GPS_Read,
        "GPS_Read",
        TASK_STACK_SIZE_GPS / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_GPS,
        g_task_stacks.gps_stack,
        &g_task_tcbs.gps_tcb
    );
    if (g_tasks.gps_task == NULL) {
        ESP_LOGE(TAG, "Failed to create GPS task");
        return ESP_FAIL;
    }

    /* Task 5: Navigation Compute (10Hz) */
    g_tasks.nav_task = xTaskCreateStatic(
        vTask_Navigation_Compute,
        "Navigation",
        TASK_STACK_SIZE_NAV / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_NAV,
        g_task_stacks.nav_stack,
        &g_task_tcbs.nav_tcb
    );
    if (g_tasks.nav_task == NULL) {
        ESP_LOGE(TAG, "Failed to create Navigation task");
        return ESP_FAIL;
    }

    /* Task 6: Telemetry (5Hz) */
    g_tasks.telemetry_task = xTaskCreateStatic(
        vTask_Telemetry,
        "Telemetry",
        TASK_STACK_SIZE_TELEMETRY / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_TELEMETRY,
        g_task_stacks.telemetry_stack,
        &g_task_tcbs.telemetry_tcb
    );
    if (g_tasks.telemetry_task == NULL) {
        ESP_LOGE(TAG, "Failed to create Telemetry task");
        return ESP_FAIL;
    }

    /* Task 7: Monitor (1Hz) */
    g_tasks.monitor_task = xTaskCreateStatic(
        vTask_Monitor,
        "Monitor",
        TASK_STACK_SIZE_MONITOR / sizeof(StackType_t),
        NULL,
        TASK_PRIORITY_MONITOR,
        g_task_stacks.monitor_stack,
        &g_task_tcbs.monitor_tcb
    );
    if (g_tasks.monitor_task == NULL) {
        ESP_LOGE(TAG, "Failed to create Monitor task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All tasks created successfully");
    return ESP_OK;
}

/**
 * @brief Run self-tests on startup
 */
static esp_err_t run_self_tests(void)
{
    ESP_LOGI(TAG, "Running startup self-tests");

    /* Check heap */
    uint32_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)free_heap);
    if (free_heap < 50000) {
        ESP_LOGW(TAG, "Low heap at startup");
    }

    /* Log task info */
    ESP_LOGI(TAG, "Task priorities:");
    ESP_LOGI(TAG, "  IMU:        %d", TASK_PRIORITY_IMU);
    ESP_LOGI(TAG, "  Attitude:   %d", TASK_PRIORITY_ATTITUDE);
    ESP_LOGI(TAG, "  Motor:      %d", TASK_PRIORITY_MOTOR);
    ESP_LOGI(TAG, "  GPS:        %d", TASK_PRIORITY_GPS);
    ESP_LOGI(TAG, "  Navigation: %d", TASK_PRIORITY_NAV);
    ESP_LOGI(TAG, "  Telemetry:  %d", TASK_PRIORITY_TELEMETRY);
    ESP_LOGI(TAG, "  Monitor:    %d", TASK_PRIORITY_MONITOR);

    return ESP_OK;
}

/**
 * @brief Application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Fixed-Wing Controller v%s", APP_VERSION_STRING);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing...");

    /* Initialize NVS (required for calibration storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS: erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* Initialize IPC mechanisms */
    ret = init_ipc();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: IPC initialization failed");
        esp_restart();
    }

    /* Run self-tests */
    ret = run_self_tests();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Self-test warnings");
    }

    /* Create all tasks */
    ret = create_tasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: Task creation failed");
        esp_restart();
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "System initialized - all tasks running");
    ESP_LOGI(TAG, "========================================");

    /* Main task can now be deleted - other tasks handle everything */
    /* Or keep it running with low priority for diagnostics */

    uint32_t loop_count = 0;
    while (1) {
        loop_count++;

        /* Periodic heartbeat */
        if ((loop_count % 60) == 0) {
            ESP_LOGI(TAG, "Heartbeat: uptime=%lu min, heap=%lu",
                     (unsigned long)(loop_count / 60),
                     (unsigned long)esp_get_free_heap_size());
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
