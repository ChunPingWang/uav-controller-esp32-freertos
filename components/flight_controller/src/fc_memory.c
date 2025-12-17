/**
 * @file fc_memory.c
 * @brief Static memory allocation helpers
 *
 * Provides static allocation for FreeRTOS objects (Constitution III).
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_heap_caps.h"

#include <string.h>

static const char *TAG = TAG_FC;

/* ==========================================================================
 * Static Buffers for Tasks (Constitution III)
 * ========================================================================== */

/* IMU Read Task */
static StackType_t s_imu_task_stack[4096 / sizeof(StackType_t)];
static StaticTask_t s_imu_task_buffer;

/* Attitude Compute Task (larger for EKF) */
static StackType_t s_attitude_task_stack[8192 / sizeof(StackType_t)];
static StaticTask_t s_attitude_task_buffer;

/* Motor Control Task */
static StackType_t s_motor_task_stack[2048 / sizeof(StackType_t)];
static StaticTask_t s_motor_task_buffer;

/* GPS Parse Task */
static StackType_t s_gps_task_stack[4096 / sizeof(StackType_t)];
static StaticTask_t s_gps_task_buffer;

/* Telemetry TX Task */
static StackType_t s_telem_tx_task_stack[4096 / sizeof(StackType_t)];
static StaticTask_t s_telem_tx_task_buffer;

/* Telemetry RX Task */
static StackType_t s_telem_rx_task_stack[4096 / sizeof(StackType_t)];
static StaticTask_t s_telem_rx_task_buffer;

/* System Monitor Task */
static StackType_t s_monitor_task_stack[2048 / sizeof(StackType_t)];
static StaticTask_t s_monitor_task_buffer;

/* ==========================================================================
 * Static Buffers for Queues
 * ========================================================================== */

/* IMU to Attitude queue */
static uint8_t s_imu_queue_storage[4 * sizeof(sensor_data_t)];
static StaticQueue_t s_imu_queue_buffer;

/* GPS to Attitude queue */
static uint8_t s_gps_att_queue_storage[2 * sizeof(gps_position_t)];
static StaticQueue_t s_gps_att_queue_buffer;

/* GPS to Controller queue */
static uint8_t s_gps_ctrl_queue_storage[2 * sizeof(gps_position_t)];
static StaticQueue_t s_gps_ctrl_queue_buffer;

/* Telemetry command queue */
static uint8_t s_telem_cmd_queue_storage[4 * sizeof(control_command_t)];
static StaticQueue_t s_telem_cmd_queue_buffer;

/* ==========================================================================
 * Static Buffers for Mutexes
 * ========================================================================== */

static StaticSemaphore_t s_attitude_mutex_buffer;
static StaticSemaphore_t s_status_mutex_buffer;
static StaticSemaphore_t s_telemetry_mutex_buffer;
static StaticSemaphore_t s_nvs_mutex_buffer;

/* ==========================================================================
 * Static Buffer for Event Group
 * ========================================================================== */

static StaticEventGroup_t s_event_group_buffer;

/* ==========================================================================
 * Global Handles
 * ========================================================================== */

fc_task_handles_t g_task_handles = {0};
fc_queue_handles_t g_queues = {0};
fc_mutex_handles_t g_mutexes = {0};
EventGroupHandle_t g_system_events = NULL;

/* Shared state */
attitude_state_t g_attitude_state = {0};
system_status_t g_system_status = {0};
home_point_t g_home_point = {0};

/* ==========================================================================
 * Public API - Static Task Creation
 * ========================================================================== */

TaskHandle_t fc_create_imu_task(TaskFunction_t func, const char *name,
                                 UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_imu_task_stack) / sizeof(StackType_t),
                             param, priority, s_imu_task_stack, &s_imu_task_buffer);
}

TaskHandle_t fc_create_attitude_task(TaskFunction_t func, const char *name,
                                      UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_attitude_task_stack) / sizeof(StackType_t),
                             param, priority, s_attitude_task_stack, &s_attitude_task_buffer);
}

TaskHandle_t fc_create_motor_task(TaskFunction_t func, const char *name,
                                   UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_motor_task_stack) / sizeof(StackType_t),
                             param, priority, s_motor_task_stack, &s_motor_task_buffer);
}

TaskHandle_t fc_create_gps_task(TaskFunction_t func, const char *name,
                                 UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_gps_task_stack) / sizeof(StackType_t),
                             param, priority, s_gps_task_stack, &s_gps_task_buffer);
}

TaskHandle_t fc_create_telem_tx_task(TaskFunction_t func, const char *name,
                                      UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_telem_tx_task_stack) / sizeof(StackType_t),
                             param, priority, s_telem_tx_task_stack, &s_telem_tx_task_buffer);
}

TaskHandle_t fc_create_telem_rx_task(TaskFunction_t func, const char *name,
                                      UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_telem_rx_task_stack) / sizeof(StackType_t),
                             param, priority, s_telem_rx_task_stack, &s_telem_rx_task_buffer);
}

TaskHandle_t fc_create_monitor_task(TaskFunction_t func, const char *name,
                                     UBaseType_t priority, void *param)
{
    return xTaskCreateStatic(func, name, sizeof(s_monitor_task_stack) / sizeof(StackType_t),
                             param, priority, s_monitor_task_stack, &s_monitor_task_buffer);
}

/* ==========================================================================
 * IPC Initialization
 * ========================================================================== */

esp_err_t fc_ipc_init(void)
{
    ESP_LOGI(TAG, "Initializing IPC mechanisms");

    /* Create queues (static) */
    g_queues.imu_to_attitude = xQueueCreateStatic(
        4, sizeof(sensor_data_t),
        s_imu_queue_storage, &s_imu_queue_buffer);

    g_queues.gps_to_attitude = xQueueCreateStatic(
        2, sizeof(gps_position_t),
        s_gps_att_queue_storage, &s_gps_att_queue_buffer);

    g_queues.gps_to_controller = xQueueCreateStatic(
        2, sizeof(gps_position_t),
        s_gps_ctrl_queue_storage, &s_gps_ctrl_queue_buffer);

    g_queues.telemetry_cmd = xQueueCreateStatic(
        4, sizeof(control_command_t),
        s_telem_cmd_queue_storage, &s_telem_cmd_queue_buffer);

    if (!g_queues.imu_to_attitude || !g_queues.gps_to_attitude ||
        !g_queues.gps_to_controller || !g_queues.telemetry_cmd) {
        ESP_LOGE(TAG, "Queue creation failed");
        return FC_ERR_QUEUE_CREATE;
    }

    /* Create mutexes (static) */
    g_mutexes.attitude_state = xSemaphoreCreateMutexStatic(&s_attitude_mutex_buffer);
    g_mutexes.system_status = xSemaphoreCreateMutexStatic(&s_status_mutex_buffer);
    g_mutexes.telemetry_data = xSemaphoreCreateMutexStatic(&s_telemetry_mutex_buffer);
    g_mutexes.nvs_access = xSemaphoreCreateMutexStatic(&s_nvs_mutex_buffer);

    if (!g_mutexes.attitude_state || !g_mutexes.system_status ||
        !g_mutexes.telemetry_data || !g_mutexes.nvs_access) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return FC_ERR_QUEUE_CREATE;
    }

    /* Create event group (static) */
    g_system_events = xEventGroupCreateStatic(&s_event_group_buffer);
    if (!g_system_events) {
        ESP_LOGE(TAG, "Event group creation failed");
        return FC_ERR_QUEUE_CREATE;
    }

    /* Initialize shared state */
    memset(&g_attitude_state, 0, sizeof(g_attitude_state));
    memset(&g_system_status, 0, sizeof(g_system_status));
    memset(&g_home_point, 0, sizeof(g_home_point));

    g_system_status.current_mode = MODE_INIT;

    ESP_LOGI(TAG, "IPC initialized: 4 queues, 4 mutexes, 1 event group");
    return ESP_OK;
}

void fc_ipc_deinit(void)
{
    /* Note: Static objects don't need to be deleted,
       but we clear handles for safety */
    memset(&g_queues, 0, sizeof(g_queues));
    memset(&g_mutexes, 0, sizeof(g_mutexes));
    g_system_events = NULL;

    ESP_LOGI(TAG, "IPC deinitialized");
}

/* ==========================================================================
 * Thread-safe Accessors
 * ========================================================================== */

esp_err_t fc_get_attitude_state(attitude_state_t *state, uint32_t timeout_ms)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.attitude_state, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *state = g_attitude_state;
        xSemaphoreGive(g_mutexes.attitude_state);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t fc_set_attitude_state(const attitude_state_t *state, uint32_t timeout_ms)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.attitude_state, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        g_attitude_state = *state;
        xSemaphoreGive(g_mutexes.attitude_state);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t fc_get_system_status(system_status_t *status, uint32_t timeout_ms)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.system_status, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *status = g_system_status;
        xSemaphoreGive(g_mutexes.system_status);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t fc_set_system_status(const system_status_t *status, uint32_t timeout_ms)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutexes.system_status, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        g_system_status = *status;
        xSemaphoreGive(g_mutexes.system_status);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

/* ==========================================================================
 * Memory Statistics
 * ========================================================================== */

void fc_log_memory_stats(void)
{
    ESP_LOGI(TAG, "Heap free: %lu bytes (internal), %lu bytes (total)",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    ESP_LOGI(TAG, "Heap min free: %lu bytes",
             (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
}
