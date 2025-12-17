/**
 * @file fc_tasks.h
 * @brief FreeRTOS task handles and IPC structures
 *
 * Defines all task handles, queues, event groups, and mutexes
 * used for inter-task communication (Constitution IV).
 */

#ifndef FC_TASKS_H
#define FC_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Task Handles
 * ========================================================================== */

/**
 * @brief Global task handles
 *
 * Initialized in main.c during system startup.
 */
typedef struct {
    TaskHandle_t imu_read;          /**< vTask_IMU_Read */
    TaskHandle_t attitude_compute;  /**< vTask_Attitude_Compute */
    TaskHandle_t motor_control;     /**< vTask_Motor_Control */
    TaskHandle_t gps_parse;         /**< vTask_GPS_Parse */
    TaskHandle_t telemetry_tx;      /**< vTask_Telemetry_TX */
    TaskHandle_t telemetry_rx;      /**< vTask_Telemetry_RX */
    TaskHandle_t system_monitor;    /**< vTask_System_Monitor */
} fc_task_handles_t;

extern fc_task_handles_t g_task_handles;

/* ==========================================================================
 * Queue Handles
 * ========================================================================== */

/**
 * @brief Global queue handles
 */
typedef struct {
    QueueHandle_t imu_to_attitude;      /**< IMU data -> Attitude estimator */
    QueueHandle_t gps_to_attitude;      /**< GPS heading -> Attitude (EKF update) */
    QueueHandle_t gps_to_controller;    /**< GPS position -> Flight controller */
    QueueHandle_t telemetry_cmd;        /**< Control commands from ground */
} fc_queue_handles_t;

extern fc_queue_handles_t g_queues;

/* ==========================================================================
 * Mutex Handles
 * ========================================================================== */

/**
 * @brief Global mutex handles
 */
typedef struct {
    SemaphoreHandle_t attitude_state;   /**< Protects attitude state access */
    SemaphoreHandle_t system_status;    /**< Protects system status access */
    SemaphoreHandle_t telemetry_data;   /**< Protects telemetry aggregation */
    SemaphoreHandle_t nvs_access;       /**< Protects NVS operations */
} fc_mutex_handles_t;

extern fc_mutex_handles_t g_mutexes;

/* ==========================================================================
 * Event Groups
 * ========================================================================== */

/**
 * @brief System event bits
 */
#define EVT_IMU_DATA_READY      (1 << 0)    /**< New IMU data available */
#define EVT_GPS_FIX_ACQUIRED    (1 << 1)    /**< GPS has valid fix */
#define EVT_GPS_FIX_LOST        (1 << 2)    /**< GPS fix lost */
#define EVT_COMM_TIMEOUT        (1 << 3)    /**< Telemetry timeout (failsafe) */
#define EVT_LOW_BATTERY         (1 << 4)    /**< Battery voltage critical */
#define EVT_MOTOR_ARMED         (1 << 5)    /**< Motors armed */
#define EVT_MOTOR_DISARMED      (1 << 6)    /**< Motors disarmed */
#define EVT_FAILSAFE_ACTIVE     (1 << 7)    /**< Failsafe mode active */
#define EVT_SELFTEST_PASS       (1 << 8)    /**< Self-test passed */
#define EVT_SELFTEST_FAIL       (1 << 9)    /**< Self-test failed */
#define EVT_HOME_SET            (1 << 10)   /**< Home point recorded */
#define EVT_CALIBRATION_REQ     (1 << 11)   /**< Calibration requested */
#define EVT_CALIBRATION_DONE    (1 << 12)   /**< Calibration complete */

/**
 * @brief Global event group handle
 */
extern EventGroupHandle_t g_system_events;

/* ==========================================================================
 * Shared State (protected by mutexes)
 * ========================================================================== */

/**
 * @brief Shared attitude state
 *
 * Updated by attitude_compute task, read by motor_control and telemetry.
 * Access MUST be protected by g_mutexes.attitude_state.
 */
extern attitude_state_t g_attitude_state;

/**
 * @brief Shared system status
 *
 * Updated by system_monitor task, read by telemetry and failsafe.
 * Access MUST be protected by g_mutexes.system_status.
 */
extern system_status_t g_system_status;

/**
 * @brief Shared home point
 *
 * Set on first GPS lock, read by RTL logic.
 */
extern home_point_t g_home_point;

/* ==========================================================================
 * Task/IPC Initialization
 * ========================================================================== */

/**
 * @brief Initialize all IPC mechanisms (queues, mutexes, events)
 *
 * Must be called before creating any tasks.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fc_ipc_init(void);

/**
 * @brief Deinitialize IPC mechanisms
 *
 * Deletes all queues, mutexes, and event groups.
 */
void fc_ipc_deinit(void);

/* ==========================================================================
 * Helper Functions
 * ========================================================================== */

/**
 * @brief Safely get current attitude state
 *
 * Thread-safe accessor using mutex.
 *
 * @param[out] state Pointer to store attitude state
 * @param timeout_ms Maximum wait time for mutex
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if mutex not acquired
 */
esp_err_t fc_get_attitude_state(attitude_state_t *state, uint32_t timeout_ms);

/**
 * @brief Safely update attitude state
 *
 * Thread-safe setter using mutex.
 *
 * @param[in] state New attitude state
 * @param timeout_ms Maximum wait time for mutex
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if mutex not acquired
 */
esp_err_t fc_set_attitude_state(const attitude_state_t *state, uint32_t timeout_ms);

/**
 * @brief Safely get system status
 *
 * @param[out] status Pointer to store system status
 * @param timeout_ms Maximum wait time for mutex
 * @return ESP_OK on success
 */
esp_err_t fc_get_system_status(system_status_t *status, uint32_t timeout_ms);

/**
 * @brief Safely update system status
 *
 * @param[in] status New system status
 * @param timeout_ms Maximum wait time for mutex
 * @return ESP_OK on success
 */
esp_err_t fc_set_system_status(const system_status_t *status, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* FC_TASKS_H */
