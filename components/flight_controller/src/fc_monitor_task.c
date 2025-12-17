/**
 * @file fc_monitor_task.c
 * @brief System monitor task (vTask_System_Monitor)
 *
 * Runs at 1Hz to check subsystem health (Phase 7, US5).
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"

static const char *TAG = TAG_FC;

/* Stub - will be implemented in Phase 7 (T071) */

void vTask_System_Monitor(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "System monitor task started");

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* TODO: Implement in T071
         * - Check subsystem health (T067)
         * - Check battery (T068, T069)
         * - Check communication timeout (T063)
         * - Update system status
         * - Trigger failsafe if needed
         */

        /* Log stack usage periodically */
        FC_LOG_STACK(TAG, NULL);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}
