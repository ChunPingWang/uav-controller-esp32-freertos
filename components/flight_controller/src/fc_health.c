/**
 * @file fc_health.c
 * @brief Subsystem health check functions
 *
 * Implements health monitoring for Phase 7 (US5).
 * Monitors: IMU, GPS, Motor, Telemetry, Battery
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_FC;

/* Health check state */
typedef struct {
    bool initialized;
    subsystem_status_t imu_status;
    subsystem_status_t gps_status;
    subsystem_status_t motor_status;
    subsystem_status_t telemetry_status;
    subsystem_status_t battery_status;

    /* Timing */
    uint64_t last_imu_update_us;
    uint64_t last_gps_update_us;
    uint64_t last_motor_update_us;
    uint64_t last_telemetry_update_us;

    /* Thresholds (microseconds) */
    uint64_t imu_timeout_us;
    uint64_t gps_timeout_us;
    uint64_t motor_timeout_us;
    uint64_t telemetry_timeout_us;
} health_state_t;

static health_state_t s_health = {0};

/* External functions from other components */
extern bool imu_sensor_is_ready(void);
extern uint8_t imu_sensor_get_quality(void);
extern bool gps_driver_is_ready(void);
extern uint8_t gps_driver_get_satellites(void);
extern float gps_driver_get_hdop(void);
extern bool motor_controller_is_ready(void);
extern bool motor_controller_is_armed(void);
extern bool xbee_driver_is_ready(void);
extern bool telemetry_is_link_lost(void);
extern float fc_battery_get_voltage(void);
extern float fc_battery_get_percent(void);
extern bool fc_battery_is_low(void);

esp_err_t fc_health_init(void)
{
    if (s_health.initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    ESP_LOGI(TAG, "Initializing health monitor");

    memset(&s_health, 0, sizeof(s_health));

    /* Set timeouts */
    s_health.imu_timeout_us = 100000;      /* 100ms - IMU should update at 100Hz */
    s_health.gps_timeout_us = 2000000;     /* 2s - GPS updates at 10Hz */
    s_health.motor_timeout_us = 500000;    /* 500ms - Motor updates at 50Hz */
    s_health.telemetry_timeout_us = 3000000; /* 3s - Link loss timeout */

    /* Initialize timestamps */
    uint64_t now = esp_timer_get_time();
    s_health.last_imu_update_us = now;
    s_health.last_gps_update_us = now;
    s_health.last_motor_update_us = now;
    s_health.last_telemetry_update_us = now;

    /* Initial status */
    s_health.imu_status = SUBSYS_INIT;
    s_health.gps_status = SUBSYS_INIT;
    s_health.motor_status = SUBSYS_INIT;
    s_health.telemetry_status = SUBSYS_INIT;
    s_health.battery_status = SUBSYS_INIT;

    s_health.initialized = true;

    ESP_LOGI(TAG, "Health monitor initialized");

    return ESP_OK;
}

subsystem_status_t fc_health_check_imu(void)
{
    if (!s_health.initialized) {
        return SUBSYS_ERROR;
    }

    /* Check if IMU is ready */
    if (!imu_sensor_is_ready()) {
        s_health.imu_status = SUBSYS_ERROR;
        return s_health.imu_status;
    }

    /* Check data quality */
    uint8_t quality = imu_sensor_get_quality();
    if (quality == 0) {
        s_health.imu_status = SUBSYS_ERROR;
    } else if (quality == 1) {
        s_health.imu_status = SUBSYS_DEGRADED;
    } else {
        s_health.imu_status = SUBSYS_OK;
    }

    /* Check data freshness (event-based update) */
    uint64_t now = esp_timer_get_time();
    EventBits_t bits = xEventGroupGetBits(g_events.event_group);
    if (bits & EVT_IMU_DATA_READY) {
        s_health.last_imu_update_us = now;
        xEventGroupClearBits(g_events.event_group, EVT_IMU_DATA_READY);
    }

    if ((now - s_health.last_imu_update_us) > s_health.imu_timeout_us) {
        s_health.imu_status = SUBSYS_ERROR;
    }

    return s_health.imu_status;
}

subsystem_status_t fc_health_check_gps(void)
{
    if (!s_health.initialized) {
        return SUBSYS_ERROR;
    }

    /* Check if GPS is ready */
    if (!gps_driver_is_ready()) {
        /* GPS not ready but might be acquiring */
        uint8_t sats = gps_driver_get_satellites();
        if (sats > 0) {
            s_health.gps_status = SUBSYS_DEGRADED;
        } else {
            s_health.gps_status = SUBSYS_ERROR;
        }
        return s_health.gps_status;
    }

    /* Check fix quality */
    uint8_t satellites = gps_driver_get_satellites();
    float hdop = gps_driver_get_hdop();

    if (satellites >= 8 && hdop < 1.5f) {
        s_health.gps_status = SUBSYS_OK;
    } else if (satellites >= 6 && hdop < 2.5f) {
        s_health.gps_status = SUBSYS_OK;
    } else if (satellites >= 4) {
        s_health.gps_status = SUBSYS_DEGRADED;
    } else {
        s_health.gps_status = SUBSYS_ERROR;
    }

    /* Check data freshness */
    uint64_t now = esp_timer_get_time();
    EventBits_t bits = xEventGroupGetBits(g_events.event_group);
    if (bits & EVT_GPS_DATA_READY) {
        s_health.last_gps_update_us = now;
        xEventGroupClearBits(g_events.event_group, EVT_GPS_DATA_READY);
    }

    if ((now - s_health.last_gps_update_us) > s_health.gps_timeout_us) {
        s_health.gps_status = SUBSYS_ERROR;
    }

    return s_health.gps_status;
}

subsystem_status_t fc_health_check_motor(void)
{
    if (!s_health.initialized) {
        return SUBSYS_ERROR;
    }

    /* Check if motor controller is ready */
    if (!motor_controller_is_ready()) {
        s_health.motor_status = SUBSYS_ERROR;
        return s_health.motor_status;
    }

    /* Motor controller is functional */
    s_health.motor_status = SUBSYS_OK;

    /* If armed, check that updates are happening */
    if (motor_controller_is_armed()) {
        /* Would check update timing here if we had timestamps */
        s_health.motor_status = SUBSYS_OK;
    }

    return s_health.motor_status;
}

subsystem_status_t fc_health_check_telemetry(void)
{
    if (!s_health.initialized) {
        return SUBSYS_ERROR;
    }

    /* Check if XBee driver is ready */
    if (!xbee_driver_is_ready()) {
        s_health.telemetry_status = SUBSYS_ERROR;
        return s_health.telemetry_status;
    }

    /* Check link status */
    if (telemetry_is_link_lost()) {
        s_health.telemetry_status = SUBSYS_ERROR;
    } else {
        s_health.telemetry_status = SUBSYS_OK;
    }

    return s_health.telemetry_status;
}

subsystem_status_t fc_health_check_battery(void)
{
    if (!s_health.initialized) {
        return SUBSYS_ERROR;
    }

    float voltage = fc_battery_get_voltage();
    float percent = fc_battery_get_percent();

    /* Check for valid readings */
    if (voltage < 5.0f || voltage > 20.0f) {
        /* Invalid reading - sensor issue */
        s_health.battery_status = SUBSYS_ERROR;
        return s_health.battery_status;
    }

    /* Check battery level */
    if (fc_battery_is_low()) {
        s_health.battery_status = SUBSYS_DEGRADED;
    } else if (percent < 50.0f) {
        s_health.battery_status = SUBSYS_OK;
    } else {
        s_health.battery_status = SUBSYS_OK;
    }

    return s_health.battery_status;
}

esp_err_t fc_health_check_all(system_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check all subsystems */
    subsystem_status_t imu = fc_health_check_imu();
    subsystem_status_t gps = fc_health_check_gps();
    subsystem_status_t motor = fc_health_check_motor();
    subsystem_status_t telemetry = fc_health_check_telemetry();
    subsystem_status_t battery = fc_health_check_battery();

    /* Count errors and degraded */
    uint8_t error_count = 0;
    uint8_t degraded_count = 0;

    if (imu == SUBSYS_ERROR) error_count++;
    else if (imu == SUBSYS_DEGRADED) degraded_count++;

    if (gps == SUBSYS_ERROR) error_count++;
    else if (gps == SUBSYS_DEGRADED) degraded_count++;

    if (motor == SUBSYS_ERROR) error_count++;
    else if (motor == SUBSYS_DEGRADED) degraded_count++;

    if (telemetry == SUBSYS_ERROR) error_count++;
    else if (telemetry == SUBSYS_DEGRADED) degraded_count++;

    if (battery == SUBSYS_ERROR) error_count++;
    else if (battery == SUBSYS_DEGRADED) degraded_count++;

    /* Update status */
    status->imu_status = imu;
    status->gps_status = gps;
    status->motor_status = motor;
    status->telemetry_status = telemetry;
    status->error_count = error_count;

    /* Get battery info */
    status->battery_voltage = fc_battery_get_voltage();
    status->battery_percent = (uint8_t)fc_battery_get_percent();

    return ESP_OK;
}
