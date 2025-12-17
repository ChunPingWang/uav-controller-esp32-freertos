/**
 * @file app_config.h
 * @brief System-wide configuration constants for Fixed-Wing Controller
 *
 * This file contains all configurable parameters for the flight controller.
 * Values are derived from spec.md requirements and plan.md architecture.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Version Information
 * ========================================================================== */

#define APP_VERSION_MAJOR   0
#define APP_VERSION_MINOR   1
#define APP_VERSION_PATCH   0
#define APP_VERSION_STRING  "0.1.0"

/* ==========================================================================
 * Task Priorities (Constitution II)
 * Hard real-time: configMAX_PRIORITIES-1 to configMAX_PRIORITIES-3
 * Background: tskIDLE_PRIORITY+1 to tskIDLE_PRIORITY+3
 * ========================================================================== */

#define TASK_PRIORITY_IMU_READ          (configMAX_PRIORITIES - 1)
#define TASK_PRIORITY_ATTITUDE_COMPUTE  (configMAX_PRIORITIES - 2)
#define TASK_PRIORITY_MOTOR_CONTROL     (configMAX_PRIORITIES - 2)
#define TASK_PRIORITY_GPS_PARSE         (tskIDLE_PRIORITY + 3)
#define TASK_PRIORITY_TELEMETRY_TX      (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TELEMETRY_RX      (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_SYSTEM_MONITOR    (tskIDLE_PRIORITY + 1)

/* ==========================================================================
 * Task Stack Sizes (bytes) - Constitution II & III
 * ========================================================================== */

#define TASK_STACK_IMU_READ             (4096)
#define TASK_STACK_ATTITUDE_COMPUTE     (8192)  /* Large for EKF matrices */
#define TASK_STACK_MOTOR_CONTROL        (2048)
#define TASK_STACK_GPS_PARSE            (4096)
#define TASK_STACK_TELEMETRY_TX         (4096)
#define TASK_STACK_TELEMETRY_RX         (4096)
#define TASK_STACK_SYSTEM_MONITOR       (2048)

/* ==========================================================================
 * Task Periods (milliseconds) - From spec.md requirements
 * ========================================================================== */

#define TASK_PERIOD_IMU_READ_MS         (10)    /* 100Hz - FR-001 */
#define TASK_PERIOD_ATTITUDE_MS         (10)    /* 100Hz - FR-006 */
#define TASK_PERIOD_MOTOR_MS            (10)    /* 100Hz */
#define TASK_PERIOD_GPS_MS              (200)   /* 5Hz - FR-013 */
#define TASK_PERIOD_TELEMETRY_TX_MS     (50)    /* 20Hz default */
#define TASK_PERIOD_SYSTEM_MONITOR_MS   (1000)  /* 1Hz */

/* ==========================================================================
 * Queue Sizes - Constitution IV
 * ========================================================================== */

#define QUEUE_SIZE_IMU_DATA             (4)
#define QUEUE_SIZE_GPS_DATA             (2)
#define QUEUE_SIZE_CONTROL_CMD          (4)
#define QUEUE_SIZE_TELEMETRY            (8)

/* ==========================================================================
 * Timing Requirements (microseconds) - From spec.md
 * ========================================================================== */

#define MAX_ISR_TIME_US                 (100)   /* Constitution V */
#define MAX_END_TO_END_LATENCY_MS       (15)    /* SC-002 */
#define MAX_MOTOR_RESPONSE_MS           (20)    /* FR-009 */
#define MAX_TELEMETRY_LATENCY_MS        (100)   /* SC-005 */
#define MAX_SELFTEST_TIME_MS            (5000)  /* FR-023 */

/* ==========================================================================
 * Communication Timeouts - From clarifications
 * ========================================================================== */

#define FAILSAFE_TIMEOUT_MS             (3000)  /* 3 seconds - FR-019/FR-024 */
#define GPS_TIMEOUT_MS                  (5000)  /* GPS signal loss threshold */

/* ==========================================================================
 * Accuracy Requirements - From spec.md
 * ========================================================================== */

#define ATTITUDE_ACCURACY_DEG           (2.0f)  /* SC-003: <2 degrees */
#define GPS_ACCURACY_M                  (5.0f)  /* SC-004: <5 meters CEP */

/* ==========================================================================
 * Motor Control Parameters
 * ========================================================================== */

#define MOTOR_THROTTLE_MIN              (0)
#define MOTOR_THROTTLE_MAX              (100)
#define MOTOR_PWM_FREQ_HZ               (50)    /* Standard ESC frequency */
#define MOTOR_PWM_MIN_US                (1000)  /* 1ms pulse */
#define MOTOR_PWM_MAX_US                (2000)  /* 2ms pulse */

/* ==========================================================================
 * EKF Parameters - From research.md
 * ========================================================================== */

#define EKF_STATE_DIM                   (6)     /* roll, pitch, yaw, bias_x/y/z */
#define EKF_UPDATE_RATE_HZ              (100)
#define EKF_GPS_MIN_SPEED_MS            (5.0f)  /* Min speed for GPS heading */

/* ==========================================================================
 * Hardware Pin Configuration (to be customized per board)
 * ========================================================================== */

/* I2C for IMU */
#define IMU_I2C_PORT                    I2C_NUM_0
#define IMU_I2C_SDA_PIN                 (21)
#define IMU_I2C_SCL_PIN                 (22)
#define IMU_I2C_FREQ_HZ                 (400000)

/* UART for GPS */
#define GPS_UART_PORT                   UART_NUM_1
#define GPS_UART_TX_PIN                 (17)
#define GPS_UART_RX_PIN                 (16)
#define GPS_UART_BAUD                   (9600)

/* UART for XBee */
#define XBEE_UART_PORT                  UART_NUM_2
#define XBEE_UART_TX_PIN                (4)
#define XBEE_UART_RX_PIN                (5)
#define XBEE_UART_BAUD                  (115200)

/* PWM for Motor/ESC */
#define MOTOR_PWM_GPIO                  (25)
#define MOTOR_PWM_CHANNEL               LEDC_CHANNEL_0

/* Battery ADC */
#define BATTERY_ADC_CHANNEL             ADC1_CHANNEL_6
#define BATTERY_LOW_THRESHOLD_MV        (3300)  /* 3.3V per cell */

/* ==========================================================================
 * NVS Namespace Keys
 * ========================================================================== */

#define NVS_NAMESPACE                   "fc_config"
#define NVS_KEY_IMU_CALIB               "imu_calib"
#define NVS_KEY_HOME_POINT              "home_point"
#define NVS_KEY_TELEMETRY_RATE          "telem_rate"

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
