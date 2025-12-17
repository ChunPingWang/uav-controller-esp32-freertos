/**
 * @file fc_errors.h
 * @brief Error codes for Fixed-Wing Flight Controller
 *
 * Defines application-specific error codes following ESP-IDF conventions.
 * All errors use esp_err_t as per Constitution VI.
 */

#ifndef FC_ERRORS_H
#define FC_ERRORS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Error Code Base
 *
 * ESP-IDF reserves 0x0000-0x00FF for core errors.
 * Application errors start at 0x1000.
 * ========================================================================== */

#define FC_ERR_BASE             (0x1000)

/* ==========================================================================
 * General Errors (0x1000 - 0x100F)
 * ========================================================================== */

#define FC_OK                   ESP_OK
#define FC_FAIL                 ESP_FAIL

/** Module not initialized */
#define FC_ERR_NOT_INIT         (FC_ERR_BASE + 0x01)

/** Already initialized */
#define FC_ERR_ALREADY_INIT     (FC_ERR_BASE + 0x02)

/** Invalid parameter */
#define FC_ERR_INVALID_PARAM    (FC_ERR_BASE + 0x03)

/** Operation timeout */
#define FC_ERR_TIMEOUT          (FC_ERR_BASE + 0x04)

/** Resource busy */
#define FC_ERR_BUSY             (FC_ERR_BASE + 0x05)

/** Not supported */
#define FC_ERR_NOT_SUPPORTED    (FC_ERR_BASE + 0x06)

/** Data not available */
#define FC_ERR_NO_DATA          (FC_ERR_BASE + 0x07)

/** Buffer overflow */
#define FC_ERR_OVERFLOW         (FC_ERR_BASE + 0x08)

/* ==========================================================================
 * IMU Sensor Errors (0x1010 - 0x101F)
 * ========================================================================== */

#define FC_ERR_IMU_BASE         (FC_ERR_BASE + 0x10)

/** IMU communication failed */
#define FC_ERR_IMU_COMM         (FC_ERR_IMU_BASE + 0x01)

/** IMU not detected */
#define FC_ERR_IMU_NOT_FOUND    (FC_ERR_IMU_BASE + 0x02)

/** IMU self-test failed */
#define FC_ERR_IMU_SELFTEST     (FC_ERR_IMU_BASE + 0x03)

/** IMU data out of range */
#define FC_ERR_IMU_RANGE        (FC_ERR_IMU_BASE + 0x04)

/** IMU not calibrated */
#define FC_ERR_IMU_NOT_CALIB    (FC_ERR_IMU_BASE + 0x05)

/** IMU data quality poor */
#define FC_ERR_IMU_QUALITY      (FC_ERR_IMU_BASE + 0x06)

/* ==========================================================================
 * Attitude Estimator Errors (0x1020 - 0x102F)
 * ========================================================================== */

#define FC_ERR_ATT_BASE         (FC_ERR_BASE + 0x20)

/** EKF diverged */
#define FC_ERR_EKF_DIVERGED     (FC_ERR_ATT_BASE + 0x01)

/** Matrix inversion failed */
#define FC_ERR_MATRIX_SINGULAR  (FC_ERR_ATT_BASE + 0x02)

/** Covariance matrix not positive definite */
#define FC_ERR_COV_INVALID      (FC_ERR_ATT_BASE + 0x03)

/** State estimate invalid */
#define FC_ERR_STATE_INVALID    (FC_ERR_ATT_BASE + 0x04)

/* ==========================================================================
 * GPS Navigator Errors (0x1030 - 0x103F)
 * ========================================================================== */

#define FC_ERR_GPS_BASE         (FC_ERR_BASE + 0x30)

/** GPS communication failed */
#define FC_ERR_GPS_COMM         (FC_ERR_GPS_BASE + 0x01)

/** GPS no fix */
#define FC_ERR_GPS_NO_FIX       (FC_ERR_GPS_BASE + 0x02)

/** GPS low accuracy */
#define FC_ERR_GPS_LOW_ACC      (FC_ERR_GPS_BASE + 0x03)

/** NMEA parse error */
#define FC_ERR_GPS_PARSE        (FC_ERR_GPS_BASE + 0x04)

/** Home point not set */
#define FC_ERR_GPS_NO_HOME      (FC_ERR_GPS_BASE + 0x05)

/* ==========================================================================
 * Motor Controller Errors (0x1040 - 0x104F)
 * ========================================================================== */

#define FC_ERR_MOTOR_BASE       (FC_ERR_BASE + 0x40)

/** Motor not armed */
#define FC_ERR_MOTOR_NOT_ARMED  (FC_ERR_MOTOR_BASE + 0x01)

/** Motor command out of range */
#define FC_ERR_MOTOR_RANGE      (FC_ERR_MOTOR_BASE + 0x02)

/** PWM initialization failed */
#define FC_ERR_MOTOR_PWM        (FC_ERR_MOTOR_BASE + 0x03)

/** Emergency stop active */
#define FC_ERR_MOTOR_ESTOP      (FC_ERR_MOTOR_BASE + 0x04)

/* ==========================================================================
 * Telemetry Errors (0x1050 - 0x105F)
 * ========================================================================== */

#define FC_ERR_TELEM_BASE       (FC_ERR_BASE + 0x50)

/** Telemetry UART failed */
#define FC_ERR_TELEM_COMM       (FC_ERR_TELEM_BASE + 0x01)

/** Packet checksum error */
#define FC_ERR_TELEM_CHECKSUM   (FC_ERR_TELEM_BASE + 0x02)

/** Unknown command */
#define FC_ERR_TELEM_CMD        (FC_ERR_TELEM_BASE + 0x03)

/** Communication timeout (failsafe trigger) */
#define FC_ERR_TELEM_TIMEOUT    (FC_ERR_TELEM_BASE + 0x04)

/* ==========================================================================
 * System/Failsafe Errors (0x1060 - 0x106F)
 * ========================================================================== */

#define FC_ERR_SYS_BASE         (FC_ERR_BASE + 0x60)

/** Low battery */
#define FC_ERR_LOW_BATTERY      (FC_ERR_SYS_BASE + 0x01)

/** Self-test failed */
#define FC_ERR_SELFTEST         (FC_ERR_SYS_BASE + 0x02)

/** Task creation failed */
#define FC_ERR_TASK_CREATE      (FC_ERR_SYS_BASE + 0x03)

/** Queue creation failed */
#define FC_ERR_QUEUE_CREATE     (FC_ERR_SYS_BASE + 0x04)

/** NVS operation failed */
#define FC_ERR_NVS              (FC_ERR_SYS_BASE + 0x05)

/** Failsafe triggered */
#define FC_ERR_FAILSAFE         (FC_ERR_SYS_BASE + 0x06)

/* ==========================================================================
 * Error Checking Macros
 * ========================================================================== */

/**
 * @brief Check error and return if not OK
 */
#define FC_RETURN_ON_ERROR(x) do {                      \
    esp_err_t __err = (x);                              \
    if (__err != ESP_OK) {                              \
        return __err;                                   \
    }                                                   \
} while(0)

/**
 * @brief Check error, log, and return if not OK
 */
#define FC_RETURN_ON_ERROR_LOG(x, tag, msg) do {        \
    esp_err_t __err = (x);                              \
    if (__err != ESP_OK) {                              \
        ESP_LOGE(tag, "%s: %s", msg, esp_err_to_name(__err)); \
        return __err;                                   \
    }                                                   \
} while(0)

/**
 * @brief Check error and goto label if not OK
 */
#define FC_GOTO_ON_ERROR(x, label) do {                 \
    esp_err_t __err = (x);                              \
    if (__err != ESP_OK) {                              \
        goto label;                                     \
    }                                                   \
} while(0)

#ifdef __cplusplus
}
#endif

#endif /* FC_ERRORS_H */
