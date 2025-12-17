/**
 * @file fc_types.h
 * @brief Common data types for Fixed-Wing Flight Controller
 *
 * Defines all shared data structures used across components.
 * Based on Key Entities from spec.md.
 */

#ifndef FC_TYPES_H
#define FC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * SensorData - Six-axis IMU raw data (spec.md entity)
 * ========================================================================== */

/**
 * @brief Raw sensor data from IMU
 *
 * Contains accelerometer and gyroscope readings with timestamp and quality.
 * Updated at 100Hz (FR-001).
 */
typedef struct {
    /* Accelerometer (m/s^2) */
    float accel_x;
    float accel_y;
    float accel_z;

    /* Gyroscope (rad/s) */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* Metadata */
    uint64_t timestamp_us;      /**< Microseconds since boot */
    uint8_t quality;            /**< Data quality: 0=invalid, 1=degraded, 2=good */
} sensor_data_t;

/* ==========================================================================
 * AttitudeState - Aircraft attitude (spec.md entity)
 * ========================================================================== */

/**
 * @brief Aircraft attitude state
 *
 * Contains roll, pitch, yaw angles and rates.
 * Updated at 100Hz by EKF (FR-005, FR-006).
 * Accuracy requirement: <2 degrees (SC-003).
 */
typedef struct {
    /* Euler angles (radians) */
    float roll;                 /**< Roll angle (-PI to PI) */
    float pitch;                /**< Pitch angle (-PI/2 to PI/2) */
    float yaw;                  /**< Yaw/heading (0 to 2*PI) */

    /* Angular rates (rad/s) */
    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    /* Metadata */
    uint64_t timestamp_us;
    bool valid;                 /**< True if attitude estimate is reliable */
} attitude_state_t;

/* ==========================================================================
 * GpsPosition - GPS location data (spec.md entity)
 * ========================================================================== */

/**
 * @brief GPS fix quality enumeration
 */
typedef enum {
    GPS_FIX_INVALID = 0,        /**< No valid fix */
    GPS_FIX_2D = 1,             /**< 2D fix (no altitude) */
    GPS_FIX_3D = 2,             /**< 3D fix (full position) */
    GPS_FIX_DGPS = 3,           /**< Differential GPS */
    GPS_FIX_RTK = 4             /**< RTK fixed solution */
} gps_fix_t;

/**
 * @brief GPS position data
 *
 * Contains location, velocity, and heading from GPS module.
 * Updated at 5Hz (FR-013). Accuracy: <5m CEP (SC-004).
 */
typedef struct {
    /* Position */
    double latitude;            /**< Degrees (-90 to 90) */
    double longitude;           /**< Degrees (-180 to 180) */
    float altitude_m;           /**< Meters above sea level */

    /* Velocity */
    float speed_ms;             /**< Ground speed (m/s) */
    float heading_deg;          /**< Track over ground (0-360 degrees) */
    float climb_rate_ms;        /**< Vertical velocity (m/s, positive up) */

    /* Quality */
    gps_fix_t fix_type;
    uint8_t satellites;         /**< Number of satellites used */
    float hdop;                 /**< Horizontal dilution of precision */

    /* Metadata */
    uint64_t timestamp_us;
    bool valid;
} gps_position_t;

/* ==========================================================================
 * HomePoint - Return-to-Launch target (spec.md entity, FR-027)
 * ========================================================================== */

/**
 * @brief Home point for RTL
 *
 * Automatically recorded on first GPS lock as per clarification.
 */
typedef struct {
    double latitude;
    double longitude;
    float altitude_m;
    bool is_set;                /**< True if home point has been recorded */
    uint64_t set_timestamp_us;  /**< When home was set */
} home_point_t;

/* ==========================================================================
 * TelemetryPacket - Downlink data (spec.md entity)
 * ========================================================================== */

/**
 * @brief Telemetry packet structure
 *
 * Contains aggregated flight data for transmission to ground station.
 * Includes sequence number and checksum (FR-020).
 */
typedef struct {
    /* Header */
    uint16_t sequence;          /**< Packet sequence number */
    uint8_t packet_type;        /**< Packet type identifier */

    /* Payload - aggregated data */
    attitude_state_t attitude;
    gps_position_t gps;
    float battery_voltage;
    uint8_t system_status;
    uint8_t flight_mode;

    /* Trailer */
    uint16_t checksum;          /**< CRC16 checksum */
    uint64_t timestamp_us;
} telemetry_packet_t;

/* ==========================================================================
 * ControlCommand - Uplink commands (spec.md entity)
 * ========================================================================== */

/**
 * @brief Command types from ground station
 */
typedef enum {
    CMD_NONE = 0,
    CMD_SET_THROTTLE,           /**< Set motor throttle (0-100%) */
    CMD_SET_MODE,               /**< Change flight mode */
    CMD_ARM,                    /**< Arm motors */
    CMD_DISARM,                 /**< Disarm motors */
    CMD_RTL,                    /**< Return to launch */
    CMD_CALIBRATE,              /**< Start calibration */
    CMD_SET_HOME                /**< Set home point manually */
} command_type_t;

/**
 * @brief Control command from ground station
 */
typedef struct {
    command_type_t type;
    union {
        uint8_t throttle_percent;   /**< For CMD_SET_THROTTLE */
        uint8_t flight_mode;        /**< For CMD_SET_MODE */
        gps_position_t position;    /**< For CMD_SET_HOME */
    } data;
    uint16_t sequence;
    uint64_t timestamp_us;
} control_command_t;

/* ==========================================================================
 * SystemStatus - Health monitoring (spec.md entity)
 * ========================================================================== */

/**
 * @brief Subsystem status flags
 */
typedef enum {
    SUBSYS_OK = 0,
    SUBSYS_WARNING = 1,
    SUBSYS_ERROR = 2,
    SUBSYS_OFFLINE = 3
} subsystem_status_t;

/**
 * @brief Flight modes
 */
typedef enum {
    MODE_INIT = 0,              /**< Initializing */
    MODE_DISARMED,              /**< Motors disarmed, safe */
    MODE_MANUAL,                /**< Manual control */
    MODE_STABILIZE,             /**< Attitude stabilization */
    MODE_RTL,                   /**< Return to launch (FR-024) */
    MODE_LOITER,                /**< Hold position (FR-025) */
    MODE_FAILSAFE               /**< Failsafe active */
} flight_mode_t;

/**
 * @brief System status aggregation
 */
typedef struct {
    /* Subsystem health */
    subsystem_status_t imu_status;
    subsystem_status_t gps_status;
    subsystem_status_t telemetry_status;
    subsystem_status_t motor_status;

    /* Flight state */
    flight_mode_t current_mode;
    bool armed;
    bool failsafe_active;

    /* Resources */
    float battery_voltage;
    float battery_percent;
    uint32_t free_heap;

    /* Communication */
    uint32_t last_command_ms;   /**< Time since last command received */
    uint8_t packet_loss_percent;

    /* Metadata */
    uint64_t uptime_ms;
    uint64_t timestamp_us;
} system_status_t;

/* ==========================================================================
 * Calibration Data
 * ========================================================================== */

/**
 * @brief IMU calibration parameters
 *
 * Stored in NVS for persistence (FR-003).
 */
typedef struct {
    /* Accelerometer offsets */
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;

    /* Accelerometer scale factors */
    float accel_scale_x;
    float accel_scale_y;
    float accel_scale_z;

    /* Gyroscope offsets */
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;

    /* Metadata */
    uint32_t calibration_date;  /**< Unix timestamp */
    bool valid;
} imu_calibration_t;

/* ==========================================================================
 * Utility Macros
 * ========================================================================== */

/** Convert degrees to radians */
#define DEG_TO_RAD(deg)     ((deg) * 0.01745329251994329577f)

/** Convert radians to degrees */
#define RAD_TO_DEG(rad)     ((rad) * 57.29577951308232087680f)

/** Mathematical constants */
#define FC_PI               (3.14159265358979323846f)
#define FC_2PI              (6.28318530717958647693f)

/** Gravity constant (m/s^2) */
#define GRAVITY_MS2         (9.80665f)

#ifdef __cplusplus
}
#endif

#endif /* FC_TYPES_H */
