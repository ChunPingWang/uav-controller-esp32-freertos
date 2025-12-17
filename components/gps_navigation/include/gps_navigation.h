/**
 * @file gps_navigation.h
 * @brief GPS and navigation controller public API
 *
 * Requirements:
 * - FR-011: GPS @ 10Hz with <3m accuracy
 * - FR-012: Home point storage
 * - FR-013: Waypoint navigation
 * - FR-014: RTL (Return-to-Launch) - FR-025
 * - FR-015: Distance/bearing calculation
 * - FR-016: NMEA parsing
 * - FR-017: Loiter mode (FR-027)
 */

#ifndef GPS_NAVIGATION_H
#define GPS_NAVIGATION_H

#include "fc_types.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/* Earth parameters */
#define EARTH_RADIUS_M          6371000.0f  /* Mean Earth radius in meters */

/* GPS thresholds */
#define GPS_MIN_SATELLITES      6           /* Minimum for valid fix */
#define GPS_HDOP_MAX            2.0f        /* Maximum HDOP for good fix */
#define GPS_MIN_SPEED_MS        5.0f        /* Min speed for valid heading (FR-027) */

/* Navigation thresholds */
#define NAV_WAYPOINT_RADIUS_M   30.0f       /* Waypoint capture radius */
#define NAV_RTL_ALTITUDE_M      50.0f       /* Default RTL altitude */
#define NAV_LOITER_RADIUS_M     100.0f      /* Default loiter radius */

/* Communication loss threshold - FR-026 */
#define COMM_LOSS_TIMEOUT_MS    3000        /* 3 seconds before RTL trigger */

/* Maximum waypoints */
#define MAX_WAYPOINTS           20

/* NMEA buffer size */
#define NMEA_MAX_LENGTH         128

/* ==========================================================================
 * Data Types
 * ========================================================================== */

/**
 * @brief GPS fix type
 */
typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3,
    GPS_FIX_DGPS = 4,
} gps_fix_type_t;

/**
 * @brief Navigation mode
 */
typedef enum {
    NAV_MODE_MANUAL = 0,    /* Manual control */
    NAV_MODE_STABILIZE,     /* Attitude stabilization only */
    NAV_MODE_LOITER,        /* Hold position (circle) - FR-027 */
    NAV_MODE_WAYPOINT,      /* Navigate to waypoints - FR-013 */
    NAV_MODE_RTL,           /* Return to launch - FR-025 */
} nav_mode_t;

/**
 * @brief GPS configuration
 */
typedef struct {
    int uart_port;
    int tx_pin;
    int rx_pin;
    uint32_t baud_rate;
    uint8_t update_rate_hz;     /* GPS update rate (typically 1-10Hz) */
} gps_config_t;

/**
 * @brief Waypoint structure
 */
typedef struct {
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_ms;             /* Target speed at waypoint */
    uint8_t action;             /* Action at waypoint (0=flythrough) */
    bool valid;
} waypoint_t;

/**
 * @brief Navigation state
 */
typedef struct {
    nav_mode_t mode;
    uint8_t current_waypoint;
    uint8_t total_waypoints;

    /* Target position */
    double target_lat_deg;
    double target_lon_deg;
    float target_alt_m;

    /* Navigation outputs */
    float target_heading_rad;   /* Heading to target */
    float distance_to_target_m; /* Distance to current target */
    float cross_track_error_m;  /* Cross-track error (signed) */

    /* Loiter parameters */
    float loiter_radius_m;
    float loiter_direction;     /* 1 = clockwise, -1 = CCW */

    /* Status */
    bool navigation_valid;
    uint64_t timestamp_us;
} nav_state_t;

/**
 * @brief NMEA sentence type
 */
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GGA,       /* Global Positioning System Fix Data */
    NMEA_RMC,       /* Recommended Minimum Navigation Information */
    NMEA_VTG,       /* Track Made Good and Ground Speed */
    NMEA_GSA,       /* GPS DOP and Active Satellites */
    NMEA_GSV,       /* GPS Satellites in View */
} nmea_type_t;

/* Default GPS configuration */
#define GPS_CONFIG_DEFAULT() { \
    .uart_port = CONFIG_GPS_UART_PORT, \
    .tx_pin = CONFIG_GPS_TX_PIN, \
    .rx_pin = CONFIG_GPS_RX_PIN, \
    .baud_rate = 9600, \
    .update_rate_hz = 10 \
}

/* ==========================================================================
 * GPS Driver API
 * ========================================================================== */

/**
 * @brief Initialize GPS module
 * @param config GPS configuration
 * @return ESP_OK on success
 */
esp_err_t gps_driver_init(const gps_config_t *config);

/**
 * @brief Deinitialize GPS module
 * @return ESP_OK on success
 */
esp_err_t gps_driver_deinit(void);

/**
 * @brief Check if GPS is ready
 * @return true if initialized and receiving data
 */
bool gps_driver_is_ready(void);

/**
 * @brief Get current GPS position
 * @param position Output position structure
 * @return ESP_OK if valid fix available
 */
esp_err_t gps_driver_get_position(gps_position_t *position);

/**
 * @brief Get GPS fix quality
 * @return Fix type (none, 2D, 3D, DGPS)
 */
gps_fix_type_t gps_driver_get_fix_type(void);

/**
 * @brief Get number of satellites in view
 * @return Satellite count
 */
uint8_t gps_driver_get_satellites(void);

/**
 * @brief Get horizontal dilution of precision
 * @return HDOP value (lower is better)
 */
float gps_driver_get_hdop(void);

/**
 * @brief Process incoming GPS data (called by task)
 * @return ESP_OK if sentence processed
 */
esp_err_t gps_driver_process(void);

/* ==========================================================================
 * NMEA Parser API (FR-016)
 * ========================================================================== */

/**
 * @brief Parse NMEA sentence
 * @param sentence Null-terminated NMEA sentence
 * @param position Output position (updated if valid)
 * @return Type of sentence parsed
 */
nmea_type_t nmea_parse_sentence(const char *sentence, gps_position_t *position);

/**
 * @brief Validate NMEA checksum
 * @param sentence NMEA sentence with checksum
 * @return true if checksum valid
 */
bool nmea_validate_checksum(const char *sentence);

/* ==========================================================================
 * Navigation Controller API
 * ========================================================================== */

/**
 * @brief Initialize navigation controller
 * @return ESP_OK on success
 */
esp_err_t navigation_controller_init(void);

/**
 * @brief Deinitialize navigation controller
 * @return ESP_OK on success
 */
esp_err_t navigation_controller_deinit(void);

/**
 * @brief Set navigation mode
 * @param mode Desired navigation mode
 * @return ESP_OK on success
 */
esp_err_t navigation_set_mode(nav_mode_t mode);

/**
 * @brief Get current navigation mode
 * @return Current mode
 */
nav_mode_t navigation_get_mode(void);

/**
 * @brief Get navigation state
 * @param state Output state structure
 * @return ESP_OK on success
 */
esp_err_t navigation_get_state(nav_state_t *state);

/**
 * @brief Update navigation with current GPS position
 * @param gps_pos Current GPS position
 * @param attitude Current attitude
 * @param command Output control command
 * @return ESP_OK on success
 */
esp_err_t navigation_update(
    const gps_position_t *gps_pos,
    const attitude_state_t *attitude,
    control_command_t *command);

/* ==========================================================================
 * Home Point API (FR-012)
 * ========================================================================== */

/**
 * @brief Set home point (typically at first GPS lock)
 * @param position Position to set as home
 * @return ESP_OK on success
 */
esp_err_t navigation_set_home(const gps_position_t *position);

/**
 * @brief Get home point
 * @param home Output home point
 * @return ESP_OK if home set
 */
esp_err_t navigation_get_home(home_point_t *home);

/**
 * @brief Check if home point is set
 * @return true if home is set
 */
bool navigation_home_is_set(void);

/**
 * @brief Clear home point
 * @return ESP_OK on success
 */
esp_err_t navigation_clear_home(void);

/* ==========================================================================
 * Waypoint API (FR-013)
 * ========================================================================== */

/**
 * @brief Clear all waypoints
 * @return ESP_OK on success
 */
esp_err_t navigation_clear_waypoints(void);

/**
 * @brief Add waypoint
 * @param waypoint Waypoint to add
 * @return ESP_OK on success, error if list full
 */
esp_err_t navigation_add_waypoint(const waypoint_t *waypoint);

/**
 * @brief Get waypoint by index
 * @param index Waypoint index (0-based)
 * @param waypoint Output waypoint
 * @return ESP_OK if valid
 */
esp_err_t navigation_get_waypoint(uint8_t index, waypoint_t *waypoint);

/**
 * @brief Get waypoint count
 * @return Number of waypoints
 */
uint8_t navigation_get_waypoint_count(void);

/* ==========================================================================
 * RTL API (FR-025, FR-026)
 * ========================================================================== */

/**
 * @brief Trigger Return-to-Launch
 * @return ESP_OK on success
 */
esp_err_t navigation_trigger_rtl(void);

/**
 * @brief Check if RTL is active
 * @return true if in RTL mode
 */
bool navigation_is_rtl_active(void);

/* ==========================================================================
 * Loiter API (FR-027)
 * ========================================================================== */

/**
 * @brief Enter loiter mode at current position
 * @param radius_m Loiter radius in meters
 * @return ESP_OK on success
 */
esp_err_t navigation_enter_loiter(float radius_m);

/* ==========================================================================
 * Navigation Math (FR-015)
 * ========================================================================== */

/**
 * @brief Calculate distance between two GPS coordinates
 * @param lat1_deg Latitude 1 in degrees
 * @param lon1_deg Longitude 1 in degrees
 * @param lat2_deg Latitude 2 in degrees
 * @param lon2_deg Longitude 2 in degrees
 * @return Distance in meters
 */
float nav_calculate_distance(double lat1_deg, double lon1_deg,
                             double lat2_deg, double lon2_deg);

/**
 * @brief Calculate bearing between two GPS coordinates
 * @param lat1_deg Latitude 1 in degrees
 * @param lon1_deg Longitude 1 in degrees
 * @param lat2_deg Latitude 2 in degrees
 * @param lon2_deg Longitude 2 in degrees
 * @return Bearing in radians (0 = North, positive = clockwise)
 */
float nav_calculate_bearing(double lat1_deg, double lon1_deg,
                            double lat2_deg, double lon2_deg);

/**
 * @brief Calculate cross-track error from line
 * @param current Current position
 * @param start Line start position
 * @param end Line end position
 * @return Cross-track error in meters (positive = right of track)
 */
float nav_calculate_crosstrack(const gps_position_t *current,
                               const gps_position_t *start,
                               const gps_position_t *end);

/* ==========================================================================
 * FreeRTOS Tasks
 * ========================================================================== */

/**
 * @brief GPS read task function
 * @param pvParameters Task parameters (unused)
 */
void vTask_GPS_Read(void *pvParameters);

/**
 * @brief Navigation compute task function
 * @param pvParameters Task parameters (unused)
 */
void vTask_Navigation_Compute(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* GPS_NAVIGATION_H */
