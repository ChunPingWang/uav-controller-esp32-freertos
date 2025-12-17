/**
 * @file navigation_controller.c
 * @brief Navigation controller implementation
 *
 * Requirements:
 * - FR-012: Home point storage
 * - FR-013: Waypoint navigation
 * - FR-015: Distance/bearing calculation
 * - FR-025: RTL
 * - FR-026: Comm loss RTL trigger
 * - FR-027: Loiter mode
 */

#include "gps_navigation.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "fc_nvs.h"
#include "esp_timer.h"

#include <string.h>
#include <math.h>

static const char *TAG = TAG_NAV;

/* ==========================================================================
 * State
 * ========================================================================== */

static bool s_initialized = false;
static nav_mode_t s_mode = NAV_MODE_MANUAL;
static nav_state_t s_state = {0};

/* Home point */
static home_point_t s_home = {0};
static bool s_home_set = false;

/* Waypoints */
static waypoint_t s_waypoints[MAX_WAYPOINTS];
static uint8_t s_waypoint_count = 0;
static uint8_t s_current_waypoint = 0;

/* Previous waypoint (for cross-track calculation) */
static gps_position_t s_prev_waypoint_pos = {0};

/* Loiter state */
static gps_position_t s_loiter_center = {0};
static float s_loiter_radius = NAV_LOITER_RADIUS_M;
static float s_loiter_direction = 1.0f;

/* ==========================================================================
 * Navigation Math (FR-015)
 * ========================================================================== */

float nav_calculate_distance(double lat1_deg, double lon1_deg,
                             double lat2_deg, double lon2_deg)
{
    /* Haversine formula */
    double lat1_rad = lat1_deg * FC_PI / 180.0;
    double lat2_rad = lat2_deg * FC_PI / 180.0;
    double dlat = (lat2_deg - lat1_deg) * FC_PI / 180.0;
    double dlon = (lon2_deg - lon1_deg) * FC_PI / 180.0;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return (float)(EARTH_RADIUS_M * c);
}

float nav_calculate_bearing(double lat1_deg, double lon1_deg,
                            double lat2_deg, double lon2_deg)
{
    double lat1_rad = lat1_deg * FC_PI / 180.0;
    double lat2_rad = lat2_deg * FC_PI / 180.0;
    double dlon_rad = (lon2_deg - lon1_deg) * FC_PI / 180.0;

    double x = sin(dlon_rad) * cos(lat2_rad);
    double y = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad);

    float bearing = (float)atan2(x, y);

    /* Normalize to [0, 2*PI] */
    while (bearing < 0.0f) {
        bearing += 2.0f * FC_PI;
    }
    while (bearing >= 2.0f * FC_PI) {
        bearing -= 2.0f * FC_PI;
    }

    return bearing;
}

float nav_calculate_crosstrack(const gps_position_t *current,
                               const gps_position_t *start,
                               const gps_position_t *end)
{
    if (current == NULL || start == NULL || end == NULL) {
        return 0.0f;
    }

    /* Distance from start to current */
    float d13 = nav_calculate_distance(start->latitude_deg, start->longitude_deg,
                                        current->latitude_deg, current->longitude_deg);

    /* Bearing from start to current */
    float brng13 = nav_calculate_bearing(start->latitude_deg, start->longitude_deg,
                                          current->latitude_deg, current->longitude_deg);

    /* Bearing from start to end */
    float brng12 = nav_calculate_bearing(start->latitude_deg, start->longitude_deg,
                                          end->latitude_deg, end->longitude_deg);

    /* Cross-track distance */
    float xte = asinf(sinf(d13 / EARTH_RADIUS_M) * sinf(brng13 - brng12)) * EARTH_RADIUS_M;

    return xte;
}

/* ==========================================================================
 * Internal Functions
 * ========================================================================== */

/**
 * @brief Calculate navigation to a target point
 */
static void nav_calculate_to_target(const gps_position_t *current,
                                    double target_lat, double target_lon,
                                    nav_state_t *state)
{
    state->target_lat_deg = target_lat;
    state->target_lon_deg = target_lon;

    state->distance_to_target_m = nav_calculate_distance(
        current->latitude_deg, current->longitude_deg,
        target_lat, target_lon);

    state->target_heading_rad = nav_calculate_bearing(
        current->latitude_deg, current->longitude_deg,
        target_lat, target_lon);

    state->navigation_valid = true;
    state->timestamp_us = esp_timer_get_time();
}

/**
 * @brief Calculate loiter orbit navigation
 */
static void nav_calculate_loiter(const gps_position_t *current,
                                 const gps_position_t *center,
                                 float radius, float direction,
                                 nav_state_t *state)
{
    /* Calculate distance and bearing to center */
    float dist_to_center = nav_calculate_distance(
        current->latitude_deg, current->longitude_deg,
        center->latitude_deg, center->longitude_deg);

    float bearing_to_center = nav_calculate_bearing(
        current->latitude_deg, current->longitude_deg,
        center->latitude_deg, center->longitude_deg);

    /* Target heading is tangent to circle */
    /* For clockwise (direction=1): heading = bearing_to_center + 90° */
    /* For CCW (direction=-1): heading = bearing_to_center - 90° */
    float tangent_offset = direction * (FC_PI / 2.0f);

    /* Also adjust for distance from ideal radius */
    float radius_error = dist_to_center - radius;
    float correction = atanf(radius_error / radius) * 0.5f;

    state->target_heading_rad = bearing_to_center + tangent_offset - correction * direction;

    /* Normalize heading */
    while (state->target_heading_rad < 0.0f) {
        state->target_heading_rad += 2.0f * FC_PI;
    }
    while (state->target_heading_rad >= 2.0f * FC_PI) {
        state->target_heading_rad -= 2.0f * FC_PI;
    }

    state->distance_to_target_m = fabsf(radius_error);
    state->cross_track_error_m = radius_error;
    state->loiter_radius_m = radius;
    state->loiter_direction = direction;
    state->navigation_valid = true;
    state->timestamp_us = esp_timer_get_time();
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

esp_err_t navigation_controller_init(void)
{
    if (s_initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    ESP_LOGI(TAG, "Initializing navigation controller");

    /* Clear state */
    s_mode = NAV_MODE_MANUAL;
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_home, 0, sizeof(s_home));
    s_home_set = false;

    /* Clear waypoints */
    memset(s_waypoints, 0, sizeof(s_waypoints));
    s_waypoint_count = 0;
    s_current_waypoint = 0;

    /* Try to load home from NVS */
    if (fc_nvs_load_home(&s_home) == ESP_OK && s_home.valid) {
        s_home_set = true;
        ESP_LOGI(TAG, "Loaded home point: %.6f, %.6f",
                 s_home.latitude_deg, s_home.longitude_deg);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Navigation controller initialized");

    return ESP_OK;
}

esp_err_t navigation_controller_deinit(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Navigation controller deinitialized");

    return ESP_OK;
}

esp_err_t navigation_set_mode(nav_mode_t mode)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (mode == s_mode) {
        return ESP_OK;
    }

    /* Validate mode change */
    if (mode == NAV_MODE_RTL && !s_home_set) {
        ESP_LOGE(TAG, "Cannot enter RTL: home not set");
        return FC_ERR_NAV_NO_HOME;
    }

    if (mode == NAV_MODE_WAYPOINT && s_waypoint_count == 0) {
        ESP_LOGE(TAG, "Cannot enter waypoint mode: no waypoints");
        return FC_ERR_NAV_NO_WAYPOINTS;
    }

    ESP_LOGI(TAG, "Navigation mode: %d -> %d", s_mode, mode);
    s_mode = mode;
    s_state.mode = mode;

    /* Reset navigation state for new mode */
    if (mode == NAV_MODE_WAYPOINT) {
        s_current_waypoint = 0;
    }

    return ESP_OK;
}

nav_mode_t navigation_get_mode(void)
{
    return s_mode;
}

esp_err_t navigation_get_state(nav_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *state = s_state;
    return ESP_OK;
}

esp_err_t navigation_update(
    const gps_position_t *gps_pos,
    const attitude_state_t *attitude,
    control_command_t *command)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (gps_pos == NULL || attitude == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state.navigation_valid = false;

    switch (s_mode) {
        case NAV_MODE_MANUAL:
        case NAV_MODE_STABILIZE:
            /* No navigation processing */
            break;

        case NAV_MODE_LOITER:
            if (gps_pos->valid) {
                nav_calculate_loiter(gps_pos, &s_loiter_center,
                                    s_loiter_radius, s_loiter_direction, &s_state);

                /* Generate heading command */
                command->yaw_rate = (s_state.target_heading_rad - attitude->yaw_rad);
                /* Normalize to [-PI, PI] */
                while (command->yaw_rate > FC_PI) command->yaw_rate -= 2 * FC_PI;
                while (command->yaw_rate < -FC_PI) command->yaw_rate += 2 * FC_PI;
            }
            break;

        case NAV_MODE_WAYPOINT:
            if (gps_pos->valid && s_waypoint_count > 0) {
                waypoint_t *wp = &s_waypoints[s_current_waypoint];

                nav_calculate_to_target(gps_pos,
                                       wp->latitude_deg, wp->longitude_deg,
                                       &s_state);

                /* Check waypoint capture */
                if (s_state.distance_to_target_m < NAV_WAYPOINT_RADIUS_M) {
                    ESP_LOGI(TAG, "Waypoint %d captured", s_current_waypoint);

                    /* Move to next waypoint */
                    s_prev_waypoint_pos = *gps_pos;
                    s_current_waypoint++;

                    if (s_current_waypoint >= s_waypoint_count) {
                        ESP_LOGI(TAG, "All waypoints complete, entering loiter");
                        navigation_enter_loiter(NAV_LOITER_RADIUS_M);
                    }
                }

                s_state.current_waypoint = s_current_waypoint;
                s_state.total_waypoints = s_waypoint_count;

                /* Generate heading command */
                float heading_error = s_state.target_heading_rad - attitude->yaw_rad;
                while (heading_error > FC_PI) heading_error -= 2 * FC_PI;
                while (heading_error < -FC_PI) heading_error += 2 * FC_PI;

                /* Simple proportional heading control */
                command->yaw_rate = heading_error * 0.5f;

                /* Set target speed */
                command->throttle = wp->speed_ms > 0 ? 50.0f : 30.0f; /* Placeholder */
            }
            break;

        case NAV_MODE_RTL:
            if (gps_pos->valid && s_home_set) {
                nav_calculate_to_target(gps_pos,
                                       s_home.latitude_deg, s_home.longitude_deg,
                                       &s_state);

                /* Check if home reached */
                if (s_state.distance_to_target_m < NAV_WAYPOINT_RADIUS_M) {
                    ESP_LOGI(TAG, "Home reached, entering loiter");
                    s_loiter_center.latitude_deg = s_home.latitude_deg;
                    s_loiter_center.longitude_deg = s_home.longitude_deg;
                    navigation_set_mode(NAV_MODE_LOITER);
                }

                /* Generate heading command */
                float heading_error = s_state.target_heading_rad - attitude->yaw_rad;
                while (heading_error > FC_PI) heading_error -= 2 * FC_PI;
                while (heading_error < -FC_PI) heading_error += 2 * FC_PI;

                command->yaw_rate = heading_error * 0.5f;
            }
            break;
    }

    return ESP_OK;
}

/* ==========================================================================
 * Home Point API (FR-012)
 * ========================================================================== */

esp_err_t navigation_set_home(const gps_position_t *position)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (position == NULL || !position->valid) {
        return ESP_ERR_INVALID_ARG;
    }

    s_home.latitude_deg = position->latitude_deg;
    s_home.longitude_deg = position->longitude_deg;
    s_home.altitude_m = position->altitude_m;
    s_home.timestamp_us = esp_timer_get_time();
    s_home.valid = true;

    s_home_set = true;

    ESP_LOGI(TAG, "Home set: %.6f, %.6f, %.1fm",
             s_home.latitude_deg, s_home.longitude_deg, s_home.altitude_m);

    /* Save to NVS */
    fc_nvs_save_home(&s_home);

    return ESP_OK;
}

esp_err_t navigation_get_home(home_point_t *home)
{
    if (home == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_home_set) {
        return FC_ERR_NAV_NO_HOME;
    }

    *home = s_home;
    return ESP_OK;
}

bool navigation_home_is_set(void)
{
    return s_home_set;
}

esp_err_t navigation_clear_home(void)
{
    memset(&s_home, 0, sizeof(s_home));
    s_home_set = false;

    ESP_LOGI(TAG, "Home cleared");
    return ESP_OK;
}

/* ==========================================================================
 * Waypoint API (FR-013)
 * ========================================================================== */

esp_err_t navigation_clear_waypoints(void)
{
    memset(s_waypoints, 0, sizeof(s_waypoints));
    s_waypoint_count = 0;
    s_current_waypoint = 0;

    ESP_LOGI(TAG, "Waypoints cleared");
    return ESP_OK;
}

esp_err_t navigation_add_waypoint(const waypoint_t *waypoint)
{
    if (waypoint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_waypoint_count >= MAX_WAYPOINTS) {
        return FC_ERR_NAV_WP_FULL;
    }

    s_waypoints[s_waypoint_count] = *waypoint;
    s_waypoints[s_waypoint_count].valid = true;
    s_waypoint_count++;

    ESP_LOGI(TAG, "Waypoint %d added: %.6f, %.6f",
             s_waypoint_count - 1,
             waypoint->latitude_deg, waypoint->longitude_deg);

    return ESP_OK;
}

esp_err_t navigation_get_waypoint(uint8_t index, waypoint_t *waypoint)
{
    if (waypoint == NULL || index >= s_waypoint_count) {
        return ESP_ERR_INVALID_ARG;
    }

    *waypoint = s_waypoints[index];
    return ESP_OK;
}

uint8_t navigation_get_waypoint_count(void)
{
    return s_waypoint_count;
}

/* ==========================================================================
 * RTL API (FR-025, FR-026)
 * ========================================================================== */

esp_err_t navigation_trigger_rtl(void)
{
    ESP_LOGW(TAG, "RTL triggered!");
    return navigation_set_mode(NAV_MODE_RTL);
}

bool navigation_is_rtl_active(void)
{
    return s_mode == NAV_MODE_RTL;
}

/* ==========================================================================
 * Loiter API (FR-027)
 * ========================================================================== */

esp_err_t navigation_enter_loiter(float radius_m)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    /* Use current position as loiter center (set by caller via GPS) */
    s_loiter_radius = radius_m > 0 ? radius_m : NAV_LOITER_RADIUS_M;
    s_loiter_direction = 1.0f; /* Clockwise */

    ESP_LOGI(TAG, "Entering loiter mode, radius=%.1fm", s_loiter_radius);

    return navigation_set_mode(NAV_MODE_LOITER);
}
