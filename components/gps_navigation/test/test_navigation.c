/**
 * @file test_navigation.c
 * @brief Unit tests for navigation math and NMEA parsing
 *
 * Tests: T044, T045 - Navigation unit tests
 */

#include "unity.h"
#include "gps_navigation.h"
#include <math.h>
#include <string.h>

/* ==========================================================================
 * Navigation Math Tests (FR-015)
 * ========================================================================== */

TEST_CASE("Distance calculation - same point", "[nav][math]")
{
    float dist = nav_calculate_distance(25.0, 121.5, 25.0, 121.5);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, dist);
}

TEST_CASE("Distance calculation - short distance", "[nav][math]")
{
    /* Approximately 111km per degree latitude at equator */
    /* 0.001 degrees should be about 111 meters */
    float dist = nav_calculate_distance(0.0, 0.0, 0.001, 0.0);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 111.0f, dist);
}

TEST_CASE("Distance calculation - known distance", "[nav][math]")
{
    /* Taipei to Kaohsiung: approximately 350km */
    float dist = nav_calculate_distance(25.0330, 121.5654,  /* Taipei */
                                         22.6273, 120.3014); /* Kaohsiung */
    TEST_ASSERT_FLOAT_WITHIN(20000.0f, 350000.0f, dist);
}

TEST_CASE("Bearing calculation - North", "[nav][math]")
{
    float bearing = nav_calculate_bearing(25.0, 121.5, 26.0, 121.5);
    /* Should be approximately 0 (North) */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, bearing);
}

TEST_CASE("Bearing calculation - East", "[nav][math]")
{
    float bearing = nav_calculate_bearing(25.0, 121.5, 25.0, 122.5);
    /* Should be approximately PI/2 (East) */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, FC_PI / 2.0f, bearing);
}

TEST_CASE("Bearing calculation - South", "[nav][math]")
{
    float bearing = nav_calculate_bearing(25.0, 121.5, 24.0, 121.5);
    /* Should be approximately PI (South) */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, FC_PI, bearing);
}

TEST_CASE("Bearing calculation - West", "[nav][math]")
{
    float bearing = nav_calculate_bearing(25.0, 121.5, 25.0, 120.5);
    /* Should be approximately 3*PI/2 (West) or -PI/2 normalized */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f * FC_PI / 2.0f, bearing);
}

/* ==========================================================================
 * NMEA Checksum Tests
 * ========================================================================== */

TEST_CASE("NMEA checksum validation - valid", "[nmea]")
{
    /* Valid GGA sentence */
    const char *sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47";
    TEST_ASSERT_TRUE(nmea_validate_checksum(sentence));
}

TEST_CASE("NMEA checksum validation - invalid", "[nmea]")
{
    /* Invalid checksum (changed last digit) */
    const char *sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*48";
    TEST_ASSERT_FALSE(nmea_validate_checksum(sentence));
}

TEST_CASE("NMEA checksum validation - no checksum", "[nmea]")
{
    const char *sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,";
    TEST_ASSERT_FALSE(nmea_validate_checksum(sentence));
}

TEST_CASE("NMEA checksum validation - no dollar sign", "[nmea]")
{
    const char *sentence = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47";
    TEST_ASSERT_FALSE(nmea_validate_checksum(sentence));
}

/* ==========================================================================
 * GPS Constants Tests
 * ========================================================================== */

TEST_CASE("Earth radius constant", "[nav]")
{
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 6371000.0f, EARTH_RADIUS_M);
}

TEST_CASE("GPS minimum satellites", "[nav]")
{
    TEST_ASSERT_EQUAL(6, GPS_MIN_SATELLITES);
}

TEST_CASE("Communication loss timeout", "[nav]")
{
    TEST_ASSERT_EQUAL(3000, COMM_LOSS_TIMEOUT_MS);
}

/* ==========================================================================
 * Waypoint Tests
 * ========================================================================== */

TEST_CASE("Waypoint structure defaults", "[nav][waypoint]")
{
    waypoint_t wp = {0};

    TEST_ASSERT_EQUAL_FLOAT(0.0, wp.latitude_deg);
    TEST_ASSERT_EQUAL_FLOAT(0.0, wp.longitude_deg);
    TEST_ASSERT_FALSE(wp.valid);
}

TEST_CASE("Max waypoints constant", "[nav][waypoint]")
{
    TEST_ASSERT_EQUAL(20, MAX_WAYPOINTS);
}

/* ==========================================================================
 * Navigation Mode Tests
 * ========================================================================== */

TEST_CASE("Navigation mode enum values", "[nav]")
{
    TEST_ASSERT_EQUAL(0, NAV_MODE_MANUAL);
    TEST_ASSERT_EQUAL(1, NAV_MODE_STABILIZE);
    TEST_ASSERT_EQUAL(2, NAV_MODE_LOITER);
    TEST_ASSERT_EQUAL(3, NAV_MODE_WAYPOINT);
    TEST_ASSERT_EQUAL(4, NAV_MODE_RTL);
}

/* ==========================================================================
 * GPS Position Structure Tests
 * ========================================================================== */

TEST_CASE("GPS position structure fields", "[nav][gps]")
{
    gps_position_t pos = {0};

    pos.latitude_deg = 25.033964;
    pos.longitude_deg = 121.564468;
    pos.altitude_m = 100.5f;
    pos.speed_ms = 15.0f;
    pos.heading_deg = 45.0f;
    pos.satellites = 10;
    pos.hdop = 1.2f;
    pos.valid = true;

    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 25.033964, pos.latitude_deg);
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 121.564468, pos.longitude_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.5f, pos.altitude_m);
    TEST_ASSERT_EQUAL(10, pos.satellites);
    TEST_ASSERT_TRUE(pos.valid);
}

/* ==========================================================================
 * Cross-track Error Tests
 * ========================================================================== */

TEST_CASE("Cross-track error - on track", "[nav][math]")
{
    gps_position_t current = {.latitude_deg = 25.0, .longitude_deg = 121.5};
    gps_position_t start = {.latitude_deg = 24.0, .longitude_deg = 121.5};
    gps_position_t end = {.latitude_deg = 26.0, .longitude_deg = 121.5};

    float xte = nav_calculate_crosstrack(&current, &start, &end);

    /* Should be near zero when on track */
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, xte);
}

TEST_CASE("Cross-track error - right of track", "[nav][math]")
{
    gps_position_t current = {.latitude_deg = 25.0, .longitude_deg = 121.6}; /* East of track */
    gps_position_t start = {.latitude_deg = 24.0, .longitude_deg = 121.5};
    gps_position_t end = {.latitude_deg = 26.0, .longitude_deg = 121.5};

    float xte = nav_calculate_crosstrack(&current, &start, &end);

    /* Should be positive (right of track) and significant */
    TEST_ASSERT_TRUE(xte > 1000.0f); /* More than 1km off track */
}

/* ==========================================================================
 * Default Configuration Tests
 * ========================================================================== */

TEST_CASE("GPS config defaults", "[nav][config]")
{
    gps_config_t config = GPS_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(9600, config.baud_rate);
    TEST_ASSERT_EQUAL(10, config.update_rate_hz);
}

TEST_CASE("Navigation thresholds", "[nav]")
{
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, NAV_WAYPOINT_RADIUS_M);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 50.0f, NAV_RTL_ALTITUDE_M);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, NAV_LOITER_RADIUS_M);
}
