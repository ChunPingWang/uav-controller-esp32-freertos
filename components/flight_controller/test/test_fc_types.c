/**
 * @file test_fc_types.c
 * @brief Unit tests for fc_types.h data structures
 *
 * Tests structure sizes, field alignments, and utility macros.
 * Following Constitution VII (TDD).
 */

#include "unity.h"
#include "fc_types.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 * Test Group Setup
 * ========================================================================== */

void setUp(void)
{
    /* Called before each test */
}

void tearDown(void)
{
    /* Called after each test */
}

/* ==========================================================================
 * Structure Size Tests
 * ========================================================================== */

TEST_CASE("sensor_data_t size is reasonable", "[fc_types]")
{
    /* Should fit in a queue element */
    TEST_ASSERT_LESS_OR_EQUAL(64, sizeof(sensor_data_t));

    /* Contains expected fields */
    sensor_data_t data = {0};
    data.accel_x = 1.0f;
    data.gyro_z = 2.0f;
    data.timestamp_us = 12345678ULL;
    data.quality = 2;

    TEST_ASSERT_EQUAL_FLOAT(1.0f, data.accel_x);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, data.gyro_z);
    TEST_ASSERT_EQUAL_UINT64(12345678ULL, data.timestamp_us);
    TEST_ASSERT_EQUAL_UINT8(2, data.quality);
}

TEST_CASE("attitude_state_t size is reasonable", "[fc_types]")
{
    TEST_ASSERT_LESS_OR_EQUAL(64, sizeof(attitude_state_t));

    attitude_state_t att = {0};
    att.roll = 0.1f;
    att.pitch = 0.2f;
    att.yaw = 0.3f;
    att.valid = true;

    TEST_ASSERT_EQUAL_FLOAT(0.1f, att.roll);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, att.pitch);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, att.yaw);
    TEST_ASSERT_TRUE(att.valid);
}

TEST_CASE("gps_position_t size is reasonable", "[fc_types]")
{
    /* GPS struct is larger due to double precision */
    TEST_ASSERT_LESS_OR_EQUAL(80, sizeof(gps_position_t));

    gps_position_t gps = {0};
    gps.latitude = 25.033964;
    gps.longitude = 121.564468;
    gps.altitude_m = 100.0f;
    gps.fix_type = GPS_FIX_3D;
    gps.satellites = 12;

    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 25.033964, gps.latitude);
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 121.564468, gps.longitude);
    TEST_ASSERT_EQUAL(GPS_FIX_3D, gps.fix_type);
    TEST_ASSERT_EQUAL_UINT8(12, gps.satellites);
}

TEST_CASE("home_point_t stores coordinates correctly", "[fc_types]")
{
    home_point_t home = {
        .latitude = 25.033964,
        .longitude = 121.564468,
        .altitude_m = 50.0f,
        .is_set = true,
        .set_timestamp_us = 1000000ULL
    };

    TEST_ASSERT_TRUE(home.is_set);
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 25.033964, home.latitude);
}

TEST_CASE("control_command_t union works correctly", "[fc_types]")
{
    control_command_t cmd = {0};

    /* Test throttle command */
    cmd.type = CMD_SET_THROTTLE;
    cmd.data.throttle_percent = 75;
    TEST_ASSERT_EQUAL(CMD_SET_THROTTLE, cmd.type);
    TEST_ASSERT_EQUAL_UINT8(75, cmd.data.throttle_percent);

    /* Test mode command */
    cmd.type = CMD_SET_MODE;
    cmd.data.flight_mode = MODE_STABILIZE;
    TEST_ASSERT_EQUAL(CMD_SET_MODE, cmd.type);
    TEST_ASSERT_EQUAL_UINT8(MODE_STABILIZE, cmd.data.flight_mode);
}

TEST_CASE("system_status_t tracks all subsystems", "[fc_types]")
{
    system_status_t status = {0};

    status.imu_status = SUBSYS_OK;
    status.gps_status = SUBSYS_WARNING;
    status.telemetry_status = SUBSYS_OK;
    status.motor_status = SUBSYS_OK;
    status.current_mode = MODE_STABILIZE;
    status.armed = true;
    status.failsafe_active = false;
    status.battery_voltage = 11.5f;
    status.battery_percent = 80.0f;

    TEST_ASSERT_EQUAL(SUBSYS_OK, status.imu_status);
    TEST_ASSERT_EQUAL(SUBSYS_WARNING, status.gps_status);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, status.current_mode);
    TEST_ASSERT_TRUE(status.armed);
    TEST_ASSERT_FALSE(status.failsafe_active);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 11.5f, status.battery_voltage);
}

/* ==========================================================================
 * Enumeration Tests
 * ========================================================================== */

TEST_CASE("gps_fix_t enum values are correct", "[fc_types]")
{
    TEST_ASSERT_EQUAL(0, GPS_FIX_INVALID);
    TEST_ASSERT_EQUAL(1, GPS_FIX_2D);
    TEST_ASSERT_EQUAL(2, GPS_FIX_3D);
    TEST_ASSERT_EQUAL(3, GPS_FIX_DGPS);
    TEST_ASSERT_EQUAL(4, GPS_FIX_RTK);
}

TEST_CASE("flight_mode_t enum values are correct", "[fc_types]")
{
    TEST_ASSERT_EQUAL(0, MODE_INIT);
    TEST_ASSERT_EQUAL(1, MODE_DISARMED);
    TEST_ASSERT_EQUAL(2, MODE_MANUAL);
    TEST_ASSERT_EQUAL(3, MODE_STABILIZE);
    TEST_ASSERT_EQUAL(4, MODE_RTL);
    TEST_ASSERT_EQUAL(5, MODE_LOITER);
    TEST_ASSERT_EQUAL(6, MODE_FAILSAFE);
}

TEST_CASE("command_type_t enum values are correct", "[fc_types]")
{
    TEST_ASSERT_EQUAL(0, CMD_NONE);
    TEST_ASSERT_EQUAL(1, CMD_SET_THROTTLE);
    TEST_ASSERT_EQUAL(2, CMD_SET_MODE);
    TEST_ASSERT_EQUAL(3, CMD_ARM);
    TEST_ASSERT_EQUAL(4, CMD_DISARM);
    TEST_ASSERT_EQUAL(5, CMD_RTL);
}

/* ==========================================================================
 * Utility Macro Tests
 * ========================================================================== */

TEST_CASE("DEG_TO_RAD converts correctly", "[fc_types]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, DEG_TO_RAD(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, FC_PI / 2.0f, DEG_TO_RAD(90.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, FC_PI, DEG_TO_RAD(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, FC_2PI, DEG_TO_RAD(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -FC_PI / 2.0f, DEG_TO_RAD(-90.0f));
}

TEST_CASE("RAD_TO_DEG converts correctly", "[fc_types]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, RAD_TO_DEG(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, RAD_TO_DEG(FC_PI / 2.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, RAD_TO_DEG(FC_PI));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 360.0f, RAD_TO_DEG(FC_2PI));
}

TEST_CASE("DEG_TO_RAD and RAD_TO_DEG are inverse", "[fc_types]")
{
    float test_values[] = {0.0f, 45.0f, 90.0f, 180.0f, 270.0f, 360.0f, -45.0f};
    int count = sizeof(test_values) / sizeof(test_values[0]);

    for (int i = 0; i < count; i++) {
        float deg = test_values[i];
        float rad = DEG_TO_RAD(deg);
        float back = RAD_TO_DEG(rad);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, deg, back);
    }
}

TEST_CASE("FC_PI constant is accurate", "[fc_types]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.0000001f, 3.14159265f, FC_PI);
}

TEST_CASE("GRAVITY_MS2 constant is accurate", "[fc_types]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 9.80665f, GRAVITY_MS2);
}

/* ==========================================================================
 * Calibration Data Tests
 * ========================================================================== */

TEST_CASE("imu_calibration_t stores offsets and scales", "[fc_types]")
{
    imu_calibration_t calib = {
        .accel_offset_x = 0.01f,
        .accel_offset_y = -0.02f,
        .accel_offset_z = 0.03f,
        .accel_scale_x = 1.001f,
        .accel_scale_y = 0.999f,
        .accel_scale_z = 1.002f,
        .gyro_offset_x = 0.001f,
        .gyro_offset_y = -0.001f,
        .gyro_offset_z = 0.0005f,
        .calibration_date = 1702700000,
        .valid = true
    };

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, calib.accel_offset_x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.001f, calib.accel_scale_x);
    TEST_ASSERT_TRUE(calib.valid);
    TEST_ASSERT_EQUAL_UINT32(1702700000, calib.calibration_date);
}

/* ==========================================================================
 * Telemetry Packet Tests
 * ========================================================================== */

TEST_CASE("telemetry_packet_t contains all required fields", "[fc_types]")
{
    telemetry_packet_t pkt = {0};

    pkt.sequence = 1234;
    pkt.packet_type = 1;
    pkt.attitude.roll = 0.1f;
    pkt.gps.latitude = 25.0;
    pkt.battery_voltage = 11.1f;
    pkt.system_status = SUBSYS_OK;
    pkt.flight_mode = MODE_STABILIZE;
    pkt.checksum = 0xABCD;

    TEST_ASSERT_EQUAL_UINT16(1234, pkt.sequence);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pkt.attitude.roll);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 25.0, pkt.gps.latitude);
    TEST_ASSERT_EQUAL_UINT16(0xABCD, pkt.checksum);
}

/* ==========================================================================
 * Test Entry Point (for ESP-IDF test framework)
 * ========================================================================== */

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
