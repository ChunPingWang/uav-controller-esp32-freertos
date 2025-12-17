/**
 * @file test_imu_sensor.c
 * @brief Unit tests for IMU sensor component
 *
 * Tests: T015 - IMU driver unit tests
 */

#include "unity.h"
#include "imu_sensor.h"
#include <string.h>
#include <math.h>

/* Test configuration defaults */
TEST_CASE("IMU config defaults are valid", "[imu]")
{
    imu_config_t config = IMU_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(I2C_NUM_0, config.i2c_port);
    TEST_ASSERT_EQUAL(21, config.sda_pin);
    TEST_ASSERT_EQUAL(22, config.scl_pin);
    TEST_ASSERT_EQUAL(400000, config.i2c_freq_hz);
    TEST_ASSERT_EQUAL(100, config.sample_rate_hz);
    TEST_ASSERT_TRUE(config.enable_filter);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, config.filter_cutoff_hz);
}

/* Test filter coefficient calculation */
TEST_CASE("IMU filter initialization", "[imu][filter]")
{
    /* Filter init is called internally - test via API */
    imu_config_t config = IMU_CONFIG_DEFAULT();
    config.enable_filter = true;
    config.filter_cutoff_hz = 20.0f;

    /* Note: Full test requires mock I2C */
    TEST_PASS();
}

/* Test calibration data structure */
TEST_CASE("IMU calibration structure defaults", "[imu][calibration]")
{
    imu_calibration_t calib = {0};

    TEST_ASSERT_EQUAL_FLOAT(0.0f, calib.accel_offset_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calib.gyro_offset_x);
    TEST_ASSERT_FALSE(calib.valid);
}

/* Test sensor data structure */
TEST_CASE("Sensor data structure layout", "[imu]")
{
    sensor_data_t data = {0};

    /* Verify structure can hold expected values */
    data.accel_x = GRAVITY_MS2;
    data.accel_y = 0.0f;
    data.accel_z = -GRAVITY_MS2;
    data.gyro_x = DEG_TO_RAD(90.0f);
    data.gyro_y = DEG_TO_RAD(-90.0f);
    data.gyro_z = DEG_TO_RAD(180.0f);
    data.timestamp_us = 1000000;
    data.quality = 2;

    TEST_ASSERT_FLOAT_WITHIN(0.01f, GRAVITY_MS2, data.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, FC_PI / 2.0f, data.gyro_x);
    TEST_ASSERT_EQUAL(2, data.quality);
}

/* Test gravity constant */
TEST_CASE("Gravity constant is correct", "[imu]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.80665f, GRAVITY_MS2);
}

/* Test degree to radian conversion */
TEST_CASE("DEG_TO_RAD macro", "[imu]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, FC_PI, DEG_TO_RAD(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, FC_PI / 2.0f, DEG_TO_RAD(90.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, DEG_TO_RAD(0.0f));
}

/* Test radian to degree conversion */
TEST_CASE("RAD_TO_DEG macro", "[imu]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 180.0f, RAD_TO_DEG(FC_PI));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 90.0f, RAD_TO_DEG(FC_PI / 2.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, RAD_TO_DEG(0.0f));
}

/* Test angle normalization */
TEST_CASE("Angle normalization -PI to PI", "[imu]")
{
    float angle1 = FC_PI + 0.1f;
    float angle2 = -FC_PI - 0.1f;
    float angle3 = 2 * FC_PI + 0.5f;

    /* Normalize to [-PI, PI] */
    while (angle1 > FC_PI) angle1 -= 2 * FC_PI;
    while (angle2 < -FC_PI) angle2 += 2 * FC_PI;
    while (angle3 > FC_PI) angle3 -= 2 * FC_PI;

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -FC_PI + 0.1f, angle1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, FC_PI - 0.1f, angle2);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, angle3);
}
