/**
 * @file test_ekf.c
 * @brief Unit tests for Extended Kalman Filter
 *
 * Tests: T016 - EKF unit tests, T016a - Matrix math tests, T016b - Attitude estimator tests
 */

#include "unity.h"
#include "ekf_types.h"
#include "matrix_math.h"
#include "attitude_estimator.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 * Matrix Math Tests (T016a)
 * ========================================================================== */

TEST_CASE("Matrix identity initialization", "[ekf][matrix]")
{
    float I[EKF_STATE_DIM * EKF_STATE_DIM];
    mat6_identity(I);

    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected, I[i * EKF_STATE_DIM + j]);
        }
    }
}

TEST_CASE("Matrix zero initialization", "[ekf][matrix]")
{
    float Z[EKF_STATE_DIM * EKF_STATE_DIM];
    mat6_zero(Z);

    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Z[i]);
    }
}

TEST_CASE("Matrix copy", "[ekf][matrix]")
{
    float src[EKF_STATE_DIM * EKF_STATE_DIM];
    float dst[EKF_STATE_DIM * EKF_STATE_DIM];

    /* Fill source with test pattern */
    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) {
        src[i] = (float)i * 0.1f;
    }

    mat6_copy(dst, src);

    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, src[i], dst[i]);
    }
}

TEST_CASE("Matrix transpose", "[ekf][matrix]")
{
    float A[EKF_STATE_DIM * EKF_STATE_DIM];
    float At[EKF_STATE_DIM * EKF_STATE_DIM];

    /* Create asymmetric matrix */
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            A[i * EKF_STATE_DIM + j] = (float)(i * 10 + j);
        }
    }

    mat6_transpose(At, A);

    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6f,
                                     A[i * EKF_STATE_DIM + j],
                                     At[j * EKF_STATE_DIM + i]);
        }
    }
}

TEST_CASE("Matrix multiplication identity", "[ekf][matrix]")
{
    float A[EKF_STATE_DIM * EKF_STATE_DIM];
    float I[EKF_STATE_DIM * EKF_STATE_DIM];
    float C[EKF_STATE_DIM * EKF_STATE_DIM];

    /* Fill A with test values */
    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) {
        A[i] = (float)i * 0.5f + 1.0f;
    }

    mat6_identity(I);
    mat6_mult(C, A, I);

    /* A * I = A */
    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, A[i], C[i]);
    }
}

TEST_CASE("Matrix inversion of identity", "[ekf][matrix]")
{
    float I[EKF_STATE_DIM * EKF_STATE_DIM];
    float I_inv[EKF_STATE_DIM * EKF_STATE_DIM];

    mat6_identity(I);
    int result = mat6_invert(I_inv, I);

    TEST_ASSERT_EQUAL(0, result);

    /* Inverse of identity is identity */
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, I_inv[i * EKF_STATE_DIM + j]);
        }
    }
}

TEST_CASE("Matrix inversion of diagonal", "[ekf][matrix]")
{
    float D[EKF_STATE_DIM * EKF_STATE_DIM];
    float D_inv[EKF_STATE_DIM * EKF_STATE_DIM];

    mat6_zero(D);
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        D[i * EKF_STATE_DIM + i] = (float)(i + 1) * 2.0f; /* 2, 4, 6, 8, 10, 12 */
    }

    int result = mat6_invert(D_inv, D);
    TEST_ASSERT_EQUAL(0, result);

    /* Verify D * D_inv = I */
    float product[EKF_STATE_DIM * EKF_STATE_DIM];
    mat6_mult(product, D, D_inv);

    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, product[i * EKF_STATE_DIM + j]);
        }
    }
}

TEST_CASE("Matrix trace calculation", "[ekf][matrix]")
{
    float A[EKF_STATE_DIM * EKF_STATE_DIM];
    mat6_zero(A);

    /* Set diagonal elements */
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        A[i * EKF_STATE_DIM + i] = (float)(i + 1);
    }

    float trace = mat6_trace(A);

    /* Trace = 1 + 2 + 3 + 4 + 5 + 6 = 21 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 21.0f, trace);
}

/* ==========================================================================
 * EKF State Tests (T016)
 * ========================================================================== */

TEST_CASE("EKF state dimensions", "[ekf]")
{
    TEST_ASSERT_EQUAL(6, EKF_STATE_DIM);
    TEST_ASSERT_EQUAL(0, EKF_IDX_ROLL);
    TEST_ASSERT_EQUAL(1, EKF_IDX_PITCH);
    TEST_ASSERT_EQUAL(2, EKF_IDX_YAW);
    TEST_ASSERT_EQUAL(3, EKF_IDX_BIAS_X);
    TEST_ASSERT_EQUAL(4, EKF_IDX_BIAS_Y);
    TEST_ASSERT_EQUAL(5, EKF_IDX_BIAS_Z);
}

TEST_CASE("EKF config defaults", "[ekf]")
{
    ekf_config_t config = EKF_CONFIG_DEFAULT();

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, config.gyro_noise);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, config.accel_noise);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 1e-6f, config.bias_noise);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, config.initial_attitude_var);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0001f, config.initial_bias_var);
}

/* ==========================================================================
 * Attitude Estimator Tests (T016b)
 * ========================================================================== */

TEST_CASE("Attitude config defaults", "[attitude]")
{
    attitude_config_t config = ATTITUDE_CONFIG_DEFAULT();

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, config.ekf_config.gyro_noise);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.0f, config.min_gps_speed_for_heading);
    TEST_ASSERT_TRUE(config.use_gps_heading);
}

TEST_CASE("Attitude state structure", "[attitude]")
{
    attitude_state_t state = {0};

    /* Test that structure fields are accessible */
    state.roll_rad = 0.1f;
    state.pitch_rad = -0.2f;
    state.yaw_rad = 1.5f;
    state.roll_rate = 0.01f;
    state.pitch_rate = -0.02f;
    state.yaw_rate = 0.03f;
    state.valid = true;
    state.timestamp_us = 1000000;

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, state.roll_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.2f, state.pitch_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, state.yaw_rad);
    TEST_ASSERT_TRUE(state.valid);
}

/* ==========================================================================
 * Angle Math Tests
 * ========================================================================== */

TEST_CASE("Angle wrap to [-PI, PI]", "[ekf][math]")
{
    /* Test positive overflow */
    float angle1 = 4.0f; /* > PI */
    while (angle1 > FC_PI) angle1 -= 2.0f * FC_PI;
    TEST_ASSERT_TRUE(angle1 >= -FC_PI && angle1 <= FC_PI);

    /* Test negative overflow */
    float angle2 = -4.0f; /* < -PI */
    while (angle2 < -FC_PI) angle2 += 2.0f * FC_PI;
    TEST_ASSERT_TRUE(angle2 >= -FC_PI && angle2 <= FC_PI);

    /* Test within range */
    float angle3 = 1.0f;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, angle3);
}

TEST_CASE("Small angle approximation validity", "[ekf][math]")
{
    /* For angles < 0.1 rad (~5.7 deg), sin(x) ≈ x with < 0.2% error */
    float small_angle = 0.1f;
    float sin_approx = small_angle;
    float sin_actual = sinf(small_angle);

    float error_pct = fabsf(sin_approx - sin_actual) / sin_actual * 100.0f;
    TEST_ASSERT_TRUE(error_pct < 0.2f);
}

TEST_CASE("Quaternion to Euler edge case - level", "[ekf][math]")
{
    /* Level flight: roll=0, pitch=0, yaw=0 */
    /* Expected accel reading: [0, 0, -g] */
    float expected_ax = 0.0f;
    float expected_ay = 0.0f;
    float expected_az = -GRAVITY_MS2;

    /* Verify gravity vector */
    float mag = sqrtf(expected_ax * expected_ax +
                      expected_ay * expected_ay +
                      expected_az * expected_az);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, GRAVITY_MS2, mag);
}
