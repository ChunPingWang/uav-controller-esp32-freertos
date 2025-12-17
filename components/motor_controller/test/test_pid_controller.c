/**
 * @file test_pid_controller.c
 * @brief Unit tests for PID controller
 *
 * Tests: T032 - PID controller unit tests
 */

#include "unity.h"
#include "motor_controller.h"
#include <math.h>

/* ==========================================================================
 * Initialization Tests
 * ========================================================================== */

TEST_CASE("PID init sets gains correctly", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 1.5f,
        .ki = 0.2f,
        .kd = 0.1f,
        .i_max = 50.0f,
        .output_max = 30.0f
    };

    pid_controller_init(&state, &gains);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, state.gains.kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, state.gains.ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, state.gains.kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 50.0f, state.gains.i_max);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 30.0f, state.gains.output_max);
}

TEST_CASE("PID init clears state", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = PID_GAINS_ROLL_DEFAULT();

    /* Pre-fill with garbage */
    state.integral = 999.0f;
    state.prev_error = 888.0f;
    state.output = 777.0f;

    pid_controller_init(&state, &gains);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, state.integral);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, state.prev_error);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, state.output);
}

/* ==========================================================================
 * P Term Tests
 * ========================================================================== */

TEST_CASE("PID proportional response", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 2.0f,
        .ki = 0.0f,  /* Disable I */
        .kd = 0.0f,  /* Disable D */
        .i_max = 100.0f,
        .output_max = 100.0f
    };

    pid_controller_init(&state, &gains);

    float error = 10.0f;
    float dt = 0.01f;

    float output = pid_controller_update(&state, error, dt);

    /* Output should be Kp * error = 2.0 * 10 = 20 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, output);
}

TEST_CASE("PID negative error handling", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 100.0f,
        .output_max = 100.0f
    };

    pid_controller_init(&state, &gains);

    float output = pid_controller_update(&state, -15.0f, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, -15.0f, output);
}

/* ==========================================================================
 * I Term Tests
 * ========================================================================== */

TEST_CASE("PID integral accumulation", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 0.0f,  /* Disable P */
        .ki = 1.0f,
        .kd = 0.0f,
        .i_max = 1000.0f,
        .output_max = 1000.0f
    };

    pid_controller_init(&state, &gains);

    float dt = 0.01f;
    float error = 10.0f;

    /* Run 10 iterations */
    float output = 0.0f;
    for (int i = 0; i < 10; i++) {
        output = pid_controller_update(&state, error, dt);
    }

    /* Integral = error * dt * 10 = 10 * 0.01 * 10 = 1.0 */
    /* Output = Ki * integral = 1.0 * 1.0 = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, output);
}

TEST_CASE("PID integral windup limit", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 0.0f,
        .ki = 1.0f,
        .kd = 0.0f,
        .i_max = 5.0f,  /* Low limit */
        .output_max = 100.0f
    };

    pid_controller_init(&state, &gains);

    float dt = 0.1f;
    float error = 100.0f;

    /* Run many iterations to saturate integral */
    float output = 0.0f;
    for (int i = 0; i < 100; i++) {
        output = pid_controller_update(&state, error, dt);
    }

    /* Output should be clamped by i_max */
    TEST_ASSERT_TRUE(output <= 5.0f);
    TEST_ASSERT_TRUE(state.integral <= 5.0f);
}

TEST_CASE("PID integral negative windup", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 0.0f,
        .ki = 1.0f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .output_max = 100.0f
    };

    pid_controller_init(&state, &gains);

    /* Large negative error */
    for (int i = 0; i < 100; i++) {
        pid_controller_update(&state, -100.0f, 0.1f);
    }

    TEST_ASSERT_TRUE(state.integral >= -5.0f);
}

/* ==========================================================================
 * D Term Tests
 * ========================================================================== */

TEST_CASE("PID derivative response", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 1.0f,
        .i_max = 100.0f,
        .output_max = 100.0f
    };

    pid_controller_init(&state, &gains);

    float dt = 0.01f;

    /* First update establishes baseline */
    pid_controller_update(&state, 0.0f, dt);

    /* Second update with changed error */
    float output = pid_controller_update(&state, 10.0f, dt);

    /* Derivative = (10 - 0) / 0.01 = 1000 */
    /* Output = Kd * derivative = 1.0 * 1000 = 1000, but clamped to 100 */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, output);
}

/* ==========================================================================
 * Output Clamping Tests
 * ========================================================================== */

TEST_CASE("PID output clamping positive", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 10.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 100.0f,
        .output_max = 25.0f  /* Low output limit */
    };

    pid_controller_init(&state, &gains);

    float output = pid_controller_update(&state, 100.0f, 0.01f);

    /* Would be 1000, but clamped to 25 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, output);
}

TEST_CASE("PID output clamping negative", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 10.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .i_max = 100.0f,
        .output_max = 25.0f
    };

    pid_controller_init(&state, &gains);

    float output = pid_controller_update(&state, -100.0f, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, output);
}

/* ==========================================================================
 * Reset Tests
 * ========================================================================== */

TEST_CASE("PID reset clears state", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = PID_GAINS_ROLL_DEFAULT();

    pid_controller_init(&state, &gains);

    /* Build up some state */
    for (int i = 0; i < 10; i++) {
        pid_controller_update(&state, 10.0f, 0.01f);
    }

    TEST_ASSERT_TRUE(fabsf(state.integral) > 0.0f);

    pid_controller_reset(&state);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, state.integral);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, state.prev_error);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, state.output);
}

/* ==========================================================================
 * Runtime Gain Update Tests
 * ========================================================================== */

TEST_CASE("PID set gains at runtime", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = PID_GAINS_ROLL_DEFAULT();

    pid_controller_init(&state, &gains);

    /* Build up integral */
    for (int i = 0; i < 10; i++) {
        pid_controller_update(&state, 5.0f, 0.01f);
    }

    float integral_before = state.integral;

    /* Change gains */
    pid_gains_t new_gains = {
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.2f,
        .i_max = 10.0f,
        .output_max = 50.0f
    };

    pid_controller_set_gains(&state, &new_gains);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, state.gains.kp);

    /* Integral should be clamped if it exceeds new limit */
    TEST_ASSERT_TRUE(state.integral <= 10.0f);
}

TEST_CASE("PID get gains", "[pid]")
{
    pid_state_t state;
    pid_gains_t gains = {
        .kp = 3.0f,
        .ki = 0.3f,
        .kd = 0.03f,
        .i_max = 33.0f,
        .output_max = 333.0f
    };

    pid_controller_init(&state, &gains);

    pid_gains_t retrieved;
    pid_controller_get_gains(&state, &retrieved);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, retrieved.kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, retrieved.ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.03f, retrieved.kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 33.0f, retrieved.i_max);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 333.0f, retrieved.output_max);
}

/* ==========================================================================
 * Default Gains Tests
 * ========================================================================== */

TEST_CASE("Default roll PID gains are valid", "[pid]")
{
    pid_gains_t gains = PID_GAINS_ROLL_DEFAULT();

    TEST_ASSERT_TRUE(gains.kp > 0.0f);
    TEST_ASSERT_TRUE(gains.ki >= 0.0f);
    TEST_ASSERT_TRUE(gains.kd >= 0.0f);
    TEST_ASSERT_TRUE(gains.i_max > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, AILERON_MAX_DEG, gains.output_max);
}

TEST_CASE("Default pitch PID gains are valid", "[pid]")
{
    pid_gains_t gains = PID_GAINS_PITCH_DEFAULT();

    TEST_ASSERT_TRUE(gains.kp > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, ELEVATOR_MAX_DEG, gains.output_max);
}

TEST_CASE("Default yaw PID gains are valid", "[pid]")
{
    pid_gains_t gains = PID_GAINS_YAW_DEFAULT();

    TEST_ASSERT_TRUE(gains.kp > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, RUDDER_MAX_DEG, gains.output_max);
}
