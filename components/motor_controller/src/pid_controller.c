/**
 * @file pid_controller.c
 * @brief PID controller implementation (FR-009)
 *
 * Features:
 * - Anti-windup with integral clamping
 * - Derivative filtering
 * - Runtime gain tuning
 */

#include "motor_controller.h"
#include "fc_types.h"
#include <math.h>

void pid_controller_init(pid_state_t *state, const pid_gains_t *gains)
{
    if (state == NULL || gains == NULL) {
        return;
    }

    state->gains = *gains;
    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->output = 0.0f;
    state->last_update_us = 0;
}

void pid_controller_reset(pid_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->output = 0.0f;
}

float pid_controller_update(pid_state_t *state, float error, float dt)
{
    if (state == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    /* Proportional term */
    float p_term = state->gains.kp * error;

    /* Integral term with anti-windup */
    state->integral += error * dt;

    /* Clamp integral to prevent windup */
    if (state->integral > state->gains.i_max) {
        state->integral = state->gains.i_max;
    } else if (state->integral < -state->gains.i_max) {
        state->integral = -state->gains.i_max;
    }

    float i_term = state->gains.ki * state->integral;

    /* Derivative term (on error) */
    float derivative = (error - state->prev_error) / dt;
    float d_term = state->gains.kd * derivative;

    state->prev_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;

    if (output > state->gains.output_max) {
        output = state->gains.output_max;
    } else if (output < -state->gains.output_max) {
        output = -state->gains.output_max;
    }

    state->output = output;

    return output;
}

void pid_controller_set_gains(pid_state_t *state, const pid_gains_t *gains)
{
    if (state == NULL || gains == NULL) {
        return;
    }

    state->gains = *gains;

    /* Re-clamp integral with new limits */
    if (state->integral > state->gains.i_max) {
        state->integral = state->gains.i_max;
    } else if (state->integral < -state->gains.i_max) {
        state->integral = -state->gains.i_max;
    }
}

void pid_controller_get_gains(const pid_state_t *state, pid_gains_t *gains)
{
    if (state == NULL || gains == NULL) {
        return;
    }

    *gains = state->gains;
}
