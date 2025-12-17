/**
 * @file motor_controller.c
 * @brief Motor controller main implementation
 *
 * Coordinates PID controllers and servo outputs for attitude control.
 * Requirements: FR-005 through FR-010
 */

#include "motor_controller.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <string.h>
#include <math.h>

static const char *TAG = TAG_MTR;

/* Controller state */
static motor_config_t s_config;
static bool s_initialized = false;
static bool s_armed = false;

/* PID controllers */
static pid_state_t s_pid_roll;
static pid_state_t s_pid_pitch;
static pid_state_t s_pid_yaw;

/* Rate limiting state */
static control_output_t s_last_output = {0};
static uint64_t s_last_output_time_us = 0;

/* ==========================================================================
 * Internal Functions
 * ========================================================================== */

/**
 * @brief Apply rate limiting to control surface command (FR-010)
 */
static float apply_rate_limit(float current, float target, float max_rate, float dt)
{
    float delta = target - current;
    float max_delta = max_rate * dt;

    if (delta > max_delta) {
        return current + max_delta;
    } else if (delta < -max_delta) {
        return current - max_delta;
    }

    return target;
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

esp_err_t motor_controller_init(const motor_config_t *config)
{
    if (s_initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    ESP_LOGI(TAG, "Initializing motor controller");

    /* Use default if not provided */
    if (config == NULL) {
        motor_config_t default_cfg = MOTOR_CONFIG_DEFAULT();
        s_config = default_cfg;
    } else {
        s_config = *config;
    }

    /* Initialize servo driver */
    esp_err_t ret = servo_driver_init(&s_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Servo driver init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize PID controllers */
    pid_gains_t roll_gains = PID_GAINS_ROLL_DEFAULT();
    pid_gains_t pitch_gains = PID_GAINS_PITCH_DEFAULT();
    pid_gains_t yaw_gains = PID_GAINS_YAW_DEFAULT();

    pid_controller_init(&s_pid_roll, &roll_gains);
    pid_controller_init(&s_pid_pitch, &pitch_gains);
    pid_controller_init(&s_pid_yaw, &yaw_gains);

    /* Initialize rate limiting state */
    memset(&s_last_output, 0, sizeof(s_last_output));
    s_last_output_time_us = esp_timer_get_time();

    s_armed = false;
    s_initialized = true;

    ESP_LOGI(TAG, "Motor controller initialized (disarmed)");

    return ESP_OK;
}

esp_err_t motor_controller_deinit(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    /* Disarm first */
    motor_controller_set_armed(false);

    /* Deinit servo driver */
    servo_driver_deinit();

    s_initialized = false;
    ESP_LOGI(TAG, "Motor controller deinitialized");

    return ESP_OK;
}

bool motor_controller_is_ready(void)
{
    return s_initialized;
}

esp_err_t motor_controller_update(
    const attitude_state_t *attitude,
    const control_command_t *command,
    control_output_t *output,
    float dt)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (attitude == NULL || command == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f; /* Default to 100Hz */
    }

    /* Calculate errors */
    float roll_error = command->roll - attitude->roll_rad;
    float pitch_error = command->pitch - attitude->pitch_rad;
    float yaw_rate_error = command->yaw_rate - attitude->yaw_rate;

    /* Run PID controllers */
    float aileron_cmd = pid_controller_update(&s_pid_roll, roll_error, dt);
    float elevator_cmd = pid_controller_update(&s_pid_pitch, pitch_error, dt);
    float rudder_cmd = pid_controller_update(&s_pid_yaw, yaw_rate_error, dt);

    /* Apply rate limiting if enabled (FR-010) */
    if (s_config.enable_rate_limit) {
        aileron_cmd = apply_rate_limit(s_last_output.aileron_deg, aileron_cmd,
                                       s_config.rate_limit_deg_s, dt);
        elevator_cmd = apply_rate_limit(s_last_output.elevator_deg, elevator_cmd,
                                        s_config.rate_limit_deg_s, dt);
        rudder_cmd = apply_rate_limit(s_last_output.rudder_deg, rudder_cmd,
                                      s_config.rate_limit_deg_s, dt);
    }

    /* Store outputs */
    output->aileron_deg = aileron_cmd;
    output->elevator_deg = elevator_cmd;
    output->rudder_deg = rudder_cmd;
    output->throttle_pct = command->throttle;
    output->timestamp_us = esp_timer_get_time();

    /* Update rate limiting state */
    s_last_output = *output;
    s_last_output_time_us = output->timestamp_us;

    return ESP_OK;
}

esp_err_t motor_controller_apply_outputs(const control_output_t *output)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    /* Apply aileron (both left and right) */
    ret = servo_set_angle(SERVO_CHANNEL_AILERON_L,
                          output->aileron_deg * s_config.aileron_l_dir,
                          s_config.aileron_min_us, s_config.aileron_max_us);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Aileron L set failed");
    }

    ret = servo_set_angle(SERVO_CHANNEL_AILERON_R,
                          output->aileron_deg * s_config.aileron_r_dir,
                          s_config.aileron_min_us, s_config.aileron_max_us);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Aileron R set failed");
    }

    /* Apply elevator */
    ret = servo_set_angle(SERVO_CHANNEL_ELEVATOR,
                          output->elevator_deg * s_config.elevator_dir,
                          s_config.elevator_min_us, s_config.elevator_max_us);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Elevator set failed");
    }

    /* Apply rudder */
    ret = servo_set_angle(SERVO_CHANNEL_RUDDER,
                          output->rudder_deg * s_config.rudder_dir,
                          s_config.rudder_min_us, s_config.rudder_max_us);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Rudder set failed");
    }

    /* Apply throttle only if armed */
    if (s_armed) {
        ret = motor_set_throttle(output->throttle_pct);
    } else {
        ret = motor_set_throttle(0.0f);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Throttle set failed");
    }

    return ESP_OK;
}

esp_err_t motor_controller_set_safe(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    ESP_LOGI(TAG, "Setting safe outputs");

    /* Center all control surfaces */
    servo_set_angle(SERVO_CHANNEL_AILERON_L, 0.0f,
                    s_config.aileron_min_us, s_config.aileron_max_us);
    servo_set_angle(SERVO_CHANNEL_AILERON_R, 0.0f,
                    s_config.aileron_min_us, s_config.aileron_max_us);
    servo_set_angle(SERVO_CHANNEL_ELEVATOR, 0.0f,
                    s_config.elevator_min_us, s_config.elevator_max_us);
    servo_set_angle(SERVO_CHANNEL_RUDDER, 0.0f,
                    s_config.rudder_min_us, s_config.rudder_max_us);

    /* Cut throttle */
    motor_set_throttle(0.0f);

    /* Reset PID integrators */
    pid_controller_reset(&s_pid_roll);
    pid_controller_reset(&s_pid_pitch);
    pid_controller_reset(&s_pid_yaw);

    /* Clear rate limiting state */
    memset(&s_last_output, 0, sizeof(s_last_output));

    return ESP_OK;
}

esp_err_t motor_controller_set_armed(bool armed)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (armed && !s_armed) {
        ESP_LOGW(TAG, "Motors ARMED");
        /* Reset integrators on arm */
        pid_controller_reset(&s_pid_roll);
        pid_controller_reset(&s_pid_pitch);
        pid_controller_reset(&s_pid_yaw);
    } else if (!armed && s_armed) {
        ESP_LOGI(TAG, "Motors DISARMED");
        /* Set safe state on disarm */
        motor_controller_set_safe();
    }

    s_armed = armed;

    return ESP_OK;
}

bool motor_controller_is_armed(void)
{
    return s_armed;
}
