/**
 * @file motor_controller.h
 * @brief Motor and servo controller public API
 *
 * Requirements:
 * - FR-005: Throttle control (0-100%)
 * - FR-006: Aileron control (±30°)
 * - FR-007: Elevator control (±25°)
 * - FR-008: Rudder control (±20°)
 * - FR-009: PID tuning support
 * - FR-010: Control surface rate limiting
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

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

/* Servo PWM parameters (standard RC servo) */
#define SERVO_PWM_FREQ_HZ       50      /* 50Hz = 20ms period */
#define SERVO_PWM_MIN_US        1000    /* 1ms pulse = min position */
#define SERVO_PWM_MID_US        1500    /* 1.5ms pulse = center */
#define SERVO_PWM_MAX_US        2000    /* 2ms pulse = max position */

/* Control surface limits (degrees) - FR-006, FR-007, FR-008 */
#define AILERON_MAX_DEG         30.0f
#define ELEVATOR_MAX_DEG        25.0f
#define RUDDER_MAX_DEG          20.0f

/* Rate limits (degrees per second) - FR-010 */
#define SERVO_RATE_LIMIT_DEG_S  120.0f  /* Max servo movement rate */

/* Throttle limits - FR-005 */
#define THROTTLE_MIN_PCT        0.0f
#define THROTTLE_MAX_PCT        100.0f
#define THROTTLE_IDLE_PCT       5.0f    /* Minimum for motor spin */

/* ==========================================================================
 * Data Types
 * ========================================================================== */

/**
 * @brief Servo channel identifiers
 */
typedef enum {
    SERVO_CHANNEL_AILERON_L = 0,
    SERVO_CHANNEL_AILERON_R,
    SERVO_CHANNEL_ELEVATOR,
    SERVO_CHANNEL_RUDDER,
    SERVO_CHANNEL_THROTTLE,
    SERVO_CHANNEL_COUNT
} servo_channel_t;

/**
 * @brief Motor controller configuration
 */
typedef struct {
    /* GPIO pins */
    int8_t aileron_l_pin;
    int8_t aileron_r_pin;
    int8_t elevator_pin;
    int8_t rudder_pin;
    int8_t throttle_pin;

    /* Servo calibration (pulse width in us) */
    uint16_t aileron_min_us;
    uint16_t aileron_max_us;
    uint16_t elevator_min_us;
    uint16_t elevator_max_us;
    uint16_t rudder_min_us;
    uint16_t rudder_max_us;

    /* Servo directions (1 or -1) */
    int8_t aileron_l_dir;
    int8_t aileron_r_dir;
    int8_t elevator_dir;
    int8_t rudder_dir;

    /* Rate limiting */
    float rate_limit_deg_s;
    bool enable_rate_limit;
} motor_config_t;

/**
 * @brief PID controller gains
 */
typedef struct {
    float kp;           /* Proportional gain */
    float ki;           /* Integral gain */
    float kd;           /* Derivative gain */
    float i_max;        /* Integral windup limit */
    float output_max;   /* Maximum output magnitude */
} pid_gains_t;

/**
 * @brief PID controller state
 */
typedef struct {
    pid_gains_t gains;
    float integral;
    float prev_error;
    float output;
    uint64_t last_update_us;
} pid_state_t;

/**
 * @brief Control input structure
 */
typedef struct {
    float roll_cmd;     /* Target roll rate or angle (rad or rad/s) */
    float pitch_cmd;    /* Target pitch rate or angle */
    float yaw_cmd;      /* Target yaw rate */
    float throttle_cmd; /* Target throttle (0-100%) */
    bool armed;         /* Motors armed flag */
    uint64_t timestamp_us;
} control_input_t;

/**
 * @brief Control output structure
 */
typedef struct {
    float aileron_deg;      /* Aileron deflection (degrees) */
    float elevator_deg;     /* Elevator deflection (degrees) */
    float rudder_deg;       /* Rudder deflection (degrees) */
    float throttle_pct;     /* Throttle percentage (0-100) */
    uint64_t timestamp_us;
} control_output_t;

/* Default configuration */
#define MOTOR_CONFIG_DEFAULT() { \
    .aileron_l_pin = CONFIG_SERVO_AILERON_L_PIN, \
    .aileron_r_pin = CONFIG_SERVO_AILERON_R_PIN, \
    .elevator_pin = CONFIG_SERVO_ELEVATOR_PIN, \
    .rudder_pin = CONFIG_SERVO_RUDDER_PIN, \
    .throttle_pin = CONFIG_MOTOR_THROTTLE_PIN, \
    .aileron_min_us = SERVO_PWM_MIN_US, \
    .aileron_max_us = SERVO_PWM_MAX_US, \
    .elevator_min_us = SERVO_PWM_MIN_US, \
    .elevator_max_us = SERVO_PWM_MAX_US, \
    .rudder_min_us = SERVO_PWM_MIN_US, \
    .rudder_max_us = SERVO_PWM_MAX_US, \
    .aileron_l_dir = 1, \
    .aileron_r_dir = -1, \
    .elevator_dir = 1, \
    .rudder_dir = 1, \
    .rate_limit_deg_s = SERVO_RATE_LIMIT_DEG_S, \
    .enable_rate_limit = true \
}

/* Default PID gains */
#define PID_GAINS_ROLL_DEFAULT() { \
    .kp = 1.0f, \
    .ki = 0.1f, \
    .kd = 0.05f, \
    .i_max = 30.0f, \
    .output_max = AILERON_MAX_DEG \
}

#define PID_GAINS_PITCH_DEFAULT() { \
    .kp = 1.2f, \
    .ki = 0.15f, \
    .kd = 0.08f, \
    .i_max = 25.0f, \
    .output_max = ELEVATOR_MAX_DEG \
}

#define PID_GAINS_YAW_DEFAULT() { \
    .kp = 0.8f, \
    .ki = 0.05f, \
    .kd = 0.02f, \
    .i_max = 20.0f, \
    .output_max = RUDDER_MAX_DEG \
}

/* ==========================================================================
 * Motor Controller API
 * ========================================================================== */

/**
 * @brief Initialize motor controller
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t motor_controller_init(const motor_config_t *config);

/**
 * @brief Deinitialize motor controller
 * @return ESP_OK on success
 */
esp_err_t motor_controller_deinit(void);

/**
 * @brief Check if motor controller is ready
 * @return true if initialized and ready
 */
bool motor_controller_is_ready(void);

/**
 * @brief Process attitude error and generate control outputs
 * @param attitude Current attitude state
 * @param command Control command (setpoint)
 * @param output Generated control outputs
 * @param dt Time step in seconds
 * @return ESP_OK on success
 */
esp_err_t motor_controller_update(
    const attitude_state_t *attitude,
    const control_command_t *command,
    control_output_t *output,
    float dt);

/**
 * @brief Apply control outputs to servos and motor
 * @param output Control outputs to apply
 * @return ESP_OK on success
 */
esp_err_t motor_controller_apply_outputs(const control_output_t *output);

/**
 * @brief Set all controls to safe positions (disarmed state)
 * @return ESP_OK on success
 */
esp_err_t motor_controller_set_safe(void);

/**
 * @brief Arm/disarm motors
 * @param armed true to arm, false to disarm
 * @return ESP_OK on success
 */
esp_err_t motor_controller_set_armed(bool armed);

/**
 * @brief Check if motors are armed
 * @return true if armed
 */
bool motor_controller_is_armed(void);

/* ==========================================================================
 * PID Controller API (FR-009)
 * ========================================================================== */

/**
 * @brief Initialize PID controller state
 * @param state PID state to initialize
 * @param gains Initial gains
 */
void pid_controller_init(pid_state_t *state, const pid_gains_t *gains);

/**
 * @brief Reset PID controller state (clear integral)
 * @param state PID state to reset
 */
void pid_controller_reset(pid_state_t *state);

/**
 * @brief Update PID controller
 * @param state PID state
 * @param error Current error (setpoint - measurement)
 * @param dt Time step in seconds
 * @return Control output
 */
float pid_controller_update(pid_state_t *state, float error, float dt);

/**
 * @brief Set PID gains at runtime (FR-009)
 * @param state PID state
 * @param gains New gains
 */
void pid_controller_set_gains(pid_state_t *state, const pid_gains_t *gains);

/**
 * @brief Get current PID gains
 * @param state PID state
 * @param gains Output gains
 */
void pid_controller_get_gains(const pid_state_t *state, pid_gains_t *gains);

/* ==========================================================================
 * Servo Driver API
 * ========================================================================== */

/**
 * @brief Initialize servo PWM outputs
 * @param config Motor configuration with pin assignments
 * @return ESP_OK on success
 */
esp_err_t servo_driver_init(const motor_config_t *config);

/**
 * @brief Deinitialize servo driver
 * @return ESP_OK on success
 */
esp_err_t servo_driver_deinit(void);

/**
 * @brief Set servo position
 * @param channel Servo channel
 * @param pulse_us Pulse width in microseconds
 * @return ESP_OK on success
 */
esp_err_t servo_set_pulse(servo_channel_t channel, uint16_t pulse_us);

/**
 * @brief Set servo position by angle
 * @param channel Servo channel
 * @param angle_deg Angle in degrees (-MAX to +MAX)
 * @param min_us Minimum pulse width
 * @param max_us Maximum pulse width
 * @return ESP_OK on success
 */
esp_err_t servo_set_angle(servo_channel_t channel, float angle_deg,
                          uint16_t min_us, uint16_t max_us);

/**
 * @brief Set throttle percentage
 * @param throttle_pct Throttle percentage (0-100)
 * @return ESP_OK on success
 */
esp_err_t motor_set_throttle(float throttle_pct);

/* ==========================================================================
 * FreeRTOS Task
 * ========================================================================== */

/**
 * @brief Motor control task function
 * @param pvParameters Task parameters (unused)
 */
void vTask_Motor_Control(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */
