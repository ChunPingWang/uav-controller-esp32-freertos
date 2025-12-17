/**
 * @file servo_driver.c
 * @brief Servo and ESC PWM driver using LEDC
 *
 * Uses ESP32 LEDC peripheral for PWM generation:
 * - 50Hz for standard servos
 * - Configurable pulse width (1000-2000us typical)
 */

#include "motor_controller.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_MTR;

/* LEDC configuration */
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_14_BIT  /* 14-bit resolution */
#define LEDC_PERIOD_US      20000              /* 20ms period (50Hz) */

/* Duty cycle = (pulse_us / period_us) * max_duty */
#define US_TO_DUTY(us)      ((uint32_t)(((us) * ((1 << LEDC_DUTY_RES) - 1)) / LEDC_PERIOD_US))

/* Channel mapping */
static const ledc_channel_t s_ledc_channels[SERVO_CHANNEL_COUNT] = {
    LEDC_CHANNEL_0,  /* SERVO_CHANNEL_AILERON_L */
    LEDC_CHANNEL_1,  /* SERVO_CHANNEL_AILERON_R */
    LEDC_CHANNEL_2,  /* SERVO_CHANNEL_ELEVATOR */
    LEDC_CHANNEL_3,  /* SERVO_CHANNEL_RUDDER */
    LEDC_CHANNEL_4,  /* SERVO_CHANNEL_THROTTLE */
};

/* Pin configuration (set during init) */
static int8_t s_servo_pins[SERVO_CHANNEL_COUNT] = {-1, -1, -1, -1, -1};
static bool s_initialized = false;

esp_err_t servo_driver_init(const motor_config_t *config)
{
    if (s_initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing servo driver");

    /* Store pin configuration */
    s_servo_pins[SERVO_CHANNEL_AILERON_L] = config->aileron_l_pin;
    s_servo_pins[SERVO_CHANNEL_AILERON_R] = config->aileron_r_pin;
    s_servo_pins[SERVO_CHANNEL_ELEVATOR] = config->elevator_pin;
    s_servo_pins[SERVO_CHANNEL_RUDDER] = config->rudder_pin;
    s_servo_pins[SERVO_CHANNEL_THROTTLE] = config->throttle_pin;

    /* Configure LEDC timer */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure each channel */
    for (int ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        if (s_servo_pins[ch] < 0) {
            continue;
        }

        ledc_channel_config_t ch_cfg = {
            .gpio_num = s_servo_pins[ch],
            .speed_mode = LEDC_MODE,
            .channel = s_ledc_channels[ch],
            .timer_sel = LEDC_TIMER,
            .duty = US_TO_DUTY(SERVO_PWM_MID_US), /* Start at center */
            .hpoint = 0,
        };

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "LEDC channel %d config failed: %s", ch, esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGD(TAG, "Servo channel %d on GPIO %d", ch, s_servo_pins[ch]);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Servo driver initialized");

    return ESP_OK;
}

esp_err_t servo_driver_deinit(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    /* Stop all channels */
    for (int ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        if (s_servo_pins[ch] >= 0) {
            ledc_stop(LEDC_MODE, s_ledc_channels[ch], 0);
        }
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Servo driver deinitialized");

    return ESP_OK;
}

esp_err_t servo_set_pulse(servo_channel_t channel, uint16_t pulse_us)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (channel >= SERVO_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_servo_pins[channel] < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Clamp pulse width */
    if (pulse_us < SERVO_PWM_MIN_US) {
        pulse_us = SERVO_PWM_MIN_US;
    } else if (pulse_us > SERVO_PWM_MAX_US) {
        pulse_us = SERVO_PWM_MAX_US;
    }

    uint32_t duty = US_TO_DUTY(pulse_us);

    esp_err_t ret = ledc_set_duty(LEDC_MODE, s_ledc_channels[channel], duty);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ledc_update_duty(LEDC_MODE, s_ledc_channels[channel]);

    return ret;
}

esp_err_t servo_set_angle(servo_channel_t channel, float angle_deg,
                          uint16_t min_us, uint16_t max_us)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    /* Map angle to pulse width
     * Angle range depends on servo type and mounting
     * Assuming symmetrical range around center
     */

    /* Determine max angle based on channel */
    float max_angle = 30.0f; /* Default */
    switch (channel) {
        case SERVO_CHANNEL_AILERON_L:
        case SERVO_CHANNEL_AILERON_R:
            max_angle = AILERON_MAX_DEG;
            break;
        case SERVO_CHANNEL_ELEVATOR:
            max_angle = ELEVATOR_MAX_DEG;
            break;
        case SERVO_CHANNEL_RUDDER:
            max_angle = RUDDER_MAX_DEG;
            break;
        default:
            break;
    }

    /* Clamp angle */
    if (angle_deg > max_angle) {
        angle_deg = max_angle;
    } else if (angle_deg < -max_angle) {
        angle_deg = -max_angle;
    }

    /* Map angle to pulse width */
    /* Center pulse = (min + max) / 2 */
    /* Range = (max - min) / 2 for each direction */
    uint16_t center_us = (min_us + max_us) / 2;
    uint16_t range_us = (max_us - min_us) / 2;

    /* Calculate pulse: center + (angle/max_angle) * range */
    float normalized = angle_deg / max_angle;
    uint16_t pulse_us = (uint16_t)(center_us + normalized * range_us);

    return servo_set_pulse(channel, pulse_us);
}

esp_err_t motor_set_throttle(float throttle_pct)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    /* Clamp throttle */
    if (throttle_pct < THROTTLE_MIN_PCT) {
        throttle_pct = THROTTLE_MIN_PCT;
    } else if (throttle_pct > THROTTLE_MAX_PCT) {
        throttle_pct = THROTTLE_MAX_PCT;
    }

    /* Map 0-100% to pulse width range */
    /* ESC typically: 1000us = 0%, 2000us = 100% */
    uint16_t pulse_us = SERVO_PWM_MIN_US +
                        (uint16_t)((throttle_pct / 100.0f) *
                                   (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));

    return servo_set_pulse(SERVO_CHANNEL_THROTTLE, pulse_us);
}
