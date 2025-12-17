/**
 * @file fc_battery.c
 * @brief Battery voltage monitoring (ADC)
 *
 * Implements battery monitoring for Phase 7 (US5).
 * Uses ESP32 ADC to measure battery voltage via voltage divider.
 */

#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

#include <string.h>
#include <math.h>

static const char *TAG = TAG_FC;

/* Battery configuration */
#define BATTERY_ADC_UNIT        ADC_UNIT_1
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_6   /* GPIO34 */
#define BATTERY_ADC_ATTEN       ADC_ATTEN_DB_12 /* 0-3.3V range */

/* Voltage divider ratio (e.g., 10K/3.3K for ~4:1) */
#define VOLTAGE_DIVIDER_RATIO   4.0f

/* LiPo voltage thresholds (3S pack) */
#define BATTERY_CELL_COUNT      3
#define CELL_FULL_VOLTAGE       4.2f    /* 100% */
#define CELL_NOMINAL_VOLTAGE    3.7f    /* ~50% */
#define CELL_LOW_VOLTAGE        3.5f    /* ~20% - warning */
#define CELL_CRITICAL_VOLTAGE   3.3f    /* ~5% - land immediately */
#define CELL_EMPTY_VOLTAGE      3.0f    /* 0% - damage risk */

/* Smoothing filter */
#define BATTERY_FILTER_ALPHA    0.1f    /* Low-pass filter coefficient */

/* State */
typedef struct {
    bool initialized;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool calibrated;

    float voltage_filtered;
    float percent_filtered;
    uint64_t last_read_us;
} battery_state_t;

static battery_state_t s_battery = {0};

/* ==========================================================================
 * Internal Functions
 * ========================================================================== */

/**
 * @brief Convert voltage to percentage using LiPo discharge curve
 */
static float voltage_to_percent(float cell_voltage)
{
    /* Simplified LiPo discharge curve approximation */
    /* Real discharge curve is non-linear */

    if (cell_voltage >= CELL_FULL_VOLTAGE) {
        return 100.0f;
    }
    if (cell_voltage <= CELL_EMPTY_VOLTAGE) {
        return 0.0f;
    }

    /* Piecewise linear approximation */
    if (cell_voltage >= 4.0f) {
        /* 4.0V - 4.2V: 80-100% (steep at top) */
        return 80.0f + (cell_voltage - 4.0f) * 100.0f;
    }
    if (cell_voltage >= 3.7f) {
        /* 3.7V - 4.0V: 30-80% (main usable range) */
        return 30.0f + (cell_voltage - 3.7f) * 166.67f;
    }
    if (cell_voltage >= 3.5f) {
        /* 3.5V - 3.7V: 10-30% (warning zone) */
        return 10.0f + (cell_voltage - 3.5f) * 100.0f;
    }
    /* 3.0V - 3.5V: 0-10% (critical) */
    return (cell_voltage - 3.0f) * 20.0f;
}

/**
 * @brief Read ADC and convert to voltage
 */
static float read_battery_voltage(void)
{
    if (!s_battery.initialized) {
        return 0.0f;
    }

    int raw_value = 0;
    esp_err_t ret = adc_oneshot_read(s_battery.adc_handle, BATTERY_ADC_CHANNEL, &raw_value);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return s_battery.voltage_filtered;
    }

    /* Convert to voltage */
    int voltage_mv = 0;
    if (s_battery.calibrated) {
        ret = adc_cali_raw_to_voltage(s_battery.cali_handle, raw_value, &voltage_mv);
        if (ret != ESP_OK) {
            /* Fallback to simple conversion */
            voltage_mv = (raw_value * 3300) / 4095;
        }
    } else {
        /* Simple conversion without calibration */
        voltage_mv = (raw_value * 3300) / 4095;
    }

    /* Apply voltage divider ratio */
    float battery_voltage = ((float)voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;

    return battery_voltage;
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

esp_err_t fc_battery_init(void)
{
    if (s_battery.initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    ESP_LOGI(TAG, "Initializing battery monitor");

    memset(&s_battery, 0, sizeof(s_battery));

    /* Configure ADC */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &s_battery.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure channel */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = BATTERY_ADC_ATTEN,
    };

    ret = adc_oneshot_config_channel(s_battery.adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_battery.adc_handle);
        return ret;
    }

    /* Try to set up calibration */
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_battery.cali_handle);
    if (ret == ESP_OK) {
        s_battery.calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        s_battery.calibrated = false;
        ESP_LOGW(TAG, "ADC calibration not available");
    }

    /* Initial reading */
    s_battery.voltage_filtered = read_battery_voltage();
    float cell_voltage = s_battery.voltage_filtered / BATTERY_CELL_COUNT;
    s_battery.percent_filtered = voltage_to_percent(cell_voltage);
    s_battery.last_read_us = esp_timer_get_time();

    s_battery.initialized = true;

    ESP_LOGI(TAG, "Battery monitor initialized: %.2fV (%.0f%%)",
             s_battery.voltage_filtered, s_battery.percent_filtered);

    return ESP_OK;
}

esp_err_t fc_battery_deinit(void)
{
    if (!s_battery.initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (s_battery.calibrated) {
        adc_cali_delete_scheme_line_fitting(s_battery.cali_handle);
    }

    adc_oneshot_del_unit(s_battery.adc_handle);

    s_battery.initialized = false;

    ESP_LOGI(TAG, "Battery monitor deinitialized");
    return ESP_OK;
}

float fc_battery_get_voltage(void)
{
    if (!s_battery.initialized) {
        return 0.0f;
    }

    /* Read new value */
    float raw_voltage = read_battery_voltage();

    /* Apply low-pass filter */
    s_battery.voltage_filtered = BATTERY_FILTER_ALPHA * raw_voltage +
                                  (1.0f - BATTERY_FILTER_ALPHA) * s_battery.voltage_filtered;

    s_battery.last_read_us = esp_timer_get_time();

    return s_battery.voltage_filtered;
}

float fc_battery_get_percent(void)
{
    if (!s_battery.initialized) {
        return 0.0f;
    }

    /* Get current voltage */
    float voltage = fc_battery_get_voltage();
    float cell_voltage = voltage / BATTERY_CELL_COUNT;

    /* Convert to percentage */
    float raw_percent = voltage_to_percent(cell_voltage);

    /* Apply filter */
    s_battery.percent_filtered = BATTERY_FILTER_ALPHA * raw_percent +
                                  (1.0f - BATTERY_FILTER_ALPHA) * s_battery.percent_filtered;

    /* Clamp to valid range */
    if (s_battery.percent_filtered < 0.0f) s_battery.percent_filtered = 0.0f;
    if (s_battery.percent_filtered > 100.0f) s_battery.percent_filtered = 100.0f;

    return s_battery.percent_filtered;
}

bool fc_battery_is_low(void)
{
    if (!s_battery.initialized) {
        return false;
    }

    float voltage = fc_battery_get_voltage();
    float cell_voltage = voltage / BATTERY_CELL_COUNT;

    return (cell_voltage < CELL_LOW_VOLTAGE);
}

bool fc_battery_is_critical(void)
{
    if (!s_battery.initialized) {
        return false;
    }

    float voltage = fc_battery_get_voltage();
    float cell_voltage = voltage / BATTERY_CELL_COUNT;

    return (cell_voltage < CELL_CRITICAL_VOLTAGE);
}

float fc_battery_get_cell_voltage(void)
{
    return fc_battery_get_voltage() / BATTERY_CELL_COUNT;
}
