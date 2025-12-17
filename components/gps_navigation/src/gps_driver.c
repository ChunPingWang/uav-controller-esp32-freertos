/**
 * @file gps_driver.c
 * @brief GPS module UART driver
 *
 * Requirements: FR-011 (10Hz GPS, <3m accuracy)
 */

#include "gps_navigation.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_GPS;

/* UART buffer size */
#define GPS_UART_BUF_SIZE       512
#define GPS_RX_BUF_SIZE         1024

/* State */
static gps_config_t s_config;
static bool s_initialized = false;
static gps_position_t s_position = {0};
static gps_fix_type_t s_fix_type = GPS_FIX_NONE;
static uint8_t s_satellites = 0;
static float s_hdop = 99.0f;

/* NMEA line buffer */
static char s_nmea_buffer[NMEA_MAX_LENGTH];
static size_t s_nmea_index = 0;

/* Forward declarations from nmea_parser.c */
extern nmea_type_t nmea_parse_gga(const char *sentence, gps_position_t *position);
extern nmea_type_t nmea_parse_rmc(const char *sentence, gps_position_t *position);
extern nmea_type_t nmea_parse_vtg(const char *sentence, gps_position_t *position);
extern nmea_type_t nmea_parse_gsa(const char *sentence, gps_position_t *position);

/* ==========================================================================
 * Public API
 * ========================================================================== */

esp_err_t gps_driver_init(const gps_config_t *config)
{
    if (s_initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    if (config == NULL) {
        gps_config_t default_cfg = GPS_CONFIG_DEFAULT();
        s_config = default_cfg;
    } else {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing GPS on UART%d (TX=%d, RX=%d) @ %lu baud",
             s_config.uart_port, s_config.tx_pin, s_config.rx_pin,
             (unsigned long)s_config.baud_rate);

    /* Configure UART */
    uart_config_t uart_cfg = {
        .baud_rate = (int)s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(s_config.uart_port, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(s_config.uart_port,
                       s_config.tx_pin,
                       s_config.rx_pin,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(s_config.uart_port, GPS_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize state */
    memset(&s_position, 0, sizeof(s_position));
    s_fix_type = GPS_FIX_NONE;
    s_satellites = 0;
    s_hdop = 99.0f;
    s_nmea_index = 0;

    s_initialized = true;
    ESP_LOGI(TAG, "GPS driver initialized");

    return ESP_OK;
}

esp_err_t gps_driver_deinit(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    uart_driver_delete(s_config.uart_port);
    s_initialized = false;

    ESP_LOGI(TAG, "GPS driver deinitialized");
    return ESP_OK;
}

bool gps_driver_is_ready(void)
{
    return s_initialized && s_position.valid;
}

esp_err_t gps_driver_get_position(gps_position_t *position)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (position == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *position = s_position;

    return s_position.valid ? ESP_OK : FC_ERR_GPS_NO_FIX;
}

gps_fix_type_t gps_driver_get_fix_type(void)
{
    return s_fix_type;
}

uint8_t gps_driver_get_satellites(void)
{
    return s_satellites;
}

float gps_driver_get_hdop(void)
{
    return s_hdop;
}

esp_err_t gps_driver_process(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    uint8_t data[GPS_UART_BUF_SIZE];
    int len = uart_read_bytes(s_config.uart_port, data, sizeof(data), 0);

    if (len <= 0) {
        return ESP_OK; /* No data available */
    }

    /* Process each byte */
    for (int i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '$') {
            /* Start of new sentence */
            s_nmea_index = 0;
            s_nmea_buffer[s_nmea_index++] = c;
        }
        else if (c == '\n' || c == '\r') {
            /* End of sentence */
            if (s_nmea_index > 0) {
                s_nmea_buffer[s_nmea_index] = '\0';

                /* Parse complete sentence */
                nmea_type_t type = nmea_parse_sentence(s_nmea_buffer, &s_position);

                if (type == NMEA_GGA) {
                    s_satellites = s_position.satellites;
                    s_hdop = s_position.hdop;
                    s_position.timestamp_us = esp_timer_get_time();

                    if (s_position.valid) {
                        s_fix_type = (s_satellites >= 4) ? GPS_FIX_3D : GPS_FIX_2D;
                    } else {
                        s_fix_type = GPS_FIX_NONE;
                    }
                }

                s_nmea_index = 0;
            }
        }
        else if (s_nmea_index < NMEA_MAX_LENGTH - 1) {
            /* Accumulate character */
            s_nmea_buffer[s_nmea_index++] = c;
        }
    }

    return ESP_OK;
}
