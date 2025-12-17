/**
 * @file xbee_driver.c
 * @brief XBee UART driver implementation
 *
 * Requirement: FR-018 (115200 baud)
 */

#include "xbee_telemetry.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_TLM;

/* State */
static xbee_config_t s_config;
static bool s_initialized = false;
static link_status_t s_link_status = {0};

esp_err_t xbee_driver_init(const xbee_config_t *config)
{
    if (s_initialized) {
        return FC_ERR_ALREADY_INIT;
    }

    if (config == NULL) {
        xbee_config_t default_cfg = XBEE_CONFIG_DEFAULT();
        s_config = default_cfg;
    } else {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing XBee on UART%d (TX=%d, RX=%d) @ %lu baud",
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

    ret = uart_driver_install(s_config.uart_port, XBEE_UART_BUF_SIZE,
                              XBEE_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize link status */
    memset(&s_link_status, 0, sizeof(s_link_status));
    s_link_status.last_rx_time_us = esp_timer_get_time();
    s_link_status.last_tx_time_us = esp_timer_get_time();

    s_initialized = true;
    ESP_LOGI(TAG, "XBee driver initialized");

    return ESP_OK;
}

esp_err_t xbee_driver_deinit(void)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    uart_driver_delete(s_config.uart_port);
    s_initialized = false;

    ESP_LOGI(TAG, "XBee driver deinitialized");
    return ESP_OK;
}

bool xbee_driver_is_ready(void)
{
    return s_initialized;
}

esp_err_t xbee_send(const uint8_t *data, size_t len)
{
    if (!s_initialized) {
        return FC_ERR_NOT_INIT;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(s_config.uart_port, data, len);
    if (written < 0) {
        return FC_ERR_XBEE_COMM;
    }

    s_link_status.last_tx_time_us = esp_timer_get_time();
    s_link_status.tx_packets++;

    return ESP_OK;
}

int xbee_receive(uint8_t *data, size_t max_len)
{
    if (!s_initialized || data == NULL || max_len == 0) {
        return 0;
    }

    int len = uart_read_bytes(s_config.uart_port, data, max_len, 0);

    if (len > 0) {
        s_link_status.last_rx_time_us = esp_timer_get_time();
        s_link_status.connected = true;
    }

    return len > 0 ? len : 0;
}

esp_err_t xbee_get_link_status(link_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Update connected status based on timeout */
    uint64_t now = esp_timer_get_time();
    uint64_t age_us = now - s_link_status.last_rx_time_us;
    s_link_status.connected = (age_us < (LINK_LOSS_TIMEOUT_MS * 1000ULL));

    *status = s_link_status;

    return ESP_OK;
}
