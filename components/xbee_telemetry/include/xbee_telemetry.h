/**
 * @file xbee_telemetry.h
 * @brief XBee telemetry communication public API
 *
 * Requirements:
 * - FR-018: XBee @ 115200 baud
 * - FR-019: Telemetry @ 5Hz (attitude, GPS, status)
 * - FR-020: Remote command reception
 * - FR-021: Packet format definition
 * - FR-022: Error detection (checksum)
 * - FR-023: Configurable data fields
 * - FR-024: Lost link detection
 */

#ifndef XBEE_TELEMETRY_H
#define XBEE_TELEMETRY_H

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

/* UART configuration - FR-018 */
#define XBEE_BAUD_RATE          115200
#define XBEE_UART_BUF_SIZE      1024

/* Telemetry rate - FR-019 */
#define TELEMETRY_RATE_HZ       5
#define TELEMETRY_PERIOD_MS     (1000 / TELEMETRY_RATE_HZ)

/* Packet format - FR-021 */
#define PACKET_SYNC_BYTE_1      0xAA
#define PACKET_SYNC_BYTE_2      0x55
#define PACKET_MAX_PAYLOAD      128
#define PACKET_HEADER_SIZE      6       /* Sync(2) + Type(1) + Flags(1) + Len(2) */
#define PACKET_FOOTER_SIZE      2       /* Checksum(2) */

/* Message types */
#define MSG_TYPE_TELEMETRY      0x01    /* Outgoing telemetry */
#define MSG_TYPE_COMMAND        0x02    /* Incoming command */
#define MSG_TYPE_ACK            0x03    /* Acknowledgment */
#define MSG_TYPE_HEARTBEAT      0x04    /* Heartbeat */
#define MSG_TYPE_CONFIG         0x05    /* Configuration */
#define MSG_TYPE_PID_TUNE       0x06    /* PID tuning - FR-009 */

/* Telemetry flags - FR-023 */
#define TELEM_FLAG_ATTITUDE     (1 << 0)
#define TELEM_FLAG_GPS          (1 << 1)
#define TELEM_FLAG_STATUS       (1 << 2)
#define TELEM_FLAG_BATTERY      (1 << 3)
#define TELEM_FLAG_NAV          (1 << 4)
#define TELEM_FLAG_DEBUG        (1 << 7)

/* Default telemetry fields */
#define TELEM_DEFAULT_FLAGS     (TELEM_FLAG_ATTITUDE | TELEM_FLAG_GPS | TELEM_FLAG_STATUS)

/* Link loss threshold - FR-024 */
#define LINK_LOSS_TIMEOUT_MS    3000    /* Same as COMM_LOSS_TIMEOUT_MS */

/* ==========================================================================
 * Data Types
 * ========================================================================== */

/**
 * @brief XBee driver configuration
 */
typedef struct {
    int uart_port;
    int tx_pin;
    int rx_pin;
    uint32_t baud_rate;
} xbee_config_t;

/**
 * @brief Packet header structure
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1;          /* PACKET_SYNC_BYTE_1 */
    uint8_t sync2;          /* PACKET_SYNC_BYTE_2 */
    uint8_t msg_type;       /* Message type */
    uint8_t flags;          /* Message flags */
    uint16_t payload_len;   /* Payload length */
} packet_header_t;

/**
 * @brief Full packet structure (for building/parsing)
 */
typedef struct {
    packet_header_t header;
    uint8_t payload[PACKET_MAX_PAYLOAD];
    uint16_t checksum;
} packet_t;

/**
 * @brief Telemetry data packet payload - FR-019
 */
typedef struct __attribute__((packed)) {
    /* Attitude (12 bytes) */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    /* GPS (24 bytes) */
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_ms;

    /* Status (8 bytes) */
    uint8_t flight_mode;
    uint8_t armed;
    uint8_t gps_fix;
    uint8_t battery_pct;
    uint16_t errors;
    uint16_t reserved;

    /* Timestamp (8 bytes) */
    uint64_t timestamp_ms;
} telemetry_data_t;

/**
 * @brief Command packet payload - FR-020
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_type;       /* Command type */
    uint8_t cmd_flags;      /* Command flags */
    uint16_t sequence;      /* Sequence number */
    uint8_t data[60];       /* Command data */
} command_data_t;

/* Command types */
typedef enum {
    CMD_SET_MODE = 0x01,
    CMD_ARM = 0x02,
    CMD_DISARM = 0x03,
    CMD_SET_WAYPOINT = 0x04,
    CMD_CLEAR_WAYPOINTS = 0x05,
    CMD_SET_HOME = 0x06,
    CMD_RTL = 0x07,
    CMD_PID_TUNE = 0x08,
    CMD_CALIBRATE_IMU = 0x09,
    CMD_REBOOT = 0xFF,
} command_type_t;

/**
 * @brief Parser state machine
 */
typedef enum {
    PARSE_STATE_SYNC1,
    PARSE_STATE_SYNC2,
    PARSE_STATE_TYPE,
    PARSE_STATE_FLAGS,
    PARSE_STATE_LEN_L,
    PARSE_STATE_LEN_H,
    PARSE_STATE_PAYLOAD,
    PARSE_STATE_CHECKSUM_L,
    PARSE_STATE_CHECKSUM_H,
} parser_state_t;

/**
 * @brief Link status
 */
typedef struct {
    bool connected;
    uint64_t last_rx_time_us;
    uint64_t last_tx_time_us;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t rx_errors;
    int8_t rssi;            /* If available from XBee */
} link_status_t;

/* Default configuration */
#define XBEE_CONFIG_DEFAULT() { \
    .uart_port = CONFIG_XBEE_UART_PORT, \
    .tx_pin = CONFIG_XBEE_TX_PIN, \
    .rx_pin = CONFIG_XBEE_RX_PIN, \
    .baud_rate = XBEE_BAUD_RATE \
}

/* ==========================================================================
 * XBee Driver API
 * ========================================================================== */

/**
 * @brief Initialize XBee driver
 * @param config Driver configuration
 * @return ESP_OK on success
 */
esp_err_t xbee_driver_init(const xbee_config_t *config);

/**
 * @brief Deinitialize XBee driver
 * @return ESP_OK on success
 */
esp_err_t xbee_driver_deinit(void);

/**
 * @brief Check if XBee driver is ready
 * @return true if initialized
 */
bool xbee_driver_is_ready(void);

/**
 * @brief Send raw bytes via XBee
 * @param data Data to send
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t xbee_send(const uint8_t *data, size_t len);

/**
 * @brief Receive data from XBee (non-blocking)
 * @param data Buffer for received data
 * @param max_len Maximum bytes to receive
 * @return Number of bytes received (0 if none)
 */
int xbee_receive(uint8_t *data, size_t max_len);

/**
 * @brief Get link status
 * @param status Output status structure
 * @return ESP_OK on success
 */
esp_err_t xbee_get_link_status(link_status_t *status);

/* ==========================================================================
 * Telemetry Protocol API - FR-021, FR-022
 * ========================================================================== */

/**
 * @brief Calculate CRC16 checksum
 * @param data Data to checksum
 * @param len Data length
 * @return CRC16 checksum
 */
uint16_t telemetry_calculate_checksum(const uint8_t *data, size_t len);

/**
 * @brief Build telemetry packet
 * @param packet Output packet
 * @param attitude Current attitude
 * @param gps Current GPS position
 * @param status System status
 * @param flags Telemetry flags (which fields to include)
 * @return ESP_OK on success
 */
esp_err_t telemetry_build_packet(
    packet_t *packet,
    const attitude_state_t *attitude,
    const gps_position_t *gps,
    const system_status_t *status,
    uint8_t flags);

/**
 * @brief Parse received packet
 * @param data Raw received data
 * @param len Data length
 * @param packet Output parsed packet
 * @return ESP_OK if valid packet parsed
 */
esp_err_t telemetry_parse_packet(const uint8_t *data, size_t len, packet_t *packet);

/**
 * @brief Process received command
 * @param packet Received packet
 * @return ESP_OK on success
 */
esp_err_t telemetry_process_command(const packet_t *packet);

/**
 * @brief Set telemetry fields to include - FR-023
 * @param flags Telemetry field flags
 * @return ESP_OK on success
 */
esp_err_t telemetry_set_fields(uint8_t flags);

/**
 * @brief Get current telemetry field flags
 * @return Current flags
 */
uint8_t telemetry_get_fields(void);

/* ==========================================================================
 * Parser API
 * ========================================================================== */

/**
 * @brief Reset parser state machine
 */
void telemetry_parser_reset(void);

/**
 * @brief Feed byte to parser
 * @param byte Received byte
 * @param packet Output packet (valid when returns true)
 * @return true if complete packet parsed
 */
bool telemetry_parser_feed(uint8_t byte, packet_t *packet);

/* ==========================================================================
 * Link Loss Detection API - FR-024
 * ========================================================================== */

/**
 * @brief Check if link is lost
 * @return true if no data received within timeout
 */
bool telemetry_is_link_lost(void);

/**
 * @brief Get time since last received packet
 * @return Time in milliseconds
 */
uint32_t telemetry_get_link_age_ms(void);

/**
 * @brief Update heartbeat (call on valid rx)
 */
void telemetry_update_heartbeat(void);

/* ==========================================================================
 * FreeRTOS Task
 * ========================================================================== */

/**
 * @brief Telemetry task function
 * @param pvParameters Task parameters (unused)
 */
void vTask_Telemetry(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* XBEE_TELEMETRY_H */
