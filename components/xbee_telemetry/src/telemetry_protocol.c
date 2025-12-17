/**
 * @file telemetry_protocol.c
 * @brief Telemetry packet protocol implementation
 *
 * Requirements:
 * - FR-021: Packet format
 * - FR-022: Error detection (CRC16)
 * - FR-023: Configurable fields
 * - FR-024: Link loss detection
 */

#include "xbee_telemetry.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_TLM;

/* State */
static uint8_t s_telemetry_flags = TELEM_DEFAULT_FLAGS;
static uint64_t s_last_heartbeat_us = 0;

/* Parser state */
static parser_state_t s_parser_state = PARSE_STATE_SYNC1;
static packet_t s_parser_packet;
static size_t s_parser_payload_idx = 0;

/* ==========================================================================
 * CRC16 Implementation (FR-022)
 * ========================================================================== */

/* CRC16-CCITT lookup table */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t telemetry_calculate_checksum(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (crc << 8) ^ crc16_table[idx];
    }

    return crc;
}

/* ==========================================================================
 * Packet Building (FR-021)
 * ========================================================================== */

esp_err_t telemetry_build_packet(
    packet_t *packet,
    const attitude_state_t *attitude,
    const gps_position_t *gps,
    const system_status_t *status,
    uint8_t flags)
{
    if (packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Build header */
    packet->header.sync1 = PACKET_SYNC_BYTE_1;
    packet->header.sync2 = PACKET_SYNC_BYTE_2;
    packet->header.msg_type = MSG_TYPE_TELEMETRY;
    packet->header.flags = flags;

    /* Build payload */
    telemetry_data_t *telem = (telemetry_data_t *)packet->payload;
    memset(telem, 0, sizeof(telemetry_data_t));

    /* Attitude data */
    if ((flags & TELEM_FLAG_ATTITUDE) && attitude != NULL) {
        telem->roll_rad = attitude->roll_rad;
        telem->pitch_rad = attitude->pitch_rad;
        telem->yaw_rad = attitude->yaw_rad;
    }

    /* GPS data */
    if ((flags & TELEM_FLAG_GPS) && gps != NULL) {
        telem->latitude_deg = gps->latitude_deg;
        telem->longitude_deg = gps->longitude_deg;
        telem->altitude_m = gps->altitude_m;
        telem->speed_ms = gps->speed_ms;
        telem->gps_fix = gps->valid ? 1 : 0;
    }

    /* Status data */
    if ((flags & TELEM_FLAG_STATUS) && status != NULL) {
        telem->flight_mode = status->flight_mode;
        telem->armed = status->armed ? 1 : 0;
        telem->battery_pct = status->battery_percent;
        telem->errors = status->error_count;
    }

    /* Timestamp */
    telem->timestamp_ms = esp_timer_get_time() / 1000ULL;

    /* Set payload length */
    packet->header.payload_len = sizeof(telemetry_data_t);

    /* Calculate checksum over header + payload */
    uint16_t crc = telemetry_calculate_checksum(
        (uint8_t *)&packet->header.msg_type,
        4 + packet->header.payload_len);  /* Type + Flags + Len + Payload */
    packet->checksum = crc;

    return ESP_OK;
}

/* ==========================================================================
 * Packet Parsing
 * ========================================================================== */

void telemetry_parser_reset(void)
{
    s_parser_state = PARSE_STATE_SYNC1;
    s_parser_payload_idx = 0;
    memset(&s_parser_packet, 0, sizeof(s_parser_packet));
}

bool telemetry_parser_feed(uint8_t byte, packet_t *packet)
{
    switch (s_parser_state) {
        case PARSE_STATE_SYNC1:
            if (byte == PACKET_SYNC_BYTE_1) {
                s_parser_packet.header.sync1 = byte;
                s_parser_state = PARSE_STATE_SYNC2;
            }
            break;

        case PARSE_STATE_SYNC2:
            if (byte == PACKET_SYNC_BYTE_2) {
                s_parser_packet.header.sync2 = byte;
                s_parser_state = PARSE_STATE_TYPE;
            } else {
                telemetry_parser_reset();
            }
            break;

        case PARSE_STATE_TYPE:
            s_parser_packet.header.msg_type = byte;
            s_parser_state = PARSE_STATE_FLAGS;
            break;

        case PARSE_STATE_FLAGS:
            s_parser_packet.header.flags = byte;
            s_parser_state = PARSE_STATE_LEN_L;
            break;

        case PARSE_STATE_LEN_L:
            s_parser_packet.header.payload_len = byte;
            s_parser_state = PARSE_STATE_LEN_H;
            break;

        case PARSE_STATE_LEN_H:
            s_parser_packet.header.payload_len |= (uint16_t)byte << 8;
            if (s_parser_packet.header.payload_len > PACKET_MAX_PAYLOAD) {
                ESP_LOGW(TAG, "Packet too large: %d", s_parser_packet.header.payload_len);
                telemetry_parser_reset();
            } else if (s_parser_packet.header.payload_len == 0) {
                s_parser_state = PARSE_STATE_CHECKSUM_L;
            } else {
                s_parser_payload_idx = 0;
                s_parser_state = PARSE_STATE_PAYLOAD;
            }
            break;

        case PARSE_STATE_PAYLOAD:
            s_parser_packet.payload[s_parser_payload_idx++] = byte;
            if (s_parser_payload_idx >= s_parser_packet.header.payload_len) {
                s_parser_state = PARSE_STATE_CHECKSUM_L;
            }
            break;

        case PARSE_STATE_CHECKSUM_L:
            s_parser_packet.checksum = byte;
            s_parser_state = PARSE_STATE_CHECKSUM_H;
            break;

        case PARSE_STATE_CHECKSUM_H:
            s_parser_packet.checksum |= (uint16_t)byte << 8;

            /* Verify checksum */
            uint16_t calc_crc = telemetry_calculate_checksum(
                (uint8_t *)&s_parser_packet.header.msg_type,
                4 + s_parser_packet.header.payload_len);

            if (calc_crc == s_parser_packet.checksum) {
                /* Valid packet */
                if (packet != NULL) {
                    *packet = s_parser_packet;
                }
                telemetry_parser_reset();
                return true;
            } else {
                ESP_LOGW(TAG, "Checksum mismatch: got %04X, expected %04X",
                         s_parser_packet.checksum, calc_crc);
            }

            telemetry_parser_reset();
            break;
    }

    return false;
}

esp_err_t telemetry_parse_packet(const uint8_t *data, size_t len, packet_t *packet)
{
    if (data == NULL || packet == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    telemetry_parser_reset();

    for (size_t i = 0; i < len; i++) {
        if (telemetry_parser_feed(data[i], packet)) {
            return ESP_OK;
        }
    }

    return FC_ERR_XBEE_PARSE;
}

/* ==========================================================================
 * Command Processing (FR-020)
 * ========================================================================== */

esp_err_t telemetry_process_command(const packet_t *packet)
{
    if (packet == NULL || packet->header.msg_type != MSG_TYPE_COMMAND) {
        return ESP_ERR_INVALID_ARG;
    }

    command_data_t *cmd = (command_data_t *)packet->payload;

    ESP_LOGI(TAG, "Command received: type=%d, seq=%d", cmd->cmd_type, cmd->sequence);

    /* Process command based on type */
    switch (cmd->cmd_type) {
        case CMD_SET_MODE:
            ESP_LOGI(TAG, "Set mode: %d", cmd->data[0]);
            /* TODO: Call navigation_set_mode() */
            break;

        case CMD_ARM:
            ESP_LOGI(TAG, "Arm command");
            /* TODO: Call motor_controller_set_armed(true) */
            break;

        case CMD_DISARM:
            ESP_LOGI(TAG, "Disarm command");
            /* TODO: Call motor_controller_set_armed(false) */
            break;

        case CMD_RTL:
            ESP_LOGW(TAG, "RTL command");
            /* TODO: Call navigation_trigger_rtl() */
            break;

        case CMD_CALIBRATE_IMU:
            ESP_LOGI(TAG, "IMU calibration command");
            /* TODO: Call imu_sensor_start_calibration() */
            break;

        case CMD_PID_TUNE:
            ESP_LOGI(TAG, "PID tune command");
            /* TODO: Update PID gains */
            break;

        case CMD_REBOOT:
            ESP_LOGW(TAG, "Reboot command");
            /* TODO: esp_restart() with safety checks */
            break;

        default:
            ESP_LOGW(TAG, "Unknown command: %d", cmd->cmd_type);
            return FC_ERR_INVALID_CMD;
    }

    return ESP_OK;
}

/* ==========================================================================
 * Configuration (FR-023)
 * ========================================================================== */

esp_err_t telemetry_set_fields(uint8_t flags)
{
    s_telemetry_flags = flags;
    ESP_LOGI(TAG, "Telemetry fields set: 0x%02X", flags);
    return ESP_OK;
}

uint8_t telemetry_get_fields(void)
{
    return s_telemetry_flags;
}

/* ==========================================================================
 * Link Loss Detection (FR-024)
 * ========================================================================== */

void telemetry_update_heartbeat(void)
{
    s_last_heartbeat_us = esp_timer_get_time();
}

bool telemetry_is_link_lost(void)
{
    uint64_t now = esp_timer_get_time();
    uint64_t age_us = now - s_last_heartbeat_us;
    return (age_us > (LINK_LOSS_TIMEOUT_MS * 1000ULL));
}

uint32_t telemetry_get_link_age_ms(void)
{
    uint64_t now = esp_timer_get_time();
    uint64_t age_us = now - s_last_heartbeat_us;
    return (uint32_t)(age_us / 1000ULL);
}
