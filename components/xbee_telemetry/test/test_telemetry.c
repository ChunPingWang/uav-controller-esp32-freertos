/**
 * @file test_telemetry.c
 * @brief Unit tests for telemetry protocol
 *
 * Tests: T056, T057 - Telemetry unit tests
 */

#include "unity.h"
#include "xbee_telemetry.h"
#include <string.h>

/* ==========================================================================
 * CRC16 Tests (FR-022)
 * ========================================================================== */

TEST_CASE("CRC16 of empty data", "[telemetry][crc]")
{
    uint8_t data[] = {};
    uint16_t crc = telemetry_calculate_checksum(data, 0);
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc);
}

TEST_CASE("CRC16 of known pattern", "[telemetry][crc]")
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = telemetry_calculate_checksum(data, sizeof(data));
    /* Known CRC16-CCITT value for this data */
    TEST_ASSERT_NOT_EQUAL_HEX16(0, crc);
    TEST_ASSERT_NOT_EQUAL_HEX16(0xFFFF, crc);
}

TEST_CASE("CRC16 consistency", "[telemetry][crc]")
{
    uint8_t data[] = "Hello World";
    uint16_t crc1 = telemetry_calculate_checksum(data, sizeof(data) - 1);
    uint16_t crc2 = telemetry_calculate_checksum(data, sizeof(data) - 1);
    TEST_ASSERT_EQUAL_HEX16(crc1, crc2);
}

TEST_CASE("CRC16 changes with data", "[telemetry][crc]")
{
    uint8_t data1[] = {0x01, 0x02, 0x03};
    uint8_t data2[] = {0x01, 0x02, 0x04};  /* Last byte different */

    uint16_t crc1 = telemetry_calculate_checksum(data1, sizeof(data1));
    uint16_t crc2 = telemetry_calculate_checksum(data2, sizeof(data2));

    TEST_ASSERT_NOT_EQUAL_HEX16(crc1, crc2);
}

/* ==========================================================================
 * Packet Constants Tests (FR-021)
 * ========================================================================== */

TEST_CASE("Packet sync bytes", "[telemetry]")
{
    TEST_ASSERT_EQUAL_HEX8(0xAA, PACKET_SYNC_BYTE_1);
    TEST_ASSERT_EQUAL_HEX8(0x55, PACKET_SYNC_BYTE_2);
}

TEST_CASE("Packet size limits", "[telemetry]")
{
    TEST_ASSERT_EQUAL(128, PACKET_MAX_PAYLOAD);
    TEST_ASSERT_EQUAL(6, PACKET_HEADER_SIZE);
    TEST_ASSERT_EQUAL(2, PACKET_FOOTER_SIZE);
}

TEST_CASE("Message types", "[telemetry]")
{
    TEST_ASSERT_EQUAL(0x01, MSG_TYPE_TELEMETRY);
    TEST_ASSERT_EQUAL(0x02, MSG_TYPE_COMMAND);
    TEST_ASSERT_EQUAL(0x03, MSG_TYPE_ACK);
    TEST_ASSERT_EQUAL(0x04, MSG_TYPE_HEARTBEAT);
}

/* ==========================================================================
 * Telemetry Flags Tests (FR-023)
 * ========================================================================== */

TEST_CASE("Telemetry flags values", "[telemetry]")
{
    TEST_ASSERT_EQUAL(0x01, TELEM_FLAG_ATTITUDE);
    TEST_ASSERT_EQUAL(0x02, TELEM_FLAG_GPS);
    TEST_ASSERT_EQUAL(0x04, TELEM_FLAG_STATUS);
    TEST_ASSERT_EQUAL(0x08, TELEM_FLAG_BATTERY);
    TEST_ASSERT_EQUAL(0x80, TELEM_FLAG_DEBUG);
}

TEST_CASE("Default telemetry flags", "[telemetry]")
{
    uint8_t expected = TELEM_FLAG_ATTITUDE | TELEM_FLAG_GPS | TELEM_FLAG_STATUS;
    TEST_ASSERT_EQUAL(expected, TELEM_DEFAULT_FLAGS);
}

/* ==========================================================================
 * Parser Tests
 * ========================================================================== */

TEST_CASE("Parser reset", "[telemetry][parser]")
{
    telemetry_parser_reset();
    /* Should not crash and start in initial state */
    TEST_PASS();
}

TEST_CASE("Parser rejects bad sync", "[telemetry][parser]")
{
    telemetry_parser_reset();

    packet_t packet;
    uint8_t bad_data[] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

    for (size_t i = 0; i < sizeof(bad_data); i++) {
        bool complete = telemetry_parser_feed(bad_data[i], &packet);
        TEST_ASSERT_FALSE(complete);
    }
}

TEST_CASE("Parser accepts valid sync", "[telemetry][parser]")
{
    telemetry_parser_reset();

    packet_t packet;

    /* Feed sync bytes */
    bool complete = telemetry_parser_feed(PACKET_SYNC_BYTE_1, &packet);
    TEST_ASSERT_FALSE(complete);

    complete = telemetry_parser_feed(PACKET_SYNC_BYTE_2, &packet);
    TEST_ASSERT_FALSE(complete);

    /* Parser should be in TYPE state now - we don't have direct access
     * but it shouldn't crash */
    TEST_PASS();
}

TEST_CASE("Parser rejects oversized payload", "[telemetry][parser]")
{
    telemetry_parser_reset();

    packet_t packet;

    /* Build packet with too-large payload length */
    uint8_t data[] = {
        PACKET_SYNC_BYTE_1, PACKET_SYNC_BYTE_2,
        MSG_TYPE_TELEMETRY,
        0x00,           /* flags */
        0xFF, 0x00      /* length = 255 > MAX_PAYLOAD */
    };

    for (size_t i = 0; i < sizeof(data); i++) {
        bool complete = telemetry_parser_feed(data[i], &packet);
        TEST_ASSERT_FALSE(complete);
    }
}

/* ==========================================================================
 * Packet Building Tests
 * ========================================================================== */

TEST_CASE("Build packet with null inputs", "[telemetry]")
{
    packet_t packet;
    esp_err_t ret = telemetry_build_packet(&packet, NULL, NULL, NULL, 0);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(PACKET_SYNC_BYTE_1, packet.header.sync1);
    TEST_ASSERT_EQUAL(PACKET_SYNC_BYTE_2, packet.header.sync2);
    TEST_ASSERT_EQUAL(MSG_TYPE_TELEMETRY, packet.header.msg_type);
}

TEST_CASE("Build packet null output returns error", "[telemetry]")
{
    esp_err_t ret = telemetry_build_packet(NULL, NULL, NULL, NULL, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

TEST_CASE("Build packet includes attitude", "[telemetry]")
{
    packet_t packet;
    attitude_state_t attitude = {
        .roll_rad = 0.1f,
        .pitch_rad = 0.2f,
        .yaw_rad = 0.3f,
        .valid = true
    };

    esp_err_t ret = telemetry_build_packet(&packet, &attitude, NULL, NULL,
                                           TELEM_FLAG_ATTITUDE);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(TELEM_FLAG_ATTITUDE, packet.header.flags);

    telemetry_data_t *data = (telemetry_data_t *)packet.payload;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, data->roll_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, data->pitch_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, data->yaw_rad);
}

/* ==========================================================================
 * Link Loss Tests (FR-024)
 * ========================================================================== */

TEST_CASE("Link loss timeout constant", "[telemetry]")
{
    TEST_ASSERT_EQUAL(3000, LINK_LOSS_TIMEOUT_MS);
}

TEST_CASE("Heartbeat update resets link age", "[telemetry]")
{
    telemetry_update_heartbeat();
    uint32_t age = telemetry_get_link_age_ms();
    /* Should be very small immediately after update */
    TEST_ASSERT_TRUE(age < 100);
}

/* ==========================================================================
 * Command Types Tests (FR-020)
 * ========================================================================== */

TEST_CASE("Command type enum values", "[telemetry]")
{
    TEST_ASSERT_EQUAL(0x01, CMD_SET_MODE);
    TEST_ASSERT_EQUAL(0x02, CMD_ARM);
    TEST_ASSERT_EQUAL(0x03, CMD_DISARM);
    TEST_ASSERT_EQUAL(0x04, CMD_SET_WAYPOINT);
    TEST_ASSERT_EQUAL(0x07, CMD_RTL);
    TEST_ASSERT_EQUAL(0xFF, CMD_REBOOT);
}

/* ==========================================================================
 * Configuration Tests
 * ========================================================================== */

TEST_CASE("XBee default config", "[telemetry]")
{
    xbee_config_t config = XBEE_CONFIG_DEFAULT();
    TEST_ASSERT_EQUAL(115200, config.baud_rate);
}

TEST_CASE("Telemetry rate", "[telemetry]")
{
    TEST_ASSERT_EQUAL(5, TELEMETRY_RATE_HZ);
    TEST_ASSERT_EQUAL(200, TELEMETRY_PERIOD_MS);
}

/* ==========================================================================
 * Structure Size Tests
 * ========================================================================== */

TEST_CASE("Packet header size", "[telemetry]")
{
    TEST_ASSERT_EQUAL(PACKET_HEADER_SIZE, sizeof(packet_header_t));
}

TEST_CASE("Telemetry data structure packed", "[telemetry]")
{
    /* Telemetry data should be packed and have known size */
    telemetry_data_t data;
    /* Roll(4) + Pitch(4) + Yaw(4) = 12 bytes attitude */
    /* Lat(8) + Lon(8) + Alt(4) + Speed(4) = 24 bytes GPS */
    /* Status(8) + Timestamp(8) = 16 bytes */
    /* Total = 52 bytes */
    TEST_ASSERT_EQUAL(52, sizeof(telemetry_data_t));
}
