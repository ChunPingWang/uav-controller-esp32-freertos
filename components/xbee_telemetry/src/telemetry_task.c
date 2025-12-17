/**
 * @file telemetry_task.c
 * @brief Telemetry FreeRTOS task
 *
 * Task: vTask_Telemetry
 * Priority: configMAX_PRIORITIES-4
 * Stack: 4096 bytes
 * Period: 200ms (5Hz) - FR-019
 */

#include "xbee_telemetry.h"
#include "fc_tasks.h"
#include "fc_log.h"
#include "fc_errors.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = TAG_TLM;

/* External function from nav_task.c */
extern void navigation_update_telemetry_time(void);

void vTask_Telemetry(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Telemetry task started");

    /* Initialize XBee driver */
    xbee_config_t config = XBEE_CONFIG_DEFAULT();
    esp_err_t ret = xbee_driver_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize XBee: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    /* Initialize heartbeat */
    telemetry_update_heartbeat();

    packet_t tx_packet;
    packet_t rx_packet;
    uint8_t rx_buffer[256];

    attitude_state_t attitude;
    gps_position_t gps;
    system_status_t status;

    uint32_t tx_count = 0;
    uint32_t rx_count = 0;
    bool link_lost_reported = false;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* ============ Receive Processing ============ */

        /* Check for incoming data */
        int rx_len = xbee_receive(rx_buffer, sizeof(rx_buffer));
        if (rx_len > 0) {
            /* Feed bytes to parser */
            for (int i = 0; i < rx_len; i++) {
                if (telemetry_parser_feed(rx_buffer[i], &rx_packet)) {
                    /* Complete packet received */
                    rx_count++;
                    telemetry_update_heartbeat();
                    navigation_update_telemetry_time();
                    link_lost_reported = false;

                    /* Process based on message type */
                    switch (rx_packet.header.msg_type) {
                        case MSG_TYPE_COMMAND:
                            telemetry_process_command(&rx_packet);
                            break;

                        case MSG_TYPE_HEARTBEAT:
                            /* Just update heartbeat, already done above */
                            ESP_LOGD(TAG, "Heartbeat received");
                            break;

                        case MSG_TYPE_CONFIG:
                            /* Handle configuration packet */
                            if (rx_packet.header.payload_len >= 1) {
                                telemetry_set_fields(rx_packet.payload[0]);
                            }
                            break;

                        default:
                            ESP_LOGD(TAG, "Unknown packet type: %d",
                                     rx_packet.header.msg_type);
                            break;
                    }

                    /* Forward to navigation for comm loss detection */
                    telemetry_packet_t telem_pkt = {
                        .msg_type = rx_packet.header.msg_type,
                        .payload_len = rx_packet.header.payload_len < 32 ?
                                       rx_packet.header.payload_len : 32,
                    };
                    memcpy(telem_pkt.payload, rx_packet.payload, telem_pkt.payload_len);
                    telem_pkt.timestamp_us = esp_timer_get_time();

                    xQueueSend(g_queues.telemetry_rx, &telem_pkt, 0);
                }
            }
        }

        /* Check for link loss - FR-024 */
        if (telemetry_is_link_lost()) {
            if (!link_lost_reported) {
                ESP_LOGW(TAG, "Link lost (no data for %lu ms)",
                         (unsigned long)telemetry_get_link_age_ms());
                link_lost_reported = true;
            }
        }

        /* ============ Transmit Processing ============ */

        /* Get current state data */
        bool got_attitude = (fc_get_attitude_state(&attitude, 0) == ESP_OK);
        bool got_gps = false;

        /* Try to get GPS from queue */
        gps_position_t gps_temp;
        if (xQueuePeek(g_queues.gps_to_nav, &gps_temp, 0) == pdTRUE) {
            gps = gps_temp;
            got_gps = true;
        }

        /* Get system status */
        fc_get_system_status(&status);

        /* Build telemetry packet */
        uint8_t flags = telemetry_get_fields();
        if (!got_attitude) flags &= ~TELEM_FLAG_ATTITUDE;
        if (!got_gps) flags &= ~TELEM_FLAG_GPS;

        ret = telemetry_build_packet(&tx_packet,
                                     got_attitude ? &attitude : NULL,
                                     got_gps ? &gps : NULL,
                                     &status,
                                     flags);

        if (ret == ESP_OK) {
            /* Send packet */
            size_t total_len = PACKET_HEADER_SIZE +
                              tx_packet.header.payload_len +
                              PACKET_FOOTER_SIZE;

            /* Build raw packet buffer */
            uint8_t tx_buffer[PACKET_HEADER_SIZE + PACKET_MAX_PAYLOAD + PACKET_FOOTER_SIZE];
            size_t idx = 0;

            /* Header */
            memcpy(&tx_buffer[idx], &tx_packet.header, PACKET_HEADER_SIZE);
            idx += PACKET_HEADER_SIZE;

            /* Payload */
            memcpy(&tx_buffer[idx], tx_packet.payload, tx_packet.header.payload_len);
            idx += tx_packet.header.payload_len;

            /* Checksum */
            tx_buffer[idx++] = tx_packet.checksum & 0xFF;
            tx_buffer[idx++] = (tx_packet.checksum >> 8) & 0xFF;

            ret = xbee_send(tx_buffer, total_len);
            if (ret == ESP_OK) {
                tx_count++;
            }
        }

        /* Periodic logging */
        if ((tx_count % 25) == 0 && tx_count > 0) { /* Every 5 seconds at 5Hz */
            link_status_t link;
            xbee_get_link_status(&link);

            ESP_LOGI(TAG, "Telemetry: TX=%lu, RX=%lu, link=%s, age=%lu ms",
                     (unsigned long)tx_count,
                     (unsigned long)rx_count,
                     link.connected ? "OK" : "LOST",
                     (unsigned long)telemetry_get_link_age_ms());

            FC_LOG_STACK(TAG, NULL);
        }

        /* Maintain 5Hz rate (200ms period) - FR-019 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
