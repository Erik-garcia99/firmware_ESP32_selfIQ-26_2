#ifndef SELFIQ_MQTT_LIB_H
#define SELFIQ_MQTT_LIB_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#include "global.h"

#define MQTT_PENDING_COUNT 16
#define MQTT_EVENT_BYTES 384
#define MQTT_QOS_AT_LEAST_ONCE 1
#define MQTT_RETAIN_DISABLED 0
#define MQTT_TEXT_AUTO_LENGTH 0
#define MQTT_STATUS_ONLINE "online"
#define MQTT_STATUS_ONLINE_LEN (sizeof(MQTT_STATUS_ONLINE) - TEXT_TERMINATOR_SIZE)
#define MQTT_HEX_CHARS_PER_BYTE 2U
#define MAX_TOPIC_LEN 180

#define MDNS_QUERY_TIMEOUT_MS 2000U
#define MQTT_DEFAULT_PORT 1883U

#define MAC_BINARY_SIZE 6U
#define MAC_TEXT_LEN 17U
#define MAC_TEXT_SIZE (MAC_TEXT_LEN + TEXT_TERMINATOR_SIZE)


#define NODE_TYPE "access_counter" // Tipo del nodo en el backend
#define NODE_MODULE "capacity_atmosphere" // Valor heredado; validar cambio junto con backend
#define MQTT_SETUP_TOPIC "selfiq/esp32_setup"
#define IDENTITY_NS "selfiq_node"

#define MQTT_CLIENT_ID_SIZE (sizeof("esp-") - 1U + MAC_BINARY_SIZE * MQTT_HEX_CHARS_PER_BYTE + TEXT_TERMINATOR_SIZE)
#define MQTT_REPLY_TOPIC_SIZE 96U
#define MQTT_URI_SIZE 64U
#define MQTT_ACK_TOPIC_SIZE 180U
#define MQTT_EVENT_ID_SIZE 80U
#define MQTT_BOOT_WORDS 2U
#define MQTT_UUID_SEPARATOR '|'
#define MQTT_REGISTRATION_SEPARATOR_BYTES 1U
#define MQTT_REGISTRATION_MAC_OFFSET 0U
#define MQTT_REGISTRATION_FIRST_SEPARATOR MAC_TEXT_LEN
#define MQTT_REGISTRATION_UUID_OFFSET (MAC_TEXT_LEN + MQTT_REGISTRATION_SEPARATOR_BYTES)
#define MQTT_REGISTRATION_SECOND_SEPARATOR (MQTT_REGISTRATION_UUID_OFFSET + UUID_TEXT_LEN)
#define MQTT_REGISTRATION_BRANCH_OFFSET (MQTT_REGISTRATION_SECOND_SEPARATOR + MQTT_REGISTRATION_SEPARATOR_BYTES)
#define MQTT_REGISTRATION_PAYLOAD_LEN (MQTT_REGISTRATION_BRANCH_OFFSET + UUID_TEXT_LEN)
#define MQTT_REGISTRATION_BUFFER_SIZE (MQTT_REGISTRATION_PAYLOAD_LEN + TEXT_TERMINATOR_SIZE)
#define MQTT_EVENT_TYPE_MAX_LEN 32U
#define MQTT_RETRY_INTERVAL_MS 15000U
#define MQTT_RETRY_TASK_STACK 4096U
#define MQTT_RETRY_TASK_PRIORITY 4U
#define MQTT_KEEPALIVE_SEC 45U
#define MQTT_RECONNECT_TIMEOUT_MS 5000U
#define MQTT_OUTBOX_BYTES 8192U

/* Call start/stop only from the connection owner (main), never MQTT callback. */
esp_err_t mqtt_register_start(const char *broker_ip, uint16_t broker_port);
void mqtt_network_stop(void);
bool mqtt_network_connected(void);
/* ESP_OK: accepted in bounded RAM queue, not yet durable. Caller handles errors.
 * Used by each sensor module, e.g. mqtt_queue_value("temperature", 23.5).
 * No UTC clock is invented: gateway attaches received_at. */
esp_err_t mqtt_queue_value(const char *event_type, double value);
#endif
