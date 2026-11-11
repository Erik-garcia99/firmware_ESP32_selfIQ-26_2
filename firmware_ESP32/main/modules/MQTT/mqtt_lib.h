#ifndef SELFIQ_MQTT_LIB_H
#define SELFIQ_MQTT_LIB_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MAX_TOPIC_LEN 180

/* Payload MQTT: comando(1), destino NVS(1). Respuesta: comando(1), destino(1), resultado(1).
 * Wi-Fi/TCP reservados: este firmware solo admite borrar la identidad backend. */
#define MQTT_REQ_ERASE_NVS 0x01U
#define MQTT_RESP_ERASE_NVS 0x81U
#define MQTT_NVS_WIFI 0x01U
#define MQTT_NVS_TCP 0x02U
#define MQTT_NVS_IDENTITY 0x03U
#define MQTT_CONTROL_REQUEST_BYTES 2U
#define MQTT_CONTROL_RESULT_BYTES 3U

#define MDNS_QUERY_TIMEOUT_MS 2000U
#define MQTT_DEFAULT_PORT 1883U

#define MAC_BINARY_SIZE 6U
#define MAC_TEXT_LEN 17U
#define MAC_TEXT_SIZE (MAC_TEXT_LEN + TEXT_TERMINATOR_SIZE)


#define NODE_TYPE "shelf_cam" // Tipo del nodo en el backend (camara de repisa)
#define NODE_MODULE "shelf_cam" // Modulo del nodo: camara de repisa
#define MQTT_SETUP_TOPIC "selfiq/esp32_setup"
#define IDENTITY_NS "selfiq_node"

#define MQTT_CLIENT_ID_SIZE (sizeof("esp-") - 1U + MAC_BINARY_SIZE * 2U + TEXT_TERMINATOR_SIZE)
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

/* Comandos JSON: roi.set con 8 ROI ronda 700-1000 B. EVENT_BYTES (384) se
 * reserva para telemetria; un comando mas grande se descartaria en silencio. */
#define MQTT_CMD_BYTES 1536U
#define MQTT_RX_BUFFER_BYTES 2048U
#define MQTT_TASK_STACK_BYTES 8192U

/* Calibracion (E5.1): cola de comandos validados y consumidor unico.
 * E5.3 conecta el consumidor real con la tarea de camara, por lo que ya no
 * hay espera simulada. El macro se conserva en 0 para diagnosticos; si se
 * sube, solo retrasa la verificacion de contexto en el host. */
#define MQTT_CAL_QUEUE_LEN 1U
#define MQTT_CAL_TASK_STACK 4096U
#define MQTT_CAL_TASK_PRIORITY 4U
#define MQTT_CAL_STUB_HOLD_MS 0U

/* Call start/stop only from the connection owner (main), never MQTT callback. */
esp_err_t mqtt_register_start(const char *broker_ip, uint16_t broker_port);
void mqtt_network_stop(void);
bool mqtt_network_connected(void);
/* ESP_OK: accepted in bounded RAM queue, not yet durable. Caller handles errors.
 * Used by each sensor module, e.g. mqtt_queue_value("temperature", 23.5).
 * No UTC clock is invented: gateway attaches received_at. */
esp_err_t mqtt_queue_value(const char *event_type, double value);
/* Igual que mqtt_queue_value pero con un objeto `data` arbitrario (JSON).
 * data_json debe ser un objeto JSON valido, p. ej. "{\"roi_id\":\"...\",\"conf\":0.9}".
 * Se almacena tal cual dentro de "data". Maximo EVENT_BYTES-1 caracteres en total.
 * Usado por modulos que publican mas de un campo (ej. roi_occ). */
esp_err_t mqtt_queue_json(const char *event_type, const char *data_json);
/* Publica un ACK pequeno (roi_ack) en .../command/ack con QoS 1.
 * No usa la cola del outbox ni la RAM de telemetria. */
esp_err_t mqtt_publish_ack_json(const char *payload);
#endif