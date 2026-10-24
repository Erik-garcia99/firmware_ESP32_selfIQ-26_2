/* SelfIQ: alta automática ESP32 -> MQTT -> Raspberry -> REST -> UUID permanente. */
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modules/MQTT/mqtt_lib.h"
#include "modules/WIFI/wifi_lib.h"

#define NODE_TYPE "shelf_cam" 
#define NODE_MODULE "camera" 
#define MQTT_SETUP_TOPIC "selfiq/esp32_setup"
#define IDENTITY_NS "selfiq_node"

static const char *TAG = "selfiq_mqtt";
static esp_mqtt_client_handle_t s_client = NULL;
static char s_mac[18] = {0};
static char s_client_id[17] = {0};  // esp- + 12 caracteres hex
static char s_reply_topic[96] = {0};
static char s_esp_id[37] = {0};
static char s_branch_id[37] = {0};
static bool s_registered = false;
static bool s_reply_ready = false;
static int s_reply_sub_id = -1;
static bool s_mqtt_connected = false;
static TaskHandle_t s_retry_task = NULL;

static bool uuid_valid(const char *v) {
  if (strlen(v) != 36) return false;
  for (int i = 0; i < 36; i++) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (v[i] != '-') return false;
    } else if (!isxdigit((unsigned char)v[i])) return false;
  }
  return true;
}

static esp_err_t save_node_identity(void) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(IDENTITY_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) return err;
  err = nvs_set_str(h, "mac", s_mac);
  if (err == ESP_OK) err = nvs_set_str(h, "node_id", s_esp_id);
  if (err == ESP_OK) err = nvs_set_str(h, "branch_id", s_branch_id);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  return err;
}

static void send_registration(void) {
  if (!s_client || !s_mqtt_connected || !s_reply_ready || s_registered) return;
  char payload[96];
  int len = snprintf(payload, sizeof(payload),
                     "{\"mac\":\"%s\",\"node_type\":\"%s\"}", s_mac, NODE_TYPE);
  if (len < 0 || len >= (int)sizeof(payload)) return;
  int id = esp_mqtt_client_publish(s_client, MQTT_SETUP_TOPIC, payload, len, 1, 0);
  ESP_LOGI(TAG, "Solicitud MQTT: %s MAC=%s (msg_id=%d)", MQTT_SETUP_TOPIC, s_mac, id);
}

static void send_status_and_subscribe(void) {
  char topic[180];
  snprintf(topic, sizeof(topic), "selfiq/%s/%s/%s/status", s_branch_id, s_esp_id, NODE_MODULE);
  esp_mqtt_client_publish(s_client, topic, "online", 6, 1, 0);
  snprintf(topic, sizeof(topic), "selfiq/%s/%s/%s/command", s_branch_id, s_esp_id, NODE_MODULE);
  esp_mqtt_client_subscribe(s_client, topic, 1);
  ESP_LOGI(TAG, "ESP32 registrada: MAC=%s ID=%s SUCURSAL=%s", s_mac, s_esp_id, s_branch_id);
}

static void mqtt_event(void *args, esp_event_base_t base, int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      s_mqtt_connected = true;
      s_registered = false; // Verificar registro en backend cada reconexión (idempotente).
      s_reply_ready = false;
      s_reply_sub_id = esp_mqtt_client_subscribe(s_client, s_reply_topic, 1);
      ESP_LOGI(TAG, "MQTT conectado; esperando suscripcion de respuesta (msg_id=%d)", s_reply_sub_id);
      break;
    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "Suscripcion MQTT confirmada (msg_id=%d)", event->msg_id);
      if (s_reply_sub_id >= 0 && event->msg_id == s_reply_sub_id) {
        s_reply_ready = true;
        send_registration();
      }
      break;
    case MQTT_EVENT_DISCONNECTED:
      s_mqtt_connected = false;
      s_reply_ready = false;
      s_reply_sub_id = -1;
      s_registered = false;
      ESP_LOGW(TAG, "MQTT desconectado; el cliente reintentará automáticamente");
      break;
    case MQTT_EVENT_DATA: {
      if (event->topic_len == (int)strlen(s_reply_topic) &&
          memcmp(event->topic, s_reply_topic, event->topic_len) == 0) {
        /* 17 caracteres MAC + | + 36 UUID + | + 36 UUID = 91 bytes. */
        if (event->current_data_offset != 0 || event->total_data_len != 91 || event->data_len != 91) {
          ESP_LOGW(TAG, "Respuesta de registro fragmentada o de tamaño incorrecto");
          break;
        }
        char response[92];
        memcpy(response, event->data, 91);
        response[91] = '\0';
        if (response[17] != '|' || response[54] != '|' ||
            strncmp(response, s_mac, 17) != 0) break;
        char esp_id[37], branch_id[37];
        memcpy(esp_id, response + 18, 36); esp_id[36] = '\0';
        memcpy(branch_id, response + 55, 36); branch_id[36] = '\0';
        if (!uuid_valid(esp_id) || !uuid_valid(branch_id)) {
          ESP_LOGE(TAG, "Respuesta con IDs inválidos");
          break;
        }
        memcpy(s_esp_id, esp_id, sizeof(s_esp_id));
        memcpy(s_branch_id, branch_id, sizeof(s_branch_id));
        if (save_node_identity() != ESP_OK) {
          ESP_LOGE(TAG, "No se pudo guardar identidad de nodo en NVS");
          break;
        }
        s_registered = true;
        send_status_and_subscribe();
      } else if (event->topic_len > 0) {
        ESP_LOGI(TAG, "Mensaje en tópico del sistema (comando; ejecución pendiente)");
      }
      break;
    }
    case MQTT_EVENT_ERROR:
      ESP_LOGW(TAG, "Error MQTT (revisar broker/red)");
      break;
    default:
      break;
  }
}

static void registration_retry(void *param) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(30000));
    if (s_mqtt_connected && !s_registered) {
      if (!s_reply_ready)
        s_reply_sub_id = esp_mqtt_client_subscribe(s_client, s_reply_topic, 1);
      else
        send_registration();
    }
  }
}

esp_err_t mqtt_register_start(const char *broker_ip, uint16_t broker_port) {
  if (!broker_ip || !*broker_ip || broker_port == 0) return ESP_ERR_INVALID_ARG;
  if (s_client != NULL) return ESP_OK;
  uint8_t mac[6];
  esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (ret != ESP_OK) return ret;
  snprintf(s_mac, sizeof(s_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(s_client_id, sizeof(s_client_id), "esp-%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(s_reply_topic, sizeof(s_reply_topic),
           "selfiq/esp32_setup/response/%s", s_client_id);

  char uri[64];
  snprintf(uri, sizeof(uri), "mqtt://%s:%u", broker_ip, (unsigned)broker_port);
  esp_mqtt_client_config_t cfg = {
      .broker.address.uri = uri,
      .credentials.client_id = s_client_id,
      .session.keepalive = 45,
      .network.reconnect_timeout_ms = 5000,
  };
  s_client = esp_mqtt_client_init(&cfg);
  if (!s_client) return ESP_FAIL;
  ret = esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event, NULL);
  if (ret == ESP_OK) ret = esp_mqtt_client_start(s_client);
  if (ret != ESP_OK) {
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    return ret;
  }
  BaseType_t started = xTaskCreate(registration_retry, "mqtt_register", 3072,
                                   NULL, 4, &s_retry_task);
  if (started != pdPASS) ESP_LOGW(TAG, "Sin tarea reintentos MQTT (se registra al conectar)");
  ESP_LOGI(TAG, "MQTT iniciando broker=%s puerto=%u client_id=%s", broker_ip,
           (unsigned)broker_port, s_client_id);
  return ESP_OK;
}
