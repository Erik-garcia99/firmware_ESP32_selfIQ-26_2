/* *********************************************
 *              selfIQ 2026-2
 *
 * @autor Erik Garcia Chavez
 * @date octuber 2026
 * @assginature : proyecto de carrera
 * @teacher : Jose Isabel Garcia Rocha
 *
 ************************************************* */

// liberias estanadar
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global.h"
#include "modules/WIFI/wifi_lib.h"
#include "modules/TCP/tcp_lib.h"
#include "modules/MQTT/mqtt_lib.h"

/* Globals kept for the existing TCP library; recovery exchanges below have a
 * private socket and never run recv_task concurrently on that socket. */
QueueHandle_t tcp_rx_queue;
EventGroupHandle_t g_tcp_event_group;
TaskHandle_t recv_task_handle, recv_proccess_handle;
TYPE_FORMAT_REQUEST_T format_reques;
TYPE_SEND_INFO_T send_info;
TYPE_ESP_WIFI_T esp_wifi;
TYPE_TCP_CLIENT_T tcp_client = {.sock = -1};
TYPE_OP_TYPE_T eOp_type;
ACTION_T eAction;
TYPE_FRAME_T eType_frame;
DATA_FIELD_T eData_field_t;
// igual aqui, se que la ip es de 16 caracteres, y se que siempre era asi, no cambiara, en el caso de IPv4, pero ahora si ahora manerjaremos IPv6, que ond? ya cambia la cosas  no, MACROSSS 
char g_rpi_ip[IPV4_TEXT_SIZE];


static const char *TAG = "MAIN";

static uint64_t legacy_sum(const char *key, const void *anchor, uint8_t type) {
  uint64_t sum = 0;
  while (*key) sum += (uint8_t)*key++;
  if (type == eTCP_CRED) return sum + *(const uint16_t *)anchor;
  const char *p = anchor;
  while (*p){
    sum += (uint8_t)*p++;
  } 
  return sum;
}

/* IEEE CRC32, format marker in upper bits. Legacy sums remain readable. */
// los numeros magicos no deben de estar asi, como en esta comparacion en el for, macro, varibale o constante no magic numbers. 
static uint32_t crc_byte(uint32_t crc, uint8_t byte) {
  crc ^= byte;
  for (int n = 0; n < CRC32_BITS_PER_BYTE; ++n) crc = (crc >> 1) ^ (CRC32_POLYNOMIAL & (0U - (crc & 1U)));
  return crc;
}

/** que onda con este ehcksum explica que tantas numeros tan largos y especificos  */
uint64_t checksum(const char *key, const void *anchor, uint8_t type) {
  uint32_t crc = crc_byte(CRC32_INITIAL_VALUE, type);
  do { 
    crc = crc_byte(crc, (uint8_t)*key); 
  }while (*key++);
  
  if (type == eTCP_CRED) {
    uint16_t port = *(const uint16_t *)anchor;
    crc = crc_byte(crc, port >> CRC32_PORT_HIGH_SHIFT); crc = crc_byte(crc, port & CRC32_PORT_LOW_MASK);
  } else {
    const char *p = anchor;
    do { 
      crc = crc_byte(crc, (uint8_t)*p); 
    } while (*p++);
  }

  return (CRC32_FORMAT_MARKER << CRC32_FORMAT_SHIFT) | (crc ^ CRC32_FINAL_XOR);
}

static bool checksum_valid(const char *key, const void *anchor, uint8_t type, uint64_t saved) {
  return saved == checksum(key, anchor, type) ||
         (saved < CRC32_LEGACY_VALUE_LIMIT && saved == legacy_sum(key, anchor, type));
}

static bool wifi_wait(void) {
  EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
      WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT,
      pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
  return (bits & WIFI_CONNECTED_BIT) && esp_wifi.connected;
}

/***
 * 
 * @ de donde saca los numeros que onda?, que esta haceindo esta coasa?
 * 
 */
static bool fetch_wifi(char *ssid, char *password) {
  uint8_t data[TCP_RESPONSE_BUFFER_SIZE];

  int size = tcp_exchange(TCP_WIFI_REQUEST_OP, data, sizeof(data));
  // de donde se saca ese 0? si indicas el ID, se supone que primero es el ID, en el frame en la aprte de data, primero ID del SSID, despues el len el tamanio del dato de LEN 

  //y lo mismo para password. 
  //entnces este 16 y el 6 de donde se lo saca? 
  //estas poneind al reves el frame como te lo mande? 
  if (size < TCP_WIFI_MIN_RESPONSE_LEN || data[0] != TCP_WIFI_RESPONSE_OP || data[1] != eDataSSID) return false;

  unsigned sl = data[2], pos = TCP_WIFI_RESPONSE_FIELD_OFFSET + sl;

  if (!sl || sl > SSID_LEN_MAX || pos + TCP_WIFI_FIELD_METADATA_BYTES > (unsigned)size || data[pos] != eDataPSWD) return false;
  unsigned pl = data[pos + 1];

  if (pl < PSWD_LEN_MIN || pl > PSWD_LEN_MAX || pos + TCP_WIFI_FIELD_METADATA_BYTES + pl != (unsigned)size ||
      memchr(data + TCP_WIFI_RESPONSE_FIELD_OFFSET, 0, sl) || memchr(data + pos + TCP_WIFI_FIELD_METADATA_BYTES, 0, pl)){
        return false;
      } 

  memcpy(ssid, data + TCP_WIFI_RESPONSE_FIELD_OFFSET, sl); ssid[sl] = 0;
  memcpy(password, data + pos + TCP_WIFI_FIELD_METADATA_BYTES, pl); password[pl] = 0;
  
  return strcmp(ssid, SSID_RPI) != 0;
}


static bool discover_broker(char *ip, uint16_t *port) {
  const char *hosts[] = {"selfiq-gateway", "raspberrypi", "rackiq-gateway"};
  if (mdns_init() != ESP_OK) return false;
  bool found = false;
  for (int i = 0; i < (int)(sizeof(hosts) / sizeof(hosts[0])) && !found; ++i) {
    esp_ip4_addr_t addr = {0};
    if (mdns_query_a(hosts[i], MDNS_QUERY_TIMEOUT_MS, &addr) == ESP_OK && addr.addr) {
      snprintf(ip, IPV4_TEXT_SIZE, IPSTR, IP2STR(&addr)); found = true;
    }
  }
  if (!found) {
    mdns_result_t *results = NULL;
    if (mdns_query_srv("selfiq-broker", "_mqtt", "_tcp", MDNS_QUERY_TIMEOUT_MS, &results) == ESP_OK) {
      for (mdns_result_t *r = results; r && !found; r = r->next) {
        if (r->addr && r->addr->addr.type == ESP_IPADDR_TYPE_V4) {
          snprintf(ip, IPV4_TEXT_SIZE, IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
          if (r->port) *port = r->port;
          found = true;
        }
      }
      mdns_query_results_free(results);
    }
  }
  mdns_free();
  return found;
}

void app_main(void) {
  /* Do not erase NVS automatically: identity/pending configuration matter. */
  esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[INTEGRITY] NVS_INIT_FAILED %s; requiere reparacion, no se borra", esp_err_to_name(err));
    return;
  }

  /* Camara independiente de las comunicaciones: una sola inicializacion. */
  ret = camera_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camara no disponible: %s", esp_err_to_name(ret));
    /* El provisioning WiFi/TCP puede continuar aunque falle la camara. */
  }

  wifi_init_sta();
  bool on_ap = false;
  unsigned rounds = 0;
  TickType_t last_probe = 0, last_mqtt = 0;
  while (1) {
    if (!esp_wifi.connected) {
      mqtt_network_stop();
      if (!on_ap && rounds < WIFI_NETWORK_ROUNDS) {
        ESP_LOGW(TAG, "[NETWORK] ronda %u/4, hasta 5 intentos", rounds + 1);
        ++rounds;
        wifi_connect_saved();
        if (wifi_wait()){
           rounds = 0; last_mqtt = 0; 
           continue; 
          }
        if(xEventGroupGetBits(g_wifi_event_group) & WIFI_NO_CREDS_BIT){
          rounds = WIFI_NETWORK_ROUNDS;
        } 
        if (rounds < WIFI_NETWORK_ROUNDS) { 
          vTaskDelay(pdMS_TO_TICKS(WIFI_ROUND_RETRY_MS)); 
          continue; 
        }
        ESP_LOGE(TAG, "[NETWORK] WIFI_RETRY_EXHAUSTED; buscando AP, sin borrar ni reiniciar");
      }
      if (wifi_set_credentials(SSID_RPI, PSWD_RPI) == ESP_OK && wifi_wait()) {
        on_ap = true; last_probe = 0; last_mqtt = 0;
        ESP_LOGI(TAG, "[NETWORK] LOCAL_AP_READY; broker %s:1883", RPI_IP_SETUP);
      } else {
        /* If gateway is absent, occasionally try saved router again. */
        on_ap = false; rounds = 0;
        ESP_LOGW(TAG, "[NETWORK] AP_UNREACHABLE; nuevo ciclo en 60s");
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECOVER_MS));
      }
      continue;
    }
    TickType_t now = xTaskGetTickCount();
    if (!mqtt_network_connected() && (!last_mqtt || now - last_mqtt >= pdMS_TO_TICKS(WIFI_RECOVER_MS))) {
      last_mqtt = now;
      uint16_t port = MQTT_DEFAULT_PORT;
      char ip[IPV4_TEXT_SIZE] = {0};
      if (on_ap) snprintf(ip, sizeof(ip), "%s", RPI_IP_SETUP);
      if (on_ap || discover_broker(ip, &port)) {
        mqtt_register_start(ip, port); // can change broker without losing queued events
      } else ESP_LOGW(TAG, "[MQTT] BROKER_NOT_FOUND; nuevo intento en 60s");
    }
    if (on_ap && (!last_probe || now - last_probe >= pdMS_TO_TICKS(TCP_UPDATE_TIMEOUT_MS))) {
      last_probe = now;
      
      uint8_t state[TCP_STATUS_BUFFER_SIZE];
      // de dide sakca este 41
      int n = tcp_exchange(TCP_STATUS_REQUEST_OP, state, sizeof(state));
      //de dide el 42?

      if (n == TCP_STATUS_RESPONSE_LEN && state[0] == TCP_STATUS_RESPONSE_OP && state[1] == TCP_STATUS_RESPONSE_VERSION) {
        ESP_LOGI(TAG, "[TCP] wifi=%u backend=%u mqtt=%u storage=%u action=%u",
                 state[2], state[3], state[4], state[5], state[6]);
        if (state[6] == TCP_STATUS_WIFI_UPDATE_ACTION) {
          char ssid[SSID_LEN_MAX + TEXT_TERMINATOR_SIZE], password[PSWD_LEN_MAX + TEXT_TERMINATOR_SIZE];
          if (fetch_wifi(ssid, password)) {
            mqtt_network_stop();
            if (wifi_set_credentials(ssid, password) == ESP_OK && wifi_wait()) {
              err = save_credentials(eWIFI_CRED); // commit ONLY after connection succeeds
              ESP_LOGI(TAG, "[NETWORK] STA_RECOVERED; persistencia=%s", esp_err_to_name(err));
              on_ap = false; rounds = 0; last_mqtt = 0;
            } else {
              ESP_LOGW(TAG, "[NETWORK] STA_TRIAL_FAILED; regresando al AP");
              wifi_set_credentials(SSID_RPI, PSWD_RPI);
              wifi_wait();
              last_mqtt = 0;
            }
            memset(password, 0, sizeof(password));
          }
        }
      } else ESP_LOGW(TAG, "[TCP] STATUS_UNAVAILABLE; MQTT local continua, sin reinicio");
    }
    vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
  }
}

esp_err_t save_credentials(uint8_t identifier) {
  nvs_handle_t h;
  esp_err_t ret;

  ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (ret != ESP_OK)
    return ret;

  if (identifier == eTCP_CRED) {
    if (tcp_client.host_ip == NULL) {
      nvs_close(h);
      return ESP_ERR_INVALID_ARG;
    }

    uint16_t port = tcp_client.host_port;
    uint64_t sum = checksum(tcp_client.host_ip, &port, eTCP_CRED);

    ret = nvs_set_str(h, KEY_TCP_IP, tcp_client.host_ip);
    if (ret == ESP_OK)
      ret = nvs_set_u16(h, KEY_TCP_PORT, port);
    if (ret == ESP_OK)
      ret = nvs_set_u64(h, KEY_TCP_SUM, sum);
    if (ret == ESP_OK)
      ret = nvs_commit(h);
    nvs_close(h);
    return ret;
  }

  if (identifier == eWIFI_CRED) {
    if (esp_wifi.ssid == NULL || esp_wifi.pswd == NULL) {
      nvs_close(h);
      return ESP_ERR_INVALID_ARG;
    }
    uint64_t sum = checksum(esp_wifi.ssid, esp_wifi.pswd, eWIFI_CRED);

    ret = nvs_set_str(h, KEY_WIFI_SSID, esp_wifi.ssid);
    if (ret == ESP_OK)
      ret = nvs_set_str(h, KEY_WIFI_PSWD, esp_wifi.pswd);
    if (ret == ESP_OK)
      ret = nvs_set_u64(h, KEY_WIFI_SUM, sum);
    if (ret == ESP_OK)
      ret = nvs_commit(h);
    nvs_close(h);
    return ret;
  }

  nvs_close(h);
  return ESP_FAIL;
}

esp_err_t load_cred(uint8_t identifier) {

  nvs_handle_t h;
  esp_err_t ret;
  ret = nvs_open(NVS_NS, NVS_READONLY, &h);
  if (ret != ESP_OK) {
    return ret;
  }

  if (identifier == eTCP_CRED) {
    char *ip = NULL;
    size_t len = 0;
    uint16_t port = 0;
    uint64_t saved_sum = 0;
    // con buffer NULL, nvs_get_str solo devuelve el tamanio (incluye el '\0')
    ret = nvs_get_str(h, KEY_TCP_IP, NULL, &len);
    if (ret == ESP_OK && (len < TCP_IP_NVS_MIN_SIZE || len > IPV4_TEXT_SIZE)) ret = ESP_ERR_INVALID_SIZE;
    if (ret == ESP_OK) {
      ip = malloc(len);
      if (ip == NULL) {
        ret = ESP_ERR_NO_MEM;
      }
    }
    if (ret == ESP_OK) {
      ret = nvs_get_str(h, KEY_TCP_IP, ip, &len);
    }
    if (ret == ESP_OK) {
      ret = nvs_get_u16(h, KEY_TCP_PORT, &port);
    }
    if (ret == ESP_OK) {
      ret = nvs_get_u64(h, KEY_TCP_SUM, &saved_sum);
    }
    nvs_close(h);

    // recalculamos la suma con lo que leimos y la comparamos con la guardada
    if (ret == ESP_OK && !checksum_valid(ip, &port, eTCP_CRED, saved_sum)) {
      ret = ESP_ERR_INVALID_CRC;
    }

    if (ret != ESP_OK) {
      free(ip);
      return ret;
    }

    free(tcp_client.host_ip);
    tcp_client.host_ip = ip;
    tcp_client.host_port = port;
    if (saved_sum < CRC32_LEGACY_VALUE_LIMIT) {
      if (save_credentials(eTCP_CRED) != ESP_OK)
        ESP_LOGW(TAG, "[INTEGRITY] migracion CRC TCP pendiente");
    }
    return ESP_OK;
  }
  if (identifier == eWIFI_CRED) {
    char *ssid = NULL, *pswd = NULL;
    size_t len_s = 0, len_p = 0;
    uint64_t saved_sum = 0;

    ret = nvs_get_str(h, KEY_WIFI_SSID, NULL, &len_s);
    if (ret == ESP_OK && (len_s < 2 || len_s > SSID_LEN_MAX + TEXT_TERMINATOR_SIZE)) ret = ESP_ERR_INVALID_SIZE;
    if (ret == ESP_OK) {
      ssid = malloc(len_s);
      if (!ssid)
        ret = ESP_ERR_NO_MEM;
    }
    if (ret == ESP_OK)
      ret = nvs_get_str(h, KEY_WIFI_SSID, ssid, &len_s);
    if (ret == ESP_OK)
      ret = nvs_get_str(h, KEY_WIFI_PSWD, NULL, &len_p);
    if (ret == ESP_OK && (len_p < PSWD_LEN_MIN + TEXT_TERMINATOR_SIZE || len_p > PSWD_LEN_MAX + TEXT_TERMINATOR_SIZE)) ret = ESP_ERR_INVALID_SIZE;
    if (ret == ESP_OK) {
      pswd = malloc(len_p);
      if (!pswd)
        ret = ESP_ERR_NO_MEM;
    }
    if (ret == ESP_OK)
      ret = nvs_get_str(h, KEY_WIFI_PSWD, pswd, &len_p);
    if (ret == ESP_OK)
      ret = nvs_get_u64(h, KEY_WIFI_SUM, &saved_sum);
    nvs_close(h);

    if (ret == ESP_OK && !checksum_valid(ssid, pswd, eWIFI_CRED, saved_sum))
      ret = ESP_ERR_INVALID_CRC;
    if (ret != ESP_OK) {
      free(ssid);
      free(pswd);
      return ret;
    }

    free(esp_wifi.ssid);
    free(esp_wifi.pswd);
    esp_wifi.ssid = ssid;
    esp_wifi.pswd = pswd;
    esp_wifi.connected = 0;
    if (saved_sum < CRC32_LEGACY_VALUE_LIMIT) {
      if (save_credentials(eWIFI_CRED) != ESP_OK)
        ESP_LOGW(TAG, "[INTEGRITY] migracion CRC WIFI pendiente");
    }
    return ESP_OK;
  }
  // algo parecido sera con wifi, mas o menos.
  nvs_close(h);
  return ESP_FAIL;
}

esp_err_t erease_cred_nvs(uint8_t identifier) {
  nvs_handle_t h;
  esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (ret != ESP_OK)
    return ret;

  const char *keys_wifi[] = {KEY_WIFI_SSID, KEY_WIFI_PSWD, KEY_WIFI_SUM};
  const char *keys_tcp[] = {KEY_TCP_IP, KEY_TCP_PORT, KEY_TCP_SUM};
  const char **keys = NULL;

  if (identifier == eWIFI_CRED)
    keys = keys_wifi;
  else if (identifier == eTCP_CRED)
    keys = keys_tcp;
  else {
    nvs_close(h);
    return ESP_ERR_INVALID_ARG;
  }

  for (int i = 0; i < (int)(sizeof(keys_wifi) / sizeof(keys_wifi[0])); ++i) {
    ret = nvs_erase_key(h, keys[i]);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
      nvs_close(h);
      return ret;
    }
  }
  ret = nvs_commit(h);
  nvs_close(h);
  return ret;
}

