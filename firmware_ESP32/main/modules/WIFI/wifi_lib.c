/* *********************************************
 *              selfIQ 2026-2
 *
 * @autor Erik Garcia Chavez
 * @date octuber 2026
 * @assginature : proyecto de carrera
 * @teacher : Jose Isabel Garcia Rocha
 *
 ************************************************* */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "freertos/task.h"
#include "global.h"
#include "wifi_lib.h"

EventGroupHandle_t g_wifi_event_group;
static const char *TAG = "wifi_lib";
static esp_ip4_addr_t s_ip;
static int s_retry_num;
static bool s_initialized;
static volatile bool s_switching;
#define WIFI_STOPPED_BIT BIT4

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *event = data;
    esp_wifi.connected = 0;
    xEventGroupClearBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
    xEventGroupSetBits(g_wifi_event_group, WIFI_STOPPED_BIT);
    if (s_switching || event->reason == WIFI_REASON_ASSOC_LEAVE) return;
    if (s_retry_num < ESP_MAX_RETRY) {
      ++s_retry_num;
      ESP_LOGW(TAG, "[NETWORK] intento %d/%u reason=%d", s_retry_num + 1, (unsigned)WIFI_TOTAL_CONNECT_ATTEMPTS, event->reason);
      if (esp_wifi_connect() == ESP_OK) return;
    }
    xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP && !s_switching) {
    s_ip = ((ip_event_got_ip_t *)data)->ip_info.ip;
    esp_wifi.ip = &s_ip;
    esp_wifi.connected = 1;
    s_retry_num = 0;
    xEventGroupClearBits(g_wifi_event_group, WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT);
    xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "[NETWORK] WIFI_READY IP=" IPSTR, IP2STR(&s_ip));
  }
}

void wifi_init_sta(void) {
  if (s_initialized) return;
  g_wifi_event_group = xEventGroupCreate();
  configASSERT(g_wifi_event_group);
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  s_initialized = true;
}

esp_err_t wifi_set_credentials(const char *ssid, const char *password) {
  if(!ssid || !password || !strlen(ssid) || strlen(ssid) > SSID_LEN_MAX ||
      strlen(password) < PSWD_LEN_MIN || strlen(password) > PSWD_LEN_MAX){
        return ESP_ERR_INVALID_ARG;
    } 
  
  char *new_ssid = strdup(ssid), *new_password = strdup(password);
  
  if (!new_ssid || !new_password) {
    free(new_ssid); free(new_password); return ESP_ERR_NO_MEM;
  }
  
  s_switching = true;
  
  xEventGroupClearBits(g_wifi_event_group, WIFI_STOPPED_BIT);
  
  if (esp_wifi_disconnect() == ESP_OK){
    xEventGroupWaitBits(g_wifi_event_group, WIFI_STOPPED_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(WIFI_DISCONNECT_TIMEOUT_MS));
  }
  
  esp_wifi.connected = 0;
  wifi_config_t cfg = {0};
  
  memcpy(cfg.sta.ssid, new_ssid, strlen(new_ssid)); // supports 32-byte SSID
  memcpy(cfg.sta.password, new_password, strlen(new_password));
  
  cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  memset(&cfg, 0, sizeof(cfg));
  xEventGroupClearBits(g_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT);
  s_retry_num = 0;
  s_switching = false;
  
  if (err != ESP_OK) {
    free(new_ssid); free(new_password);
    xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
    return err;
  }
  free(esp_wifi.ssid); free(esp_wifi.pswd);
  esp_wifi.ssid = new_ssid; esp_wifi.pswd = new_password;
  err = esp_wifi_connect();
  if (err != ESP_OK) xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
  return err;
}

void wifi_connect_saved(void) {
  xEventGroupClearBits(
      g_wifi_event_group,
      WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT);

  esp_err_t err = load_cred(eWIFI_CRED);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "[INTEGRITY] load_cred(WIFI) fallo: %s (0x%x)",
        esp_err_to_name(err),
        (unsigned int)err);

    xEventGroupSetBits(g_wifi_event_group, WIFI_NO_CREDS_BIT);
    return;
  }

  ESP_LOGI(TAG, "[NETWORK] credenciales cargadas; conectando a '%s'",
           esp_wifi.ssid);

  err = wifi_set_credentials(esp_wifi.ssid, esp_wifi.pswd);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "[NETWORK] wifi_set_credentials fallo: %s (0x%x)",
        esp_err_to_name(err),
        (unsigned int)err);

    xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
  }
}
