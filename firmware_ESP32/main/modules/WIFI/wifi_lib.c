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
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <nvs_flash.h>

/* librerias propias*/
#include "esp_wifi_types_generic.h"
#include "global.h"
#include "wifi_lib.h"

// varibales globales
EventGroupHandle_t g_wifi_event_group = NULL;

static const char *TAG = "wifi_lib";
static int s_retry_num = 0;
static uint8_t s_initialized = FALSE;
static esp_ip4_addr_t s_ip; // almacenamiento real de la IP

static void wifi_event_handler(void *args, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *d =
        (wifi_event_sta_disconnected_t *)event_data;

    // desconexion pedida por nosotros (esp_wifi_disconnect), no reintentar
    if (d->reason == WIFI_REASON_ASSOC_LEAVE) {
      return;
    }

    esp_wifi.connected = 0;

    if (s_retry_num < ESP_MAX_RETRY) {
      s_retry_num++;
      ESP_LOGW(TAG, "reintentando WIFI (%d/%d) reason=%d", s_retry_num,
               ESP_MAX_RETRY, d->reason);
      esp_wifi_connect();
    } else {
      ESP_LOGE(TAG, "no se pudo conectar con las credenciales guardadas");
      xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    s_ip = event->ip_info.ip; // copia, event_data no sobrevive al handler
    esp_wifi.ip = &s_ip;
    esp_wifi.connected = 1;
    s_retry_num = 0;
    ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&s_ip));
    xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}
/*
void wifi_init_sta(void) {
  if (s_initialized) {
    return;
  }

  g_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // las credenciales se persisten en flash (NVS, namespace nvs.net80211)
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  s_initialized = TRUE;
}*/

void wifi_init_sta(void) {
  if (s_initialized)
    return;

  g_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  /* RAM: la fuente de verdad es nuestra NVS custom con checksum */
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  s_initialized = TRUE;
}

// intenta conectar con lo guardado en flash y activa uno de los 3 bits:
//  WIFI_NO_CREDS_BIT -> no hay credenciales
//  WIFI_FAIL_BIT     -> hay credenciales pero no conecto (async, por el
//  handler) WIFI_CONNECTED_BIT-> hay credenciales y conecto (async, por el
//  handler)
/*void wifi_connect_saved(void) {
  xEventGroupClearBits(g_wifi_event_group,
                       WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT);
  s_retry_num = 0;
  esp_wifi.connected = 0;

  wifi_config_t saved = {0};
  if (esp_wifi_get_config(WIFI_IF_STA, &saved) != ESP_OK ||
      saved.sta.ssid[0] == '\0') {
    ESP_LOGW(TAG, "no hay credenciales guardadas en flash");
    xEventGroupSetBits(g_wifi_event_group, WIFI_NO_CREDS_BIT);
    return;
  }

  ESP_LOGI(TAG, "conectando a '%s' con credenciales guardadas", saved.sta.ssid);
  esp_wifi_disconnect(); // limpia intento previo (el handler lo ignora)
  ESP_ERROR_CHECK(esp_wifi_connect());
}*/

void wifi_connect_saved(void) {
  xEventGroupClearBits(g_wifi_event_group,
                       WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT);
  s_retry_num = 0;
  esp_wifi.connected = 0;

  if (load_cred(eWIFI_CRED) != ESP_OK) {
    ESP_LOGW(TAG, "sin credenciales validas en NVS custom");
    xEventGroupSetBits(g_wifi_event_group, WIFI_NO_CREDS_BIT);
    return;
  }

  wifi_config_t cfg = {0};
  cfg.sta.threshold.authmode =
      (strlen(esp_wifi.pswd) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
  strncpy((char *)cfg.sta.ssid, esp_wifi.ssid, sizeof(cfg.sta.ssid) - 1);
  strncpy((char *)cfg.sta.password, esp_wifi.pswd,
          sizeof(cfg.sta.password) - 1);

  if (esp_wifi_set_config(WIFI_IF_STA, &cfg) != ESP_OK) {
    xEventGroupSetBits(g_wifi_event_group, WIFI_NO_CREDS_BIT);
    return;
  }

  ESP_LOGI(TAG, "conectando a '%s'", esp_wifi.ssid);
  esp_wifi_disconnect();
  ESP_ERROR_CHECK(esp_wifi_connect());
}

// asigna ssid/pswd con el tamaño exacto, libera lo anterior
static esp_err_t wifi_store_credentials(const char *ssid, const char *pswd) {
  if (ssid == NULL || pswd == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t l_ssid = strlen(ssid);
  size_t l_pswd = strlen(pswd);

  // limites del stack: ssid 1..32, pswd 0 (red abierta) o 8..63
  if (l_ssid == 0 || l_ssid > SSID_LEN_MAX || l_pswd > PSWD_LEN_MAX ||
      (l_pswd > 0 && l_pswd < PSWD_LEN_MIN)) {
    return ESP_ERR_INVALID_ARG;
  }

  char *new_ssid = malloc(l_ssid + 1);
  char *new_pswd = malloc(l_pswd + 1);
  if (!new_ssid || !new_pswd) {
    free(new_ssid);
    free(new_pswd);
    return ESP_ERR_NO_MEM;
  }
  memcpy(new_ssid, ssid, l_ssid + 1);
  memcpy(new_pswd, pswd, l_pswd + 1);

  free(esp_wifi.ssid);
  free(esp_wifi.pswd);
  esp_wifi.ssid = new_ssid;
  esp_wifi.pswd = new_pswd;
  return ESP_OK;
}

// libera la copia temporal en heap (las credenciales ya quedaron en flash)
static void wifi_free_credentials(void) {
  free(esp_wifi.ssid);
  free(esp_wifi.pswd);
  esp_wifi.ssid = NULL;
  esp_wifi.pswd = NULL;
}

// provisioning: recibe credenciales nuevas, las guarda en flash e intenta
// conectar
/*esp_err_t wifi_set_credentials(const char *ssid, const char *pswd) {
  esp_err_t err = wifi_store_credentials(ssid, pswd);
  if (err != ESP_OK) {
    return err;
  }

  wifi_config_t cfg = {0};
  cfg.sta.threshold.authmode =
      (strlen(esp_wifi.pswd) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
  strncpy((char *)cfg.sta.ssid, esp_wifi.ssid, sizeof(cfg.sta.ssid) - 1);
  strncpy((char *)cfg.sta.password, esp_wifi.pswd,
          sizeof(cfg.sta.password) - 1);

  // el stack persiste esto en flash (WIFI_STORAGE_FLASH)
  err = esp_wifi_set_config(WIFI_IF_STA, &cfg);

  memset(&cfg, 0, sizeof(cfg));
  wifi_free_credentials();

  if (err != ESP_OK) {
    return err;
  }

  wifi_connect_saved(); // reinicia el flujo de los 3 bits
  return ESP_OK;
}*/

esp_err_t wifi_set_credentials(const char *ssid, const char *pswd) {
  esp_err_t err = wifi_store_credentials(ssid, pswd);
  if (err != ESP_OK)
    return err;

  wifi_config_t cfg = {0};
  cfg.sta.threshold.authmode =
      (strlen(esp_wifi.pswd) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
  strncpy((char *)cfg.sta.ssid, esp_wifi.ssid, sizeof(cfg.sta.ssid) - 1);
  strncpy((char *)cfg.sta.password, esp_wifi.pswd,
          sizeof(cfg.sta.password) - 1);

  err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  memset(&cfg, 0, sizeof(cfg));
  if (err != ESP_OK)
    return err;

  s_retry_num = 0;
  esp_wifi.connected = 0;
  xEventGroupClearBits(g_wifi_event_group,
                       WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT);
  esp_wifi_disconnect();
  return esp_wifi_connect();
}
