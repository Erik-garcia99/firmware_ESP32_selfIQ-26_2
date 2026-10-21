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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

// logs
#include "esp_err.h"
#include "esp_log.h"
// recursos ESP
#include "esp_mac.h"
#include "esp_system.h"

// flash
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "mdns.h"

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// librerias propias
#include "global.h"
#include "modules/TCP/tcp_lib.h"
#include "modules/WIFI/wifi_lib.h"
#include "modules/MQTT/mqtt_lib.h"
#include "portmacro.h"

//+++++++++++++++++++++++++++++++++++ colas

QueueHandle_t
    tcp_rx_queue; // cola que usaremos como swap entre el modulo de TCP que
                  // recibe el frame y main que desempaqueta la infromacion.

//++++++++++++++++++++++++++++++++++ grupo de eventos
EventGroupHandle_t g_tcp_event_group;

//++++++++++++++++++++++++++++++++ varibales globales

static const char *TAG = "MAIN: ";
TaskHandle_t recv_task_handle = NULL;
TaskHandle_t recv_proccess_handle = NULL;
char g_rpi_ip[16] = {0};

//++++++++++++++++++++++++++++++++ estrucutras
TYPE_FORMAT_REQUEST_T format_reques = {0};
TYPE_SEND_INFO_T send_info = {0};
TYPE_ESP_WIFI_T esp_wifi = {0};
TYPE_TCP_CLIENT_T tcp_client = {.sock = -1};

//++++++++++++++++++++++++++++++++++ enums

TYPE_OP_TYPE_T eOp_type;
ACTION_T eAction;
TYPE_FRAME_T eType_frame;
DATA_FIELD_T eData_field_t;

//+++++++++++++++++++++++++++++++funciones

/*********************************/
/*guardamos un checksum, para identificar la integridad de los datos */
/*********************************/
/*
 * @brief funcion que se encarga de actualiuzar las credencuales.
 *
 * */
esp_err_t update_setup_cred(char *key, void *anchor, uint8_t identifier);

/**
 * @brief levanta el tcp client.
 */

esp_err_t setup_tcp(void);

/*
 * @brief funcion utilizado para inicliazar con credeniclaes por defecto para
 * relizar la comunicacoin con la RPI por medio de socket TCP, para relizar el
 * setup con las credencilaes de la red a utilizar.
 *
 *
 *
 * */
static esp_err_t setup_credentials_default(uint8_t identifier);

static esp_err_t parse_and_apply_wifi_creds(TYPE_FORMAT_REQUEST_T *frame,
                                            uint8_t *offset);

static esp_err_t connect_to_rpi_ap(void);

//++++++++++++++++++++++++++++++++ tareas

/**
 * @param tarea encargada de decifrar, parsear y taratr los dato recbidisos por
 * el protocolo TCP
 *
 * */

void tcp_proccess_task(void *params);

void task_wifi_supervisor(void *params);
void task_broker_discovery(void *params);

/*
void app_main(void) {

  // variables
  esp_err_t ret;
  EventBits_t bits = 0;
  //  varibale que indica las veces que se intenera aparte de las 5 de la propia
  //  librerbia antes de reinicar la ESP tratnado de limpar posibles errores
  static uint8_t global_attempts = 0;
  // inicalizacion de varibales
  tcp_rx_queue = xQueueCreate(10, sizeof(TYPE_FORMAT_REQUEST_T *));

  // incializacion de flash para cargar o escribir en flash
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // WIFI: primero intenta con las credenciales guardadas en flash
  wifi_init_sta();
  xTaskCreate(task_wifi_supervisor, "wifi_sup", 3072, NULL, 4, NULL);

  wifi_connect_saved();

  while (1) {
    // primero se intenta conectar con las credenclaes que puede estar guardasd
    // en flash

    bits = xEventGroupWaitBits(g_wifi_event_group,
                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT |
                                   WIFI_NO_CREDS_BIT,
                               pdTRUE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGI(TAG, "WIFI conectado");
      break;
      // si me pude conectar al WIFI el proximo paso es conectarme a TCP
    } else if (bits & WIFI_NO_CREDS_BIT) {

      ESP_LOGW(TAG,
               "imposble conectarse, no hay crenciales o fallo al intentar");
      // tenemos 2 opciones
       realmente la esp32 viene vacia y no tiene crencuales guardadas --> por
       * lo que lo mas porabale es que la red ed la rasberry este levantada al
       * igual que le TCP_server, por lo que por este no hay problema
       *
       * puede que las credenicales esten bien pero hay una falla en internet,
       *
       * en otro caso tiene credennciales guardadas, en este caso, pueden ser
       * uno de los sigueintes casos :
       *
       * la red esta caida.
       * se cambio las credenciales de red
       * se corrompieron las credncieles
       *
       * --> al final cada ESP con MQTT tendra que enviar un keep_alive,
       * entpnces si no se recibe un keep_alive en un lapso de 5 a 10 minutos se
       * va a concluir que algo paso con la conexion en ese esp32
       *
       * por lo que indicara en la APP se se incie comunicacion, es cunado se
       * levanta TCP, y tiene que inciar una comnucacion entre la rasberry y la
       * esp32.
       *
       *
      do {
        ret = setup_credentials_default(eWIFI_CRED);

        if (global_attempts >= 4 && ret != ESP_OK) {
          // ya lo habremos borrado entonces lo que queda es reinciar
          ESP_LOGE(TAG, "no se pudo conectar a la AP de la RPi, reiniciando");
          esp_restart();
        }
        global_attempts++;
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "reintentando conexion a AP RPi en 60 s");
          // vTaskDelay(pdMS_TO_TICKS(60000));
        }

      } while (ret != ESP_OK);

      global_attempts = 0;
      ESP_LOGI(TAG, "conectado a AP '%s'", SSID_RPI);

      // si se pudo conctar ahora vamos a conectarnos al TCP SERVER
      do {
        ret = setup_credentials_default(eTCP_CRED);
        if (global_attempts > 4 && ret != ESP_OK) {
          // ya lo habremos borrado entonces lo que queda es reinciar
          ESP_LOGE(TAG, "no se pudo conectar al TCP de la RPi, reiniciando");
          esp_restart();
        }
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "reintentando TCP a RPi en 60 s");
          // vTaskDelay(pdMS_TO_TICKS(60000));
        }
        global_attempts++;
      } while (ret != ESP_OK);

      global_attempts = 0;
      ESP_LOGI(TAG,
               "TCP establecido con la RPi, esperando credenciales WiFi...");

      bits = xEventGroupWaitBits(g_tcp_event_group,
                                 TCP_WIFI_UPDATE | TCP_DISCONNECTED, pdTRUE,
                                 pdFALSE, portMAX_DELAY);

      if (bits & TCP_WIFI_UPDATE) {
        // ya se actualizaron las credeniclaes y se establecio la conexion
        // cunado llegue a este punto es porque ya estamos conectados al WIFI
        // del local o casa final en donde estaran conecado todo lo demas.

        ESP_LOGI(TAG, "creds WiFi recibidas, reconectando a red normal...");

        vTaskDelay(pdMS_TO_TICKS(500));  deja que muera recv_task
        // entonces como ultimo las tareas de TCP no las necesitamos por ahora

        vTaskDelete(recv_task_handle);
        // vTaskDelete(recv_proccess_handle);

        wifi_connect_saved();
        xEventGroupWaitBits(g_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE,
                            pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "en red normal, iniciando mDNS discovery");
        xTaskCreate(task_broker_discovery, "broker_disc", 4096, NULL, 5, NULL);
      } else if (bits & TCP_DISCONNECTED) {
        // el servidor cerro la conexion, deberia de estar intentando volvernos
        // a conectar.
      }

      // setup_tcp();

      // primero debemos de inciar sesion con el wifi de la rasberry
      // ret = update_setup_cred(SSID_RPI, PSWD_RPI, eWIFI_CRED);

      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ERROR al establecer credeniclaes WIFI de RPI: %s",
                 esp_err_to_name(ret));
      }

      wifi_init_sta();
      wifi_connect_saved();
      // en este caso no hay cedeniclaes, por lo que empezamos a levantar todo
      // para reliazar la peticion de credenciales

      // bits = xEventGroupWaitBits(g_wifi_event_group,
      // WIFI_FAIL_BIT | WIFI_CONNECTED_BIT, pdTRUE,
      // pdFALSE, portMAX_DELAY);

      // si falla que me indicara, posiblemnete la
      // red aun no esta levantada, por lo que lo que
      // haremos es limpiar las credenicales. que
      // hemos guardado y esperar 1 minuto mas para
      // darle tiempo a la RPI en levantar los
      // servicios.

      //@NOTE si, entonces igual aqui tendre otros 4
      // intentos, lo que da un
      // total de 20 inentos tanto para WIFI como
      // para TCP de poder conectarse, de no hacerlo
      // algo paso, y vamos a reinicar la rasberry
      if (bits & WIFI_FAIL_BIT) {
        if (global_attempts > 4) {
          ret = erease_cred_nvs(eWIFI_CRED);
          global_attempts = 0;
        }

        ret = erease_cred_nvs(eWIFI_CRED);
        global_attempts++;
        if (ret != ESP_OK) {
          ESP_LOGE(TAG,
                   "tipo de error al borrar "
                   "credenucales: %s",
                   esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
        continue;
      }
      // en otro caso se conecto entonces emepzamos
      // la conexion con TCP ahora vamos a actualizar
      // las credenicaeles de TCP
      // ret = update_setup_cred(RPI_IP_SETUP, &RPI_PORT_SETUP, eTCP_CRED);

      // inicamos el socket y la comunicacion con el
      // servidor.
      // setup_tcp();

      // continue;
    } else if (bits & WIFI_FAIL_BIT) {
      **
       * en este pude ser que las crecenicales hayan cambiado
       * o puede que no haya red, por lo quedebemos de levantar el servicio en
       * local o puede que se hayan corrompide en algun reinciio repentino. por
       * lo que sera necesario despues pedirlas.
       *
       *
       *
      ESP_LOGE(TAG, "fallo de conexion WiFi (el supervisor se encarga)");
    }
  }
}*/

void app_main(void) {
  esp_err_t ret;
  EventBits_t bits;
  uint8_t attempts = 0;

  tcp_rx_queue = xQueueCreate(10, sizeof(TYPE_FORMAT_REQUEST_T *));
  g_tcp_event_group = xEventGroupCreate();
  configASSERT(tcp_rx_queue != NULL);
  configASSERT(g_tcp_event_group != NULL);

  /* Un solo consumidor de frames para todos los intentos de TCP. */
  BaseType_t created = xTaskCreate(tcp_proccess_task, "tcp_proccess_task",
                                   4096, NULL, 6, &recv_proccess_handle);
  configASSERT(created == pdPASS);

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* Stack WiFi primero: crea g_wifi_event_group */
  wifi_init_sta();

  /* Supervisor DESHABILITADO por ahora: app_main maneja reconexion y
   * reprovisioning. Cuando el flujo este maduro, se puede reactivar solo
   * para monitoreo pasivo. */
  // xTaskCreate(task_wifi_supervisor, "wifi_sup", 3072, NULL, 4, NULL);

  while (1) {

    /* ============================================================
     * FASE 1: intentar con lo que haya en NVS
     * ============================================================ */
    ESP_LOGI(TAG, "intentando con credenciales guardadas en NVS");
    wifi_connect_saved();

    bits = xEventGroupWaitBits(
        g_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_NO_CREDS_BIT, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    /* ¿En que red estamos? */
    bool in_real_net = false;
    bool in_rpi_ap = false;

    if (bits & WIFI_CONNECTED_BIT) {
      if (esp_wifi.ssid && strcmp(esp_wifi.ssid, SSID_RPI) == 0) {
        in_rpi_ap = true;
        ESP_LOGW(TAG, "conectado al AP del RPi '%s' (provisioning pendiente)",
                 esp_wifi.ssid);
      } else {
        in_real_net = true;
        ESP_LOGI(TAG, "conectado a red real '%s'", esp_wifi.ssid);
      }
    } else if (bits & WIFI_NO_CREDS_BIT) {
      ESP_LOGW(TAG, "sin credenciales validas en NVS");
    } else if (bits & WIFI_FAIL_BIT) {
      ESP_LOGW(TAG, "credenciales guardadas no funcionan, borrando");
      erease_cred_nvs(eWIFI_CRED);
    }

    if (in_real_net) {
      ESP_LOGI(TAG, "RED FINAL: '%s' IP=" IPSTR, esp_wifi.ssid,
               IP2STR(esp_wifi.ip));
      break; /* salir del while — estamos en la red buena */
    }

    /* ============================================================
     * FASE 2: provisioning por TCP (si no estamos ya en el AP del RPi)
     * ============================================================ */
    if (!in_rpi_ap) {
      if (connect_to_rpi_ap() != ESP_OK) {
        attempts++;
        ESP_LOGE(TAG, "no se pudo conectar al AP del RPi (%u/5)", attempts);
        if (attempts >= 5) {
          ESP_LOGE(TAG, "agotados intentos, reiniciando");
          esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    /* Configurar IP/puerto del RPi (persiste: es estable) */
    char rpi_ip[] = RPI_IP_SETUP;
    uint16_t rpi_port = RPI_PORT_SETUP;
    if (update_setup_cred(rpi_ip, &rpi_port, eTCP_CRED) != ESP_OK) {
      ESP_LOGE(TAG, "no se pudo configurar IP/puerto del RPi");
      attempts++;
      if (attempts >= 5)
        esp_restart();
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    /* Limpiar bits de TCP antes del nuevo ciclo */
    xEventGroupClearBits(g_tcp_event_group, TCP_CONNECTED | TCP_FAIL |
                                                TCP_DISCONNECTED |
                                                TCP_WIFI_UPDATE);

    /* Levantar TCP + enviar ReqWifi */
    if (setup_tcp() != ESP_OK) {
      attempts++;
      ESP_LOGE(TAG, "setup_tcp fallo (%u/5)", attempts);
      if (attempts >= 5)
        esp_restart();
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    /* Esperar credenciales reales por TCP */
    ESP_LOGI(TAG, "TCP establecido, esperando credenciales reales...");
    bits = xEventGroupWaitBits(g_tcp_event_group,
                               TCP_WIFI_UPDATE | TCP_FAIL | TCP_DISCONNECTED,
                               pdTRUE, pdFALSE,
                               pdMS_TO_TICKS(TCP_UPDATE_TIMEOUT_MS));

    /* El servidor puede cerrar justo despues de enviar la respuesta;
       recv_task marca DISCONNECTED antes de que el parser vacie la cola. */
    if ((bits & TCP_DISCONNECTED) && !(bits & TCP_WIFI_UPDATE)) {
      bits |= xEventGroupWaitBits(g_tcp_event_group, TCP_WIFI_UPDATE,
                                  pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
    }

    if (!(bits & TCP_WIFI_UPDATE)) {
      attempts++;
      ESP_LOGE(TAG, "no llegaron credenciales por TCP (%u/5)", attempts);
      if (attempts >= 5)
        esp_restart();
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    /* ============================================================
     * FASE 3: las creds reales ya estan en NVS y en el stack WiFi
     *         (parse_and_apply_wifi_creds → update_setup_cred →
     *          wifi_set_credentials + save_credentials).
     *         wifi_set_credentials ya llamo esp_wifi_connect().
     *         Solo falta esperar a que conecte o falle.
     * ============================================================ */
    ESP_LOGI(TAG, "creds reales recibidas, intentando conectar...");
    bits = xEventGroupWaitBits(g_wifi_event_group,
                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdTRUE,
                               pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGI(TAG, "RED FINAL: '%s' IP=" IPSTR, esp_wifi.ssid,
               IP2STR(esp_wifi.ip));
      attempts = 0;
      break; /* salir del while — exito */
    }

    /* Las creds reales no funcionaron: borrar y volver a empezar */
    attempts++;
    ESP_LOGW(TAG, "creds reales no funcionan (%u/5), reprovisioning", attempts);
    if (attempts >= 5) {
      ESP_LOGE(TAG, "agotados intentos, reiniciando");
      esp_restart();
    }
    erease_cred_nvs(eWIFI_CRED);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }

  /* ============================================================
   * Aqui ya estamos en la red real. Por ahora solo se reporta.
   * Mas adelante: mDNS discovery, broker, etc.
   * ============================================================ */
  ESP_LOGI(TAG, "==================================================");
  ESP_LOGI(TAG, "  RED FINAL: '%s'", esp_wifi.ssid);
  ESP_LOGI(TAG, "  IP:        " IPSTR, IP2STR(esp_wifi.ip));
  ESP_LOGI(TAG, "==================================================");

  xTaskCreate(task_broker_discovery, "broker_disc", 4096, NULL, 5, NULL);
  uint8_t wifi_offline_checks = 0;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(60000));
    /* Wi-Fi es responsabilidad del stack ESP32, MQTT reintenta por su cuenta.
       No modificar una STA conectada solo porque Railway/Internet no responda. */
    if (!esp_wifi.connected) {
      wifi_offline_checks++;
      if (wifi_offline_checks >= 5) {
        ESP_LOGE(TAG, "STA sin conexion por 5 verificaciones; reiniciando aprovisionamiento");
        esp_restart();
      }
      ESP_LOGW(TAG, "STA desconectada; recuperación WiFi intento %u/5", wifi_offline_checks);
      wifi_connect_saved();
    } else {
      wifi_offline_checks = 0;
    }
  }
}

// el seungdo parametro represetna en WIFI la contrasneia
// en TCP represeta en puerto
// como en uno es string y en otro es numerico, se utiliza de tipoc void* para
// evitar el problema de casting, accedindo a memoria
esp_err_t update_setup_cred(char *key, void *anchor, uint8_t identifier) {

  if (identifier == eTCP_CRED) {
    // entonces debemos de actualizar las credenicales de TCP
    uint16_t port = *(uint16_t *)anchor;

    tcp_client.host_port = port;
    char *new_ip = realloc(tcp_client.host_ip, strlen(key) + 1);
    if (new_ip == NULL) {
      ESP_LOGE(TAG, "no se pudo reservar memoria para IP TCP");
      return ESP_ERR_NO_MEM;
    }
    tcp_client.host_ip = new_ip;
    strcpy(tcp_client.host_ip, key);

    // pasamos a guardar los datos en flash.
    // y regresamos el valor si es que se pudo guardar o no.
    return save_credentials(eTCP_CRED);

  } else if (identifier == eWIFI_CRED) {

    /*esp_wifi.ssid = realloc(esp_wifi.ssid, strlen(key));
    esp_err_tp_wifi.pswd = realloc(esp_wifi.pswd, strlen(anchor));*/
    esp_err_t werr = wifi_set_credentials(key, (const char *)anchor);

    /*infromacionf (esp_wifi.ssid == NULL && esp_wifi.pswd == NULL) {
      ESP_LOGE(TAG, "sin memoria para asingar credenicales.");
      return ESP_FAIL;
    }*/
    if (werr != ESP_OK) {
      ESP_LOGE(TAG, "wifi_set_credentials fallo: %s", esp_err_to_name(werr));
      return werr;
    }

    // strcpy(esp_wifi.ssid, key);
    // strcpy(esp_wifi.pswd, anchor);
    // esp_wifi.connected = 0;

    return save_credentials(eWIFI_CRED);
  }
  return ESP_FAIL;
}

/*
esp_err_t setup_credentials_default(uint8_t identifier) {

  char *key = NULL;
  void *anchor = NULL;
  // uint16_t anchor_PORT = 0;
  uint8_t len_key = 0;
  uint8_t len_anchor = 0;
  esp_err_t ret;
  uint16_t tcp_port = RPI_PORT_SETUP;
  EventBits_t bits = 0;

  if (identifier == eWIFI_CRED) {
    // copiando el SSID a una varibale auxiliar
    len_key = strlen(SSID_RPI);
    key = malloc(len_key + 1);
    memcpy(key, SSID_RPI, len_key);
    key[len_key] = '\0';

    // copiando PSWD a una vairbale axilizar
    len_anchor = strlen(PSWD_RPI);
    char *pswd = malloc(len_anchor + 1);
    if (pswd == NULL) {
      free(key);
      return ESP_ERR_NO_MEM;
    }
    memcpy(pswd, PSWD_RPI, len_anchor);
    pswd[len_anchor] = '\0';
    anchor = pswd;

  } else if (identifier == eTCP_CRED) {

    // copiando la IP  a una varibale auxiliar
    len_key = strlen(RPI_IP_SETUP);
    key = malloc(len_key + 1);
    memcpy(key, SSID_RPI, len_key);
    key[len_key] = '\0';

    // agregamos el puerto a una varibale auxilizar
    anchor = &tcp_port;
  } else {
    ESP_LOGE(TAG, "FATAL ERROR!, %d no esta dentro de los identificadores de "
                  "inicilizacion de comunicacion");
    return ESP_ERR_INVALID_ARG;
  }

  // ahora podemos trbjar con las variables porque el proceso de ambos es muy
  // similar.

  // primero debemos de inciar sesion con el wifi de la rasberry
  ret = update_setup_cred(key, &anchor, identifier);

  if (identifier == eWIFI_CRED) {
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ERROR al establecer credeniclaes WIFI de RPI: %s",
               esp_err_to_name(ret));
      ret = erease_cred_nvs(eWIFI_CRED);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "tipo de error al borrar : credenucales: %s",
                 esp_err_to_name(ret));
      }
      vTaskDelay(pdMS_TO_TICKS(60000));
      return ESP_FAIL;
    }

    wifi_init_sta();
    wifi_connect_saved();

    bits = xEventGroupWaitBits(g_wifi_event_group,
                               WIFI_FAIL_BIT | WIFI_CONNECTED_BIT, pdTRUE,
                               pdFALSE, portMAX_DELAY);

    if (bits & WIFI_FAIL_BIT) {
      ret = erease_cred_nvs(eWIFI_CRED);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "tipo de error al borrar : credenucales: %s",
                 esp_err_to_name(ret));
      }
      vTaskDelay(pdMS_TO_TICKS(60000));
      return ESP_FAIL;
    }
    return ESP_OK;
  } else if (identifier == eTCP_CRED) {

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ERROR al establecer credeniclaes TCP de RPI: %s",
               esp_err_to_name(ret));
      ret = erease_cred_nvs(eTCP_CRED);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "tipo de error al borrar : credenucales: %s",
                 esp_err_to_name(ret));
      }
      vTaskDelay(pdMS_TO_TICKS(60000));
      return ESP_FAIL;
    }

    // pudo estavelecer las crednecilaes TCP ahora veremos si nos podemos
    // conectar
    setup_tcp();

    bits = xEventGroupWaitBits(g_tcp_event_group,
                               TCP_FAIL | TCP_CONNECTED | TCP_DISCONNECTED,
                               pdTRUE, pdFALSE, portMAX_DELAY);

    if (bits & TCP_CONNECTED) {
      // se logro la conxion
      return ESP_OK;
    } else if (bits & TCP_FAIL) {
      ESP_LOGE(TAG, "error al establecer la conexion con el servidor!");
      return ESP_FAIL;
    }
    return ESP_OK;
  }

  return ret;
}*/

static esp_err_t setup_credentials_default(uint8_t identifier) {

  char *key = NULL;
  void *anchor = NULL;
  uint16_t tcp_port = RPI_PORT_SETUP;
  EventBits_t bits = 0;
  esp_err_t ret;

  if (identifier == eWIFI_CRED) {
    key = strdup(SSID_RPI);
    anchor = strdup(PSWD_RPI);
    if (!key || !anchor) {
      free(key);
      free(anchor);
      return ESP_ERR_NO_MEM;
    }
  } else if (identifier == eTCP_CRED) {
    key = strdup(RPI_IP_SETUP);
    anchor = &tcp_port;
    if (!key)
      return ESP_ERR_NO_MEM;
  } else {
    ESP_LOGE(TAG, "identifier %d invalido", identifier);
    return ESP_ERR_INVALID_ARG;
  }

  ret = update_setup_cred(key, anchor, identifier);
  if (identifier == eWIFI_CRED) {
    free(key);
    free(anchor);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "setup cred fallo (%u): %s", identifier,
             esp_err_to_name(ret));
    erease_cred_nvs(identifier);
    vTaskDelay(pdMS_TO_TICKS(60000));
    return ESP_FAIL;
  }

  if (identifier == eWIFI_CRED) {
    bits = xEventGroupWaitBits(g_wifi_event_group,
                               WIFI_FAIL_BIT | WIFI_CONNECTED_BIT, pdTRUE,
                               pdFALSE, portMAX_DELAY);
    if (bits & WIFI_FAIL_BIT) {
      erease_cred_nvs(eWIFI_CRED);
      vTaskDelay(pdMS_TO_TICKS(60000));
      return ESP_FAIL;
    }
    return ESP_OK;
  }

  /* TCP */
  setup_tcp();
  bits = xEventGroupWaitBits(g_tcp_event_group,
                             TCP_FAIL | TCP_CONNECTED | TCP_DISCONNECTED,
                             pdTRUE, pdFALSE, portMAX_DELAY);
  if (bits & TCP_CONNECTED)
    return ESP_OK;

  if (bits & TCP_FAIL)
    ESP_LOGE(TAG, "fallo al conectar TCP");
  return ESP_FAIL;
}

// en este puntp ya transformamos valores en trama binaria en su rpresentacion
// ascii o lo dejamos en froma numericos
//
// // SSID y PSWD para WIFI -> STRING
// //IP - STRING - PORT - uint16;
/*esp_err_t save_credentials(uint8_t identifier) {

  esp_err_t ret;
  nvs_handle_t h;

  uint16_t port = 0;
  uint64_t sum = 0;

  if (identifier == eTCP_CRED) {

    // vamos a guardar las credeniclaes de TCP en flash, para levanatar
    // servicios mas rapido y comprbar corrupciones en los datos

    if (tcp_client.host_ip == NULL) {
      ESP_LOGE(TAG, "estado de IP invalida");
      return ESP_ERR_INVALID_ARG;
    }
    port = tcp_client.host_port;
    sum = checksum(tcp_client.host_ip, &port, eTCP_CRED);

    ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
      // ESP_LOGE(TAG, "tipo de error: %s", esp_err_to_name(ret));
      return ret;
    }
    ret = nvs_set_str(h, KEY_TCP_IP, tcp_client.host_ip);
    if (ret == ESP_OK) {
      ret = nvs_set_u16(h, KEY_TCP_PORT, port);
    }
    if (ret == ESP_OK) {
      ret = nvs_set_u64(h, KEY_TCP_SUM, sum);
    }
    if (ret == ESP_OK) {
      ret = nvs_commit(h); // sin commit no se escribe en flash
    }
    nvs_close(h);
    return ret;
  } else if (identifier == eWIFI_CRED) {

    if (esp_wifi.ssid == NULL) {
      ESP_LOGE(TAG, "sin nombre de red");
      return ESP_ERR_INVALID_ARG;
    }

    uint64_t sum = checksum(esp_wifi.ssid, &esp_wifi.pswd, eWIFI_CRED);

    ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
      // ESP_LOGE(TAG, "tipo de error: %s", esp_err_to_name(ret));
      return ret;
    }

    ret = nvs_set_str(h, KEY_WIFI_SSID, esp_wifi.ssid);
    if (ret == ESP_OK) {
      ret = nvs_set_str(h, KEY_WIFI_PSWD, esp_wifi.pswd);
    }
    if (ret == ESP_OK) {
      ret = nvs_set_u64(h, KEY_WIFI_SUM, sum);
    }
    if (ret == ESP_OK) {
      ret = nvs_commit(h); // sin commit no se escribe en flash
    }
    nvs_close(h);
    return ret;
  }

  // si el identificador es uno no estbalecido pues fallo.
  return ESP_FAIL;
}*/

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
    if (ret == ESP_OK && checksum(ip, &port, eTCP_CRED) != saved_sum) {
      ret = ESP_ERR_INVALID_CRC;
    }

    if (ret != ESP_OK) {
      free(ip);
      return ret;
    }

    free(tcp_client.host_ip);
    tcp_client.host_ip = ip;
    tcp_client.host_port = port;
    return ESP_OK;
  }
  if (identifier == eWIFI_CRED) {
    char *ssid = NULL, *pswd = NULL;
    size_t len_s = 0, len_p = 0;
    uint64_t saved_sum = 0;

    ret = nvs_get_str(h, KEY_WIFI_SSID, NULL, &len_s);
    if (ret == ESP_OK) {
      ssid = malloc(len_s);
      if (!ssid)
        ret = ESP_ERR_NO_MEM;
    }
    if (ret == ESP_OK)
      ret = nvs_get_str(h, KEY_WIFI_SSID, ssid, &len_s);
    if (ret == ESP_OK)
      ret = nvs_get_str(h, KEY_WIFI_PSWD, NULL, &len_p);
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

    if (ret == ESP_OK && checksum(ssid, pswd, eWIFI_CRED) != saved_sum)
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

  for (int i = 0; i < 3; ++i) {
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

// vamos a sumar los valores, totales de los datos
// SSID + pswd
// IP + PORT
uint64_t checksum(const char *key, const void *anchor, uint8_t identifier) {
  uint64_t total = 0;

  // key siempre es un string (SSID o IP): sumamos el valor de cada caracter
  for (size_t i = 0; key[i] != '\0'; i++) {
    total += (uint8_t)key[i];
  }

  if (identifier == eTCP_CRED) {
    // anchor es el puerto: es un numero, se suma directo
    total += *(const uint16_t *)anchor;

  } else if (identifier == eWIFI_CRED) {
    // anchor es la contrasenia: otro string, se suma caracter por caracter
    const char *pswd = (const char *)anchor;
    for (size_t i = 0; pswd[i] != '\0'; i++) {
      total += (uint8_t)pswd[i];
    }
  }

  return total;
}

esp_err_t setup_tcp(void) {
  esp_err_t ret;
  int attempts = 0;
  const int max_attempts = 3;

  while (attempts < max_attempts) {
    ret = tcp_cliente_init();

    if (ret != ESP_OK) {
      attempts++;
      ESP_LOGW(TAG, "TCP connect fallo (intento %d/%d)", attempts,
               max_attempts);
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    /* Peticion inicial: AB CD 01 11. Enviar antes de crear recv_task;
       la respuesta quedara en el buffer del socket hasta que lo leamos. */
    memset(&send_info, 0, sizeof(send_info));
    send_info.op_type = OPReqCredWifi;
    send_info.format_request.header = HEADER;
    send_info.format_request.len = 1;
    send_info.format_request.type_frame = eWifi;
    send_info.format_request.action = eReqWifi;

    ESP_LOGI(TAG, "solicitando credenciales WiFi a la RPi (AB CD 01 11)");

    if (send_message() == ESP_OK) {
      BaseType_t created = xTaskCreate(recv_task, "recv_task", 3072, NULL,
                                       5, &recv_task_handle);
      if (created != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear recv_task");
        close(tcp_client.sock);
        tcp_client.sock = -1;
        tcp_client.connected = 0;
        return ESP_ERR_NO_MEM;
      }
      xEventGroupSetBits(g_tcp_event_group, TCP_CONNECTED);
      return ESP_OK;
    }

    /* send fallo: limpiar y reintentar */
    ESP_LOGE(TAG, "send_message fallo, cerrando socket y reintentando");
    if (tcp_client.sock >= 0) {
      close(tcp_client.sock);
      tcp_client.sock = -1;
    }
    tcp_client.connected = 0;
    xEventGroupClearBits(g_tcp_event_group, TCP_CONNECTED);
    vTaskDelay(pdMS_TO_TICKS(500));
    attempts++;
  }

  ESP_LOGE(TAG, "setup_tcp: agotados %d intentos", max_attempts);
  xEventGroupSetBits(g_tcp_event_group, TCP_FAIL);
  return ESP_FAIL;
}

/* ------------------------------------------------------------------ */
/* parser credenciales WiFi (solo red normal)                          */
/* formato: |id_ssid|len|SSID|id_pswd|len|pswd|                       */
/* ------------------------------------------------------------------ */
static esp_err_t parse_and_apply_wifi_creds(TYPE_FORMAT_REQUEST_T *frame,
                                            uint8_t *offset) {
  uint8_t id, len;
  char *ssid = NULL, *pswd = NULL;
  esp_err_t ret = ESP_FAIL;
  size_t total = (frame->len >= 1) ? (size_t)(frame->len - 1) : 0;

  /* LEN incluye type|action; los TLV viven en data[0..total-1]. */
  if (total < 4 || total > MAX_DATA)
    return ESP_ERR_INVALID_SIZE;

  /* SSID */
  if ((size_t)(*offset) + 2 > total)
    return ESP_ERR_INVALID_SIZE;
  id = frame->data[(*offset)++];
  if (id != eDataSSID)
    return ESP_ERR_INVALID_ARG;
  len = frame->data[(*offset)++];
  if (len == 0 || len > SSID_LEN_MAX || (size_t)*offset + len > total)
    return ESP_ERR_INVALID_SIZE;
  ssid = malloc(len + 1);
  if (!ssid)
    return ESP_ERR_NO_MEM;
  memcpy(ssid, &frame->data[*offset], len);
  *offset += len;
  ssid[len] = '\0';

  /* Contraseña */
  if ((size_t)(*offset) + 2 > total)
    goto done;
  id = frame->data[(*offset)++];
  if (id != eDataPSWD)
    goto done;
  len = frame->data[(*offset)++];
  if (len < PSWD_LEN_MIN || len > PSWD_LEN_MAX ||
      (size_t)*offset + len != total)
    goto done;
  pswd = malloc(len + 1);
  if (!pswd) {
    ret = ESP_ERR_NO_MEM;
    goto done;
  }
  memcpy(pswd, &frame->data[*offset], len);
  *offset += len;
  pswd[len] = '\0';

  /* El AP solo se usa temporalmente; jamas se guarda como red normal. */
  if (strcmp(ssid, SSID_RPI) == 0) {
    ESP_LOGE(TAG, "la RPi envio SSID del AP en lugar del WiFi normal");
    ret = ESP_ERR_INVALID_ARG;
    goto done;
  }
  ESP_LOGI(TAG, "credenciales recibidas para SSID='%s'", ssid);
  ret = update_setup_cred(ssid, pswd, eWIFI_CRED);

done:
  free(ssid);
  free(pswd);
  return ret;
}

// tareas

void tcp_proccess_task(void *params) {
  TYPE_FORMAT_REQUEST_T *frame = NULL;

  while (1) {
    if (xQueueReceive(tcp_rx_queue, &frame, portMAX_DELAY) != pdTRUE)
      continue;
    if (frame == NULL)
      continue;

    if (frame->header == ACK && frame->len == 0xFF &&
        frame->type_frame == eRespACK) {
      ESP_LOGE(TAG, "RPi devolvio NACK de protocolo");
      xEventGroupSetBits(g_tcp_event_group, TCP_FAIL);
    } else if (frame->header == ACK && frame->type_frame == eWifi &&
               frame->action == eRespWifi) {
      uint8_t offset = 0;
      esp_err_t ret = parse_and_apply_wifi_creds(frame, &offset);
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "credenciales normales recibidas y guardadas en NVS");
        xEventGroupSetBits(g_tcp_event_group, TCP_WIFI_UPDATE);
      } else {
        ESP_LOGE(TAG, "credenciales TCP invalidas: %s",
                 esp_err_to_name(ret));
        xEventGroupSetBits(g_tcp_event_group, TCP_FAIL);
      }
    } else {
      ESP_LOGW(TAG, "frame inesperado: header=%04X type=%u action=%u",
               frame->header, (unsigned)frame->type_frame,
               (unsigned)frame->action);
    }

    vPortFree(frame);
    frame = NULL;
  }
}

void task_wifi_supervisor(void *param) {
  uint8_t tries = 0;

  while (1) {
    EventBits_t bits = xEventGroupWaitBits(
        g_wifi_event_group, WIFI_FAIL_BIT | WIFI_CONNECTED_BIT, pdFALSE,
        pdFALSE, pdMS_TO_TICKS(WIFI_RECOVER_MS));

    if (bits & WIFI_FAIL_BIT) {
      xEventGroupClearBits(g_wifi_event_group, WIFI_FAIL_BIT);
      tries++;
      ESP_LOGW(TAG, "WiFi caido (intento %u/%u)", tries, WIFI_RECOVER_TRIES);

      if (tries >= WIFI_RECOVER_TRIES) {
        ESP_LOGE(TAG, "agotados reintentos, borrando creds y reiniciando");
        erease_cred_nvs(eWIFI_CRED);
        esp_restart();
      } else {
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECOVER_MS));
        wifi_connect_saved();
      }
    } else if (bits & WIFI_CONNECTED_BIT) {
      tries = 0;
    }
  }
}

/* ------------------------------------------------------------------ */
/* task_broker_discovery (mDNS)                                         */
/* ------------------------------------------------------------------ */
void task_broker_discovery(void *pv) {
  const char *hosts[] = {"selfiq-gateway", "raspberrypi", "rackiq-gateway"};

  while (1) {
    char ip[16] = {0};
    bool found = false;
    uint16_t broker_port = 1883;

    if (mdns_init() == ESP_OK) {
      esp_ip4_addr_t addr = {0};

      for (int i = 0; i < 3 && !found; i++) {
        if (mdns_query_a(hosts[i], 3000, &addr) == ESP_OK && addr.addr) {
          snprintf(ip, sizeof(ip), IPSTR, IP2STR(&addr));
          found = true;
        }
      }

      if (!found) {
        mdns_result_t *res = NULL;
        if (mdns_query_srv("selfiq-broker", "_mqtt", "_tcp", 3000, &res) ==
                ESP_OK &&
            res) {
          for (mdns_result_t *r = res; r; r = r->next) {
            if (r->addr && r->addr->addr.type == ESP_IPADDR_TYPE_V4) {
              snprintf(ip, sizeof(ip), IPSTR,
                       IP2STR(&r->addr->addr.u_addr.ip4));
              found = true;
              if (r->port) broker_port = r->port;
              break;
            }
          }
          mdns_query_results_free(res);
        }
      }
      mdns_free();
    }

    if (found) {
      ESP_LOGI(TAG, "RPi en red normal: %s", ip);
      strncpy(g_rpi_ip, ip, sizeof(g_rpi_ip) - 1);
      ESP_LOGI(TAG, "iniciando registro MQTT con gateway %s:%u", g_rpi_ip, (unsigned)broker_port);
      if (mqtt_register_start(g_rpi_ip, broker_port) != ESP_OK) {
        ESP_LOGE(TAG, "falló iniciar cliente MQTT; reintento mDNS");
        vTaskDelay(pdMS_TO_TICKS(30000));
        continue;
      }
      vTaskDelete(NULL);
      return;
    }

    ESP_LOGW(TAG, "mDNS sin resultados, reintento en 30 s");
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

/* ------------------------------------------------------------------ */
/* Conecta al AP de provisioning del RPi SIN persistir en NVS.        */
/* Usa wifi_set_credentials() directo, NO update_setup_cred(), para   */
/* que SelfIQ-Admi no se guarde como si fuera la red real.            */
/* ------------------------------------------------------------------ */
static esp_err_t connect_to_rpi_ap(void) {
  ESP_LOGI(TAG, "conectando al AP del RPi '%s'", SSID_RPI);

  esp_err_t err = wifi_set_credentials(SSID_RPI, PSWD_RPI);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi_set_credentials(SelfIQ-Admi) fallo: %s",
             esp_err_to_name(err));
    return err;
  }

  EventBits_t bits = xEventGroupWaitBits(
      g_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdTRUE, pdFALSE,
      pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "conectado al AP del RPi, IP=" IPSTR, IP2STR(esp_wifi.ip));
    return ESP_OK;
  }

  ESP_LOGE(TAG, "no se pudo conectar al AP del RPi");
  return ESP_FAIL;
}
