/* *********************************************
 *              selfIQ 2026-2
 *
 * @autor Erik Garcia Chavez
 * @date octuber 2026
 * @assginature : proyecto de carrera
 * @teacher : Jose Isabel Garcia Rocha
 *
 ************************************************* */

#ifndef WIFI_LIB_H
#define WIFI_LIB_H

#include <esp_err.h>
#include <esp_event.h>
#include <esp_netif_ip_addr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
//++++++++++++++++++++++ macros
// intentara conectarse con las credenicales actuales 5 veces, esperara 1
// minutos antes de volver a petdir de nuevo las credenciales.
#define ESP_MAX_RETRY 5

// bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_NO_CREDS_BIT BIT2
#define WIFI_UPDATE BIT3

// credenciales de WIFI por defecto de la rasberry pi
#define SSID_RPI "SelfIQ-Admi"
#define PSWD_RPI "$elfIQ-4dmi-1"

// tiempos del mecanismo de recuperacion
#define WIFI_RECOVER_MS 60000 // espera entre reintentos al perder la red
#define WIFI_RECOVER_TRIES 5  // reintentos antes de borrar credenciales
#define WIFI_CONNECT_TIMEOUT_MS                                                \
  60000                    // espera maxima al conectar a la red nueva
#define TCP_RETRY_MS 60000 // espera antes de repetir el flujo con la RPI
#define TCP_UPDATE_TIMEOUT_MS                                                  \
  300000 // espera maxima de las credenciales de la RPI

//++++++++++++++++ grupos de eventos
extern EventGroupHandle_t g_wifi_event_group;

//++++++++++++++++++ estrucutras

/**
 *
 * @brief esta estrucutura guaradaremos infromacion de la conexion o del estado
 * de la conexion
 *
 * ssid - nombre de la red a coenctarse @memberof esp_wifi_t
 * pswd - contrasenia de la red o de la cuenta @memberof esp_wifi_t
 * ip - ip otorgada al dispositivo por parte de la red @memberof esp_wifi_t
 * connected - 1 o 0 indicando que se pudo realizar la conexion y se eucneutra
 * coinectado el ESP @memberof esp_wifi_t
 *
 */
typedef struct {
  char *ssid;
  char *pswd;
  esp_ip4_addr_t *ip;
  int connected;
} TYPE_ESP_WIFI_T;

extern TYPE_ESP_WIFI_T esp_wifi;

//++++++++++++++++++ funciones
void wifi_init_sta(void);      // init del stack, una sola vez
void wifi_connect_saved(void); // intenta conectar con lo guardado en flash
esp_err_t wifi_set_credentials(const char *ssid,
                               const char *pswd); // guarda en flash y conecta

#endif
