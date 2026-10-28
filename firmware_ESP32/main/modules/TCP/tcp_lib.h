/* *********************************************
 *              selfIQ 2026-2
 *
 * @autor Erik Garcia Chavez
 * @date octuber 2026
 * @assginature : proyecto de carrera
 * @teacher : Jose Isabel Garcia Rocha
 *
 ************************************************* */
#ifndef TCP_LIB_H
#define TCP_LIB_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/event_groups.h"
#include "global.h"

/* Indices en la trama completa: HEADER[2], LEN, TYPE/ACTION, DATA. */
enum { TCP_HEADER_HIGH_OFFSET, TCP_HEADER_LOW_OFFSET, TCP_LENGTH_OFFSET,
       TCP_OPERATION_OFFSET, TCP_DATA_OFFSET };
/* tcp_exchange devuelve desde TYPE/ACTION; no incluye HEADER ni LEN. */
enum { TCP_WIFI_OPERATION_OFFSET, TCP_WIFI_SSID_ID_OFFSET,
       TCP_WIFI_SSID_LENGTH_OFFSET };
enum { TCP_STATUS_OPERATION_OFFSET, TCP_STATUS_VERSION_OFFSET,
       TCP_STATUS_WIFI_OFFSET, TCP_STATUS_BACKEND_OFFSET, TCP_STATUS_MQTT_OFFSET,
       TCP_STATUS_STORAGE_OFFSET, TCP_STATUS_ACTION_OFFSET };
#define TCP_FIELD_LENGTH_OFFSET 1U
#define TCP_RX_QUEUE_WAIT_MS 100U
#define TCP_BYTE_MASK 0xFFU
#define TCP_NACK_PADDING_VALUE 0U

//+++++++++++++++++++++ macros

#define RPI_IP_SETUP "192.168.50.1"
#define RPI_PORT_SETUP 5000

//+++++++++++++++ bits de grupo de eventos

#define TCP_RETRY_SERVER BIT4
#define TCP_DISCONNECTED BIT5 // servidor cerro la conexion inesperadamente
#define TCP_UPDATE BIT6
#define TCP_FAIL BIT7
#define TCP_CONNECTED BIT8
#define TCP_WIFI_UPDATE BIT9
#define TCP_IP_NVS_MIN_SIZE 2U

#define TCP_HEADER_BYTES 2U
#define TCP_LENGTH_BYTES 1U
#define TCP_OPERATION_BYTES 1U
#define TCP_PREFIX_BYTES (TCP_HEADER_BYTES + TCP_LENGTH_BYTES)
#define TCP_FRAME_OVERHEAD (TCP_PREFIX_BYTES + TCP_OPERATION_BYTES)
#define TCP_REQUEST_PAYLOAD_LEN TCP_OPERATION_BYTES
//para sacar el header del frame si es ACK - NACK o HEADER de peticion de RPI->ESP32
#define TCP_HEADER_HIGH ((HEADER >> TCP_HEADER_SHIFT) & TCP_BYTE_MASK)
#define TCP_HEADER_LOW (HEADER & TCP_BYTE_MASK)
#define TCP_ACK_HIGH ((ACK >> TCP_HEADER_SHIFT) & TCP_BYTE_MASK)
#define TCP_ACK_LOW (ACK & TCP_BYTE_MASK)

// WIFI + eReqWifi = 0x11 -> socilicitar las credencales WIFI 
#define TCP_WIFI_REQUEST_OP 0x11U
#define TCP_WIFI_RESPONSE_OP 0x12U
#define TCP_STATUS_REQUEST_OP 0x41U
#define TCP_STATUS_RESPONSE_OP 0x42U
#define TCP_WIFI_MIN_RESPONSE_LEN 6U
#define TCP_WIFI_RESPONSE_FIELD_OFFSET 3U
#define TCP_WIFI_FIELD_METADATA_BYTES 2U
#define TCP_STATUS_RESPONSE_LEN 7U
#define TCP_STATUS_RESPONSE_VERSION 1U
#define TCP_STATUS_WIFI_UPDATE_ACTION 2U
#define TCP_STATUS_BUFFER_SIZE 16U
#define TCP_RESPONSE_BUFFER_SIZE (MAX_DATA + TCP_OPERATION_BYTES)


#define TCP_MAX_CONNECT_ATTEMPTS 5
#define TCP_CONNECT_RETRY_MS 1000U
#define TCP_CONNECTION_RETRY_MS 2000U
#define TCP_SOCKET_TIMEOUT_SEC 5
#define TCP_MAX_RECV_TIMEOUTS 3
#define TCP_HEADER_SHIFT 8U
#define TCP_ACTION_BITS 4U
#define TCP_NIBBLE_MASK 0x0FU
#define TCP_NACK_MARKER 0xFFU
#define TCP_NACK_FRAME_BYTES 5U


// +++++++++++++++++++ colas
extern QueueHandle_t tcp_rx_queue;
//++++++++++++++++++++++++ grupo de eventos

extern EventGroupHandle_t g_tcp_event_group;

// +++++++++++++++++++++++++estrucutras

/**
 * @brief estrucutra con  la informacion sobre el proceso/ de la conexion de
 * socker TCP
 *
 * host_ip - direccion IP al servidor a conectarse @memberof tcp_client_t
 * host_port - puerto del servidor a conectarse @memberof tcp_client_t
 * sock - descriptor del socket creador @memberof tcp_client_t
 * connected -  bandera indicando que se realziara la conexion @memberof
 * tcp_client_t
 * @logged_in - bandera que indica que se logro loggear al servidor @memberof
 * tcp_client_t
 *
 */

typedef struct {
  char *host_ip;
  uint16_t host_port;
  int sock;
  int connected;
} TYPE_TCP_CLIENT_T;

extern TYPE_TCP_CLIENT_T tcp_client;

// +++++++++++++++++++++ funciones

/**
 * @brief funcion encarga de crear el socket con el que se va a conectar al
 * servidor. lo intentara 5 veces antes de salir e indicar que no se pudo
 * conectar
 *
 * @param tcp_client es la estrucutra que tiene la informacion del socket de la
 * conexion con el servidor
 *
 * @return ESP_OK cunado se pude establecer la conexion
 * @return ESP_FAIL cunado no se pudo establecer la conexion
 *
 *
 */
esp_err_t tcp_cliente_init();

// funcion que debe de realizar el login
/**
 * @brief funcion que funcionara para enivar los datos hacia el servidor
 *
 * @param sockfd parametro en donde vendra el descriptro con el sokcer abierto
 * en el momento para enviar la infocmacion
 * @param msg dentro de esta estrucutra se encontrara la informacion del datos a
 * enviar, en especial <op> indicando que estrucutra es la que se va a enviar
 *
 *
 *
 * @return ESP_OK si se envio correctamente
 * @return ESP_FAIL ocurrio un error en el envio de los datos.
 *
 *
 */
esp_err_t send_message(void);

// +++++++++++++++++ tareas
/**
 * @brief tarea encargada de recibir lo que manda el servidor hacia la ESP
 *
 * solo parte el payload en sus respectivas v aribales, mas no anlaiza lo que
 * llego
 */
void recv_task(void *params);


int tcp_exchange(uint8_t operation, uint8_t *out, size_t cap);

#endif
