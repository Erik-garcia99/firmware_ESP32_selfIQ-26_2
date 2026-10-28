/* *********************************************
 *              selfIQ 2026-2
 *
 * @autor Erik Garcia Chavez
 * @date octuber 2026
 * @assginature : proyecto de carrera
 * @teacher : Jose Isabel Garcia Rocha
 *
 ************************************************* */

// librerias estandar

// liberias estanadar
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <fcntl.h>
// logs
#include "esp_err.h"
#include "esp_log.h"

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// librerias propias

#include "global.h"
#include "modules/TCP/tcp_lib.h"

//+++++++++++++++++++++ varibales globales

const char *TAG = "TCP_CLIENT : ";

// funciones

esp_err_t tcp_cliente_init() {

  // reinicamos todo antes de inciar
  tcp_client.connected = 0;
  if (tcp_client.sock >= 0) {
    close(tcp_client.sock);
    tcp_client.sock = -1;
  }

  for (int n_retry = 0; n_retry < TCP_MAX_CONNECT_ATTEMPTS; n_retry++) {

    ESP_LOGI(TAG, "intento %d de establecer conexion", n_retry);

    // creamos el descriptor del socket
    tcp_client.sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_client.sock < 0) {
      ESP_LOGE(TAG, "error al crear el descriptor del socket(): %d", errno);
      vTaskDelay(pdMS_TO_TICKS(TCP_CONNECT_RETRY_MS));
      continue; // saltamos todo y se vuelve a intentar.
    }
    // timeout de recv
    struct timeval timeout = {.tv_sec = TCP_SOCKET_TIMEOUT_SEC, .tv_usec = 0};
    setsockopt(tcp_client.sock, SOL_SOCKET, SO_RCVTIMEO, &timeout,
               sizeof(timeout));

    // direccion del servidor
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(tcp_client.host_port),
    };

    inet_pton(AF_INET, tcp_client.host_ip, &server_addr.sin_addr);

    // conectar
    if (connect(tcp_client.sock, (struct sockaddr *)&server_addr,
                sizeof(server_addr)) < 0) {
      ESP_LOGE(TAG, "descriptor <connect()> fallo  %d", errno);
      close(tcp_client.sock);
      tcp_client.sock = -1;
      vTaskDelay(pdMS_TO_TICKS(TCP_CONNECTION_RETRY_MS));
      continue;
    }

    // exito, salir de inmediato
    tcp_client.connected = 1;

    ESP_LOGI(TAG, "EXITO!, conectado a %s:%hu", tcp_client.host_ip,
             tcp_client.host_port);
    return ESP_OK;
  }
  ESP_LOGI(TAG, "fail!, no se pudo conectar al %s:%hu", tcp_client.host_ip,
           tcp_client.host_port);
  // se agotaron los 5 intentos
  tcp_client.connected = 0;
  tcp_client.sock = -1;
  return ESP_FAIL;
}

/* TCP es un flujo de bytes: un recv() no equivale a un frame completo. */
static bool recv_exact(int sock, uint8_t *dst, size_t total) {
  size_t got = 0;
  int timeouts = 0;
  while (got < total) {
    int n = recv(sock, dst + got, total - got, 0);
    if (n > 0) {
      got += (size_t)n;
      timeouts = 0;
    } else if (n == 0) {
      return false;
    } else if (errno == EINTR) {
      continue;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      if (++timeouts >= TCP_MAX_RECV_TIMEOUTS)
        return false;
    } else {
      ESP_LOGE(TAG, "recv fallo errno=%d", errno);
      return false;
    }
  }
  return true;
}

// la cola de los datos que llegan a donde los esta envinado, no veo que se use en otro lugar mas que aqui, no como mi codigo original, que hacia una cola con una copia de los datos que llegaban y los enviaba a otra tarea en main.c y ahi se decodificadbaa y se decidia con forme al coentenido del frame. 
void recv_task(void *params) {
  int sock = tcp_client.sock;
  uint8_t hdr[TCP_FRAME_OVERHEAD];

  while (1) {
    /* Cabecera 2B + LEN 1B + TYPE|ACTION 1B. */
    if (!recv_exact(sock, hdr, sizeof(hdr)))
      break;

    uint16_t head = ((uint16_t)hdr[TCP_HEADER_HIGH_OFFSET] << TCP_HEADER_SHIFT) | hdr[TCP_HEADER_LOW_OFFSET];
    uint8_t frame_len = hdr[TCP_LENGTH_OFFSET];
    uint8_t type = (hdr[TCP_OPERATION_OFFSET] >> TCP_ACTION_BITS) & TCP_NIBBLE_MASK;
    uint8_t action = hdr[TCP_OPERATION_OFFSET] & TCP_NIBBLE_MASK;
    bool is_nack = (head == ACK && frame_len == TCP_NACK_MARKER &&
                    type == TCP_NIBBLE_MASK && action == TCP_NIBBLE_MASK);

    if (head != HEADER && head != ACK) {
      ESP_LOGE(TAG, "header TCP invalido: 0x%04X", head);
      break;
    }
    if (!is_nack && (frame_len < TCP_OPERATION_BYTES || frame_len > MAX_DATA + TCP_OPERATION_BYTES)) {
      ESP_LOGE(TAG, "LEN TCP invalido: %u", frame_len);
      break;
    }

    TYPE_FORMAT_REQUEST_T *frame = pvPortMalloc(sizeof(*frame));
    if (frame == NULL) {
      ESP_LOGE(TAG, "sin memoria para frame TCP");
      break;
    }
    memset(frame, 0, sizeof(*frame));
    frame->header = head;
    frame->len = frame_len;
    frame->type_frame = type;
    frame->action = action;

    /* NACK legado incluye 1 byte de relleno; las respuestas normales usan
       LEN = type|action (1B) + datos. */
    size_t data_len = is_nack ? TCP_OPERATION_BYTES : (size_t)(frame_len - TCP_OPERATION_BYTES);
    if (data_len > 0 && !recv_exact(sock, frame->data, data_len)) {
      vPortFree(frame);
      break;
    }

    ESP_LOGI(TAG, "RX frame: header=%04X len=%u type=%u action=%u",
             head, frame_len, type, action);
    if (xQueueSend(tcp_rx_queue, &frame, pdMS_TO_TICKS(TCP_RX_QUEUE_WAIT_MS)) != pdTRUE) {
      ESP_LOGW(TAG, "cola RX llena");
      vPortFree(frame);
      break;
    }
    /* El parser puede terminar el provisioning mientras llega el siguiente
       recv. El EOF posterior es normal. */
  }

  tcp_client.connected = 0;
  if (tcp_client.sock == sock && sock >= 0) {
    close(sock);
    tcp_client.sock = -1;
  }
  xEventGroupSetBits(g_tcp_event_group, TCP_DISCONNECTED);
  vTaskDelete(NULL);
}

esp_err_t send_message(void) {
  TYPE_FORMAT_REQUEST_T *f = &send_info.format_request;
  uint8_t buffer[TCP_FRAME_OVERHEAD + MAX_DATA];
  int offset = 0;

  if (tcp_client.sock < 0) {
    ESP_LOGE(TAG, "send_message: socket cerrado");
    return ESP_FAIL;
  }

  if (send_info.op_type == OP_NACK) {
    /* NACK: wire = 54 33 FF FF 00 (LEN=0xFF, type=0xF, action=0xF, 1 byte pad)
     */
    buffer[TCP_HEADER_HIGH_OFFSET] = TCP_ACK_HIGH;
    buffer[TCP_HEADER_LOW_OFFSET] = TCP_ACK_LOW;
    buffer[TCP_LENGTH_OFFSET] = TCP_NACK_MARKER;
    buffer[TCP_OPERATION_OFFSET] = TCP_NACK_MARKER;
    buffer[TCP_DATA_OFFSET] = TCP_NACK_PADDING_VALUE;
    offset = TCP_NACK_FRAME_BYTES;
  } else {
    /* HEADER big-endian */
    uint16_t h_be = htons(f->header);
    memcpy(buffer + offset, &h_be, TCP_HEADER_BYTES);
    offset += TCP_HEADER_BYTES;

    /* LEN = 1 (byte type|action) + N (data) */
    if (f->len < TCP_OPERATION_BYTES) {
      ESP_LOGE(TAG, "send_message: LEN invalido (%u)", f->len);
      return ESP_FAIL;
    }
    buffer[offset++] = f->len;

    /* Byte type|action — HIGH nibble = type, LOW = action (como lee la RPi) */
    buffer[offset++] =
        ((uint8_t)(f->type_frame & TCP_NIBBLE_MASK) << TCP_ACTION_BITS) | ((uint8_t)(f->action & TCP_NIBBLE_MASK));

    uint8_t data_len = f->len - TCP_OPERATION_BYTES;
    if (data_len > MAX_DATA) {
      ESP_LOGE(TAG, "send_message: data_len > MAX_DATA (%u)", data_len);
      return ESP_FAIL;
    }
    if (data_len > 0) {
      memcpy(buffer + offset, f->data, data_len);
      offset += data_len;
    }
  }

  /* Envio completo (send puede ser parcial) */
  int total = 0;
  while (total < offset) {
    int sent = send(tcp_client.sock, buffer + total, offset - total, 0);
    if (sent <= 0) {
      ESP_LOGE(TAG, "send() fallo errno: %d", errno);
      return ESP_FAIL;
    }
    total += sent;
  }

  ESP_LOGI(TAG, "TX %d B  header=0x%04X len=%u type=%u action=%u", total,
           f->header, f->len, (unsigned)f->type_frame, (unsigned)f->action);
  return ESP_OK;
}





/* Bounded connect + exact reads: handles partial TCP frames, no helper tasks. */

// es mas esta funcion para que funciona exactamente?
int tcp_exchange(uint8_t operation, uint8_t *out, size_t cap) {
  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (sock < 0){
    return -1;
  } 

  struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(RPI_PORT_SETUP)};
  inet_pton(AF_INET, RPI_IP_SETUP, &address.sin_addr);
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  int rc = connect(sock, (struct sockaddr *)&address, sizeof(address));
  if (rc < 0 && errno != EINPROGRESS) goto fail;
  if (rc < 0) {
    fd_set writable; FD_ZERO(&writable); FD_SET(sock, &writable);
    struct timeval timeout = {.tv_sec = TCP_SOCKET_TIMEOUT_SEC};
    if (select(sock + 1, NULL, &writable, NULL, &timeout) <= 0) goto fail;
    int error = 0; socklen_t len = sizeof(error);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error) goto fail;
  }
  fcntl(sock, F_SETFL, flags);
  struct timeval timeout = {.tv_sec = TCP_SOCKET_TIMEOUT_SEC};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  const uint8_t request[] = {TCP_HEADER_HIGH, TCP_HEADER_LOW, TCP_REQUEST_PAYLOAD_LEN, operation};
  
  for (size_t sent = 0; sent < sizeof(request);) {
    int n = send(sock, request + sent, sizeof(request) - sent, 0);
    if (n <= 0) goto fail;
    sent += n;
  }
  uint8_t prefix[TCP_PREFIX_BYTES];
  for (size_t got = 0; got < sizeof(prefix);) {
    int n = recv(sock, prefix + got, sizeof(prefix) - got, 0);
    if (n <= 0) goto fail;
    got += n;
  }
  if (prefix[TCP_HEADER_HIGH_OFFSET] != TCP_ACK_HIGH || prefix[TCP_HEADER_LOW_OFFSET] != TCP_ACK_LOW || prefix[TCP_LENGTH_OFFSET] == 0 || prefix[TCP_LENGTH_OFFSET] > cap) goto fail;
  for (size_t got = 0; got < prefix[TCP_LENGTH_OFFSET];) {
    int n = recv(sock, out + got, prefix[TCP_LENGTH_OFFSET] - got, 0);
    if (n <= 0) goto fail;
    got += n;
  }
  close(sock);
  return prefix[TCP_LENGTH_OFFSET];
fail:
  close(sock);
  return -1;
}
