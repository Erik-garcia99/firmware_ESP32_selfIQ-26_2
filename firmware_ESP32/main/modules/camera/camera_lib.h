#ifndef SELFIQ_CAMERA_LIB_H
#define SELFIQ_CAMERA_LIB_H

#include "esp_camera.h"
#include "esp_err.h"

/* Etapa 1: captura JPEG QVGA para verificar estabilidad junto con Wi-Fi. */
#define CAMERA_XCLK_HZ 20000000U
#define CAMERA_INITIAL_FRAME_SIZE FRAMESIZE_QVGA
#define CAMERA_JPEG_QUALITY 12
#define CAMERA_JPEG_QUALITY_MIN 0
#define CAMERA_JPEG_QUALITY_MAX 63
#define CAMERA_FRAME_BUFFER_COUNT 2U
#define CAMERA_CAPTURE_INTERVAL_MS 1000U
#define CAMERA_INIT_RETRY_MS 5000U
#define CAMERA_TASK_STACK_BYTES 8192U
#define CAMERA_HEALTH_INTERVAL_MS 60000U
#define CAMERA_TASK_PRIORITY 2U
#define CAMERA_LOG_EVERY_FRAMES 10U
#define CAMERA_REQ_ID_MAX 64U
#define CAMERA_REQ_ID_SIZE (CAMERA_REQ_ID_MAX + 1U)
#define CAMERA_SNAPSHOT_QUEUE_LEN 4U

/* Llamar desde app_main. ESP_OK indica tarea creada, no sensor listo.
 * Inicio idempotente; no llamar simultaneamente desde varias tareas.
 * Mientras corre, esta tarea es la unica propietaria de la camara:
 * no llamar capture/set/deinit desde otras tareas. */
esp_err_t camera_start_task(void);

/* AI-Thinker ESP32-CAM, compatible con OV2640/OV3660 si usan ese pinout. */
esp_err_t camera_init(void);
camera_fb_t *camera_capture(void);
void camera_release(camera_fb_t *frame);
esp_err_t camera_set_resolution(framesize_t resolution);
esp_err_t camera_set_quality(int quality); /* 0..63, menor = mejor JPEG */
esp_err_t camera_deinit(void);

/* La tarea de camara es la unica propietaria de esp_camera_fb_get/frame2jpg.
 * Encola una solicitud de snapshot; se atiende en el siguiente ciclo (<=1 s).
 * No bloquea ni captura desde la tarea MQTT. */
esp_err_t camera_request_snapshot(const char *req_id);

/* Destino del HTTP POST del snapshot. Llamar cuando la identidad y el broker
 * esten listos. Cadenas vacias deshabilitan el envio. */
void camera_set_snapshot_target(const char *gateway_ip, const char *node_id,
                                const char *branch_id);

#endif
