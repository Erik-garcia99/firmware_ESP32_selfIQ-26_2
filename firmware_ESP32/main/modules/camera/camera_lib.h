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
#define CAMERA_FRAME_BUFFER_COUNT 1U
#define CAMERA_CAPTURE_INTERVAL_MS 1000U
#define CAMERA_INIT_RETRY_MS 5000U
#define CAMERA_TASK_STACK_BYTES 4096U
#define CAMERA_TASK_PRIORITY 2U
#define CAMERA_LOG_EVERY_FRAMES 10U

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

#endif
