#ifndef SELFIQ_CAMERA_LIB_H
#define SELFIQ_CAMERA_LIB_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"

#include "modules/calibration/calibration_capture.h"
#include "modules/calibration/calibration_frame.h"

/* Captura JPEG gris. El sensor del nodo es OV3660 (2048x1536): a QVGA se
 * descartaba el 97.6% de los pixeles y cada ROI quedaba con ~1 px por celda
 * del descriptor 8x8. VGA (640x480) cuadruplica la densidad y coincide con
 * camera_frame_width/height del esquema. Bajarla a QVGA solo si falta RAM
 * (VGA gris usa 307200 B por buffer, x2 en PSRAM). */
#define CAMERA_XCLK_HZ 20000000U
#define CAMERA_INITIAL_FRAME_SIZE FRAMESIZE_VGA
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

/* E5.3: contrato de frame de calibración. Debe coincidir con la resolución
 * configurada arriba y con la huella de contexto del backend (640x480).
 * Cambiar cualquiera de los dos exige subir camera_visual_revision en
 * esp32_node: el trigger marca las zonas como needs_recalibration. */
#define CAMERA_CAL_FRAME_WIDTH 640U
#define CAMERA_CAL_FRAME_HEIGHT 480U

/* Una sola operación física pendiente. Si la cola está llena, el consumidor
 * MQTT responde `busy`; nunca se encola una segunda captura en silencio. */
#define CAMERA_CAL_QUEUE_LEN 1U
#define CAMERA_CAL_FRAME_ATTEMPTS 4U
#define CAMERA_CAL_FRAME_RETRY_MS 40U
/* VGA gris con quality 12 ronda 40-70 KB; 64 KB rechazaba capturas buenas. */
#define CAMERA_CAL_MAX_JPEG_BYTES (96U * 1024U)
#define CAMERA_CAL_MULTIPART_BYTES (CAMERA_CAL_MAX_JPEG_BYTES + 2048U)

/* Uploader independiente: nunca comparte la tarea de cámara. La cola es
 * acotada y, si se llena, la captura sigue siendo válida y el fallo es
 * recuperable (no se bloquea el siguiente frame). */
#define CAMERA_CAL_UPLOAD_ENABLE 1
#define CAMERA_CAL_UPLOAD_QUEUE_LEN 1U
#define CAMERA_CAL_UPLOAD_TASK_STACK 6144U
#define CAMERA_CAL_UPLOAD_TASK_PRIORITY 3U
#define CAMERA_CAL_HTTP_TIMEOUT_MS 8000U

typedef enum {
    CAMERA_CAL_OK = 0,
    CAMERA_CAL_E_ARGUMENT,
    CAMERA_CAL_E_CONTEXT,
    CAMERA_CAL_E_FRAME,
    CAMERA_CAL_E_DESCRIPTOR,
    CAMERA_CAL_E_JPEG,
    CAMERA_CAL_E_NO_MEM,
} camera_cal_status_t;

enum {
    CAMERA_CAL_KIND_EMPTY = 0,
    CAMERA_CAL_KIND_FULL = 1,
};

/* Copia acotada de la orden `calibrate` (Anexo A.7.1). No contiene punteros ni
 * el payload MQTT. `request_us` es el instante monotónico en que el consumidor
 * aceptó la orden; el frame debe ser posterior. */
typedef struct {
    char calibration_id[CAL_CAPTURE_UUID_SIZE];
    char roi_id[CAL_CAPTURE_UUID_SIZE];
    char capture_id[CAL_CAPTURE_UUID_SIZE];
    char context_hash[CAL_CAPTURE_HASH_HEX_SIZE];
    char issued_at[CAL_CAPTURE_TIMESTAMP_SIZE];
    uint32_t config_version;
    uint32_t calibration_version;
    uint8_t kind;
    int64_t request_us;
} camera_cal_request_t;

/* Metadatos de la captura. El JPEG no se expone aquí: lo posee el uploader. */
typedef struct {
    camera_cal_request_t request;
    camera_cal_status_t status;
    cal_frame_err_t frame_error;
    cal_err_t build_error;
    uint8_t descriptor[CAL_CAPTURE_DESCRIPTOR_BYTES];
    size_t descriptor_len;
    uint32_t descriptor_crc;
    uint8_t mean;
    uint8_t contrast;
    uint8_t clipped_pct;
    char image_sha256[CAL_CAPTURE_SHA256_HEX_SIZE];
    uint32_t image_bytes;
    uint16_t width;
    uint16_t height;
    bool upload_enqueued;
    const char *detail; /* cadena estática; NULL si no hay detalle */
} camera_cal_result_t;

typedef void (*camera_cal_result_cb_t)(const camera_cal_result_t *result);

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

/* E5.3: encola una captura de calibración en la MISMA tarea propietaria de la
 * cámara, separada de `snapshot.request`. Devuelve ESP_ERR_NO_MEM si ya hay una
 * operación pendiente (el consumidor debe responder `busy`). */
esp_err_t camera_request_calibration(const camera_cal_request_t *request);

/* El handler se invoca desde la tarea de cámara, una vez construidos
 * descriptor/JPEG y encolada (o descartada) la subida. Debe ser corto y no
 * bloquear: E5.4 lo usará para persistir y publicar el ACK. */
void camera_set_cal_result_handler(camera_cal_result_cb_t handler);

/* Código estable para ACK/log (`invalid_frame`, `config_mismatch`, ...). */
const char *camera_cal_status_code(camera_cal_status_t status);

/* Destino del HTTP POST del snapshot. Llamar cuando la identidad y el broker
 * esten listos. Cadenas vacias deshabilitan el envio. */
void camera_set_snapshot_target(const char *gateway_ip, const char *node_id,
                                const char *branch_id);

#endif
