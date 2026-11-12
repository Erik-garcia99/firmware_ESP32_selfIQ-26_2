#include "camera_lib.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "modules/MQTT/mqtt_lib.h"
#include "modules/calibration/calibration_frame.h"
#include "modules/roi/roi_store.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "global.h"

static const char *TAG = "SELFIQ_CAMERA";
static bool camera_ready = false;
static TaskHandle_t camera_task_handle = NULL;
static QueueHandle_t s_snapshot_queue = NULL;
static QueueHandle_t s_cal_queue = NULL;
static QueueHandle_t s_cal_upload_queue = NULL;
static TaskHandle_t s_cal_upload_task = NULL;
static SemaphoreHandle_t s_target_lock = NULL;
static camera_cal_result_cb_t s_cal_result_cb = NULL;
static cal_frame_stats_t s_cal_frame_stats;
static char s_gateway_ip[IPV4_TEXT_SIZE];
static char s_node_id[UUID_TEXT_SIZE];
static char s_branch_id[UUID_TEXT_SIZE];

/* GPIO del modelo AI-Thinker ESP32-CAM clasico. */
#define CAM_PIN_PWDN   32
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27
#define CAM_PIN_D7     35
#define CAM_PIN_D6     34
#define CAM_PIN_D5     39
#define CAM_PIN_D4     36
#define CAM_PIN_D3     21
#define CAM_PIN_D2     19
#define CAM_PIN_D1     18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22

/* Item interno del uploader: posee el JPEG hasta que la tarea de subida
 * termina. La cola copia la estructura, nunca el buffer de imagen. */
typedef struct {
    camera_cal_request_t request;
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
    uint32_t uptime_ms;
    uint8_t *jpeg;
    size_t jpeg_len;
} camera_cal_upload_t;

esp_err_t camera_init(void)
{
    if (camera_ready) return ESP_OK;

    bool psram_available = heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0;
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = CAMERA_XCLK_HZ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = CAMERA_INITIAL_FRAME_SIZE,
        .jpeg_quality = CAMERA_JPEG_QUALITY,
        .fb_count = CAMERA_FRAME_BUFFER_COUNT,
        .fb_location = psram_available ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    if (!psram_available) {
        ESP_LOGW(TAG, "PSRAM no disponible: usando QVGA en RAM interna");
    }
    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar camara: %s", esp_err_to_name(ret));
        return ret;
    }
    camera_ready = true;
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        ESP_LOGI(TAG, "Camara inicializada: PID=0x%04x (%s), PSRAM=%s",
                 sensor->id.PID, sensor->id.PID == OV3660_PID ? "OV3660" : "otro sensor",
                 psram_available ? "si" : "no");
    }
    return ESP_OK;
}

camera_fb_t *camera_capture(void)
{
    if (!camera_ready) {
        ESP_LOGE(TAG, "La camara no esta inicializada");
        return NULL;
    }
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) ESP_LOGE(TAG, "Fallo la captura");
    return frame;
}

void camera_release(camera_fb_t *frame)
{
    if (frame) esp_camera_fb_return(frame);
}

esp_err_t camera_set_resolution(framesize_t resolution)
{
    if (!camera_ready) return ESP_ERR_INVALID_STATE;
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) return ESP_FAIL;
    return sensor->set_framesize(sensor, resolution) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_set_quality(int quality)
{
    if (!camera_ready) return ESP_ERR_INVALID_STATE;
    if (quality < CAMERA_JPEG_QUALITY_MIN || quality > CAMERA_JPEG_QUALITY_MAX) return ESP_ERR_INVALID_ARG;
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) return ESP_FAIL;
    return sensor->set_quality(sensor, quality) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_deinit(void)
{
    if (!camera_ready) return ESP_OK;
    esp_err_t ret = esp_camera_deinit();
    if (ret == ESP_OK) camera_ready = false;
    return ret;
}

void camera_set_snapshot_target(const char *gateway_ip, const char *node_id,
                                const char *branch_id)
{
    if (!s_target_lock) return;
    xSemaphoreTake(s_target_lock, portMAX_DELAY);
    snprintf(s_gateway_ip, sizeof(s_gateway_ip), "%s", gateway_ip ? gateway_ip : "");
    snprintf(s_node_id, sizeof(s_node_id), "%s", node_id ? node_id : "");
    snprintf(s_branch_id, sizeof(s_branch_id), "%s", branch_id ? branch_id : "");
    xSemaphoreGive(s_target_lock);
}

esp_err_t camera_request_snapshot(const char *req_id)
{
    if (!req_id || !*req_id || strlen(req_id) > CAMERA_REQ_ID_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!s_snapshot_queue)
        return ESP_ERR_INVALID_STATE;

    char item[CAMERA_REQ_ID_SIZE] = {0};
    snprintf(item, sizeof(item), "%s", req_id);
    if (xQueueSend(s_snapshot_queue, item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "snapshot: cola llena; solicitud descartada");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t camera_request_calibration(const camera_cal_request_t *request)
{
    if (!request || !request->capture_id[0] || !request->roi_id[0] ||
        !request->calibration_id[0] || !request->context_hash[0] ||
        request->request_us <= 0)
        return ESP_ERR_INVALID_ARG;
    if (request->kind != CAMERA_CAL_KIND_EMPTY &&
        request->kind != CAMERA_CAL_KIND_FULL)
        return ESP_ERR_INVALID_ARG;
    if (!s_cal_queue) return ESP_ERR_INVALID_STATE;
    if (xQueueSend(s_cal_queue, request, 0) != pdTRUE) {
        ESP_LOGW(TAG, "CAL: ya hay una captura pendiente; se rechaza otra");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "CAL: captura encolada capture=%s kind=%s roi=%s",
             request->capture_id,
             request->kind == CAMERA_CAL_KIND_FULL ? "full" : "empty",
             request->roi_id);
    return ESP_OK;
}

void camera_set_cal_result_handler(camera_cal_result_cb_t handler)
{
    s_cal_result_cb = handler;
}

const char *camera_cal_status_code(camera_cal_status_t status)
{
    switch (status) {
        case CAMERA_CAL_OK: return "ok";
        case CAMERA_CAL_E_ARGUMENT: return "invalid_request";
        case CAMERA_CAL_E_CONTEXT: return "config_mismatch";
        case CAMERA_CAL_E_FRAME: return "invalid_frame";
        case CAMERA_CAL_E_DESCRIPTOR: return "invalid_frame";
        case CAMERA_CAL_E_JPEG: return "capture_failed";
        case CAMERA_CAL_E_NO_MEM: return "capture_failed";
        default: return "capture_failed";
    }
}

static int64_t frame_timestamp_us(const camera_fb_t *frame)
{
    return (int64_t)frame->timestamp.tv_sec * 1000000LL +
           (int64_t)frame->timestamp.tv_usec;
}

static uint32_t normalized_millionths(float value)
{
    if (!(value > 0.0f)) return 0U;
    if (value >= 1.0f) return 1000000U;
    return (uint32_t)(value * 1000000.0f + 0.5f);
}

/* Copia atómica del ROI vigente: nunca se retiene un puntero a roi_store. */
static bool calibration_roi(const char *roi_id, uint32_t config_version,
                            cal_roi_t *out)
{
    if (!roi_id || !out) return false;
    roi_blob_t snapshot;
    if (!roi_store_snapshot(&snapshot)) return false;
    if (snapshot.config_version != config_version) return false;
    for (size_t i = 0; i < snapshot.count; ++i) {
        if (strcmp(snapshot.rois[i].id, roi_id)) continue;
        out->x = normalized_millionths(snapshot.rois[i].x);
        out->y = normalized_millionths(snapshot.rois[i].y);
        out->w = normalized_millionths(snapshot.rois[i].w);
        out->h = normalized_millionths(snapshot.rois[i].h);
        return true;
    }
    return false;
}

static void camera_cal_report(const camera_cal_request_t *request,
                              camera_cal_status_t status,
                              cal_frame_err_t frame_error,
                              cal_err_t build_error,
                              const cal_capture_t *capture,
                              uint32_t uptime_ms,
                              bool upload_enqueued,
                              const char *detail)
{
    camera_cal_result_t result;
    memset(&result, 0, sizeof(result));
    result.request = *request;
    result.status = status;
    result.frame_error = frame_error;
    result.build_error = build_error;
    result.upload_enqueued = upload_enqueued;
    result.detail = detail;
    if (capture) {
        size_t copy = capture->descriptor_len;
        if (copy > sizeof(result.descriptor)) copy = sizeof(result.descriptor);
        memcpy(result.descriptor, capture->descriptor_bytes, copy);
        result.descriptor_len = copy;
        result.descriptor_crc = capture->descriptor_crc;
        result.mean = capture->descriptor.mean;
        result.contrast = capture->descriptor.contrast;
        result.clipped_pct = capture->descriptor.clipped_pct;
        memcpy(result.image_sha256, capture->image_sha256,
               sizeof(result.image_sha256));
        result.image_bytes = capture->image_bytes;
        result.width = capture->width;
        result.height = capture->height;
    }

    if (status == CAMERA_CAL_OK) {
        ESP_LOGI(TAG,
                 "CAL_CAPTURE_OK: capture=%s kind=%s crc=%08lx bytes=%lu "
                 "sha=%.16s... upload=%s",
                 request->capture_id,
                 request->kind == CAMERA_CAL_KIND_FULL ? "full" : "empty",
                 (unsigned long)result.descriptor_crc,
                 (unsigned long)result.image_bytes, result.image_sha256,
                 upload_enqueued ? "queued" : "skipped");
    } else {
        ESP_LOGW(TAG,
                 "CAL_CAPTURE_FAIL: capture=%s status=%d code=%s detail=%s "
                 "frame=%s build=%s rejected=%lu",
                 request->capture_id, (int)status,
                 camera_cal_status_code(status), detail ? detail : "?",
                 cal_frame_error_reason(frame_error),
                 cal_error_code(build_error),
                 (unsigned long)cal_frame_rejected_total(&s_cal_frame_stats));
    }

    if (s_cal_result_cb) s_cal_result_cb(&result);
}

/* Solo la llama la tarea de cámara. Toma uno o varios frames hasta obtener uno
 * posterior a la orden, extrae el descriptor del MISMO frame del que se genera
 * el JPEG y libera camera_fb_t en todas las ramas. */
static void camera_handle_calibration(const camera_cal_request_t *request)
{
    cal_roi_t roi;
    if (!calibration_roi(request->roi_id, request->config_version, &roi)) {
        camera_cal_report(request, CAMERA_CAL_E_CONTEXT, CAL_FRAME_OK, CAL_OK,
                          NULL, 0, false, "context_changed");
        return;
    }

    cal_frame_err_t last_frame_error = CAL_FRAME_OK;
    for (uint32_t attempt = 0; attempt < CAMERA_CAL_FRAME_ATTEMPTS; ++attempt) {
        camera_fb_t *frame = camera_capture();
        if (!frame) {
            last_frame_error = CAL_FRAME_E_ARGUMENT;
            vTaskDelay(pdMS_TO_TICKS(CAMERA_CAL_FRAME_RETRY_MS));
            continue;
        }

        const cal_frame_info_t info = {
            .is_grayscale = frame->format == PIXFORMAT_GRAYSCALE,
            .width = frame->width,
            .height = frame->height,
            .len = frame->len,
            .captured_us = frame_timestamp_us(frame),
        };
        const int64_t now_us = esp_timer_get_time();
        cal_frame_err_t frame_error =
            cal_frame_check(&info, CAMERA_CAL_FRAME_WIDTH,
                            CAMERA_CAL_FRAME_HEIGHT, request->request_us,
                            now_us, CAL_FRAME_MAX_AGE_MS, &s_cal_frame_stats);
        if ((frame_error == CAL_FRAME_E_STALE ||
             frame_error == CAL_FRAME_E_TOO_OLD) &&
            attempt + 1U < CAMERA_CAL_FRAME_ATTEMPTS) {
            last_frame_error = frame_error;
            camera_release(frame);
            vTaskDelay(pdMS_TO_TICKS(CAMERA_CAL_FRAME_RETRY_MS));
            continue;
        }
        if (frame_error != CAL_FRAME_OK) {
            camera_release(frame);
            camera_cal_report(request, CAMERA_CAL_E_FRAME, frame_error, CAL_OK,
                              NULL, 0, false,
                              cal_frame_error_reason(frame_error));
            return;
        }

        cal_capture_t capture;
        cal_err_t build_error =
            cal_capture_build(frame->buf, frame->len, frame->width,
                              frame->height, frame->width, roi, &capture);
        if (build_error != CAL_OK) {
            camera_release(frame);
            camera_cal_report(request, CAMERA_CAL_E_DESCRIPTOR, CAL_FRAME_OK,
                              build_error, NULL, 0, false,
                              cal_error_code(build_error));
            return;
        }

        uint8_t *jpeg = NULL;
        size_t jpeg_len = 0;
        if (!frame2jpg(frame, (uint8_t)CAL_CAPTURE_JPEG_QUALITY, &jpeg,
                       &jpeg_len) ||
            !jpeg || jpeg_len == 0U || jpeg_len > CAMERA_CAL_MAX_JPEG_BYTES) {
            free(jpeg);
            camera_release(frame);
            camera_cal_report(request, CAMERA_CAL_E_JPEG, CAL_FRAME_OK, CAL_OK,
                              NULL, 0, false, "jpeg_encode");
            return;
        }

        build_error = cal_capture_attach_jpeg(
            &capture, jpeg, jpeg_len, (uint16_t)frame->width,
            (uint16_t)frame->height);
        camera_release(frame);
        if (build_error != CAL_OK) {
            free(jpeg);
            camera_cal_report(request, CAMERA_CAL_E_JPEG, CAL_FRAME_OK,
                              build_error, NULL, 0, false,
                              cal_error_code(build_error));
            return;
        }

        /* Revalidar el contexto tras el trabajo costoso: un roi.set concurrente
         * no puede colar una captura de la configuración anterior. */
        cal_roi_t verified;
        if (!calibration_roi(request->roi_id, request->config_version, &verified)) {
            free(jpeg);
            camera_cal_report(request, CAMERA_CAL_E_CONTEXT, CAL_FRAME_OK,
                              CAL_OK, NULL, 0, false, "context_changed");
            return;
        }

        const uint32_t uptime_ms = (uint32_t)(now_us / 1000);
        bool upload_enqueued = false;
        const char *detail = NULL;
#if CAMERA_CAL_UPLOAD_ENABLE
        if (s_cal_upload_queue) {
            camera_cal_upload_t upload;
            memset(&upload, 0, sizeof(upload));
            upload.request = *request;
            memcpy(upload.descriptor, capture.descriptor_bytes,
                   capture.descriptor_len);
            upload.descriptor_len = capture.descriptor_len;
            upload.descriptor_crc = capture.descriptor_crc;
            upload.mean = capture.descriptor.mean;
            upload.contrast = capture.descriptor.contrast;
            upload.clipped_pct = capture.descriptor.clipped_pct;
            memcpy(upload.image_sha256, capture.image_sha256,
                   sizeof(upload.image_sha256));
            upload.image_bytes = capture.image_bytes;
            upload.width = capture.width;
            upload.height = capture.height;
            upload.uptime_ms = uptime_ms;
            upload.jpeg = jpeg;
            upload.jpeg_len = jpeg_len;
            if (xQueueSend(s_cal_upload_queue, &upload, 0) == pdTRUE) {
                upload_enqueued = true;
                jpeg = NULL; /* propiedad del uploader */
            } else {
                detail = "upload_queue_full";
            }
        } else {
            detail = "upload_disabled";
        }
#else
        detail = "upload_disabled";
#endif
        free(jpeg);
        if (!upload_enqueued) {
            ESP_LOGW(TAG, "CAL_UPLOAD_SKIPPED: capture=%s reason=%s",
                     request->capture_id, detail ? detail : "?");
        }
        camera_cal_report(request, CAMERA_CAL_OK, CAL_FRAME_OK, CAL_OK,
                          &capture, uptime_ms, upload_enqueued, detail);
        return;
    }

    camera_cal_report(request, CAMERA_CAL_E_FRAME, last_frame_error, CAL_OK,
                      NULL, 0, false, "no_fresh_frame");
}

/* Solo la llama la tarea de camara, con el frame todavia valido. */
static void camera_send_snapshot(camera_fb_t *frame, const char *req_id)
{
    char gateway[IPV4_TEXT_SIZE] = {0};
    char node[UUID_TEXT_SIZE] = {0};
    char branch[UUID_TEXT_SIZE] = {0};
    if (s_target_lock) {
        xSemaphoreTake(s_target_lock, portMAX_DELAY);
        memcpy(gateway, s_gateway_ip, sizeof(gateway));
        memcpy(node, s_node_id, sizeof(node));
        memcpy(branch, s_branch_id, sizeof(branch));
        xSemaphoreGive(s_target_lock);
    }
    if (!gateway[0] || !node[0] || !branch[0]) {
        ESP_LOGE(TAG, "snapshot: destino no configurado (gateway/identidad)");
        return;
    }

    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    if (!frame2jpg(frame, (uint8_t)CAMERA_JPEG_QUALITY, &jpg, &jpg_len) ||
        !jpg || jpg_len == 0) {
        ESP_LOGE(TAG, "snapshot: frame2jpg fallo");
        free(jpg);
        return;
    }

    ESP_LOGI(TAG, "snapshot: JPEG %ux%u, %u bytes; enviando al gateway",
             (unsigned)frame->width, (unsigned)frame->height, (unsigned)jpg_len);

    char url[128];
    snprintf(url, sizeof(url), "http://%s:80/camera/snapshot", gateway);
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "snapshot: no se pudo crear cliente HTTP");
        free(jpg);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_header(client, "X-Req-Id", req_id);
    esp_http_client_set_header(client, "X-Node-Id", node);
    esp_http_client_set_header(client, "X-Branch-Id", branch);
    esp_http_client_set_post_field(client, (const char *)jpg, (int)jpg_len);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    free(jpg);

    if (err == ESP_OK && status >= 200 && status < 300) {
        ESP_LOGI(TAG, "snapshot: enviado OK (HTTP %d) req=%s", status, req_id);
    } else {
        ESP_LOGE(TAG, "snapshot: fallo HTTP err=%s status=%d req=%s",
                 esp_err_to_name(err), status, req_id);
    }
}

static bool camera_cal_build_meta(const camera_cal_upload_t *upload, char *out,
                                  size_t capacity)
{
    if (!upload->request.issued_at[0]) return false;
    char descriptor_b64[CAL_CAPTURE_DESCRIPTOR_BYTES * 2U] = {0};
    size_t descriptor_b64_len = 0;
    if (cal_base64_encode(upload->descriptor, upload->descriptor_len,
                          descriptor_b64, sizeof(descriptor_b64),
                          &descriptor_b64_len) != CAL_OK ||
        descriptor_b64_len == 0)
        return false;

    int written = snprintf(
        out, capacity,
        "{\"calibration_id\":\"%s\",\"calibration_version\":%lu,"
        "\"roi_id\":\"%s\",\"capture_id\":\"%s\",\"kind\":\"%s\","
        "\"context_hash\":\"%s\",\"descriptor_crc\":\"%08lx\","
        "\"format_version\":%u,\"captured_at\":\"%s\","
        "\"clock_quality\":\"unknown\",\"uptime_ms\":%lu,"
        "\"image_sha256\":\"%s\",\"image_bytes\":%lu,"
        "\"width\":%u,\"height\":%u,\"descriptor\":\"%s\"}",
        upload->request.calibration_id,
        (unsigned long)upload->request.calibration_version,
        upload->request.roi_id, upload->request.capture_id,
        upload->request.kind == CAMERA_CAL_KIND_FULL ? "full" : "empty",
        upload->request.context_hash, (unsigned long)upload->descriptor_crc,
        (unsigned)CAL_DESCRIPTOR_FORMAT_VERSION, upload->request.issued_at,
        (unsigned long)upload->uptime_ms, upload->image_sha256,
        (unsigned long)upload->image_bytes, (unsigned)upload->width,
        (unsigned)upload->height, descriptor_b64);
    return written > 0 && (size_t)written < capacity;
}

static bool camera_cal_write(esp_http_client_handle_t client, const void *data,
                             size_t length)
{
    const uint8_t *bytes = data;
    size_t sent = 0;
    while (sent < length) {
        int written = esp_http_client_write(client, (const char *)bytes + sent,
                                            (int)(length - sent));
        if (written <= 0) return false;
        sent += (size_t)written;
    }
    return true;
}

/* Tarea independiente: una subida lenta no bloquea la captura ni MQTT. */
static void camera_cal_upload(camera_cal_upload_t *upload)
{
    char gateway[IPV4_TEXT_SIZE] = {0};
    char node[UUID_TEXT_SIZE] = {0};
    char branch[UUID_TEXT_SIZE] = {0};
    if (s_target_lock) {
        xSemaphoreTake(s_target_lock, portMAX_DELAY);
        memcpy(gateway, s_gateway_ip, sizeof(gateway));
        memcpy(node, s_node_id, sizeof(node));
        memcpy(branch, s_branch_id, sizeof(branch));
        xSemaphoreGive(s_target_lock);
    }
    if (!gateway[0] || !node[0] || !branch[0]) {
        ESP_LOGW(TAG, "CAL_UPLOAD_SKIPPED: capture=%s reason=no_target",
                 upload->request.capture_id);
        return;
    }

    char meta[1024];
    if (!camera_cal_build_meta(upload, meta, sizeof(meta))) {
        ESP_LOGW(TAG, "CAL_UPLOAD_SKIPPED: capture=%s reason=meta",
                 upload->request.capture_id);
        return;
    }

    static const char *boundary = "selfiq-cal-ref-2f4b";
    char meta_head[192];
    char image_head[224];
    int meta_head_len = snprintf(
        meta_head, sizeof(meta_head),
        "--%s\r\nContent-Disposition: form-data; name=\"meta\"\r\n"
        "Content-Type: application/json\r\n\r\n",
        boundary);
    int image_head_len = snprintf(
        image_head, sizeof(image_head),
        "\r\n--%s\r\nContent-Disposition: form-data; name=\"image\"; "
        "filename=\"reference.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n",
        boundary);
    char tail[96];
    int tail_len = snprintf(tail, sizeof(tail), "\r\n--%s--\r\n", boundary);
    if (meta_head_len <= 0 || image_head_len <= 0 || tail_len <= 0 ||
        (size_t)meta_head_len >= sizeof(meta_head) ||
        (size_t)image_head_len >= sizeof(image_head) ||
        (size_t)tail_len >= sizeof(tail)) {
        ESP_LOGW(TAG, "CAL_UPLOAD_SKIPPED: capture=%s reason=multipart",
                 upload->request.capture_id);
        return;
    }

    const size_t meta_len = strlen(meta);
    const size_t content_length = (size_t)meta_head_len + meta_len +
                                  (size_t)image_head_len + upload->jpeg_len +
                                  (size_t)tail_len;
    if (content_length > (size_t)CAMERA_CAL_MULTIPART_BYTES) {
        ESP_LOGW(TAG, "CAL_UPLOAD_SKIPPED: capture=%s reason=too_large",
                 upload->request.capture_id);
        return;
    }

    char url[128];
    snprintf(url, sizeof(url),
             "http://%s:80/camera/calibration-reference", gateway);
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = CAMERA_CAL_HTTP_TIMEOUT_MS,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGW(TAG, "CAL_UPLOAD_SKIPPED: capture=%s reason=no_client",
                 upload->request.capture_id);
        return;
    }

    char content_type[128];
    snprintf(content_type, sizeof(content_type),
             "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_header(client, "X-Node-Id", node);
    esp_http_client_set_header(client, "X-Branch-Id", branch);
    esp_http_client_set_header(client, "X-Capture-Id",
                               upload->request.capture_id);

    bool sent = esp_http_client_open(client, (int)content_length) == ESP_OK &&
                camera_cal_write(client, meta_head, (size_t)meta_head_len) &&
                camera_cal_write(client, meta, meta_len) &&
                camera_cal_write(client, image_head, (size_t)image_head_len) &&
                camera_cal_write(client, upload->jpeg, upload->jpeg_len) &&
                camera_cal_write(client, tail, (size_t)tail_len);
    if (sent) sent = esp_http_client_fetch_headers(client) >= 0;
    int status = sent ? esp_http_client_get_status_code(client) : 0;
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (sent && status >= 200 && status < 300) {
        ESP_LOGI(TAG,
                 "CAL_UPLOAD_OK: capture=%s HTTP %d bytes=%lu sha=%.16s...",
                 upload->request.capture_id, status,
                 (unsigned long)upload->image_bytes, upload->image_sha256);
    } else {
        ESP_LOGW(TAG,
                 "CAL_UPLOAD_FAIL: capture=%s HTTP %d bytes=%lu (recuperable)",
                 upload->request.capture_id, status,
                 (unsigned long)upload->image_bytes);
    }
}

static void camera_cal_upload_task(void *arg)
{
    (void)arg;
    camera_cal_upload_t upload;
    for (;;) {
        if (xQueueReceive(s_cal_upload_queue, &upload, portMAX_DELAY) != pdTRUE)
            continue;
        camera_cal_upload(&upload);
        free(upload.jpeg);
    }
}

/* Captura estable en grises + salud del nodo cada 60 s + snapshots.
 * Esta tarea es la unica que llama a camera_capture/frame2jpg. Solicitudes de
 * snapshot y de calibración llegan por colas separadas y se atienden en el
 * siguiente frame. La calibración tiene prioridad sobre el snapshot. */
static void camera_task(void *arg)
{
    (void)arg;
    uint32_t captured = 0;
    uint32_t failed = 0;
    uint32_t frames_in_window = 0;
    TickType_t last_health = xTaskGetTickCount();
    bool first_capture = true;
    char pending[CAMERA_REQ_ID_SIZE] = {0};

    for (;;) {
        if (!camera_ready) {
            if (camera_init() != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(CAMERA_INIT_RETRY_MS));
                continue;
            }
            ESP_LOGI(TAG,
                     "CAPTURE_STARTED: intervalo=%u ms; grises %ux%u, %u buffers PSRAM",
                     (unsigned)CAMERA_CAPTURE_INTERVAL_MS,
                     (unsigned)CAMERA_CAL_FRAME_WIDTH,
                     (unsigned)CAMERA_CAL_FRAME_HEIGHT,
                     (unsigned)CAMERA_FRAME_BUFFER_COUNT);
        }

        TickType_t cycle_start = xTaskGetTickCount();

        camera_cal_request_t cal_request;
        const bool has_calibration =
            s_cal_queue &&
            xQueueReceive(s_cal_queue, &cal_request, 0) == pdTRUE;

        if (has_calibration) {
            camera_handle_calibration(&cal_request);
        } else {
            int64_t started_us = esp_timer_get_time();
            camera_fb_t *frame = camera_capture();
            int64_t elapsed_us = esp_timer_get_time() - started_us;
            if (frame) {
                size_t width = frame->width;
                size_t height = frame->height;
                size_t bytes = frame->len;
                bool valid = frame->buf != NULL && bytes > 0 &&
                             frame->format == PIXFORMAT_GRAYSCALE;
                if (valid) {
                    ++captured;
                    ++frames_in_window;
                    if (first_capture || captured % CAMERA_LOG_EVERY_FRAMES == 0U) {
                        ESP_LOGI(TAG, "CAPTURE_OK: %ux%u bytes=%u capture_us=%lld ok=%lu fail=%lu heap=%u",
                                 (unsigned)width, (unsigned)height, (unsigned)bytes,
                                 (long long)elapsed_us, (unsigned long)captured,
                                 (unsigned long)failed,
                                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
                        first_capture = false;
                    }
                    if (pending[0]) {
                        camera_send_snapshot(frame, pending);
                        pending[0] = '\0';
                    }
                } else {
                    ++failed;
                    ESP_LOGW(TAG, "CAPTURE_INVALID: buffer vacio o formato no gris");
                    if (pending[0]) {
                        ESP_LOGE(TAG, "snapshot: captura invalida; solicitud descartada");
                        pending[0] = '\0';
                    }
                }
                camera_release(frame);
            } else {
                ++failed;
                if (pending[0]) {
                    ESP_LOGE(TAG, "snapshot: captura invalida; solicitud descartada");
                    pending[0] = '\0';
                }
            }
        }

        /* Publicar salud cada 60 s (fijo, no depende del perfil). */
        TickType_t now = xTaskGetTickCount();
        if (now - last_health >= pdMS_TO_TICKS(CAMERA_HEALTH_INTERVAL_MS)) {
            last_health = now;
            float fps = (float)frames_in_window * 1000.0f / (float)CAMERA_HEALTH_INTERVAL_MS;
            uint32_t heap = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            uint32_t uptime_s = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
            /* Perfil actual: 0 = normal (Fase 7B lo cambia via cfg.set). */
            const int profile = 0;

            esp_err_t r;
            r = mqtt_queue_value("cam_fps", (double)fps);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
                ESP_LOGW(TAG, "mqtt_queue_value(cam_fps)=%s", esp_err_to_name(r));
            r = mqtt_queue_value("cam_heap", (double)heap);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
                ESP_LOGW(TAG, "mqtt_queue_value(cam_heap)=%s", esp_err_to_name(r));
            r = mqtt_queue_value("cam_fails", (double)failed);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
                ESP_LOGW(TAG, "mqtt_queue_value(cam_fails)=%s", esp_err_to_name(r));
            r = mqtt_queue_value("cam_uptime", (double)uptime_s);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
                ESP_LOGW(TAG, "mqtt_queue_value(cam_uptime)=%s", esp_err_to_name(r));
            r = mqtt_queue_value("cam_profile", (double)profile);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
                ESP_LOGW(TAG, "mqtt_queue_value(cam_profile)=%s", esp_err_to_name(r));

            ESP_LOGI(TAG, "HEALTH: fps=%.1f heap=%u fails=%lu uptime=%us profile=%d frames_60s=%u cal_ok=%lu cal_rej=%lu",
                     (double)fps, (unsigned)heap, (unsigned long)failed,
                     (unsigned long)uptime_s, profile, (unsigned)frames_in_window,
                     (unsigned long)s_cal_frame_stats.accepted,
                     (unsigned long)cal_frame_rejected_total(&s_cal_frame_stats));
            frames_in_window = 0;
        }

        /* Esperar el resto del ciclo, despertando si llega un snapshot. */
        TickType_t elapsed_ticks = xTaskGetTickCount() - cycle_start;
        TickType_t interval = pdMS_TO_TICKS(CAMERA_CAPTURE_INTERVAL_MS);
        TickType_t wait = elapsed_ticks < interval ? interval - elapsed_ticks : 0;
        if (elapsed_ticks >= interval) {
            ESP_LOGW(TAG, "CAPTURE_SLOW: ciclo excede intervalo configurado");
            wait = interval;
        }
        char request[CAMERA_REQ_ID_SIZE];
        if (xQueueReceive(s_snapshot_queue, request, wait) == pdTRUE) {
            memcpy(pending, request, sizeof(pending));
        }
    }
}

esp_err_t camera_start_task(void)
{
    if (camera_task_handle != NULL) return ESP_OK;
    if (!s_snapshot_queue) {
        s_snapshot_queue = xQueueCreate(CAMERA_SNAPSHOT_QUEUE_LEN, CAMERA_REQ_ID_SIZE);
        if (!s_snapshot_queue) return ESP_ERR_NO_MEM;
    }
    if (!s_cal_queue) {
        s_cal_queue = xQueueCreate(CAMERA_CAL_QUEUE_LEN, sizeof(camera_cal_request_t));
        if (!s_cal_queue) return ESP_ERR_NO_MEM;
    }
#if CAMERA_CAL_UPLOAD_ENABLE
    if (!s_cal_upload_queue) {
        s_cal_upload_queue = xQueueCreate(CAMERA_CAL_UPLOAD_QUEUE_LEN,
                                          sizeof(camera_cal_upload_t));
        if (!s_cal_upload_queue) return ESP_ERR_NO_MEM;
    }
    if (!s_cal_upload_task &&
        xTaskCreate(camera_cal_upload_task, "cal_upload",
                    CAMERA_CAL_UPLOAD_TASK_STACK, NULL,
                    CAMERA_CAL_UPLOAD_TASK_PRIORITY, &s_cal_upload_task) != pdPASS) {
        s_cal_upload_task = NULL;
        return ESP_ERR_NO_MEM;
    }
#endif
    if (!s_target_lock) {
        s_target_lock = xSemaphoreCreateMutex();
        if (!s_target_lock) return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(camera_task, "selfiq_camera", CAMERA_TASK_STACK_BYTES,
                    NULL, CAMERA_TASK_PRIORITY, &camera_task_handle) != pdPASS) {
        camera_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
