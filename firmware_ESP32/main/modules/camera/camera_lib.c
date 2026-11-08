#include "camera_lib.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "modules/MQTT/mqtt_lib.h"
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
static SemaphoreHandle_t s_target_lock = NULL;
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

/* Captura estable en grises + salud del nodo cada 60 s + snapshots.
 * Esta tarea es la unica que llama a camera_capture/frame2jpg. Solicitudes de
 * snapshot llegan por cola y se atienden en el siguiente frame (<=1 s). */
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
            ESP_LOGI(TAG, "CAPTURE_STARTED: intervalo=%u ms; grises QVGA, 2 buffers PSRAM",
                     (unsigned)CAMERA_CAPTURE_INTERVAL_MS);
        }

        TickType_t cycle_start = xTaskGetTickCount();
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

            ESP_LOGI(TAG, "HEALTH: fps=%.1f heap=%u fails=%lu uptime=%us profile=%d frames_60s=%u",
                     (double)fps, (unsigned)heap, (unsigned long)failed,
                     (unsigned long)uptime_s, profile, (unsigned)frames_in_window);
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
