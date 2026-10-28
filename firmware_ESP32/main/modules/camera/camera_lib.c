#include "camera_lib.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SELFIQ_CAMERA";
static bool camera_ready = false;
static TaskHandle_t camera_task_handle = NULL;

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
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = CAMERA_INITIAL_FRAME_SIZE,
        .jpeg_quality = CAMERA_JPEG_QUALITY,
        .fb_count = CAMERA_FRAME_BUFFER_COUNT,
        .fb_location = psram_available ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
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

/* Diagnostico inicial: no publica imagenes ni inventa mediciones de ROI.
 * Siempre devuelve el buffer al driver antes de esperar el proximo ciclo. */
static void camera_task(void *arg)
{
    (void)arg;
    uint32_t captured = 0;
    uint32_t failed = 0;
    for (;;) {
        if (!camera_ready) {
            if (camera_init() != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(CAMERA_INIT_RETRY_MS));
                continue;
            }
            ESP_LOGI(TAG, "CAPTURE_STARTED: intervalo=%u ms; sin analisis ROI",
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
                         frame->format == PIXFORMAT_JPEG;
            camera_release(frame);
            if (valid) {
                ++captured;
                if (captured == 1U || captured % CAMERA_LOG_EVERY_FRAMES == 0U) {
                    ESP_LOGI(TAG, "CAPTURE_OK: %ux%u bytes=%u capture_us=%lld ok=%lu fail=%lu heap=%u",
                             (unsigned)width, (unsigned)height, (unsigned)bytes,
                             (long long)elapsed_us, (unsigned long)captured,
                             (unsigned long)failed,
                             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
                }
            } else {
                ++failed;
                ESP_LOGW(TAG, "CAPTURE_INVALID: buffer vacio o formato no JPEG");
            }
        } else {
            ++failed;
        }
        /* Si captura tarda demasiado, ceder CPU sin intentar recuperar ciclos. */
        TickType_t elapsed_ticks = xTaskGetTickCount() - cycle_start;
        TickType_t interval = pdMS_TO_TICKS(CAMERA_CAPTURE_INTERVAL_MS);
        if (elapsed_ticks < interval) {
            vTaskDelay(interval - elapsed_ticks);
        } else {
            ESP_LOGW(TAG, "CAPTURE_SLOW: ciclo excede intervalo configurado");
            vTaskDelay(interval);
        }
    }
}

esp_err_t camera_start_task(void)
{
    if (camera_task_handle != NULL) return ESP_OK;
    if (xTaskCreate(camera_task, "selfiq_camera", CAMERA_TASK_STACK_BYTES,
                    NULL, CAMERA_TASK_PRIORITY, &camera_task_handle) != pdPASS) {
        camera_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
