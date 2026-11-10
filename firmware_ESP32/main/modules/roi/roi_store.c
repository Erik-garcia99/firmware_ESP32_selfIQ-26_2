#include "roi_store.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

static const char *TAG = "SELFIQ_ROI";
static SemaphoreHandle_t s_lock;
static roi_blob_t s_blob;
static bool s_valid;

esp_err_t roi_store_lock_init(void) {
  if (s_lock) return ESP_OK;
  s_lock = xSemaphoreCreateMutex();
  return s_lock ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t roi_store_init(void) {
  esp_err_t err = roi_store_lock_init();
  if (err != ESP_OK) return err;

  memset(&s_blob, 0, sizeof(s_blob));
  s_valid = false;

  nvs_handle_t handle;
  err = nvs_open(ROI_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) return err;

  roi_blob_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  size_t length = sizeof(tmp);
  err = nvs_get_blob(handle, ROI_NVS_KEY, &tmp, &length);
  nvs_close(handle);
  if (err != ESP_OK) return err;

  if (length != sizeof(tmp) || !roi_blob_verify(&tmp)) {
    ESP_LOGE(TAG, "ROI: NVS invalida (len=%u); se arranca sin zonas",
             (unsigned)length);
    return ESP_ERR_INVALID_CRC;
  }

  xSemaphoreTake(s_lock, portMAX_DELAY);
  s_blob = tmp;
  s_valid = true;
  xSemaphoreGive(s_lock);

  ESP_LOGI(TAG, "ROI: count=%u version=%lu crc=OK",
           (unsigned)s_blob.count, (unsigned long)s_blob.config_version);
  return ESP_OK;
}

esp_err_t roi_store_save(const roi_def_t *rois, size_t count,
                         uint32_t config_version) {
  if (count > ROI_MAX || (count && !rois))
    return ESP_ERR_INVALID_ARG;
  if (!s_lock) return ESP_ERR_INVALID_STATE;

  roi_blob_t next;
  memset(&next, 0, sizeof(next));
  next.count = (uint16_t)count;
  next.config_version = config_version;
  if (count)
    memcpy(next.rois, rois, count * sizeof(roi_def_t));
  roi_blob_seal(&next);

  /* Comparar en RAM, bajo lock, sin tocar NVS si es idéntico. */
  xSemaphoreTake(s_lock, portMAX_DELAY);
  bool same = s_valid && !memcmp(&next, &s_blob, sizeof(next));
  xSemaphoreGive(s_lock);
  if (same) {
    ESP_LOGI(TAG, "ROI sin cambios: count=%u version=%lu",
             (unsigned)count, (unsigned long)config_version);
    return ESP_OK;
  }

  /* NVS fuera del lock (es lento). Si falla, NO se publica el cambio. */
  nvs_handle_t handle;
  esp_err_t err = nvs_open(ROI_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ROI: nvs_open: %s", esp_err_to_name(err));
    return err;
  }
  err = nvs_set_blob(handle, ROI_NVS_KEY, &next, sizeof(next));
  if (err == ESP_OK)
    err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ROI: nvs write: %s", esp_err_to_name(err));
    return err;
  }

  xSemaphoreTake(s_lock, portMAX_DELAY);
  s_blob = next;
  s_valid = true;
  xSemaphoreGive(s_lock);

  ESP_LOGI(TAG, "NVS guardado: %u ROI, version=%lu, crc=OK",
           (unsigned)count, (unsigned long)config_version);
  return ESP_OK;
}

bool roi_store_snapshot(roi_blob_t *out) {
  if (!out || !s_lock) return false;
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(ROI_STORE_SNAPSHOT_TIMEOUT_MS))
      != pdTRUE)
    return false;
  bool ok = s_valid;
  if (ok) *out = s_blob;
  xSemaphoreGive(s_lock);
  return ok;
}

const roi_blob_t *roi_store_current(void) {
  /* No usar desde tareas concurrentes. Solo para arranque/diagnóstico. */
  return s_valid ? &s_blob : NULL;
}

bool roi_store_has_config(void) { return s_valid; }