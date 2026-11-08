#include "roi_store.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "SELFIQ_ROI";
static roi_blob_t s_blob;
static bool s_valid = false;

esp_err_t roi_store_init(void) {
  memset(&s_blob, 0, sizeof(s_blob));
  s_valid = false;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(ROI_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK)
    return err;

  size_t length = sizeof(s_blob);
  err = nvs_get_blob(handle, ROI_NVS_KEY, &s_blob, &length);
  nvs_close(handle);
  if (err != ESP_OK)
    return err;

  if (length != sizeof(s_blob) || !roi_blob_verify(&s_blob)) {
    ESP_LOGE(TAG, "ROI: NVS invalida (len=%u); se arranca sin zonas",
             (unsigned)length);
    memset(&s_blob, 0, sizeof(s_blob));
    return ESP_ERR_INVALID_CRC;
  }

  s_valid = true;
  ESP_LOGI(TAG, "ROI: count=%u version=%lu crc=OK",
           (unsigned)s_blob.count, (unsigned long)s_blob.config_version);
  return ESP_OK;
}

esp_err_t roi_store_save(const roi_def_t *rois, size_t count,
                         uint32_t config_version) {
  if (count > ROI_MAX || (count && !rois))
    return ESP_ERR_INVALID_ARG;

  roi_blob_t next;
  memset(&next, 0, sizeof(next));
  next.count = (uint16_t)count;
  next.config_version = config_version;
  if (count)
    memcpy(next.rois, rois, count * sizeof(roi_def_t));
  roi_blob_seal(&next);

  if (s_valid && !memcmp(&next, &s_blob, sizeof(next))) {
    ESP_LOGI(TAG, "ROI sin cambios: count=%u version=%lu",
             (unsigned)count, (unsigned long)config_version);
    return ESP_OK;
  }

  nvs_handle_t handle;
  esp_err_t err = nvs_open(ROI_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ROI: no se pudo abrir NVS: %s", esp_err_to_name(err));
    return err;
  }
  err = nvs_set_blob(handle, ROI_NVS_KEY, &next, sizeof(next));
  if (err == ESP_OK)
    err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ROI: fallo al guardar en NVS: %s", esp_err_to_name(err));
    return err;
  }

  s_blob = next;
  s_valid = true;
  ESP_LOGI(TAG, "NVS guardado: %u ROI, version=%lu, crc=OK",
           (unsigned)count, (unsigned long)config_version);
  return ESP_OK;
}

const roi_blob_t *roi_store_current(void) { return s_valid ? &s_blob : NULL; }

bool roi_store_has_config(void) { return s_valid; }
