#ifndef SELFIQ_ROI_STORE_H
#define SELFIQ_ROI_STORE_H

#include "esp_err.h"
#include "roi_core.h"

#define ROI_NVS_NAMESPACE "selfiq_roi"
#define ROI_NVS_KEY "set"

/* Carga la configuracion al arranque. Devuelve ESP_ERR_NVS_NOT_FOUND si
 * todavia no hay zonas guardadas. Si magic/formato/CRC no coinciden deja el
 * estado vacio y devuelve ESP_ERR_INVALID_CRC. */
esp_err_t roi_store_init(void);

/* Guarda el conjunto y actualiza la copia en RAM. No escribe NVS si el blob
 * es identico al actual (evita desgaste). */
esp_err_t roi_store_save(const roi_def_t *rois, size_t count,
                         uint32_t config_version);

/* Copia vigente; NULL si nunca se guardo una configuracion valida. */
const roi_blob_t *roi_store_current(void);

bool roi_store_has_config(void);

#endif
