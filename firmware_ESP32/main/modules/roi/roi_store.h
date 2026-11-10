#ifndef SELFIQ_ROI_STORE_H
#define SELFIQ_ROI_STORE_H

#include <stdbool.h>

#include "esp_err.h"
#include "roi_core.h"

#define ROI_NVS_NAMESPACE "selfiq_roi"
#define ROI_NVS_KEY "set"
#define ROI_STORE_SNAPSHOT_TIMEOUT_MS 200U

/* Crea el mutex interno. Idempotente. Llamar antes de roi_store_init().
 * Devuelve ESP_ERR_NO_MEM si no hay heap para crear el mutex. */
esp_err_t roi_store_lock_init(void);

/* Carga la configuracion al arranque. Devuelve ESP_ERR_NVS_NOT_FOUND si
 * todavia no hay zonas guardadas. Si magic/formato/CRC no coinciden deja el
 * estado vacio y devuelve ESP_ERR_INVALID_CRC. */
esp_err_t roi_store_init(void);

/* Guarda el conjunto y actualiza la copia en RAM. No escribe NVS si el blob
 * es identico al actual (evita desgaste). Thread-safe respecto a snapshot. */
esp_err_t roi_store_save(const roi_def_t *rois, size_t count,
                         uint32_t config_version);

/* Copia atomica del blob vigente bajo mutex. Devuelve false si no hay
 * configuracion valida o si no se pudo tomar el lock en
 * ROI_STORE_SNAPSHOT_TIMEOUT_MS. `out` es del llamador; NUNCA retiene un
 * puntero al estado interno, por lo que es seguro frente a roi_store_save()
 * concurrente. */
bool roi_store_snapshot(roi_blob_t *out);

/* Deprecado: retiene puntero al estado interno, no thread-safe frente a
 * roi_store_save(). Solo válido en arranque cuando nadie más escribe.
 * Migrar a roi_store_snapshot(); el compilador lo marcará como warning
 * hasta que se retire (E5.2). */
__attribute__((deprecated("usa roi_store_snapshot()")))
const roi_blob_t *roi_store_current(void);

bool roi_store_has_config(void);

#endif