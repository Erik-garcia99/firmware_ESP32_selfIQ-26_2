#ifndef SELFIQ_CALIBRATION_STORE_H
#define SELFIQ_CALIBRATION_STORE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "calibration_core.h"

/* Almacenamiento por ROI, nunca se guardan fotografias. UUID como texto. */
#define CAL_STORE_UUID_CHARS 36U
#define CAL_STORE_HASH_CHARS 64U
#define CAL_STORE_DESCRIPTOR_BYTES (CAL_DESCRIPTOR_HEADER_BYTES + CAL_DESCRIPTOR_CELLS)
#define CAL_STORE_NAMESPACE "selfiq_cal"

typedef enum {
    CAL_STORE_EMPTY = 0,
    CAL_STORE_FULL = 1
} cal_store_kind_t;

typedef struct {
    char roi_id[CAL_STORE_UUID_CHARS + 1U];
    char calibration_id[CAL_STORE_UUID_CHARS + 1U];
    char context_hash[CAL_STORE_HASH_CHARS + 1U];
    uint32_t config_version;
    uint32_t calibration_version;
    char capture_id[CAL_STORE_UUID_CHARS + 1U];
    cal_store_kind_t kind;
    const cal_descriptor_t *descriptor;
} cal_store_reference_t;

typedef struct {
    char roi_id[CAL_STORE_UUID_CHARS + 1U];
    char calibration_id[CAL_STORE_UUID_CHARS + 1U];
    char context_hash[CAL_STORE_HASH_CHARS + 1U];
    uint32_t config_version;
    uint32_t calibration_version;
    bool has_empty;
    bool has_full;
    char empty_capture_id[CAL_STORE_UUID_CHARS + 1U];
    char full_capture_id[CAL_STORE_UUID_CHARS + 1U];
    uint8_t empty_descriptor[CAL_STORE_DESCRIPTOR_BYTES];
    uint8_t full_descriptor[CAL_STORE_DESCRIPTOR_BYTES];
} cal_store_pair_t;

/* Requiere nvs_flash_init() previo. No modifica registros ni elimina NVS. */
esp_err_t calibration_store_init(void);
/* Guarda en un unico borrador durable por ROI. No reemplaza pareja activa. */
esp_err_t calibration_store_save_draft(const cal_store_reference_t *ref);
esp_err_t calibration_store_get_draft(const char *roi_id, cal_store_pair_t *out);
/* Solo cambia selector despues de escribir y verificar banco inactivo.
 * Los IDs deben coincidir con las capturas revisadas por backend. */
esp_err_t calibration_store_commit(const char *roi_id, const char *calibration_id,
                                   const char *empty_capture_id,
                                   const char *full_capture_id);
esp_err_t calibration_store_get_active(const char *roi_id, cal_store_pair_t *out);
/* Cuerpo JSON para status, desde NVS validada; ausencias son null, errores no.
 * Un borrador parcial usa null para el CRC de la referencia aún inexistente. */
esp_err_t calibration_store_status_body(const char *roi_id, char *out, size_t size);
/* Borra exclusivamente borrador de esa sesion; no modifica la pareja activa. */
esp_err_t calibration_store_cancel(const char *roi_id, const char *calibration_id);

#endif
