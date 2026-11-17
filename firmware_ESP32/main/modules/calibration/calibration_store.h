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
/* RAM-only generation for camera cache coherence. Odd = commit in progress.
 * Check before AND after loading/measuring. Not a persisted version number. */
uint32_t calibration_store_revision(void);
/* Cuerpo JSON para status, desde NVS validada; ausencias son null, errores no.
 * Un borrador parcial usa null para el CRC de la referencia aún inexistente. */
esp_err_t calibration_store_status_body(const char *roi_id, char *out, size_t size);
/* Borra exclusivamente borrador de esa sesion; no modifica la pareja activa. */
esp_err_t calibration_store_cancel(const char *roi_id, const char *calibration_id);

/* Referencia local del motor de ocupacion, una por ROI.
 * NO sustituye al descriptor 8x8 canonico: ese sigue siendo el formato que
 * viaja a app y backend. Este blob es interno del nodo, se construye en la
 * misma captura de calibracion (cuando el nodo todavia tiene el frame) y se
 * liga a config_version + calibration_version. Si la app recalibra, la version
 * cambia y la referencia vieja deja de aceptarse sin borrar nada.
 * Formato opaco de 16 B por tile (patron seguido de guarda). */
#define CAL_KERNEL_TILE_BYTES 16U
#define CAL_KERNEL_MAX_TILES 1024U

typedef struct {
    char roi_id[CAL_STORE_UUID_CHARS + 1U];
    uint32_t config_version;
    uint32_t calibration_version;
    uint32_t full_pixels; /* pixeles ocupados en la referencia de lleno */
    uint32_t rect[4];     /* x, y, ancho, alto en pixeles del frame */
    uint16_t tile_count;  /* tiles de patron */
    uint16_t guard_count; /* tiles de guarda de fondo */
    /* save: origen de los tiles. get: destino (tiles_capacity obligatorio). */
    uint8_t *tiles;
    size_t tiles_capacity;
    size_t tiles_bytes; /* save: debe ser 16*(tile_count+guard_count). get: leido */
} cal_kernel_ref_t;

/* Guarda o reemplaza la referencia de esa ROI. Rechaza si la ROI no pertenece
 * a la configuracion aplicada o si los conteos no cuadran con tiles_bytes. */
esp_err_t calibration_store_save_kernel_ref(const cal_kernel_ref_t *ref);
/* Lee la referencia solo si config_version y calibration_version coinciden con
 * `ref`; si cambio la calibracion devuelve ESP_ERR_INVALID_STATE. Con
 * ref->tiles NULL devuelve solo metadatos. */
esp_err_t calibration_store_get_kernel_ref(cal_kernel_ref_t *ref);
/* Actualiza solo los pixeles de lleno de una referencia ya guardada
 * (lectura-modificacion-escritura verificada). Se usa en la captura de "lleno",
 * cuando el nodo ya no tiene el frame vacio. */
esp_err_t calibration_store_set_kernel_full_pixels(const char *roi_id,
                                                   uint32_t config_version,
                                                   uint32_t calibration_version,
                                                   uint32_t full_pixels);
/* Borra la referencia de esa ROI. No toca pareja ni descriptores. */
esp_err_t calibration_store_drop_kernel_ref(const char *roi_id);

/* Informe del borrado total del namespace de calibracion. */
typedef struct {
    size_t keys_removed;
    size_t free_entries_before;
    size_t available_entries_before;
    size_t free_entries_after;
    size_t available_entries_after;
    bool verified_empty;
} cal_store_erase_report_t;

/* Mantenimiento explicito: borra TODAS las claves de CAL_STORE_NAMESPACE
 * (borradores, bancos y selectores de cualquier ROI). No abre ni modifica
 * selfiq_cfg (Wi-Fi/broker/identidad), selfiq_roi ni selfiq_vexp.
 * El nodo queda sin pareja de calibracion hasta que la app vuelva a calibrar.
 * Verifica que el namespace quedo vacio y devuelve estadisticas antes/despues.
 * Rechaza (ESP_ERR_INVALID_STATE) si hay un commit en curso. */
esp_err_t calibration_store_erase_all(cal_store_erase_report_t *report);

#endif
