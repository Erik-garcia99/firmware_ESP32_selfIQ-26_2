#ifndef STUB_NVS_H
#define STUB_NVS_H
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef int nvs_handle_t;
#define NVS_DEFAULT_PART_NAME "nvs"
#define NVS_READONLY 0
#define NVS_READWRITE 1
esp_err_t nvs_open(const char *, int, nvs_handle_t *);
void nvs_close(nvs_handle_t);
esp_err_t nvs_get_blob(nvs_handle_t, const char *, void *, size_t *);
esp_err_t nvs_set_blob(nvs_handle_t, const char *, const void *, size_t);
esp_err_t nvs_get_u8(nvs_handle_t, const char *, uint8_t *);
esp_err_t nvs_set_u8(nvs_handle_t, const char *, uint8_t);
esp_err_t nvs_erase_key(nvs_handle_t, const char *);
esp_err_t nvs_erase_all(nvs_handle_t);
esp_err_t nvs_commit(nvs_handle_t);

typedef struct {
    size_t used_entries;
    size_t free_entries;
    size_t available_entries;
    size_t total_entries;
    size_t namespace_count;
} nvs_stats_t;
esp_err_t nvs_get_stats(const char *part_name, nvs_stats_t *stats);

/* Enumeracion de claves: imitacion minima de la API de ESP-IDF. */
typedef struct nvs_opaque_iterator_t *nvs_iterator_t;
typedef enum { NVS_TYPE_ANY = 0xff } nvs_type_t;
typedef struct {
    char namespace_name[16];
    nvs_type_t type;
    char key[16];
} nvs_entry_info_t;
esp_err_t nvs_entry_find(const char *part_name, const char *namespace_name,
                         nvs_type_t type, nvs_iterator_t *output_iterator);
esp_err_t nvs_entry_next(nvs_iterator_t *iterator);
esp_err_t nvs_entry_info(const nvs_iterator_t iterator, nvs_entry_info_t *out_info);
void nvs_release_iterator(nvs_iterator_t iterator);
#endif
