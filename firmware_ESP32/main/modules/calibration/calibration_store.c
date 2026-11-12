#include "calibration_store.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "nvs.h"
#include "esp_log.h"
#include "modules/roi/roi_store.h"
#include "modules/roi/roi_core.h"

static const char *TAG = "SELFIQ_CAL_STORE";

/* Formato binario fijo, sin structs con padding. Cada banco es autocontenido.
 * (magic, version, flags, IDs, hash, versiones, capturas, descriptores, CRC).
 */
#define CAL_RECORD_MAGIC 0x43414c31UL
#define CAL_RECORD_VERSION 1U
#define CAL_RECORD_ID_OFFSET 6U
#define CAL_RECORD_CAL_ID_OFFSET (CAL_RECORD_ID_OFFSET + CAL_STORE_UUID_CHARS)
#define CAL_RECORD_HASH_OFFSET (CAL_RECORD_CAL_ID_OFFSET + CAL_STORE_UUID_CHARS)
#define CAL_RECORD_CONFIG_OFFSET (CAL_RECORD_HASH_OFFSET + CAL_STORE_HASH_CHARS)
#define CAL_RECORD_VERSION_OFFSET (CAL_RECORD_CONFIG_OFFSET + 4U)
#define CAL_RECORD_EMPTY_ID_OFFSET (CAL_RECORD_VERSION_OFFSET + 4U)
#define CAL_RECORD_FULL_ID_OFFSET (CAL_RECORD_EMPTY_ID_OFFSET + CAL_STORE_UUID_CHARS)
#define CAL_RECORD_EMPTY_DESC_OFFSET (CAL_RECORD_FULL_ID_OFFSET + CAL_STORE_UUID_CHARS)
#define CAL_RECORD_FULL_DESC_OFFSET (CAL_RECORD_EMPTY_DESC_OFFSET + CAL_STORE_DESCRIPTOR_BYTES)
#define CAL_RECORD_CRC_OFFSET (CAL_RECORD_FULL_DESC_OFFSET + CAL_STORE_DESCRIPTOR_BYTES)
#define CAL_RECORD_LENGTH (CAL_RECORD_CRC_OFFSET + 4U)
#define CAL_FLAG_EMPTY 0x01U
#define CAL_FLAG_FULL 0x02U

static void write32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4U; ++i) p[i] = (uint8_t)(v >> (i * 8U));
}
static uint32_t read32(const uint8_t *p) {
    uint32_t v = 0U;
    for (unsigned i = 0; i < 4U; ++i) v |= (uint32_t)p[i] << (i * 8U);
    return v;
}
static uint32_t crc32_bytes(const uint8_t *p, size_t n) {
    uint32_t crc = 0xffffffffU;
    while (n--) {
        crc ^= *p++;
        for (unsigned b = 0; b < 8U; ++b)
            crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return crc ^ 0xffffffffU;
}
static bool uuid_ok(const char *id) {
    return id && strnlen(id, CAL_STORE_UUID_CHARS + 1U) == CAL_STORE_UUID_CHARS &&
           roi_uuid_valid(id);
}
static bool hash_ok(const char *hash) {
    if (!hash || strnlen(hash, CAL_STORE_HASH_CHARS + 1U) != CAL_STORE_HASH_CHARS)
        return false;
    for (size_t i = 0; i < CAL_STORE_HASH_CHARS; ++i)
        if (!isxdigit((unsigned char)hash[i])) return false;
    return true;
}
static bool roi_matches(const char *roi_id, uint32_t version) {
    const roi_blob_t *current = roi_store_current();
    if (!current || current->config_version != version) return false;
    for (size_t i = 0; i < current->count; ++i)
        if (!strcmp(current->rois[i].id, roi_id)) return true;
    return false;
}
static bool descriptor_ok(const uint8_t *src) {
    if (src[0] != CAL_DESCRIPTOR_FORMAT_VERSION ||
        src[1] != CAL_DESCRIPTOR_GRID_WIDTH) return false;
    cal_descriptor_t descriptor;
    return cal_descriptor_decode(src, CAL_STORE_DESCRIPTOR_BYTES, &descriptor) == CAL_OK;
}
static void encode(const cal_store_pair_t *p, uint8_t *dst) {
    memset(dst, 0, CAL_RECORD_LENGTH);
    write32(dst, CAL_RECORD_MAGIC);
    dst[4] = CAL_RECORD_VERSION;
    dst[5] = (p->has_empty ? CAL_FLAG_EMPTY : 0U) |
             (p->has_full ? CAL_FLAG_FULL : 0U);
    memcpy(dst + CAL_RECORD_ID_OFFSET, p->roi_id, CAL_STORE_UUID_CHARS);
    memcpy(dst + CAL_RECORD_CAL_ID_OFFSET, p->calibration_id, CAL_STORE_UUID_CHARS);
    memcpy(dst + CAL_RECORD_HASH_OFFSET, p->context_hash, CAL_STORE_HASH_CHARS);
    write32(dst + CAL_RECORD_CONFIG_OFFSET, p->config_version);
    write32(dst + CAL_RECORD_VERSION_OFFSET, p->calibration_version);
    if (p->has_empty) {
        memcpy(dst + CAL_RECORD_EMPTY_ID_OFFSET, p->empty_capture_id, CAL_STORE_UUID_CHARS);
        memcpy(dst + CAL_RECORD_EMPTY_DESC_OFFSET, p->empty_descriptor, CAL_STORE_DESCRIPTOR_BYTES);
    }
    if (p->has_full) {
        memcpy(dst + CAL_RECORD_FULL_ID_OFFSET, p->full_capture_id, CAL_STORE_UUID_CHARS);
        memcpy(dst + CAL_RECORD_FULL_DESC_OFFSET, p->full_descriptor, CAL_STORE_DESCRIPTOR_BYTES);
    }
    write32(dst + CAL_RECORD_CRC_OFFSET, crc32_bytes(dst, CAL_RECORD_CRC_OFFSET));
}
static esp_err_t decode(const uint8_t *src, cal_store_pair_t *p) {
    if (read32(src) != CAL_RECORD_MAGIC || src[4] != CAL_RECORD_VERSION ||
        (src[5] & ~(CAL_FLAG_EMPTY | CAL_FLAG_FULL)) != 0U ||
        read32(src + CAL_RECORD_CRC_OFFSET) != crc32_bytes(src, CAL_RECORD_CRC_OFFSET))
        return ESP_ERR_INVALID_CRC;
    memset(p, 0, sizeof(*p));
    memcpy(p->roi_id, src + CAL_RECORD_ID_OFFSET, CAL_STORE_UUID_CHARS);
    memcpy(p->calibration_id, src + CAL_RECORD_CAL_ID_OFFSET, CAL_STORE_UUID_CHARS);
    memcpy(p->context_hash, src + CAL_RECORD_HASH_OFFSET, CAL_STORE_HASH_CHARS);
    p->config_version = read32(src + CAL_RECORD_CONFIG_OFFSET);
    p->calibration_version = read32(src + CAL_RECORD_VERSION_OFFSET);
    p->has_empty = (src[5] & CAL_FLAG_EMPTY) != 0U;
    p->has_full = (src[5] & CAL_FLAG_FULL) != 0U;
    if (p->has_empty) {
        memcpy(p->empty_capture_id, src + CAL_RECORD_EMPTY_ID_OFFSET, CAL_STORE_UUID_CHARS);
        memcpy(p->empty_descriptor, src + CAL_RECORD_EMPTY_DESC_OFFSET, CAL_STORE_DESCRIPTOR_BYTES);
        if (!uuid_ok(p->empty_capture_id) || !descriptor_ok(p->empty_descriptor))
            return ESP_ERR_INVALID_CRC;
    }
    if (p->has_full) {
        memcpy(p->full_capture_id, src + CAL_RECORD_FULL_ID_OFFSET, CAL_STORE_UUID_CHARS);
        memcpy(p->full_descriptor, src + CAL_RECORD_FULL_DESC_OFFSET, CAL_STORE_DESCRIPTOR_BYTES);
        if (!uuid_ok(p->full_capture_id) || !descriptor_ok(p->full_descriptor))
            return ESP_ERR_INVALID_CRC;
    }
    if (!uuid_ok(p->roi_id) || !uuid_ok(p->calibration_id) ||
        !hash_ok(p->context_hash) || !p->calibration_version)
        return ESP_ERR_INVALID_CRC;
    return ESP_OK;
}
/* CRC32 de UUID completo para nombres cortos de clave. Una colision no se
 * acepta silenciosamente: cada blob contiene el UUID y se verifica al leer. */
static esp_err_t make_key(char kind, const char *roi, char dst[16]) {
    if (!uuid_ok(roi)) return ESP_ERR_INVALID_ARG;
    int n = snprintf(dst, 16, "%c%08lx", kind,
                     (unsigned long)crc32_bytes((const uint8_t *)roi, CAL_STORE_UUID_CHARS));
    return n == 9 ? ESP_OK : ESP_FAIL;
}
static esp_err_t load_blob(nvs_handle_t handle, const char *key,
                           const char *roi_id, cal_store_pair_t *out) {
    uint8_t bytes[CAL_RECORD_LENGTH];
    size_t len = sizeof(bytes);
    esp_err_t err = nvs_get_blob(handle, key, bytes, &len);
    if (err != ESP_OK) return err;
    if (len != sizeof(bytes)) return ESP_ERR_INVALID_SIZE;
    err = decode(bytes, out);
    if (err == ESP_OK && strcmp(roi_id, out->roi_id)) return ESP_ERR_INVALID_STATE;
    return err;
}
static esp_err_t write_blob(nvs_handle_t handle, const char *key,
                            const cal_store_pair_t *pair) {
    uint8_t bytes[CAL_RECORD_LENGTH];
    encode(pair, bytes);
    cal_store_pair_t existing;
    esp_err_t err = load_blob(handle, key, pair->roi_id, &existing);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
        return err; /* Incluye colision de clave y dato previo corrupto. */
    err = nvs_set_blob(handle, key, bytes, sizeof(bytes));
    if (err == ESP_OK) err = nvs_commit(handle);
    if (err != ESP_OK) return err;
    cal_store_pair_t check;
    err = load_blob(handle, key, pair->roi_id, &check);
    if (err != ESP_OK) return err;
    uint8_t verify[CAL_RECORD_LENGTH];
    encode(&check, verify);
    return memcmp(bytes, verify, sizeof(bytes)) == 0 ? ESP_OK : ESP_ERR_INVALID_CRC;
}
static esp_err_t selector(nvs_handle_t handle, const char *roi_id, uint8_t *value) {
    char key[16];
    esp_err_t err = make_key('s', roi_id, key);
    if (err != ESP_OK) return err;
    err = nvs_get_u8(handle, key, value);
    if (err == ESP_OK && *value > 1U) return ESP_ERR_INVALID_CRC;
    return err;
}

esp_err_t calibration_store_init(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(CAL_STORE_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_close(h);
    ESP_LOGI(TAG, "NVS lista: banco doble, descriptor 8x8, version=1");
    return ESP_OK;
}

static esp_err_t get_draft(const char *roi_id, cal_store_pair_t *out, bool current) {
    if (!out) return ESP_ERR_INVALID_ARG;
    char key[16];
    esp_err_t err = make_key('d', roi_id, key);
    if (err != ESP_OK) return err;
    nvs_handle_t h;
    err = nvs_open(CAL_STORE_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    err = load_blob(h, key, roi_id, out);
    nvs_close(h);
    if (err == ESP_OK && current && !roi_matches(roi_id, out->config_version))
        return ESP_ERR_INVALID_STATE;
    return err;
}

esp_err_t calibration_store_get_draft(const char *roi_id, cal_store_pair_t *out) {
    return get_draft(roi_id, out, true);
}

static esp_err_t get_active(const char *roi_id, cal_store_pair_t *out, bool current) {
    if (!out) return ESP_ERR_INVALID_ARG;
    char key[16];
    esp_err_t err = make_key('a', roi_id, key);
    if (err != ESP_OK) return err;
    nvs_handle_t h;
    err = nvs_open(CAL_STORE_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    uint8_t bank;
    err = selector(h, roi_id, &bank);
    if (err == ESP_OK) {
        key[0] = bank ? 'b' : 'a';
        err = load_blob(h, key, roi_id, out);
    }
    nvs_close(h);
    if (err == ESP_OK && (!out->has_empty || !out->has_full ||
                          (current && !roi_matches(roi_id, out->config_version))))
        return ESP_ERR_INVALID_STATE;
    return err;
}

esp_err_t calibration_store_get_active(const char *roi_id, cal_store_pair_t *out) {
    return get_active(roi_id, out, true);
}

esp_err_t calibration_store_save_draft(const cal_store_reference_t *ref) {
    if (!ref || !ref->descriptor || !uuid_ok(ref->roi_id) ||
        !uuid_ok(ref->calibration_id) || !uuid_ok(ref->capture_id) ||
        !hash_ok(ref->context_hash) || !ref->calibration_version ||
        (ref->kind != CAL_STORE_EMPTY && ref->kind != CAL_STORE_FULL))
        return ESP_ERR_INVALID_ARG;
    if (!roi_matches(ref->roi_id, ref->config_version)) return ESP_ERR_INVALID_STATE;
    uint8_t serialized[CAL_STORE_DESCRIPTOR_BYTES];
    size_t n = 0;
    if (cal_descriptor_encode(ref->descriptor, serialized, sizeof(serialized), &n) != CAL_OK ||
        n != sizeof(serialized) || ref->descriptor->grid_side != CAL_DESCRIPTOR_GRID_WIDTH)
        return ESP_ERR_INVALID_ARG;
    cal_store_pair_t pair;
    char key[16];
    esp_err_t err = make_key('d', ref->roi_id, key);
    if (err != ESP_OK) return err;
    nvs_handle_t h;
    err = nvs_open(CAL_STORE_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = load_blob(h, key, ref->roi_id, &pair);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        memset(&pair, 0, sizeof(pair));
    } else if (err == ESP_OK && pair.config_version != ref->config_version) {
        /* Borrador de una configuracion ROI invalidada; nunca se activa. */
        memset(&pair, 0, sizeof(pair));
    } else if (err != ESP_OK) {
        nvs_close(h);
        return err; /* Colision UUID o corrupcion: no sobrescribir. */
    }
    if (pair.calibration_id[0]) {
        if (strcmp(pair.calibration_id, ref->calibration_id)) {
            /* El backend cerró la sesión anterior y su cancelación nunca llegó
             * al nodo. Solo una versión mayor reemplaza el borrador: un replay
             * tardío de una operación vieja no debe pisar la sesión vigente. */
            if (ref->kind != CAL_STORE_EMPTY ||
                ref->calibration_version <= pair.calibration_version) {
                nvs_close(h);
                return ESP_ERR_INVALID_STATE;
            }
            ESP_LOGW(TAG, "Borrador de sesión anterior reemplazado: %lu -> %lu",
                     (unsigned long)pair.calibration_version,
                     (unsigned long)ref->calibration_version);
            memset(&pair, 0, sizeof(pair));
        } else if (strcmp(pair.context_hash, ref->context_hash) ||
                   pair.config_version != ref->config_version ||
                   pair.calibration_version != ref->calibration_version) {
            nvs_close(h);
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (ref->kind == CAL_STORE_FULL && !pair.has_empty) {
        nvs_close(h);
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(pair.roi_id, ref->roi_id, sizeof(pair.roi_id));
    memcpy(pair.calibration_id, ref->calibration_id, sizeof(pair.calibration_id));
    memcpy(pair.context_hash, ref->context_hash, sizeof(pair.context_hash));
    pair.config_version = ref->config_version;
    pair.calibration_version = ref->calibration_version;
    /* Repetir exactamente la misma captura no debe volver a escribir flash.
     * Un capture_id reutilizado con contenido diferente es conflicto. */
    const bool already_present = ref->kind == CAL_STORE_EMPTY ? pair.has_empty : pair.has_full;
    if (already_present) {
        const char *old_id = ref->kind == CAL_STORE_EMPTY ? pair.empty_capture_id : pair.full_capture_id;
        const uint8_t *old_bytes = ref->kind == CAL_STORE_EMPTY ? pair.empty_descriptor : pair.full_descriptor;
        if (!strcmp(old_id, ref->capture_id)) {
            if (!memcmp(old_bytes, serialized, sizeof(serialized))) {
                nvs_close(h);
                return ESP_OK;
            }
            nvs_close(h);
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (ref->kind == CAL_STORE_EMPTY) {
        pair.has_empty = true;
        memcpy(pair.empty_capture_id, ref->capture_id, sizeof(pair.empty_capture_id));
        memcpy(pair.empty_descriptor, serialized, sizeof(serialized));
    } else {
        pair.has_full = true;
        memcpy(pair.full_capture_id, ref->capture_id, sizeof(pair.full_capture_id));
        memcpy(pair.full_descriptor, serialized, sizeof(serialized));
    }
    err = write_blob(h, key, &pair);
    nvs_close(h);
    if (err == ESP_OK) ESP_LOGI(TAG, "Borrador guardado: tipo=%s version=%lu",
                              ref->kind == CAL_STORE_EMPTY ? "empty" : "full",
                              (unsigned long)pair.calibration_version);
    return err;
}

esp_err_t calibration_store_commit(const char *roi_id, const char *calibration_id,
                                   const char *empty_capture_id,
                                   const char *full_capture_id) {
    if (!uuid_ok(roi_id) || !uuid_ok(calibration_id) ||
        !uuid_ok(empty_capture_id) || !uuid_ok(full_capture_id))
        return ESP_ERR_INVALID_ARG;
    cal_store_pair_t pair;
    esp_err_t err = calibration_store_get_draft(roi_id, &pair);
    if (err != ESP_OK) return err;
    if (!pair.has_empty || !pair.has_full ||
        strcmp(pair.calibration_id, calibration_id) ||
        strcmp(pair.empty_capture_id, empty_capture_id) ||
        strcmp(pair.full_capture_id, full_capture_id)) return ESP_ERR_INVALID_STATE;
    if (!memcmp(pair.empty_descriptor, pair.full_descriptor, CAL_STORE_DESCRIPTOR_BYTES))
        return ESP_ERR_INVALID_STATE; /* Capturas identicas; similitud real: E5. */
    cal_store_pair_t active;
    esp_err_t active_err = calibration_store_get_active(roi_id, &active);
    if (active_err == ESP_OK &&
        !strcmp(active.calibration_id, pair.calibration_id)) {
        if (!strcmp(active.empty_capture_id, pair.empty_capture_id) &&
            !strcmp(active.full_capture_id, pair.full_capture_id) &&
            !memcmp(active.empty_descriptor, pair.empty_descriptor, CAL_STORE_DESCRIPTOR_BYTES) &&
            !memcmp(active.full_descriptor, pair.full_descriptor, CAL_STORE_DESCRIPTOR_BYTES))
            return ESP_OK; /* Replay idempotente de confirmacion. */
        return ESP_ERR_INVALID_STATE;
    }
    if (active_err == ESP_OK && active.calibration_version >= pair.calibration_version)
        return ESP_ERR_INVALID_STATE; /* Evitar rollback a una sesion antigua. */
    if (active_err != ESP_OK && active_err != ESP_ERR_NVS_NOT_FOUND &&
        active_err != ESP_ERR_INVALID_STATE)
        return active_err;
    char selector_key[16], bank_key[16];
    err = make_key('s', roi_id, selector_key);
    if (err != ESP_OK) return err;
    err = make_key('a', roi_id, bank_key);
    if (err != ESP_OK) return err;
    nvs_handle_t h;
    err = nvs_open(CAL_STORE_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    uint8_t old = 1U; /* Sin selector: escribir banco a (0). */
    esp_err_t sel_err = selector(h, roi_id, &old);
    if (sel_err != ESP_OK && sel_err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(h);
        return sel_err;
    }
    const uint8_t next = (uint8_t)(1U - old);
    bank_key[0] = next ? 'b' : 'a';
    err = write_blob(h, bank_key, &pair);
    if (err == ESP_OK) err = nvs_set_u8(h, selector_key, next);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) {
        /* E5.5: releer selector y banco recién escritos. El ACK de commit solo
         * es cierto si el almacenamiento confirma el cambio. */
        nvs_handle_t vh;
        cal_store_pair_t check;
        uint8_t confirmed = 0xFFU;
        esp_err_t verr = nvs_open(CAL_STORE_NAMESPACE, NVS_READONLY, &vh);
        if (verr == ESP_OK) {
            if (selector(vh, roi_id, &confirmed) != ESP_OK || confirmed != next) {
                verr = ESP_ERR_INVALID_STATE;
            } else if (load_blob(vh, bank_key, roi_id, &check) != ESP_OK) {
                verr = ESP_ERR_INVALID_STATE;
            } else {
                uint8_t expected[CAL_RECORD_LENGTH], actual[CAL_RECORD_LENGTH];
                encode(&pair, expected);
                encode(&check, actual);
                if (memcmp(expected, actual, sizeof(expected)))
                    verr = ESP_ERR_INVALID_CRC;
            }
            nvs_close(vh);
        }
        if (verr != ESP_OK) {
            ESP_LOGE(TAG, "Pareja no verificable en NVS: %s",
                     esp_err_to_name(verr));
            err = verr;
        }
    }
    if (err == ESP_OK) ESP_LOGI(TAG, "Pareja activada version=%lu banco=%u",
                              (unsigned long)pair.calibration_version, (unsigned)next);
    return err;
}

esp_err_t calibration_store_cancel(const char *roi_id, const char *calibration_id) {
    if (!uuid_ok(roi_id) || !uuid_ok(calibration_id)) return ESP_ERR_INVALID_ARG;
    char key[16];
    esp_err_t err = make_key('d', roi_id, key);
    if (err != ESP_OK) return err;
    nvs_handle_t h;
    err = nvs_open(CAL_STORE_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    cal_store_pair_t draft;
    /* Permitir cancelar un borrador obsoleto si version ROI cambio. */
    err = load_blob(h, key, roi_id, &draft);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(h);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        nvs_close(h);
        return err;
    }
    if (strcmp(draft.calibration_id, calibration_id)) {
        nvs_close(h);
        return ESP_ERR_INVALID_STATE;
    }
    err = nvs_erase_key(h, key);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static bool status_entry(char *out, size_t size, const cal_store_pair_t *pair,
                         bool draft) {
    char empty[11] = "null", full[11] = "null";
    if (pair->has_empty)
        snprintf(empty, sizeof(empty), "\"%08lx\"", (unsigned long)
                 crc32_bytes(pair->empty_descriptor, CAL_STORE_DESCRIPTOR_BYTES));
    if (pair->has_full)
        snprintf(full, sizeof(full), "\"%08lx\"", (unsigned long)
                 crc32_bytes(pair->full_descriptor, CAL_STORE_DESCRIPTOR_BYTES));
    int n = snprintf(out, size,
                     "{\"calibration_version\":%lu,\"%s\":\"%s\","
                     "\"empty_crc\":%s,\"full_crc\":%s}",
                     (unsigned long)pair->calibration_version,
                     draft ? "calibration_id" : "context_hash",
                     draft ? pair->calibration_id : pair->context_hash,
                     empty, full);
    return n >= 0 && (size_t)n < size;
}

esp_err_t calibration_store_status_body(const char *roi_id, char *out, size_t size) {
    if (!out || !size) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';
    cal_store_pair_t pair;
    char active[256] = "null", draft[256] = "null";
    /* Status informa también historia de otra config_version: el backend
     * compara el contexto. No confundir una pareja obsoleta con NVS corrupta. */
    esp_err_t err = get_active(roi_id, &pair, false);
    if (err == ESP_OK) {
        if (!status_entry(active, sizeof(active), &pair, false)) return ESP_ERR_INVALID_SIZE;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) return err;
    err = get_draft(roi_id, &pair, false);
    if (err == ESP_OK) {
        if (!status_entry(draft, sizeof(draft), &pair, true)) return ESP_ERR_INVALID_SIZE;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) return err;
    int n = snprintf(out, size, "\"active\":%s,\"draft\":%s", active, draft);
    if (n < 0 || (size_t)n >= size) {
        out[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}
