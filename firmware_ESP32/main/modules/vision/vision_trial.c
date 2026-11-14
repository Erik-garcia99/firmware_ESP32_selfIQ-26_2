#include "vision_trial.h"
#include "local_pattern.h"
#include "local_guard.h"
#include "vision_runtime.h"
#include "vision_pilot.h"
#include "modules/camera/camera_lib.h"
#include "modules/roi/roi_store.h"
#include "modules/calibration/calibration_store.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include <stdlib.h>
#include <string.h>

#define TRIAL_MAGIC 0x3158504cU
#define TRIAL_VERSION 1U
#define TRIAL_NAMESPACE "selfiq_vexp"
#define TRIAL_KEY "reference"
#define TRIAL_FRAMES 3U
#define TRIAL_BLOB_MAX 12000U
static const char *TAG = "VISION_TRIAL";
typedef struct {
    char req[37], session[37], roi[37], action[12];
    int64_t requested_us;
    bool discard_current;
} trial_request_t;
typedef struct {
    uint32_t magic, version, length, crc, full_pixels, tile_count, guard_count;
    uint32_t rect[4];
    cal_store_pair_t binding;
    char session[37];
} trial_header_t;
static QueueHandle_t queue;
static lp_context_t *pattern;
static lg_context_t *guard;
static trial_header_t header;
static unsigned stage; /* 0 uncalibrated, 1 empty_ready, 2 durable ready */
static bool attempted_load;
static char seen_req[8][37], seen_session[8][37], seen_action[8][12], seen_roi[8][37];
static unsigned seen_next;
static bool seen_discard_current[8];
/* Camera-task-only diagnostics; never stored in the reference format. */
static esp_err_t last_save_error;
static const char *last_save_step = "not_started";
static size_t last_save_bytes;
static nvs_stats_t cleanup_before;
static bool cleanup_before_valid;
static vision_pilot_health_t pilot_health;
static bool pilot_binding_checked, pilot_binding_valid;
static uint32_t pilot_binding_revision, pilot_binding_config;
static int64_t pilot_checked_us, pilot_previous_us;
/* Read-only sample diagnostics. No sensor setters or persisted-format changes.
 * Register values are read near capture, not atomically latched to the frame. */
static struct {
    bool present, sensor_ok;
    uint32_t roi_mean_q8, roi_clipped_pct, background_mean_q8;
    uint32_t reference_background_mean_q8, frame_crc, pattern_crc, guard_crc;
    int sensor_pid, manual, exposure_raw, gain_raw;
    int64_t captured_us, sensor_read_us;
} frame_diag;

static void diagnose_frame(const camera_fb_t *f) {
    memset(&frame_diag, 0, sizeof(frame_diag));
    uint64_t sum = 0, clipped_count = 0, background = 0, reference = 0;
    for (uint32_t y = pattern->y; y < pattern->y + pattern->h; ++y)
        for (uint32_t x = pattern->x; x < pattern->x + pattern->w; ++x) {
            unsigned value = f->buf[(size_t)y * f->width + x];
            sum += value;
            clipped_count += value <= 2 || value >= 253;
        }
    uint32_t pixels = pattern->w * pattern->h;
    frame_diag.roi_mean_q8 = (uint32_t)(sum * 256 / pixels);
    frame_diag.roi_clipped_pct = (uint32_t)(clipped_count * 100 / pixels);
    for (uint32_t i = 0; i < guard->count; ++i) {
        for (uint32_t y = 0; y < 16; ++y)
            for (uint32_t x = 0; x < 16; ++x)
                background += f->buf[(size_t)(guard->y[i] + y) * f->width +
                                     guard->x[i] + x];
        for (uint32_t k = 0; k < 16; ++k) reference += guard->ref[i][k];
    }
    if (guard->count) {
        frame_diag.background_mean_q8 = (uint32_t)(background / guard->count);
        frame_diag.reference_background_mean_q8 =
            (uint32_t)(reference * 16 / guard->count);
    }
    frame_diag.frame_crc = roi_crc32(f->buf, f->len);
    frame_diag.pattern_crc = roi_crc32((const uint8_t *)pattern->reference,
                                      pattern->count * sizeof(pattern->reference[0]));
    frame_diag.guard_crc = roi_crc32((const uint8_t *)guard->ref,
                                    guard->count * sizeof(guard->ref[0]));
    frame_diag.captured_us = (int64_t)f->timestamp.tv_sec * 1000000 + f->timestamp.tv_usec;
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) frame_diag.sensor_pid = sensor->id.PID;
    if (sensor && sensor->id.PID == OV3660_PID && sensor->get_reg) {
        frame_diag.manual = sensor->get_reg(sensor, 0x3503, 0x03);
        frame_diag.exposure_raw = sensor->get_reg(sensor, 0x3500, 0x0fffff);
        frame_diag.gain_raw = sensor->get_reg(sensor, 0x350a, 0x03ff);
        frame_diag.sensor_ok = frame_diag.manual >= 0 &&
            frame_diag.exposure_raw >= 0 && frame_diag.gain_raw >= 0;
    }
    frame_diag.sensor_read_us = esp_timer_get_time();
    frame_diag.present = true;
}

static void diagnostic_json(cJSON *root) {
    if (!frame_diag.present) return;
    cJSON *d = cJSON_AddObjectToObject(root, "diagnostics");
    frame_diag.present = false;
    if (!d) return;
    cJSON_AddNumberToObject(d, "version", 1);
    cJSON_AddNumberToObject(d, "roi_mean_q8", frame_diag.roi_mean_q8);
    cJSON_AddNumberToObject(d, "roi_clipped_pct", frame_diag.roi_clipped_pct);
    cJSON_AddNumberToObject(d, "background_mean_q8", frame_diag.background_mean_q8);
    cJSON_AddNumberToObject(d, "reference_background_mean_q8", frame_diag.reference_background_mean_q8);
    cJSON_AddNumberToObject(d, "frame_crc32", frame_diag.frame_crc);
    cJSON_AddNumberToObject(d, "pattern_crc32", frame_diag.pattern_crc);
    cJSON_AddNumberToObject(d, "guard_crc32", frame_diag.guard_crc);
    cJSON_AddNumberToObject(d, "full_reference_pixels", header.full_pixels);
    cJSON_AddNumberToObject(d, "captured_us", (double)frame_diag.captured_us);
    cJSON_AddNumberToObject(d, "sensor_read_us", (double)frame_diag.sensor_read_us);
    cJSON_AddNumberToObject(d, "sensor_pid", frame_diag.sensor_pid);
    cJSON_AddStringToObject(d, "sensor_status",
        frame_diag.sensor_ok ? "ok" :
        frame_diag.sensor_pid == OV3660_PID ? "read_failed" : "unsupported_or_unavailable");
    if (frame_diag.sensor_ok) {
        cJSON_AddBoolToObject(d, "auto_exposure", !(frame_diag.manual & 1));
        cJSON_AddBoolToObject(d, "auto_gain", !(frame_diag.manual & 2));
        cJSON_AddNumberToObject(d, "exposure_raw", frame_diag.exposure_raw);
        cJSON_AddNumberToObject(d, "gain_raw", frame_diag.gain_raw);
    } else {
        cJSON_AddNullToObject(d, "auto_exposure");
        cJSON_AddNullToObject(d, "auto_gain");
        cJSON_AddNullToObject(d, "exposure_raw");
        cJSON_AddNullToObject(d, "gain_raw");
    }
}

static uint32_t normalized(float v) {
    return (uint32_t)(v * 1000000.0f + 0.5f);
}
static bool current_binding(const char *roi, cal_store_pair_t *pair, cal_rect_t *rect) {
    roi_blob_t snap;
    memset(pair, 0, sizeof(*pair));
    if (!roi_store_snapshot(&snap) ||
        calibration_store_get_active(roi, pair) != ESP_OK ||
        !pair->has_empty || !pair->has_full || pair->config_version != snap.config_version)
        return false;
    for (size_t i = 0; i < snap.count; ++i) {
        roi_def_t *r = &snap.rois[i];
        if (strcmp(r->id, roi)) continue;
        cal_roi_t geometry = {normalized(r->x), normalized(r->y),
                              normalized(r->w), normalized(r->h)};
        return cal_roi_to_rect(CAMERA_CAL_FRAME_WIDTH, CAMERA_CAL_FRAME_HEIGHT,
                               geometry, rect) == CAL_OK;
    }
    return false;
}
static bool same_binding(const cal_store_pair_t *a, const cal_store_pair_t *b) {
    return a->config_version == b->config_version &&
           a->calibration_version == b->calibration_version &&
           !memcmp(a->roi_id, b->roi_id, sizeof(a->roi_id)) &&
           !memcmp(a->calibration_id, b->calibration_id, sizeof(a->calibration_id)) &&
           !memcmp(a->context_hash, b->context_hash, sizeof(a->context_hash)) &&
           !memcmp(a->empty_capture_id, b->empty_capture_id, sizeof(a->empty_capture_id)) &&
           !memcmp(a->full_capture_id, b->full_capture_id, sizeof(a->full_capture_id)) &&
           !memcmp(a->empty_descriptor, b->empty_descriptor, sizeof(a->empty_descriptor)) &&
           !memcmp(a->full_descriptor, b->full_descriptor, sizeof(a->full_descriptor));
}
static bool bound_now(const char *roi) {
    cal_store_pair_t pair;
    cal_rect_t r;
    return current_binding(roi, &pair, &r) && same_binding(&header.binding, &pair) &&
           header.rect[0] == r.left && header.rect[1] == r.top &&
           header.rect[2] == (uint32_t)(r.right - r.left) &&
           header.rect[3] == (uint32_t)(r.bottom - r.top);
}
static const char *state(void) {
    return stage == 2 ? "ready" : stage == 1 ? "empty_ready" : "uncalibrated";
}
static void reply(const trial_request_t *q, const char *status,
                  const lp_result_t *result, unsigned index,
                  uint32_t matched, uint32_t informative, int64_t elapsed) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "type", "vision.experimental.result");
    cJSON_AddBoolToObject(root, "experimental", true);
    cJSON_AddStringToObject(root, "req_id", q->req);
    cJSON_AddStringToObject(root, "session_id", q->session);
    cJSON_AddStringToObject(root, "roi_id", q->roi);
    cJSON_AddStringToObject(root, "action", q->action);
    cJSON_AddStringToObject(root, "status", status);
    cJSON_AddStringToObject(root, "state", state());
    cJSON_AddBoolToObject(root, "persisted", stage == 2);
    cJSON_AddBoolToObject(root, "valid", result && result->relative_pct != 255);
    cJSON_AddNumberToObject(root, "sample_index", index);
    cJSON_AddNumberToObject(root, "processing_us", (double)elapsed);
    cJSON_AddNumberToObject(root, "uptime_ms", (double)(esp_timer_get_time() / 1000));
    cJSON_AddNumberToObject(root, "matched", matched);
    cJSON_AddNumberToObject(root, "informative", informative);
    cJSON_AddNumberToObject(root, "heap_internal",
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(root, "free_psram", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    bool cleanup = !strcmp(q->action, "cleanup");
    bool storage_failure = !strcmp(status, "persistence_or_context_failed") ||
                           !strcmp(status, "cleanup_failed");
    if (storage_failure) {
        cJSON_AddStringToObject(root, "failure_reason",
            cleanup ? "cleanup_failed" :
            last_save_error == ESP_OK ? "context_changed" : "persistence_failed");
        cJSON_AddStringToObject(root, "storage_error", esp_err_to_name(last_save_error));
        cJSON_AddNumberToObject(root, "storage_error_code", last_save_error);
        cJSON_AddStringToObject(root, "storage_step",
            !cleanup && last_save_error == ESP_OK ? "context_recheck" : last_save_step);
        cJSON_AddNumberToObject(root, "reference_bytes", last_save_bytes);
    }
    if (storage_failure || cleanup) {
        nvs_stats_t stats;
        esp_err_t stats_err = nvs_get_stats(NULL, &stats);
        if (stats_err == ESP_OK) {
            cJSON_AddNumberToObject(root, "nvs_free_entries", stats.free_entries);
            cJSON_AddNumberToObject(root, "nvs_available_entries", stats.available_entries);
        } else cJSON_AddStringToObject(root, "nvs_stats_error", esp_err_to_name(stats_err));
    }
    if (cleanup) {
        cJSON_AddStringToObject(root, "cleanup_scope", TRIAL_NAMESPACE "/" TRIAL_KEY);
        cJSON_AddStringToObject(root, "cleanup_policy",
            q->discard_current ? "discard_matching_session" : "obsolete_only");
        cJSON_AddBoolToObject(root, "removed", !strcmp(status, "cleanup_completed"));
        if (cleanup_before_valid)
            cJSON_AddNumberToObject(root, "nvs_available_entries_before",
                                   cleanup_before.available_entries);
    }
    if (result) {
        cJSON_AddNumberToObject(root, "area_pct", result->area_pct);
        cJSON_AddNumberToObject(root, "relative_unclamped_q8", result->relative_unclamped_q8);
        cJSON_AddNumberToObject(root, "weak_reference_pixels", result->weak_reference_pixels);
    }
    if (result && result->relative_pct != 255)
        cJSON_AddNumberToObject(root, "occupancy_pct", result->relative_pct);
    else cJSON_AddNullToObject(root, "occupancy_pct");
    diagnostic_json(root);
    char *text = cJSON_PrintUnformatted(root);
    if (text) {
        ESP_LOGI(TAG, "%s", text);
        esp_err_t err = mqtt_publish_trial_json(text);
        if (err != ESP_OK) ESP_LOGW(TAG, "diagnostic delivery: %s", esp_err_to_name(err));
        cJSON_free(text);
    }
    cJSON_Delete(root);
}
static bool allocate(void) {
    if (!pattern) pattern = heap_caps_calloc(1, sizeof(*pattern), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!guard) guard = heap_caps_calloc(1, sizeof(*guard), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return pattern && guard;
}
static bool setup(const trial_header_t *h) {
    return lp_init(pattern, CAMERA_CAL_FRAME_WIDTH, CAMERA_CAL_FRAME_HEIGHT,
                   h->rect[0], h->rect[1], h->rect[2], h->rect[3]) == LP_OK &&
           !lg_init(guard, CAMERA_CAL_FRAME_WIDTH, CAMERA_CAL_FRAME_HEIGHT,
                    h->rect[0], h->rect[1], h->rect[2], h->rect[3]);
}
static size_t blob_size(void) {
    return sizeof(header) + 16U * (pattern->count + guard->count);
}
static esp_err_t save(void) {
    size_t size = blob_size();
    last_save_bytes = size;
    last_save_error = ESP_OK;
    last_save_step = "size_check";
    if (size > TRIAL_BLOB_MAX) return last_save_error = ESP_ERR_INVALID_SIZE;
    last_save_step = "allocate_buffers";
    uint8_t *bytes = heap_caps_calloc(1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *verify = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!bytes || !verify) {
        free(bytes); free(verify);
        return last_save_error = ESP_ERR_NO_MEM;
    }
    header.magic = TRIAL_MAGIC; header.version = TRIAL_VERSION;
    header.length = size; header.crc = 0;
    header.tile_count = pattern->count; header.guard_count = guard->count;
    memcpy(bytes, &header, sizeof(header));
    size_t at = sizeof(header);
    for (uint32_t i = 0; i < pattern->count; ++i)
        for (unsigned k = 0; k < 16; ++k) bytes[at++] = (uint8_t)pattern->reference[i][k];
    for (uint32_t i = 0; i < guard->count; ++i)
        for (unsigned k = 0; k < 16; ++k) bytes[at++] = (uint8_t)guard->ref[i][k];
    ((trial_header_t *)bytes)->crc = roi_crc32(bytes, size);
    nvs_handle_t h;
    last_save_step = "nvs_open";
    esp_err_t err = nvs_open(TRIAL_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        last_save_step = "nvs_set_blob";
        err = nvs_set_blob(h, TRIAL_KEY, bytes, size);
        if (err == ESP_OK) {
            last_save_step = "nvs_commit";
            err = nvs_commit(h);
        }
        size_t read_size = size;
        if (err == ESP_OK) {
            last_save_step = "nvs_readback";
            err = nvs_get_blob(h, TRIAL_KEY, verify, &read_size);
        }
        if (err == ESP_OK) {
            last_save_step = "verify_bytes";
            if (read_size != size || memcmp(bytes, verify, size)) err = ESP_FAIL;
        }
        nvs_close(h);
    }
    free(bytes); free(verify);
    last_save_error = err;
    return last_save_error;
}
static void load(const char *roi) {
    attempted_load = true;
    nvs_handle_t h;
    if (nvs_open(TRIAL_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    size_t size = 0;
    esp_err_t err = nvs_get_blob(h, TRIAL_KEY, NULL, &size);
    if (err != ESP_OK || size < sizeof(header) || size > TRIAL_BLOB_MAX) { nvs_close(h); return; }
    uint8_t *bytes = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!bytes) { nvs_close(h); return; }
    err = nvs_get_blob(h, TRIAL_KEY, bytes, &size);
    nvs_close(h);
    if (err != ESP_OK) { free(bytes); return; }
    trial_header_t *saved = (trial_header_t *)bytes;
    uint32_t crc = saved->crc;
    saved->crc = 0;
    bool valid = saved->magic == TRIAL_MAGIC && saved->version == TRIAL_VERSION &&
                 saved->length == size && roi_crc32(bytes, size) == crc &&
                 saved->session[36] == '\0' && roi_uuid_valid(saved->session);
    if (valid) {
        header = *saved;
        valid = bound_now(roi) && setup(&header) &&
                header.tile_count == pattern->count && header.guard_count == guard->count &&
                blob_size() == size && header.full_pixels > 0 &&
                header.full_pixels <= pattern->w * pattern->h &&
                (uint64_t)header.full_pixels * 25 >= pattern->w * pattern->h;
    }
    if (valid) {
        size_t at = sizeof(header);
        for (uint32_t i = 0; i < pattern->count; ++i)
            for (unsigned k = 0; k < 16; ++k) pattern->reference[i][k] = bytes[at++];
        for (uint32_t i = 0; i < guard->count; ++i)
            for (unsigned k = 0; k < 16; ++k) guard->ref[i][k] = bytes[at++];
        pattern->samples = TRIAL_FRAMES; pattern->ready = 1;
        guard->samples = TRIAL_FRAMES; guard->ready = 1;
        stage = 2;
    }
    free(bytes);
}

/* 1 definitely obsolete, 0 current, -1 cannot establish safely.
 * Check the stored ROI's own context, not just the requested ROI: another
 * still-valid experimental ROI must never be mistaken for garbage. */
static int obsolete_reference(const trial_header_t *saved) {
    roi_blob_t snap;
    if (!roi_store_snapshot(&snap)) return -1;
    bool found = false;
    for (size_t i = 0; i < snap.count; ++i)
        if (!memcmp(snap.rois[i].id, saved->binding.roi_id, sizeof(snap.rois[i].id)))
            found = true;
    if (!found || snap.config_version != saved->binding.config_version) return 1;
    cal_store_pair_t active;
    cal_rect_t rect;
    if (!current_binding(saved->binding.roi_id, &active, &rect)) return -1;
    return !same_binding(&saved->binding, &active) ||
           saved->rect[0] != rect.left || saved->rect[1] != rect.top ||
           saved->rect[2] != (uint32_t)(rect.right - rect.left) ||
           saved->rect[3] != (uint32_t)(rect.bottom - rect.top);
}

static void discard_trial_ram(void) {
    stage = 0;
    attempted_load = true;
    memset(&header, 0, sizeof(header));
    free(pattern); pattern = NULL;
    free(guard); guard = NULL;
}

static const char *cleanup_reference(const trial_request_t *q) {
    last_save_error = ESP_OK;
    last_save_bytes = 0;
    cleanup_before_valid = nvs_get_stats(NULL, &cleanup_before) == ESP_OK;
    /* Explicit discard must not invalidate another session pending in RAM. */
    if (q->discard_current && stage &&
        (strcmp(header.session, q->session) || strcmp(header.binding.roi_id, q->roi)))
        return "cleanup_refused_ram_session";
    nvs_handle_t h;
    last_save_step = "cleanup_open_readonly";
    esp_err_t err = nvs_open(TRIAL_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        if (q->discard_current) discard_trial_ram();
        return "cleanup_no_reference";
    }
    if (err != ESP_OK) { last_save_error = err; return "cleanup_failed"; }
    last_save_step = "cleanup_read_size";
    size_t size = 0;
    err = nvs_get_blob(h, TRIAL_KEY, NULL, &size);
    if (err != ESP_OK) {
        nvs_close(h);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            if (q->discard_current) discard_trial_ram();
            return "cleanup_no_reference";
        }
        last_save_error = err;
        return "cleanup_failed";
    }
    last_save_bytes = size;
    last_save_step = "cleanup_validate_blob";
    if (size < sizeof(header) || size > TRIAL_BLOB_MAX) {
        nvs_close(h); last_save_error = ESP_ERR_INVALID_SIZE;
        return "cleanup_failed";
    }
    uint8_t *bytes = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!bytes) {
        nvs_close(h); last_save_error = ESP_ERR_NO_MEM;
        return "cleanup_failed";
    }
    err = nvs_get_blob(h, TRIAL_KEY, bytes, &size);
    nvs_close(h);
    if (err != ESP_OK) { free(bytes); last_save_error = err; return "cleanup_failed"; }
    trial_header_t saved = *(trial_header_t *)bytes;
    ((trial_header_t *)bytes)->crc = 0;
    bool valid = saved.magic == TRIAL_MAGIC && saved.version == TRIAL_VERSION &&
        saved.length == size && roi_crc32(bytes, size) == saved.crc &&
        saved.binding.roi_id[36] == '\0' && roi_uuid_valid(saved.binding.roi_id) &&
        saved.tile_count > 0 && saved.tile_count <= LP_MAX_TILES &&
        saved.guard_count >= 16 && saved.guard_count <= LG_MAX &&
        sizeof(header) + 16U * (saved.tile_count + saved.guard_count) == size;
    free(bytes);
    if (!valid) { last_save_error = ESP_ERR_INVALID_STATE; return "cleanup_failed"; }
    if (q->discard_current &&
        (memcmp(saved.binding.roi_id, q->roi, sizeof(q->roi)) ||
         memcmp(saved.session, q->session, sizeof(q->session))))
        return "cleanup_refused_session";
    cal_store_pair_t target, recheck;
    cal_rect_t rect, again;
    if (!current_binding(q->roi, &target, &rect)) return "cleanup_context_unavailable";
    if (!q->discard_current) {
        int obsolete = obsolete_reference(&saved);
        if (!obsolete) return "cleanup_refused_current";
        if (obsolete < 0) return "cleanup_context_unavailable";
    }
    /* Recheck after reads, just before mutation; no automatic delete on save
     * failure and no erase-all fallback, even if NVS is completely full. */
    if (!current_binding(q->roi, &recheck, &again) || !same_binding(&target, &recheck) ||
        rect.left != again.left || rect.top != again.top ||
        rect.right != again.right || rect.bottom != again.bottom ||
        (!q->discard_current && obsolete_reference(&saved) != 1))
        return "cleanup_context_changed";
    last_save_step = "cleanup_open_readwrite";
    err = nvs_open(TRIAL_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) { last_save_error = err; return "cleanup_failed"; }
    last_save_step = "nvs_erase_key";
    err = nvs_erase_key(h, TRIAL_KEY);
    if (err == ESP_OK) {
        last_save_step = "nvs_commit";
        err = nvs_commit(h);
    }
    if (err == ESP_OK) {
        last_save_step = "cleanup_verify_absence";
        size_t remaining = 0;
        esp_err_t read = nvs_get_blob(h, TRIAL_KEY, NULL, &remaining);
        err = read == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : read == ESP_OK ? ESP_FAIL : read;
    }
    nvs_close(h);
    last_save_error = err;
    if (err != ESP_OK) return "cleanup_failed";
    if (q->discard_current) {
        discard_trial_ram();
        return "cleanup_completed";
    }
    if (stage && same_binding(&header.binding, &saved.binding)) stage = 0;
    /* A new, valid empty pending in RAM is deliberately preserved. */
    return "cleanup_completed";
}
esp_err_t vision_trial_init(void) {
    if (!queue) {
        queue = xQueueCreate(1, sizeof(trial_request_t));
        if (queue)
            ESP_LOGI(TAG, "E3_PILOT_V1: automatic local measurement; "
                     "single experimental reference slot; roi_occ disabled");
    }
    return queue ? ESP_OK : ESP_ERR_NO_MEM;
}

/* Transitional E3 adapter: consumes the existing durable single-ROI slot.
 * All calls execute in camera_task, including manual trial requests.
 * No canonical descriptor or NVS format is changed by automatic measurement. */
static bool pilot_roi_bound(const roi_blob_t *snapshot) {
    if (snapshot->config_version != header.binding.config_version) return false;
    for (size_t i = 0; i < snapshot->count; ++i) {
        const roi_def_t *roi = &snapshot->rois[i];
        if (strcmp(roi->id, header.binding.roi_id)) continue;
        cal_rect_t rect;
        cal_roi_t geometry = {normalized(roi->x), normalized(roi->y),
                              normalized(roi->w), normalized(roi->h)};
        return cal_roi_to_rect(VR_WIDTH, VR_HEIGHT, geometry, &rect) == CAL_OK &&
            header.rect[0] == rect.left && header.rect[1] == rect.top &&
            header.rect[2] == (uint32_t)(rect.right - rect.left) &&
            header.rect[3] == (uint32_t)(rect.bottom - rect.top);
    }
    return false;
}

static void pilot_reject(const char *status, bool missing) {
    pilot_health.status = status;
    ++pilot_health.rejected;
    if (missing) ++pilot_health.references_missing;
    else ++pilot_health.context_rejected;
}

static void pilot_process(const camera_fb_t *frame, int64_t now) {
    roi_blob_t snapshot;
    uint32_t revision = calibration_store_revision();
    if ((revision & 1U) || !roi_store_snapshot(&snapshot)) {
        pilot_reject("context_unavailable", false);
        return;
    }
    if (!attempted_load && snapshot.count) {
        if (!allocate()) {
            pilot_reject("psram_required", true);
            return;
        }
        for (size_t i = 0; i < snapshot.count; ++i) {
            load(snapshot.rois[i].id);
            if (stage == 2) break;
        }
        pilot_binding_checked = false;
    }
    if (stage != 2) {
        pilot_health.other_rois_skipped += snapshot.count;
        pilot_reject("references_required", true);
        return;
    }
    if (snapshot.count > 1) pilot_health.other_rois_skipped += snapshot.count - 1U;
    memcpy(pilot_health.roi_id, header.binding.roi_id, sizeof(pilot_health.roi_id));
    pilot_health.config_version = header.binding.config_version;
    pilot_health.calibration_version = header.binding.calibration_version;
    if (!pilot_roi_bound(&snapshot)) {
        pilot_reject("context_changed", false);
        return;
    }
    /* Active NVS is read only at load/invalidation, not at each measurement.
     * A failed read may be retried at most once/minute in an unchanged context. */
    if (!pilot_binding_checked || pilot_binding_revision != revision ||
        pilot_binding_config != snapshot.config_version ||
        (!pilot_binding_valid && now - pilot_checked_us >= 60000000)) {
        cal_store_pair_t active;
        cal_rect_t rect;
        pilot_binding_valid = current_binding(header.binding.roi_id, &active, &rect) &&
                              same_binding(&active, &header.binding);
        pilot_binding_revision = revision;
        pilot_binding_config = snapshot.config_version;
        pilot_binding_checked = true;
        pilot_checked_us = now;
    }
    if (!pilot_binding_valid || calibration_store_revision() != revision) {
        pilot_reject("context_changed", false);
        return;
    }
    vr_frame_t input = {0};
    input.now_us = esp_timer_get_time();
    input.previous_us = pilot_previous_us;
    if (frame) {
        input.gray = frame->buf;
        input.length = frame->len;
        input.width = frame->width;
        input.height = frame->height;
        input.grayscale = frame->format == PIXFORMAT_GRAYSCALE;
        input.captured_us = (int64_t)frame->timestamp.tv_sec * 1000000 +
                            frame->timestamp.tv_usec;
    }
    vr_result_t result;
    vr_status_t status = vr_measure(pattern, guard, header.full_pixels, &input, &result);
    if (input.captured_us > pilot_previous_us && input.captured_us <= input.now_us)
        pilot_previous_us = input.captured_us;
    roi_blob_t after;
    if (!roi_store_snapshot(&after) || !pilot_roi_bound(&after) ||
        calibration_store_revision() != revision) {
        pilot_binding_checked = false;
        pilot_reject("context_changed", false);
        return;
    }
    pilot_health.status = vr_status_name(status);
    pilot_health.matched = result.matched;
    pilot_health.informative = result.informative;
    if (status != VR_OK) {
        ++pilot_health.rejected;
        pilot_health.frame_rejected += status == VR_FRAME;
        pilot_health.clipping_rejected += status == VR_CLIPPING;
        pilot_health.background_rejected += status == VR_BACKGROUND;
        pilot_health.weak_rejected += status == VR_WEAK_REFERENCE;
        return;
    }
    ++pilot_health.accepted;
    pilot_health.valid = true;
    pilot_health.occupancy_pct = result.value.relative_pct;
    pilot_health.relative_unclamped_q8 = result.value.relative_unclamped_q8;
    pilot_health.captured_us = input.captured_us;
}

void vision_pilot_process_frame(const camera_fb_t *frame) {
    int64_t started = esp_timer_get_time();
    ++pilot_health.frames;
    pilot_health.valid = false;
    pilot_health.occupancy_pct = 255;
    pilot_health.relative_unclamped_q8 = 0;
    pilot_health.matched = pilot_health.informative = 0;
    pilot_health.captured_us = 0;
    pilot_health.roi_id[0] = '\0';
    pilot_health.config_version = pilot_health.calibration_version = 0;
    pilot_process(frame, started);
    pilot_health.processing_us = (uint32_t)(esp_timer_get_time() - started);
    if (pilot_health.processing_us > pilot_health.max_processing_us)
        pilot_health.max_processing_us = pilot_health.processing_us;
}

void vision_pilot_health(vision_pilot_health_t *out) {
    if (!out) return;
    *out = pilot_health;
    if (!out->status) {
        out->status = "not_started";
        out->occupancy_pct = 255;
    }
}

void vision_pilot_health_log(uint32_t stack_min_free) {
    vision_pilot_health_t h;
    vision_pilot_health(&h);
    ESP_LOGI(TAG, "E3_PILOT: roi=%s cfg=%lu cal=%lu status=%s valid=%u pct=%d "
             "raw_q8=%lu match=%lu/%lu frames=%lu accepted=%lu rejected=%lu "
             "frame_rej=%lu clip_rej=%lu bg_rej=%lu weak_rej=%lu ctx_rej=%lu "
             "refs_missing=%lu other_roi_skips=%lu us=%lu max_us=%lu "
             "stack_min=%lu publication=disabled",
             h.roi_id[0] ? h.roi_id : "-", (unsigned long)h.config_version,
             (unsigned long)h.calibration_version, h.status, (unsigned)h.valid,
             h.valid ? (int)h.occupancy_pct : -1,
             (unsigned long)h.relative_unclamped_q8, (unsigned long)h.matched,
             (unsigned long)h.informative, (unsigned long)h.frames,
             (unsigned long)h.accepted, (unsigned long)h.rejected,
             (unsigned long)h.frame_rejected, (unsigned long)h.clipping_rejected,
             (unsigned long)h.background_rejected, (unsigned long)h.weak_rejected,
             (unsigned long)h.context_rejected, (unsigned long)h.references_missing,
             (unsigned long)h.other_rois_skipped, (unsigned long)h.processing_us,
             (unsigned long)h.max_processing_us, (unsigned long)stack_min_free);
}

esp_err_t vision_trial_submit(const cJSON *root, bool retained) {
    trial_request_t q = {0};
    const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(root, "confirm_experimental");
    if (retained || !cJSON_IsTrue(enabled) || !queue) return ESP_ERR_INVALID_STATE;
    const char *keys[] = {"req_id", "session_id", "roi_id", "action"};
    char *dest[] = {q.req, q.session, q.roi, q.action};
    size_t sizes[] = {sizeof(q.req), sizeof(q.session), sizeof(q.roi), sizeof(q.action)};
    for (unsigned i = 0; i < 4; ++i) {
        const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, keys[i]);
        if (!cJSON_IsString(item) || strlen(item->valuestring) >= sizes[i]) return ESP_ERR_INVALID_ARG;
        strcpy(dest[i], item->valuestring);
    }
    if (!strcmp(q.action, "cleanup") &&
        !cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root, "confirm_cleanup")))
        return ESP_ERR_INVALID_ARG;
    const cJSON *discard = cJSON_GetObjectItemCaseSensitive(root, "confirm_discard_current");
    if (discard && (!cJSON_IsBool(discard) || strcmp(q.action, "cleanup")))
        return ESP_ERR_INVALID_ARG;
    q.discard_current = cJSON_IsTrue(discard);
    if (!roi_uuid_valid(q.req) || !roi_uuid_valid(q.roi) || !roi_uuid_valid(q.session) ||
        (strcmp(q.action, "empty") && strcmp(q.action, "full") &&
         strcmp(q.action, "sample") && strcmp(q.action, "status") &&
         strcmp(q.action, "cleanup"))) return ESP_ERR_INVALID_ARG;
    q.requested_us = esp_timer_get_time();
    return xQueueSend(queue, &q, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}
static camera_fb_t *fresh(int64_t after) {
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
        camera_fb_t *f = camera_capture();
        if (f && f->buf && f->format == PIXFORMAT_GRAYSCALE &&
            f->width == CAMERA_CAL_FRAME_WIDTH && f->height == CAMERA_CAL_FRAME_HEIGHT &&
            f->len == (size_t)f->width * f->height) {
            int64_t stamp = (int64_t)f->timestamp.tv_sec * 1000000 + f->timestamp.tv_usec;
            int64_t now = esp_timer_get_time();
            if (stamp > after && stamp <= now && now - stamp <= 500000) return f;
        }
        camera_release(f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return NULL;
}
static bool clipped(const camera_fb_t *f) {
    uint32_t count = 0;
    for (uint32_t y = pattern->y; y < pattern->y + pattern->h; ++y)
        for (uint32_t x = pattern->x; x < pattern->x + pattern->w; ++x) {
            uint8_t p = f->buf[(size_t)y * f->width + x];
            count += p <= 5 || p >= 250;
        }
    return (uint64_t)count * 100 > (uint64_t)pattern->w * pattern->h * 20;
}
bool vision_trial_service(void) {
    trial_request_t q;
    if (!queue || xQueueReceive(queue, &q, 0) != pdTRUE) return false;
    pilot_binding_checked = false;
    frame_diag.present = false;
    for (unsigned i = 0; i < 8; ++i) {
        if (strcmp(seen_req[i], q.req)) continue;
        bool same = !strcmp(seen_session[i], q.session) &&
                    !strcmp(seen_action[i], q.action) && !strcmp(seen_roi[i], q.roi) &&
                    seen_discard_current[i] == q.discard_current;
        reply(&q, same ? "duplicate_not_reexecuted" : "request_id_conflict", NULL, 0, 0, 0, 0);
        return true;
    }
    unsigned slot = seen_next++ % 8;
    strcpy(seen_req[slot], q.req); strcpy(seen_session[slot], q.session);
    strcpy(seen_action[slot], q.action); strcpy(seen_roi[slot], q.roi);
    seen_discard_current[slot] = q.discard_current;
    if (!strcmp(q.action, "cleanup")) {
        const char *status = cleanup_reference(&q);
        reply(&q, status, NULL, 0, 0, 0, 0);
        return true;
    }
    if (!allocate()) { reply(&q, "psram_required", NULL, 0, 0, 0, 0); return true; }
    if (!attempted_load && strcmp(q.action, "empty")) load(q.roi);
    if (stage && !bound_now(q.roi)) stage = 0;
    if (!strcmp(q.action, "status")) {
        reply(&q, state(), NULL, 0, 0, 0, 0);
        return true;
    }
    if (!strcmp(q.action, "empty")) {
        if (stage && !strcmp(q.session, header.session)) {
            reply(&q, "new_session_required", NULL, 0, 0, 0, 0);
            return true;
        }
        stage = 0; attempted_load = true;
        memset(&header, 0, sizeof(header));
        cal_rect_t r;
        if (!current_binding(q.roi, &header.binding, &r)) {
            reply(&q, "active_context_required", NULL, 0, 0, 0, 0); return true;
        }
        header.rect[0] = r.left; header.rect[1] = r.top;
        header.rect[2] = r.right - r.left; header.rect[3] = r.bottom - r.top;
        strcpy(header.session, q.session);
        if (!setup(&header)) { reply(&q, "unsupported_roi", NULL, 0, 0, 0, 0); return true; }
    } else if (strcmp(header.session, q.session) ||
               (strcmp(q.action, "full") == 0 ? stage != 1 : stage != 2)) {
        reply(&q, "calibration_required", NULL, 0, 0, 0, 0); return true;
    }
    uint32_t full_counts[TRIAL_FRAMES] = {0};
    int64_t after = q.requested_us;
    for (unsigned i = 0; i < TRIAL_FRAMES; ++i) {
        if (!bound_now(q.roi)) {
            stage = 0; reply(&q, "context_changed", NULL, i, 0, 0, 0); return true;
        }
        camera_fb_t *f = fresh(after);
        if (!f) {
            if (!strcmp(q.action, "empty")) stage = 0;
            reply(&q, "invalid_or_stale_frame", NULL, i, 0, 0, 0); return true;
        }
        after = (int64_t)f->timestamp.tv_sec * 1000000 + f->timestamp.tv_usec;
        int64_t started = esp_timer_get_time();
        uint32_t matched = 0, informative = 0;
        lp_result_t result = {0};
        const char *failure = NULL;
        if (clipped(f)) failure = "excessive_clipping";
        if (!failure && !strcmp(q.action, "empty")) {
            if (lp_add_empty(pattern, f->buf, f->len, f->width) != LP_OK ||
                lg_add(guard, f->buf, f->len, f->width)) failure = "reference_failed";
        } else if (!failure) {
            if (lg_check(guard, f->buf, f->len, f->width, &matched, &informative))
                failure = "background_inconclusive";
            else if (lp_measure(pattern, f->buf, f->len, f->width,
                     !strcmp(q.action, "sample") ? header.full_pixels : 0, &result) != LP_OK)
                failure = "measurement_failed";
            else if ((uint64_t)result.weak_reference_pixels * 100 > (uint64_t)result.total_pixels * 25)
                failure = "weak_reference";
        }
        int64_t processing_us = esp_timer_get_time() - started;
        if (!strcmp(q.action, "sample")) diagnose_frame(f);
        camera_release(f);
        if (!bound_now(q.roi)) { stage = 0; failure = "context_changed"; }
        if (failure) {
            reply(&q, failure, NULL, i + 1, matched, informative, processing_us);
            return true;
        }
        if (!strcmp(q.action, "sample"))
            reply(&q, "measurement", &result, i + 1, matched, informative, processing_us);
        if (!strcmp(q.action, "full")) full_counts[i] = result.occupied_pixels;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    if (!strcmp(q.action, "empty")) {
        if (lp_finish_empty(pattern) != LP_OK || lg_finish(guard)) {
            reply(&q, "reference_failed", NULL, 0, 0, 0, 0); return true;
        }
        stage = 1;
        reply(&q, "empty_captured", NULL, TRIAL_FRAMES, 0, 0, 0);
    } else if (!strcmp(q.action, "full")) {
        for (unsigned i = 0; i < TRIAL_FRAMES; ++i)
            for (unsigned j = i + 1; j < TRIAL_FRAMES; ++j)
                if (full_counts[j] < full_counts[i]) {
                    uint32_t tmp = full_counts[i]; full_counts[i] = full_counts[j]; full_counts[j] = tmp;
                }
        header.full_pixels = full_counts[1];
        uint32_t total = pattern->w * pattern->h;
        if ((uint64_t)header.full_pixels * 25 < total ||
            (uint64_t)(full_counts[2] - full_counts[0]) * 100 > (uint64_t)header.full_pixels * 20) {
            reply(&q, "indistinguishable_or_unstable_full", NULL, 0, 0, 0, 0); return true;
        }
        if (!bound_now(q.roi)) {
            stage = 0; reply(&q, "context_changed", NULL, 0, 0, 0, 0); return true;
        }
        esp_err_t err = save();
        if (err == ESP_OK && bound_now(q.roi)) stage = 2;
        else ESP_LOGE(TAG, "experimental reference not activated: %s", esp_err_to_name(err));
        reply(&q, stage == 2 ? "full_captured" : "persistence_or_context_failed",
              NULL, TRIAL_FRAMES, 0, 0, 0);
    } else reply(&q, "sample_done", NULL, TRIAL_FRAMES, 0, 0, 0);
    return true;
}
