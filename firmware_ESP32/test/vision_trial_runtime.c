#include "../main/modules/vision/vision_trial.c"
#include <assert.h>
#include <stdio.h>

static uint8_t pixels[640 * 480], persisted[TRIAL_BLOB_MAX], pending[TRIAL_BLOB_MAX];
static size_t persisted_size, pending_size, queue_size;
static uint8_t queued[512];
static bool has_message, fail_commit, stale;
static bool fail_set_space, pending_write, pending_erase;
static bool extra_roi, binding_read_failed;
static unsigned frames, released, writes, measurements;
static unsigned erases;
static int expected_storage_error = ESP_FAIL;
static const char *expected_storage_step = "nvs_commit";
static int64_t clock_us = 1000000;
static cal_store_pair_t active;
static char last_status[80], last_command[512];
static camera_fb_t framebuffer;
static lp_context_t saved_pattern;
static lg_context_t saved_guard;
static uint32_t saved_full_pixels;
static bool sensor_read_failed, sensor_missing;
static int sensor_pid = OV3660_PID;
static bool discard_confirm;
static uint32_t active_revision;
static unsigned binding_reads, storage_opens, revision_after_snapshot;
static float roi_shift;
static bool snapshot_failed;
static unsigned roi_publications;
static unsigned cam_moved_events;
static bool queue_full;
static bool production_fixture;
static uint32_t fixture_version;
static unsigned kernel_reads;
static unsigned last_published_value;
esp_err_t mqtt_queue_json(const char *type, const char *data) {
    if (!strcmp(type, "cam_moved")) {
        cJSON *root = cJSON_Parse(data);
        assert(root);
        cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "value");
        assert(v && v->valueint == 1);
        assert(cJSON_GetObjectItemCaseSensitive(root, "roi_id"));
        assert(cJSON_GetObjectItemCaseSensitive(root, "config_version"));
        assert(cJSON_GetObjectItemCaseSensitive(root, "calibration_version"));
        assert(cJSON_GetObjectItemCaseSensitive(root, "streak"));
        assert(cJSON_GetObjectItemCaseSensitive(root, "uptime_ms"));
        cJSON_Delete(root);
        if (queue_full) return ESP_ERR_NO_MEM;
        ++cam_moved_events;
        return ESP_OK;
    }
    assert(!strcmp(type, "roi_occ"));
    cJSON *root = cJSON_Parse(data);
    assert(root);
    assert(cJSON_GetObjectItemCaseSensitive(root, "roi_id"));
    assert(cJSON_GetObjectItemCaseSensitive(root, "value")->valueint <= 100);
    assert(cJSON_GetObjectItemCaseSensitive(root, "captured_uptime_ms"));
    /* Fase 6: solo se publica la mediana de la ventana, nunca el instantaneo. */
    if (using_production)
        assert(cJSON_GetObjectItemCaseSensitive(root, "value")->valueint ==
               pilot_health.occupancy_smoothed_pct);
    last_published_value =
        (unsigned)cJSON_GetObjectItemCaseSensitive(root, "value")->valueint;
    cJSON_Delete(root);
    if (queue_full) return ESP_ERR_NO_MEM;
    ++roi_publications;
    return ESP_OK;
}
uint32_t calibration_store_revision(void) { return active_revision; }
static int mock_get_reg(sensor_t *sensor, int reg, int mask) {
    (void)sensor;
    if (sensor_read_failed) return -1;
    if (reg == 0x3503) { assert(mask == 3); return 0; }
    if (reg == 0x3500) { assert(mask == 0x0fffff); return 12345; }
    assert(reg == 0x350a && mask == 0x03ff);
    return 32;
}
sensor_t *esp_camera_sensor_get(void) {
    static sensor_t sensor;
    sensor.id.PID = sensor_pid;
    sensor.get_reg = mock_get_reg;
    return sensor_missing ? NULL : &sensor;
}

void mock_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
const char *esp_err_to_name(int err) { (void)err; return "mock"; }
void *heap_caps_calloc(size_t n, size_t size, unsigned caps) {
    (void)caps; return calloc(n, size);
}
void *heap_caps_malloc(size_t size, unsigned caps) { (void)caps; return malloc(size); }
size_t heap_caps_get_free_size(unsigned caps) { (void)caps; return 100000; }
int64_t esp_timer_get_time(void) { return ++clock_us; }
void vTaskDelay(unsigned ms) { clock_us += ms * 1000; }
QueueHandle_t xQueueCreate(unsigned count, size_t size) {
    assert(count == 1 && size <= sizeof(queued)); queue_size = size; return queued;
}
int xQueueSend(QueueHandle_t q, const void *data, unsigned timeout) {
    (void)q; (void)timeout;
    if (has_message) return 0;
    memcpy(queued, data, queue_size); has_message = true; return pdTRUE;
}
int xQueueReceive(QueueHandle_t q, void *data, unsigned timeout) {
    (void)q; (void)timeout;
    if (!has_message) return 0;
    memcpy(data, queued, queue_size); has_message = false; return pdTRUE;
}
camera_fb_t *camera_capture(void) {
    clock_us += 200000;
    framebuffer.buf = pixels; framebuffer.len = sizeof(pixels);
    framebuffer.width = 640; framebuffer.height = 480;
    framebuffer.format = PIXFORMAT_GRAYSCALE;
    int64_t stamp = stale ? 0 : clock_us - 10;
    framebuffer.timestamp.tv_sec = stamp / 1000000;
    framebuffer.timestamp.tv_usec = stamp % 1000000;
    ++frames;
    return &framebuffer;
}
void camera_release(camera_fb_t *f) { if (f) ++released; }
bool roi_store_snapshot(roi_blob_t *out) {
    if (snapshot_failed) return false;
    if (revision_after_snapshot && !--revision_after_snapshot) active_revision += 2;
    memset(out, 0, sizeof(*out)); out->count = 1;
    out->config_version = active.config_version;
    strcpy(out->rois[0].id, active.roi_id);
    out->rois[0].x = .25f + roi_shift; out->rois[0].y = .25f;
    out->rois[0].w = .5f; out->rois[0].h = .5f;
    if (extra_roi) {
        out->count = 2;
        strcpy(out->rois[1].id, "20000000-0000-0000-0000-000000000002");
        out->rois[1].x = .01f; out->rois[1].y = .01f;
        out->rois[1].w = .1f; out->rois[1].h = .1f;
    }
    roi_blob_seal(out); /* el snapshot del dispositivo siempre viene sellado */
    return true;
}
esp_err_t calibration_store_get_kernel_ref(cal_kernel_ref_t *ref) {
    ++kernel_reads;
    if (!production_fixture) return ESP_ERR_NVS_NOT_FOUND;
    if (ref->calibration_version != fixture_version) return ESP_ERR_INVALID_STATE;
    lp_context_t pat;
    lg_context_t grd;
    bool second = strcmp(ref->roi_id, active.roi_id) != 0;
    if (lp_init(&pat, 640, 480, second ? 6 : 160, second ? 4 : 120,
                second ? 65 : 320, second ? 49 : 240) != LP_OK) abort();
    if (lg_init(&grd, 640, 480, pat.x, pat.y, pat.w, pat.h)) abort();
    size_t bytes = 16U * (pat.count + grd.count);
    if (ref->tiles) {
        assert(ref->tiles_capacity >= bytes);
        /* References from the deterministic empty test image. */
        assert(lp_add_empty(&pat, pixels, sizeof(pixels), 640) == LP_OK);
        assert(lp_finish_empty(&pat) == LP_OK);
        assert(!lg_add(&grd, pixels, sizeof(pixels), 640));
        assert(!lg_finish(&grd));
        size_t at = 0;
        for (unsigned i = 0; i < pat.count; ++i)
            for (unsigned k = 0; k < 16; ++k) ref->tiles[at++] = pat.reference[i][k];
        for (unsigned i = 0; i < grd.count; ++i)
            for (unsigned k = 0; k < 16; ++k) ref->tiles[at++] = grd.ref[i][k];
    }
    ref->rect[0] = pat.x; ref->rect[1] = pat.y;
    ref->rect[2] = pat.w; ref->rect[3] = pat.h;
    ref->tile_count = pat.count; ref->guard_count = grd.count;
    ref->tiles_bytes = bytes; ref->full_pixels = pat.w * pat.h / 2;
    return ESP_OK;
}
esp_err_t calibration_store_get_active(const char *roi, cal_store_pair_t *out) {
    ++binding_reads;
    if (binding_read_failed) return ESP_FAIL;
    if (extra_roi && !strcmp(roi, "20000000-0000-0000-0000-000000000002")) {
        *out = active;
        strcpy(out->roi_id, roi);
        return ESP_OK;
    }
    if (strcmp(roi, active.roi_id)) return ESP_FAIL;
    *out = active; return ESP_OK;
}
static unsigned wipe_calls;
esp_err_t calibration_store_erase_all(cal_store_erase_report_t *report) {
    ++wipe_calls;
    if (!report) return ESP_OK;
    memset(report, 0, sizeof(*report));
    report->keys_removed = 7;
    report->free_entries_before = 196;
    report->available_entries_before = 70;
    report->free_entries_after = 400;
    report->available_entries_after = 274;
    report->verified_empty = true;
    return ESP_OK;
}
int nvs_open(const char *name, int mode, nvs_handle_t *out) {
    ++storage_opens;
    assert(!strcmp(name, TRIAL_NAMESPACE)); (void)mode; *out = 1;
    return ESP_OK;
}
int nvs_set_blob(nvs_handle_t h, const char *key, const void *data, size_t size) {
    (void)h; assert(!strcmp(key, TRIAL_KEY)); assert(size <= sizeof(pending));
    if (fail_set_space) return ESP_ERR_NVS_NOT_ENOUGH_SPACE;
    memcpy(pending, data, size); pending_size = size;
    pending_write = true; pending_erase = false;
    return ESP_OK;
}
int nvs_commit(nvs_handle_t h) {
    (void)h;
    if (fail_commit) return ESP_FAIL;
    if (pending_erase) { persisted_size = 0; ++erases; }
    else if (pending_write) {
        memcpy(persisted, pending, pending_size); persisted_size = pending_size;
        ++writes;
    }
    pending_write = pending_erase = false;
    return ESP_OK;
}
int nvs_erase_key(nvs_handle_t h, const char *key) {
    (void)h;
    assert(!strcmp(key, TRIAL_KEY));
    if (!persisted_size) return ESP_ERR_NVS_NOT_FOUND;
    pending_write = false; pending_erase = true;
    return ESP_OK;
}
int nvs_get_blob(nvs_handle_t h, const char *key, void *data, size_t *size) {
    (void)h; assert(!strcmp(key, TRIAL_KEY));
    if (!persisted_size) return ESP_ERR_NVS_NOT_FOUND;
    if (!data) { *size = persisted_size; return ESP_OK; }
    if (*size < persisted_size) return ESP_FAIL;
    memcpy(data, persisted, persisted_size); *size = persisted_size; return ESP_OK;
}
void nvs_close(nvs_handle_t h) {
    (void)h; pending_write = pending_erase = false;
}
int nvs_get_stats(const char *partition, nvs_stats_t *stats) {
    assert(!partition);
    memset(stats, 0, sizeof(*stats));
    stats->free_entries = 196;
    stats->available_entries = persisted_size ? 70 : 300;
    return ESP_OK;
}
esp_err_t mqtt_publish_trial_json(const char *text) {
    cJSON *root = cJSON_Parse(text);
    assert(root);
    strcpy(last_status, cJSON_GetObjectItem(root, "status")->valuestring);
    assert(cJSON_IsTrue(cJSON_GetObjectItem(root, "experimental")));
    const cJSON *d = cJSON_GetObjectItem(root, "diagnostics");
    bool frame_result = !strcmp(last_status, "measurement") ||
                        !strcmp(last_status, "background_inconclusive");
    assert((d != NULL) == frame_result);
    if (d) {
        assert(cJSON_GetObjectItem(d, "roi_mean_q8")->valueint == 80 * 256);
        assert(cJSON_GetObjectItem(d, "roi_clipped_pct")->valueint == 0);
        assert(cJSON_GetObjectItem(d, "background_mean_q8")->valueint == 80 * 256);
        assert(cJSON_GetObjectItem(d, "reference_background_mean_q8")->valueint == 80 * 256);
        assert(cJSON_GetObjectItem(d, "full_reference_pixels")->valuedouble == header.full_pixels);
        assert(cJSON_GetObjectItem(d, "pattern_crc32")->valuedouble ==
               roi_crc32((const uint8_t *)pattern->reference,
                         pattern->count * sizeof(pattern->reference[0])));
        assert(cJSON_GetObjectItem(d, "sensor_read_us")->valuedouble >
               cJSON_GetObjectItem(d, "captured_us")->valuedouble);
        if (sensor_read_failed || sensor_missing || sensor_pid != OV3660_PID) {
            assert(cJSON_IsNull(cJSON_GetObjectItem(d, "exposure_raw")));
            assert(cJSON_IsNull(cJSON_GetObjectItem(d, "gain_raw")));
        } else {
            assert(cJSON_IsTrue(cJSON_GetObjectItem(d, "auto_exposure")));
            assert(cJSON_IsTrue(cJSON_GetObjectItem(d, "auto_gain")));
            assert(cJSON_GetObjectItem(d, "exposure_raw")->valueint == 12345);
            assert(cJSON_GetObjectItem(d, "gain_raw")->valueint == 32);
        }
    }
    if (!strcmp(last_status, "persistence_or_context_failed") ||
        !strcmp(last_status, "cleanup_failed")) {
        assert(!strcmp(cJSON_GetObjectItem(root, "failure_reason")->valuestring,
            !strcmp(last_status, "cleanup_failed") ? "cleanup_failed" : "persistence_failed"));
        assert(!strcmp(cJSON_GetObjectItem(root, "storage_step")->valuestring,
                       expected_storage_step));
        assert(cJSON_GetObjectItem(root, "storage_error_code")->valueint == expected_storage_error);
        assert(cJSON_GetObjectItem(root, "reference_bytes")->valueint > 0);
        assert(cJSON_GetObjectItem(root, "nvs_available_entries")->valueint == 70);
    }
    const cJSON *scope = cJSON_GetObjectItem(root, "cleanup_scope");
    if (scope) {
        assert(!strcmp(scope->valuestring, "selfiq_vexp/reference"));
        assert(!strcmp(cJSON_GetObjectItem(root, "cleanup_policy")->valuestring,
                       discard_confirm ? "discard_matching_session" : "obsolete_only"));
        assert(cJSON_IsTrue(cJSON_GetObjectItem(root, "removed")) ==
               !strcmp(last_status, "cleanup_completed"));
    }
    const cJSON *wipe_scope = cJSON_GetObjectItem(root, "wipe_scope");
    if (wipe_scope) {
        assert(!strcmp(wipe_scope->valuestring,
                       "selfiq_cal,selfiq_vexp/reference"));
        assert(!strcmp(cJSON_GetObjectItem(root, "wipe_policy")->valuestring,
                       "all_calibration_keys"));
        assert(cJSON_GetObjectItem(root, "cal_keys_removed")->valueint == 7);
        assert(cJSON_IsTrue(cJSON_GetObjectItem(root, "cal_verified_empty")));
        assert(cJSON_IsTrue(cJSON_GetObjectItem(root, "trial_removed")));
        assert(cJSON_IsTrue(cJSON_GetObjectItem(root, "roi_blob_ok")));
        assert(cJSON_GetObjectItem(root, "nvs_available_entries_before")->valueint == 70);
        assert(cJSON_GetObjectItem(root, "nvs_available_entries")->valueint == 300);
    }
    if (cJSON_IsTrue(cJSON_GetObjectItem(root, "valid"))) {
        assert(stage == 2);
        assert(cJSON_IsNumber(cJSON_GetObjectItem(root, "occupancy_pct")));
        ++measurements;
    } else assert(cJSON_IsNull(cJSON_GetObjectItem(root, "occupancy_pct")));
    cJSON_Delete(root); return ESP_OK;
}
static void image(bool full, bool changed_background) {
    for (unsigned y = 0; y < 480; ++y)
        for (unsigned x = 0; x < 640; ++x) {
            uint8_t v = (x / 4 + y / 4) % 2 ? 40 : 120;
            if ((full && x >= 160 && x < 480 && y >= 120 && y < 360) ||
                changed_background) v = 160 - v;
            pixels[y * 640 + x] = v;
        }
}
static void flatten_guard(const lg_context_t *g, uint32_t limit) {
    for (uint32_t i = 0; i < limit && i < g->count; ++i)
        for (unsigned y = 0; y < 16; ++y)
            for (unsigned x = 0; x < 16; ++x)
                pixels[(size_t)(g->y[i] + y) * 640 + g->x[i] + x] = 80;
}
static void submit(unsigned id, unsigned session, const char *action, bool retained) {
    snprintf(last_command, sizeof(last_command),
        "{\"confirm_experimental\":true,\"confirm_cleanup\":true,%s"
        "\"req_id\":\"00000000-0000-0000-0000-%012u\","
        "\"session_id\":\"10000000-0000-0000-0000-%012u\","
        "\"roi_id\":\"20000000-0000-0000-0000-000000000001\",\"action\":\"%s\"}",
        discard_confirm ? "\"confirm_discard_current\":true," : "",
        id, session, action);
    cJSON *root = cJSON_Parse(last_command);
    assert(root);
    esp_err_t err = vision_trial_submit(root, retained);
    assert(retained ? err != ESP_OK : err == ESP_OK);
    cJSON_Delete(root);
    if (!retained) assert(vision_trial_service());
    assert(frames == released);
}
/* El borrado total exige el literal exacto: no se acepta un booleano. */
static void submit_wipe(unsigned id, unsigned session, bool confirm) {
    snprintf(last_command, sizeof(last_command),
        "{\"confirm_experimental\":true,%s"
        "\"req_id\":\"00000000-0000-0000-0000-%012u\","
        "\"session_id\":\"10000000-0000-0000-0000-%012u\","
        "\"roi_id\":\"20000000-0000-0000-0000-000000000001\","
        "\"action\":\"wipe_calibration\"}",
        confirm ? "\"confirm_wipe_cal\":\"ERASE_CAL\"," : "", id, session);
    cJSON *root = cJSON_Parse(last_command);
    assert(root);
    esp_err_t err = vision_trial_submit(root, false);
    cJSON_Delete(root);
    if (!confirm) { assert(err != ESP_OK); return; }
    assert(err == ESP_OK);
    assert(vision_trial_service());
}
static void reboot_ram(void) {
    for (unsigned i = 0; i < production_count; ++i) {
        free(prod_pattern[i]); prod_pattern[i] = NULL;
        free(prod_guard[i]); prod_guard[i] = NULL;
    }
    production_count = production_index = 0;
    using_production = false;
    for (unsigned i = 0; i < PROD_MAX_ROI; ++i) {
        median_reset(i);
        production_published_us[i] = production_attempt_us[i] = 0;
        production_last_published_pct[i] = 0;
        production_guard_fail_streak[i] = production_ok_streak[i] = 0;
        production_cam_moved_active[i] = production_cam_moved_queued[i] = false;
    }
    production_published = production_queue_rejected = 0;
    roi_publications = cam_moved_events = 0;
    free(pattern); free(guard); pattern = NULL; guard = NULL;
    stage = 0; attempted_load = false; memset(&header, 0, sizeof(header));
    memset(seen_req, 0, sizeof(seen_req)); seen_next = 0;
    memset(&pilot_health, 0, sizeof(pilot_health));
    pilot_binding_checked = pilot_binding_valid = false;
    pilot_previous_us = 0;
}

static void automatic_frame(void) {
    camera_fb_t *frame = camera_capture();
    vision_pilot_process_frame(frame);
    camera_release(frame);
    assert(frames == released);
}

static void test_frame_evaluator(void) {
    lp_context_t *p = calloc(1, sizeof(*p));
    lg_context_t *g = calloc(1, sizeof(*g));
    assert(p && g);
    image(false, false);
    assert(lp_init(p, 640, 480, 160, 120, 320, 240) == LP_OK);
    assert(lg_init(g, 640, 480, 160, 120, 320, 240) == 0);
    assert(lp_add_empty(p, pixels, sizeof(pixels), 640) == LP_OK);
    assert(lp_finish_empty(p) == LP_OK);
    assert(lg_add(g, pixels, sizeof(pixels), 640) == 0 && lg_finish(g) == 0);
    image(true, false);
    lp_result_t full;
    assert(lp_measure(p, pixels, sizeof(pixels), 640, 0, &full) == LP_OK);
    vr_frame_t f = {pixels, sizeof(pixels), 640, 480, true, 1000000, 1100000, 0};
    vr_result_t result;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_OK);
    assert(result.value.relative_pct == 100);
    for (unsigned y = 120; y < 360; ++y)
        for (unsigned x = 160; x < 320; ++x)
            pixels[y * 640 + x] = (x / 4 + y / 4) % 2 ? 40 : 120;
    lp_result_t expected;
    assert(lp_measure(p, pixels, sizeof(pixels), 640, full.occupied_pixels, &expected) == LP_OK);
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_OK);
    assert(!memcmp(&result.value, &expected, sizeof(expected)));
    image(false, false);
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_OK);
    assert(result.value.relative_pct == 0);
    /* Bandas 15/11/2026: la guarda parcialmente alterada mide y avisa; casi
     * toda alterada cae bajo el 25 % y se rechaza. El ROI no se toca. */
    flatten_guard(g, g->count / 2);
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_OK);
    assert(result.guard_warn == 1 && result.value.relative_pct == 0);
    assert(result.matched * 100U < result.informative * LG_GUARD_PASS_PCT);
    image(false, false);
    flatten_guard(g, g->count - g->count / 5);
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_BACKGROUND);
    assert(result.value.relative_pct == 255);
    image(false, false);
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_OK);
    assert(result.guard_warn == 0);
    f.length--;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_FRAME);
    assert(result.value.relative_pct == 255);
    f.length++; f.grayscale = false;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_FRAME);
    f.grayscale = true; f.width = 320;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_FRAME);
    f.width = 640; f.now_us = 1600000;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_FRAME);
    f.now_us = 999999;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_FRAME);
    f.now_us = 1100000; f.previous_us = f.captured_us;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_FRAME);
    f.previous_us = 0;
    assert(vr_measure(p, g, 0, &f, &result) == VR_UNCALIBRATED);
    assert(vr_measure(NULL, g, full.occupied_pixels, &f, &result) == VR_UNCALIBRATED);
    unsigned cols = p->cols;
    p->cols = 0;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_UNCALIBRATED);
    p->cols = cols;
    uint16_t guard_x = g->x[0];
    g->x[0] = 640;
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_UNCALIBRATED);
    g->x[0] = guard_x;
    image(false, true);
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_BACKGROUND);
    assert(result.value.relative_pct == 255);
    memset(pixels, 255, sizeof(pixels));
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_CLIPPING);
    image(true, false);
    memset(p->reference, 0, sizeof(p->reference));
    assert(vr_measure(p, g, full.occupied_pixels, &f, &result) == VR_WEAK_REFERENCE);
    assert(result.value.relative_pct == 255);
    free(p); free(g);
}
/* Fase 6: ventana deslizante de 5 muestras. Un pico aislado no mueve la
 * mediana; un cambio sostenido converge en 3 muestras. */
static void test_median_window(void) {
    unsigned slot = 0;
    median_reset(slot);
    assert(production_median_count[slot] == 0);
    assert(median_push(slot, 58) == 58);
    assert(median_push(slot, 20) == 39);   /* (20+58)/2 */
    assert(median_push(slot, 53) == 53);
    assert(median_push(slot, 56) == 54);   /* (53+56)/2 */
    assert(median_push(slot, 100) == 56);
    assert(production_median_count[slot] == PROD_MEDIAN_SAMPLES);
    /* La ventana ya esta llena: entra 0, sale 58. */
    assert(median_push(slot, 0) == 53);
    assert(median_push(slot, 0) == 53);
    assert(median_push(slot, 0) == 0);
    assert(median_push(slot, 0) == 0);
    assert(median_push(slot, 0) == 0);
    /* Reinicio de ventana (calibracion nueva): no arrastra historia. */
    median_reset(slot);
    assert(production_median_count[slot] == 0);
    assert(median_push(slot, 7) == 7);
    assert(median_push(slot, 255) == 131); /* valor fuera de rango: solo se
                                            * promedia, nunca se publica asi */
    median_reset(slot);
}

/* Fase 6 incremento 2: histeresis de publicacion y latido de 30 s. */
static void test_publish_hysteresis(void) {
    unsigned slot = 0;
    median_reset(slot);
    production_published_us[slot] = production_attempt_us[slot] = 0;
    production_last_published_pct[slot] = 0;

    /* Primera lectura tras cargar la referencia: siempre se publica. */
    assert(production_should_publish(slot, 0, 1000000));
    production_published_us[slot] = 1000000;
    production_last_published_pct[slot] = 50;

    /* Cambio de 3 puntos antes del latido: no publica. */
    assert(!production_should_publish(slot, 53, 6000000));
    assert(!production_should_publish(slot, 47, 6000000));

    /* Cambio de 6 puntos: publica de inmediato. */
    assert(production_should_publish(slot, 56, 6000000));
    assert(production_should_publish(slot, 44, 6000000));

    /* Sin cambios durante 30 s: publica el latido. */
    assert(!production_should_publish(slot, 50, 1000000 + PROD_HEARTBEAT_US - 1));
    assert(production_should_publish(slot, 50, 1000000 + PROD_HEARTBEAT_US));

    /* Recargar la calibracion reinicia el estado: el primer valor vuelve a salir. */
    median_reset(slot);
    production_published_us[slot] = production_attempt_us[slot] = 0;
    production_last_published_pct[slot] = 0;
    assert(production_should_publish(slot, 0, 99000000));
    median_reset(slot);
}

/* Regresión observada en placa: la guarda sana oscila 47-51/54 (87-94 %) en una
 * calibración y 73-83/88 (83-94 %) en el soak v2 (cfg=70). El umbral único al
 * 90 % y después la banda 85-92 % daban CAM_MOVED falsos cuando bajaba al
 * 83-85 %. Con banda gris 75-80 % ni dispara en sano ni recupera con lecturas
 * intermedias, y una guarda sana del 83 % sí cuenta para recuperar. */
static void test_cam_moved_recovery_guard_pct(void) {
    unsigned slot = 0;
    median_reset(slot);
    production_guard_fail_streak[slot] = production_ok_streak[slot] = 0;
    production_cam_moved_active[slot] = production_cam_moved_queued[slot] = false;
    production_published_us[slot] = production_attempt_us[slot] = 0;
    production_last_published_pct[slot] = 0;
    using_production = true;
    strcpy(production_roi_id[slot], "20000000-0000-0000-0000-000000000001");
    memset(&production_binding[slot], 0, sizeof(production_binding[slot]));
    production_binding[slot].config_version = 1;
    production_binding[slot].calibration_version = 1;

    unsigned moved_before = cam_moved_events;
    /* Guarda al 50 %: dispara tras 5 frames. */
    for (unsigned i = 0; i < PROD_CAM_MOVED_FRAMES; ++i)
        cam_moved_track(slot, 1000000 + i, VR_OK, 30, 60);
    assert(production_cam_moved_active[slot]);
    assert(cam_moved_events == moved_before + 1);

    /* Dos buenas y una mala NO recuperan. */
    cam_moved_track(slot, 2000000, VR_OK, 60, 60);
    cam_moved_track(slot, 2000001, VR_OK, 60, 60);
    cam_moved_track(slot, 2000002, VR_OK, 30, 60);
    assert(production_cam_moved_active[slot]);
    /* Zona gris (77 %) tampoco cuenta como buena. */
    cam_moved_track(slot, 2000003, VR_OK, 77, 100);
    assert(production_cam_moved_active[slot]);
    assert(cam_moved_events == moved_before + 1);

    /* Tres buenas (100 %) sí, sin reenviar el aviso. */
    cam_moved_track(slot, 3000000, VR_OK, 60, 60);
    cam_moved_track(slot, 3000001, VR_OK, 60, 60);
    cam_moved_track(slot, 3000002, VR_OK, 60, 60);
    assert(!production_cam_moved_active[slot]);
    assert(cam_moved_events == moved_before + 1);

    /* La guarda sana real del soak NO dispara desde cero: 47/54 = 87 % con la
     * primera calibración y 74/88 = 84 % con la del v2 (debajo del viejo 85). */
    production_guard_fail_streak[slot] = 0;
    for (unsigned i = 0; i < PROD_CAM_MOVED_FRAMES; ++i)
        cam_moved_track(slot, 4000000 + i, VR_OK, 47, 54);
    assert(!production_cam_moved_active[slot]);
    assert(cam_moved_events == moved_before + 1);
    for (unsigned i = 0; i < PROD_CAM_MOVED_FRAMES; ++i)
        cam_moved_track(slot, 5000000 + i, VR_OK, 74, 88);
    assert(!production_cam_moved_active[slot]);
    assert(cam_moved_events == moved_before + 1);
    /* Y una guarda sana al 83 % (73/88) sí recupera tras un episodio real. */
    for (unsigned i = 0; i < PROD_CAM_MOVED_FRAMES; ++i)
        cam_moved_track(slot, 1000000 + i, VR_OK, 30, 60);
    assert(production_cam_moved_active[slot]);
    for (unsigned i = 0; i < PROD_CAM_MOVED_RECOVERY; ++i)
        cam_moved_track(slot, 6000000 + i, VR_OK, 73, 88);
    assert(!production_cam_moved_active[slot]);
    assert(cam_moved_events == moved_before + 2);
    using_production = false;
}

int main(void) {
    strcpy(active.roi_id, "20000000-0000-0000-0000-000000000001");
    strcpy(active.calibration_id, "30000000-0000-0000-0000-000000000001");
    memset(active.context_hash, 'a', 64);
    active.config_version = 16; active.calibration_version = 2;
    active.has_empty = active.has_full = true;
    assert(vision_trial_init() == ESP_OK);
    test_median_window();
    test_publish_hysteresis();
    test_cam_moved_recovery_guard_pct();
    image(false, false);
    submit(1, 1, "empty", true); assert(stage == 0 && !frames);
    submit(2, 1, "sample", false); assert(!strcmp(last_status, "calibration_required"));
    submit(3, 1, "empty", false); assert(stage == 1 && writes == 0);
    image(true, false);
    submit(4, 1, "full", false); assert(stage == 2 && writes == 1);
    saved_pattern = *pattern;
    saved_guard = *guard;
    saved_full_pixels = header.full_pixels;
    unsigned old_frames = frames;
    submit(4, 1, "full", false);
    assert(frames == old_frames && writes == 1);
    assert(!strcmp(last_status, "duplicate_not_reexecuted"));
    submit(4, 1, "empty", false); assert(!strcmp(last_status, "request_id_conflict"));
    submit(5, 1, "sample", false); assert(measurements == 3);
    image(false, true);
    submit(6, 1, "sample", false);
    assert(!strcmp(last_status, "background_inconclusive") && measurements == 3);
    reboot_ram();
    submit(7, 1, "status", false); assert(stage == 2);
    /* Status alone does not prove that a restored reference can measure.
     * Check all features/anchor positions and recapture an identical scene. */
    assert(header.full_pixels == saved_full_pixels);
    assert(pattern->count == saved_pattern.count);
    assert(!memcmp(pattern->reference, saved_pattern.reference, sizeof(pattern->reference)));
    assert(!memcmp(guard, &saved_guard, sizeof(*guard)));
    image(true, false);
    lp_result_t before, after;
    assert(lp_measure(&saved_pattern, pixels, sizeof(pixels), 640,
                      saved_full_pixels, &before) == LP_OK);
    assert(lp_measure(pattern, pixels, sizeof(pixels), 640,
                      header.full_pixels, &after) == LP_OK);
    assert(!memcmp(&before, &after, sizeof(before)));
    unsigned measurements_before = measurements;
    submit(15, 1, "sample", false);
    assert(measurements == measurements_before + 3);
    assert(!strcmp(last_status, "sample_done"));
    unsigned before_diagnostic_samples = measurements;
    sensor_read_failed = true;
    submit(101, 1, "sample", false);
    sensor_read_failed = false; sensor_missing = true;
    submit(102, 1, "sample", false);
    sensor_missing = false; sensor_pid = 0x2640;
    submit(103, 1, "sample", false);
    sensor_pid = OV3660_PID;
    assert(measurements == before_diagnostic_samples + 9 && writes == 1);
    persisted[persisted_size - 1] ^= 1;
    reboot_ram();
    submit(8, 1, "status", false); assert(stage == 0);
    persisted[persisted_size - 1] ^= 1;
    reboot_ram();
    submit(9, 1, "status", false); assert(stage == 2);
    ++active.config_version;
    submit(10, 1, "sample", false); assert(stage == 0);
    image(false, false);
    submit(11, 2, "empty", false); assert(stage == 1);
    image(true, false); fail_commit = true;
    submit(12, 2, "full", false); assert(stage == 1 && writes == 1);
    assert(!strcmp(last_status, "persistence_or_context_failed"));
    fail_commit = false;
    stale = true;
    submit(13, 3, "empty", false);
    assert(stage == 0 && !strcmp(last_status, "invalid_or_stale_frame"));
    reboot_ram();
    stale = false;
    /* Cleanup cannot delete a still-current reference, even with another
     * session ID. It does not need a frame or a new calibration capture. */
    active.config_version = 16;
    unsigned before_cleanup_frames = frames;
    submit(16, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_current") && erases == 0);
    assert(frames == before_cleanup_frames && persisted_size);
    cJSON *unconfirmed = cJSON_Parse(last_command);
    cJSON_DeleteItemFromObject(unconfirmed, "confirm_cleanup");
    assert(vision_trial_submit(unconfirmed, false) != ESP_OK);
    cJSON_Delete(unconfirmed);
    /* Changing the request target must not turn another ROI's current
     * reference into something safe to delete. */
    extra_roi = true;
    cJSON *other_target = cJSON_Parse(last_command);
    cJSON_ReplaceItemInObject(other_target, "req_id",
        cJSON_CreateString("00000000-0000-0000-0000-000000000099"));
    cJSON_ReplaceItemInObject(other_target, "roi_id",
        cJSON_CreateString("20000000-0000-0000-0000-000000000002"));
    assert(vision_trial_submit(other_target, false) == ESP_OK);
    cJSON_Delete(other_target);
    assert(vision_trial_service());
    assert(!strcmp(last_status, "cleanup_refused_current") && erases == 0);
    extra_roi = false;
    binding_read_failed = true;
    submit(100, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_context_unavailable") && erases == 0);
    binding_read_failed = false;
    persisted[persisted_size - 1] ^= 1;
    expected_storage_step = "cleanup_validate_blob";
    expected_storage_error = ESP_ERR_INVALID_STATE;
    submit(17, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_failed") && erases == 0 && persisted_size);
    persisted[persisted_size - 1] ^= 1;
    active.config_version = 17;
    fail_commit = true;
    expected_storage_step = "nvs_commit"; expected_storage_error = ESP_FAIL;
    submit(18, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_failed") && erases == 0 && persisted_size);
    fail_commit = false;
    cal_store_pair_t canonical_before = active;
    submit(19, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_completed") && erases == 1 && !persisted_size);
    assert(!memcmp(&active, &canonical_before, sizeof(active)));
    assert(frames == before_cleanup_frames);
    submit(20, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_no_reference") && erases == 1);
    image(false, false);
    submit(21, 4, "empty", false); assert(stage == 1);
    image(true, false);
    submit(22, 4, "full", false); assert(stage == 2 && writes == 2);
    submit(23, 4, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_current") && persisted_size);
    /* Reproduce full's no-space error, remove the obsolete reference only,
     * preserve a new empty in RAM and retry full without recapturing empty. */
    active.config_version = 18;
    image(false, false);
    submit(24, 5, "empty", false); assert(stage == 1);
    image(true, false);
    fail_set_space = true;
    expected_storage_step = "nvs_set_blob";
    expected_storage_error = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
    submit(25, 5, "full", false);
    assert(stage == 1 && writes == 2 && persisted_size);
    assert(!strcmp(last_status, "persistence_or_context_failed"));
    fail_set_space = false;
    submit(26, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_completed") && stage == 1 && !persisted_size);
    submit(27, 5, "full", false);
    assert(stage == 2 && writes == 3 && !strcmp(last_status, "full_captured"));
    /* Explicit discard is separate from obsolete-only cleanup: never infer
     * permission from a new UUID. Check persisted session even after boot. */
    unsigned before_discard_frames = frames, before_discard_erases = erases;
    canonical_before = active;
    discard_confirm = true;
    submit(200, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_ram_session") && persisted_size);
    reboot_ram();
    submit(201, 9, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_session") && persisted_size);
    submit(202, 5, "cleanup", true);
    cJSON *missing_confirmation = cJSON_Parse(last_command);
    cJSON_DeleteItemFromObject(missing_confirmation, "confirm_cleanup");
    assert(vision_trial_submit(missing_confirmation, false) != ESP_OK);
    cJSON_Delete(missing_confirmation);
    cJSON *bad_flag = cJSON_Parse(last_command);
    cJSON_ReplaceItemInObject(bad_flag, "confirm_discard_current", cJSON_CreateString("true"));
    assert(vision_trial_submit(bad_flag, false) != ESP_OK);
    cJSON_Delete(bad_flag);
    cJSON *wrong_action = cJSON_Parse(last_command);
    cJSON_ReplaceItemInObject(wrong_action, "action", cJSON_CreateString("empty"));
    assert(vision_trial_submit(wrong_action, false) != ESP_OK);
    cJSON_Delete(wrong_action);
    cJSON *wrong_roi = cJSON_Parse(last_command);
    cJSON_ReplaceItemInObject(wrong_roi, "roi_id",
        cJSON_CreateString("20000000-0000-0000-0000-000000000002"));
    assert(vision_trial_submit(wrong_roi, false) == ESP_OK);
    cJSON_Delete(wrong_roi);
    assert(vision_trial_service());
    assert(!strcmp(last_status, "cleanup_refused_session") && persisted_size);
    persisted[persisted_size - 1] ^= 1;
    expected_storage_step = "cleanup_validate_blob";
    expected_storage_error = ESP_ERR_INVALID_STATE;
    submit(203, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_failed") && persisted_size);
    persisted[persisted_size - 1] ^= 1;
    discard_confirm = false;
    submit(204, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_current") && persisted_size);
    discard_confirm = true;
    submit(204, 5, "cleanup", false);
    assert(!strcmp(last_status, "request_id_conflict") && persisted_size);
    fail_commit = true;
    expected_storage_step = "nvs_commit"; expected_storage_error = ESP_FAIL;
    submit(205, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_failed") && persisted_size);
    fail_commit = false;
    binding_read_failed = true;
    submit(206, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_context_unavailable") && persisted_size);
    binding_read_failed = false;
    assert(erases == before_discard_erases);
    submit(207, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_completed") && !persisted_size);
    assert(stage == 0 && pattern == NULL && guard == NULL);
    assert(erases == before_discard_erases + 1);
    submit(207, 5, "cleanup", false);
    assert(!strcmp(last_status, "duplicate_not_reexecuted"));
    assert(erases == before_discard_erases + 1 && frames == before_discard_frames);
    assert(!memcmp(&active, &canonical_before, sizeof(active)));
    reboot_ram();
    discard_confirm = false;
    submit(208, 5, "status", false);
    assert(!strcmp(last_status, "uncalibrated") && stage == 0);
    discard_confirm = true;
    submit(209, 5, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_no_reference") && !pattern && !guard);
    discard_confirm = false;
    image(false, false);
    submit(210, 6, "empty", false);
    image(true, false);
    submit(211, 6, "full", false);
    assert(stage == 2 && writes == 4 && persisted_size);
    image(false, false);
    submit(212, 7, "empty", false);
    assert(stage == 1);
    discard_confirm = true;
    submit(213, 6, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_ram_session") && stage == 1 && persisted_size);
    submit(214, 7, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_refused_session") && stage == 1 && persisted_size);
    discard_confirm = false;
    reboot_ram();
    submit(215, 6, "status", false);
    assert(stage == 2 && pattern && guard);
    discard_confirm = true;
    submit(216, 6, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_completed") && stage == 0 && !persisted_size);
    assert(!pattern && !guard && !memcmp(&active, &canonical_before, sizeof(active)));
    discard_confirm = false;
    image(false, false);
    submit(217, 8, "empty", false);
    assert(stage == 1 && !persisted_size);
    discard_confirm = true;
    submit(218, 8, "cleanup", false);
    assert(!strcmp(last_status, "cleanup_no_reference") && stage == 0 && !pattern && !guard);
    discard_confirm = false;
    reboot_ram();
    test_frame_evaluator();
    image(false, false);
    submit(220, 10, "empty", false);
    image(true, false);
    submit(221, 10, "full", false);
    assert(stage == 2);
    reboot_ram();
    unsigned writes_before_auto = writes, erases_before_auto = erases;
    unsigned publications_before_auto = measurements;
    automatic_frame();
    assert(pilot_health.valid && pilot_health.occupancy_pct == 100 && stage == 2);
    unsigned reads_after_load = binding_reads, opens_after_load = storage_opens;
    automatic_frame();
    automatic_frame();
    assert(binding_reads == reads_after_load && storage_opens == opens_after_load);
    assert(writes == writes_before_auto && erases == erases_before_auto);
    assert(measurements == publications_before_auto);
    /* Duplicate/invalid frames do not retain the previous valid percentage. */
    vision_pilot_process_frame(&framebuffer);
    assert(!pilot_health.valid && pilot_health.occupancy_pct == 255);
    assert(!strcmp(pilot_health.status, "invalid_or_stale_frame"));
    vision_pilot_process_frame(NULL);
    assert(!pilot_health.valid && pilot_health.occupancy_pct == 255);
    image(false, false);
    automatic_frame();
    assert(pilot_health.valid && pilot_health.occupancy_pct == 0);
    /* Banda de aviso en el piloto: mide, cuenta guard_warn y no rechaza. */
    unsigned guard_warns_before = pilot_health.guard_warn;
    flatten_guard(guard, guard->count / 2);
    automatic_frame();
    assert(pilot_health.valid && pilot_health.occupancy_pct == 0);
    assert(pilot_health.guard_warn == guard_warns_before + 1);
    image(false, true);
    automatic_frame();
    assert(!pilot_health.valid && pilot_health.background_rejected == 1);
    image(true, false);
    ++active_revision; /* commit in progress */
    automatic_frame();
    assert(!pilot_health.valid && !strcmp(pilot_health.status, "context_unavailable"));
    ++active_revision;
    automatic_frame();
    assert(pilot_health.valid);
    revision_after_snapshot = 2; /* epoch changes during the measured frame */
    automatic_frame();
    assert(!pilot_health.valid && !strcmp(pilot_health.status, "context_changed"));
    automatic_frame();
    assert(pilot_health.valid);
    roi_shift = .01f; /* geometry mismatch, even if caller forgot version */
    automatic_frame();
    assert(!pilot_health.valid && !strcmp(pilot_health.status, "context_changed"));
    roi_shift = 0;
    extra_roi = true;
    automatic_frame();
    assert(pilot_health.valid && pilot_health.other_rois_skipped > 0);
    extra_roi = false;
    snapshot_failed = true;
    automatic_frame();
    assert(!pilot_health.valid);
    snapshot_failed = false;
    ++active.calibration_version; active_revision += 2;
    automatic_frame();
    /* Reload discards the old slot; new active version has no matching kernel.
     * Geometry mismatches above still report context_changed without reloading. */
    assert(!pilot_health.valid && !strcmp(pilot_health.status, "references_required"));
    assert(pilot_health.occupancy_pct == 255);
    --active.calibration_version; active_revision += 2;
    automatic_frame();
    assert(pilot_health.valid);
    ++active.config_version;
    automatic_frame();
    assert(!pilot_health.valid);
    --active.config_version;
    assert(writes == writes_before_auto && erases == erases_before_auto);
    vision_pilot_health_t health;
    vision_pilot_health(&health);
    assert(health.frames == health.accepted + health.rejected);
    vision_pilot_health_log(4096);
    /* Mantenimiento: borrado total de calibracion en una sola accion. */
    image(false, false);
    submit(90, 11, "empty", false);
    assert(stage == 1);
    image(true, false);
    submit(91, 11, "full", false);
    assert(stage == 2 && persisted_size);
    unsigned erases_before_wipe = erases, opens_before_wipe = storage_opens;
    submit_wipe(300, 12, false);           /* sin el literal: rechazado */
    assert(wipe_calls == 0 && erases == erases_before_wipe && stage == 2);
    submit_wipe(301, 12, true);
    assert(wipe_calls == 1);
    assert(!strcmp(last_status, "wipe_completed"));
    assert(erases == erases_before_wipe + 1 && !persisted_size);
    assert(stage == 0 && pattern == NULL && guard == NULL);
    submit(302, 12, "status", false);
    assert(!strcmp(last_status, "uncalibrated"));
    /* No se relee el blob borrado ni se mide sin recalibrar. */
    submit(303, 12, "sample", false);
    assert(!strcmp(last_status, "calibration_required"));
    assert(storage_opens == opens_before_wipe + 1);
    reboot_ram();
    /* Production: two ROI, cached references, throttled telemetry and no
     * publishing on rejects. The preceding experimental run publishes nothing. */
    assert(roi_publications == 0);
    production_fixture = true;
    fixture_version = active.calibration_version;
    extra_roi = true;
    image(false, false);
    automatic_frame();
    assert(using_production && production_count == 2 && pilot_health.valid);
    assert(roi_publications == 1);
    /* Fase 6: la primera muestra vale por si misma; no arranca en cero falso. */
    assert(pilot_health.median_samples == 1);
    assert(pilot_health.occupancy_pct == 0);
    assert(pilot_health.occupancy_smoothed_pct == 0);
    assert(last_published_value == 0);
    unsigned kernel_after_load = kernel_reads;
    automatic_frame();
    assert(pilot_health.valid && roi_publications == 2);
    automatic_frame();
    assert(kernel_reads == kernel_after_load && roi_publications == 2);
    assert(production_index == 1);
    clock_us += PROD_HEARTBEAT_US;
    queue_full = true;
    automatic_frame();
    assert(roi_publications == 2 && production_queue_rejected);
    queue_full = false;
    automatic_frame();
    assert(roi_publications == 3);
    clock_us += PROD_HEARTBEAT_US;
    unsigned window_before_stale[PROD_MAX_ROI];
    for (unsigned i = 0; i < PROD_MAX_ROI; ++i)
        window_before_stale[i] = production_median_count[i];
    stale = true;
    automatic_frame();
    assert(!pilot_health.valid && roi_publications == 3);
    /* Un rechazo no aporta muestra ni vacia la ventana. */
    for (unsigned i = 0; i < PROD_MAX_ROI; ++i)
        assert(production_median_count[i] == window_before_stale[i]);
    stale = false;
    /* Invalid first ROI must not block the second slot. */
    roi_shift = .01f;
    automatic_frame();
    assert(!pilot_health.valid);
    automatic_frame();
    assert(pilot_health.valid);
    roi_shift = 0;
    unsigned before_revision = roi_publications;
    ++active.calibration_version; active_revision += 2;
    automatic_frame();
    assert(!using_production && !pilot_health.valid);
    assert(roi_publications == before_revision);
    --active.calibration_version; active_revision += 2;
    automatic_frame();
    assert(using_production && pilot_health.valid);
    assert(roi_publications == before_revision + 1);
    /* Fase 6: un pico aislado de 100 % no se publica; manda la mediana.
     * Primero se llena la ventana de ambos slots con lecturas vacias. */
    for (unsigned i = 0; i < 2U * PROD_MEDIAN_SAMPLES; ++i) automatic_frame();
    assert(production_median_count[0] == PROD_MEDIAN_SAMPLES);
    assert(production_median_count[1] == PROD_MEDIAN_SAMPLES);
    assert(pilot_health.occupancy_smoothed_pct == 0);
    unsigned published_before_spike = roi_publications;
    clock_us += PROD_HEARTBEAT_US;
    image(true, false);
    unsigned spikes = 0;
    for (unsigned i = 0; i < 2; ++i) {
        unsigned before_frame = roi_publications;
        automatic_frame();
        assert(pilot_health.valid);
        spikes += pilot_health.occupancy_pct == 100;
        assert(pilot_health.occupancy_smoothed_pct == 0);
        if (roi_publications != before_frame) assert(last_published_value == 0);
    }
    assert(spikes == 1); /* solo la ROI que cubre la imagen llena mide 100 */
    assert(roi_publications > published_before_spike);

    /* Fase 6 incremento 3: 5 rechazos de fondo consecutivos -> un solo
     * cam_moved, sin publicar value mientras la ROI esté marcada como movida. */
    production_fixture = false;
    extra_roi = false;
    reboot_ram();
    production_fixture = true;
    fixture_version = active.calibration_version;
    image(false, false);
    automatic_frame();
    assert(using_production && production_count == 1);
    for (unsigned i = 0; i < PROD_MEDIAN_SAMPLES; ++i) automatic_frame();
    clock_us += PROD_HEARTBEAT_US;
    unsigned pubs_before_move = roi_publications;
    unsigned moved_before = cam_moved_events;
    flatten_guard(prod_guard[0], prod_guard[0]->count - prod_guard[0]->count / 5);
    /* Un rechazo de frame en medio rompe la racha: no dispara. */
    stale = true;
    automatic_frame();
    stale = false;
    for (unsigned i = 0; i < PROD_CAM_MOVED_FRAMES - 1U; ++i) automatic_frame();
    assert(cam_moved_events == moved_before);
    automatic_frame();
    assert(cam_moved_events == moved_before + 1);
    assert(roi_publications == pubs_before_move);
    /* Insistir no reenvía el aviso ni publica porcentaje. */
    for (unsigned i = 0; i < 3U; ++i) automatic_frame();
    assert(cam_moved_events == moved_before + 1);
    assert(roi_publications == pubs_before_move);
    /* Recuperación: 3 VR_OK limpian el estado y se reanuda la publicación. */
    image(false, false);
    for (unsigned i = 0; i < PROD_CAM_MOVED_RECOVERY + 1U; ++i) automatic_frame();
    assert(!production_cam_moved_active[0]);
    assert(cam_moved_events == moved_before + 1);
    assert(roi_publications > pubs_before_move);

    production_fixture = false;
    extra_roi = false;
    reboot_ram();
    puts("vision_trial: runtime, safe cleanup/discard, CRC, context, no-space recovery and release OK");
    return 0;
}
