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
    return true;
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
static void reboot_ram(void) {
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
int main(void) {
    strcpy(active.roi_id, "20000000-0000-0000-0000-000000000001");
    strcpy(active.calibration_id, "30000000-0000-0000-0000-000000000001");
    memset(active.context_hash, 'a', 64);
    active.config_version = 16; active.calibration_version = 2;
    active.has_empty = active.has_full = true;
    assert(vision_trial_init() == ESP_OK);
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
    assert(!pilot_health.valid && !strcmp(pilot_health.status, "context_changed"));
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
    reboot_ram();
    puts("vision_trial: runtime, safe cleanup/discard, CRC, context, no-space recovery and release OK");
    return 0;
}
