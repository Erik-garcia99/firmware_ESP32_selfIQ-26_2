#include "vision_runtime.h"
#include <string.h>

vr_status_t vr_measure(lp_context_t *pattern, const lg_context_t *guard,
                       uint32_t full_pixels, const vr_frame_t *f,
                       vr_result_t *out) {
    if (!out) return VR_MEASUREMENT;
    memset(out, 0, sizeof(*out));
    out->value.relative_pct = 255;
    out->value.area_pct = 255;
    if (!f || !f->gray || !f->grayscale ||
        f->width != VR_WIDTH || f->height != VR_HEIGHT ||
        f->length != (size_t)VR_WIDTH * VR_HEIGHT ||
        f->captured_us <= 0 || f->captured_us <= f->previous_us ||
        f->captured_us > f->now_us || f->now_us - f->captured_us > VR_MAX_AGE_US)
        return out->status = VR_FRAME;
    if (!pattern || !guard || !pattern->ready || !guard->ready ||
        pattern->width != VR_WIDTH || pattern->height != VR_HEIGHT ||
        guard->width != VR_WIDTH || guard->height != VR_HEIGHT ||
        pattern->w < 4 || pattern->h < 4 ||
        pattern->x >= VR_WIDTH || pattern->y >= VR_HEIGHT ||
        pattern->w > VR_WIDTH - pattern->x || pattern->h > VR_HEIGHT - pattern->y ||
        pattern->cols != (pattern->w + 15U) / 16U ||
        pattern->rows != (pattern->h + 15U) / 16U ||
        pattern->count != pattern->cols * pattern->rows ||
        pattern->count > LP_MAX_TILES || guard->count < 16 || guard->count > LG_MAX ||
        !full_pixels || full_pixels > pattern->w * pattern->h ||
        (uint64_t)full_pixels * 25 < (uint64_t)pattern->w * pattern->h)
        return out->status = VR_UNCALIBRATED;
    for (uint32_t i = 0; i < guard->count; ++i)
        if (guard->x[i] > VR_WIDTH - 16U || guard->y[i] > VR_HEIGHT - 16U)
            return out->status = VR_UNCALIBRATED;
    uint32_t clipped = 0;
    for (uint32_t y = pattern->y; y < pattern->y + pattern->h; ++y)
        for (uint32_t x = pattern->x; x < pattern->x + pattern->w; ++x) {
            unsigned v = f->gray[(size_t)y * VR_WIDTH + x];
            clipped += v <= 5 || v >= 250;
        }
    if ((uint64_t)clipped * 100 > (uint64_t)pattern->w * pattern->h * 20)
        return out->status = VR_CLIPPING;
    if (lg_check(guard, f->gray, f->length, VR_WIDTH, &out->matched, &out->informative))
        return out->status = VR_BACKGROUND;
    lp_result_t value;
    if (lp_measure(pattern, f->gray, f->length, VR_WIDTH, full_pixels, &value) != LP_OK ||
        value.relative_pct == 255)
        return out->status = VR_MEASUREMENT;
    if ((uint64_t)value.weak_reference_pixels * 100 > (uint64_t)value.total_pixels * 25)
        return out->status = VR_WEAK_REFERENCE;
    out->value = value;
    return out->status = VR_OK;
}

const char *vr_status_name(vr_status_t status) {
    switch (status) {
    case VR_OK: return "ok";
    case VR_FRAME: return "invalid_or_stale_frame";
    case VR_UNCALIBRATED: return "references_required";
    case VR_CLIPPING: return "excessive_clipping";
    case VR_BACKGROUND: return "background_inconclusive";
    case VR_MEASUREMENT: return "measurement_failed";
    case VR_WEAK_REFERENCE: return "weak_reference";
    default: return "unknown";
    }
}
