#ifndef SELFIQ_VISION_RUNTIME_H
#define SELFIQ_VISION_RUNTIME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "local_guard.h"
#include "local_pattern.h"

/* E3 pure frame evaluator. Same frozen kernel/quality limits as the pilot.
 * No allocation, NVS, capture, JPEG, network or retained frame pointers. */
#define VR_WIDTH 640U
#define VR_HEIGHT 480U
#define VR_MAX_AGE_US 500000
typedef enum {
    VR_OK = 0, VR_FRAME, VR_UNCALIBRATED, VR_CLIPPING,
    VR_BACKGROUND, VR_MEASUREMENT, VR_WEAK_REFERENCE, VR_STATUS_COUNT
} vr_status_t;
typedef struct {
    const uint8_t *gray;
    size_t length;
    uint32_t width, height;
    bool grayscale;
    int64_t captured_us, now_us, previous_us;
} vr_frame_t;
typedef struct {
    vr_status_t status;
    lp_result_t value; /* relative_pct=255 on EVERY rejected result */
    uint32_t matched, informative;
} vr_result_t;

vr_status_t vr_measure(lp_context_t *pattern, const lg_context_t *guard,
                       uint32_t full_pixels, const vr_frame_t *frame,
                       vr_result_t *result);
const char *vr_status_name(vr_status_t status);
#endif
