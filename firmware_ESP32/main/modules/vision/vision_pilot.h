#ifndef SELFIQ_VISION_PILOT_H
#define SELFIQ_VISION_PILOT_H

#include <stdbool.h>
#include <stdint.h>
#include "modules/camera/camera_lib.h"

/* E3 first increment, single-ROI reference slot from the existing pilot.
 * Camera-task-only API. Automatic, local diagnostics; never roi_occ.
 * Future app capture integration/multi-ROI storage are NOT implemented here. */
typedef struct {
    uint32_t frames, accepted, rejected, context_rejected, references_missing;
    uint32_t other_rois_skipped, processing_us, max_processing_us;
    uint32_t relative_unclamped_q8, matched, informative;
    uint32_t frame_rejected, clipping_rejected, background_rejected, weak_rejected;
    uint32_t config_version, calibration_version;
    int64_t captured_us;
    bool valid;
    uint8_t occupancy_pct; /* 255 if invalid, not a stock-zero reading */
    char roi_id[37];
    const char *status;
} vision_pilot_health_t;

void vision_pilot_process_frame(const camera_fb_t *frame);
void vision_pilot_health_log(uint32_t stack_min_free);
/* Copy current camera-task state; do not call concurrently from MQTT. */
void vision_pilot_health(vision_pilot_health_t *out);
#endif
