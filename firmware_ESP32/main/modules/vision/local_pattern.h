#ifndef SELFIQ_EXPERIMENTAL_LOCAL_PATTERN_H
#define SELFIQ_EXPERIMENTAL_LOCAL_PATTERN_H

#include <stddef.h>
#include <stdint.h>

/* Experimental shared host/device kernel; not a persisted format.
 * Defaults frozen before evaluating this candidate, not product acceptance.
 * 16 px tiles, 4x4 block means per tile, positive NCC >=0.70 => background.
 * Local mean shifts and positive contrast scaling do not change NCC.
 * Ignore <=5 gray levels mean-centered absolute difference (noise floor).
 * Keep foreground tiles with >=3 foreground tiles in their 3x3 neighborhood.
 */
#define LP_MAX_TILES 1600U
#define LP_FEATURES 16U
#define LP_VERSION 1U

enum { LP_OK = 0, LP_ARGUMENT, LP_FRAME, LP_UNCALIBRATED, LP_CAPACITY };

typedef struct {
    uint32_t width, height, x, y, w, h, cols, rows, count;
    uint16_t samples;
    uint8_t ready;
    uint16_t reference[LP_MAX_TILES][LP_FEATURES];
    uint8_t mask[LP_MAX_TILES];
} lp_context_t;

typedef struct {
    uint32_t occupied_pixels, total_pixels, raw_occupied_pixels;
    uint32_t weak_reference_pixels;
    uint32_t relative_unclamped_q8; /* percentage * 256 */
    uint8_t area_pct, relative_pct; /* relative=255 without usable full */
} lp_result_t;

int lp_init(lp_context_t *ctx, uint32_t width, uint32_t height,
            uint32_t x, uint32_t y, uint32_t w, uint32_t h);
int lp_add_empty(lp_context_t *ctx, const uint8_t *gray, size_t length,
                 uint32_t stride);
int lp_finish_empty(lp_context_t *ctx);
int lp_measure(lp_context_t *ctx, const uint8_t *gray, size_t length,
               uint32_t stride, uint32_t full_pixels, lp_result_t *out);
size_t lp_context_bytes(void);

#endif
