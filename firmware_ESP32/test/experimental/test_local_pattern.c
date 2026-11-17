#include "local_pattern.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static lp_context_t ctx;
static uint8_t pixels[80U * 80U];

int main(void) {
    lp_result_t result;
    assert(lp_init(NULL, 64, 64, 0, 0, 64, 64) == LP_ARGUMENT);
    assert(lp_init(&ctx, 64, 64, 0, 0, 65, 64) == LP_ARGUMENT);
    assert(lp_init(&ctx, 4096, 4096, 0, 0, 4096, 4096) == LP_CAPACITY);
    assert(lp_init(&ctx, 64, 64, 0, 0, 64, 64) == LP_OK);
    assert(lp_measure(&ctx, pixels, sizeof(pixels), 80, 0, &result) == LP_UNCALIBRATED);
    assert(result.area_pct == 255);
    assert(lp_add_empty(&ctx, pixels, 1, 80) == LP_FRAME);
    assert(lp_measure(&ctx, NULL, sizeof(pixels), 80, 0, &result) == LP_FRAME);
    assert(lp_measure(&ctx, pixels, sizeof(pixels), 80, 0, NULL) == LP_ARGUMENT);

    for (unsigned h = 4; h <= 79; h += 3) {
        for (unsigned w = 4; w <= 79; w += 3) {
            for (unsigned y = 0; y < h; ++y)
                for (unsigned x = 0; x < w; ++x)
                    pixels[y * 80U + x] = (uint8_t)((x * 31U + y * 13U) % 256U);
            assert(lp_init(&ctx, w, h, 0, 0, w, h) == LP_OK);
            for (unsigned n = 0; n < 9; ++n)
                assert(lp_add_empty(&ctx, pixels, sizeof(pixels), 80) == LP_OK);
            assert(lp_add_empty(&ctx, pixels, sizeof(pixels), 80) == LP_ARGUMENT);
            assert(lp_finish_empty(&ctx) == LP_OK);
            assert(lp_finish_empty(&ctx) == LP_UNCALIBRATED);
            assert(lp_measure(&ctx, pixels, sizeof(pixels), 80, 0, &result) == LP_OK);
            assert(result.total_pixels == w * h);
            assert(result.occupied_pixels == 0);
            for (unsigned y = 0; y < h; ++y)
                for (unsigned x = 0; x < w; ++x)
                    pixels[y * 80U + x] = (uint8_t)(255U - pixels[y * 80U + x]);
            assert(lp_measure(&ctx, pixels, sizeof(pixels), 80, w * h, &result) == LP_OK);
            assert(result.area_pct <= 100 && result.relative_pct <= 100);
            assert(result.occupied_pixels <= result.total_pixels);
        }
    }
    /* Maximum contrast stresses the int64 covariance products. */
    assert(lp_init(&ctx, 64, 64, 0, 0, 64, 64) == LP_OK);
    for (unsigned y = 0; y < 64; ++y)
        for (unsigned x = 0; x < 64; ++x)
            pixels[y * 80U + x] = ((x / 4U + y / 4U) % 2U) ? 255U : 0U;
    assert(lp_add_empty(&ctx, pixels, sizeof(pixels), 80) == LP_OK);
    assert(lp_finish_empty(&ctx) == LP_OK);
    for (unsigned i = 0; i < sizeof(pixels); ++i)
        pixels[i] = (uint8_t)(255U - pixels[i]);
    assert(lp_measure(&ctx, pixels, sizeof(pixels), 80, 4096, &result) == LP_OK);
    assert(result.area_pct == 100);
    puts("local_pattern C: bounds, strides, 676 geometries, covariance extremes OK");
    return 0;
}
