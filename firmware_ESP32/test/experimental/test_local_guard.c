#include "../../main/modules/vision/local_guard.h"
#include <assert.h>
#include <stdio.h>

static lg_context_t guard;
static uint8_t pixels[160U * 160U], changed[160U * 160U];
int main(void) {
    uint32_t seed = 20261113, matched, informative;
    uint8_t blocks[40 * 40];
    for (unsigned i = 0; i < sizeof(blocks); ++i) {
        seed = seed * 1664525U + 1013904223U;
        blocks[i] = (uint8_t)(30U + seed % 130U);
    }
    for (unsigned y = 0; y < 160; ++y)
        for (unsigned x = 0; x < 160; ++x)
            pixels[y * 160 + x] = blocks[y / 4 * 40 + x / 4];
    assert(!lg_init(&guard, 160, 160, 60, 60, 40, 40));
    assert(guard.count <= LG_MAX);
    assert(lg_check(&guard, pixels, sizeof(pixels), 160, &matched, &informative));
    for (unsigned i = 0; i < 3; ++i)
        assert(!lg_add(&guard, pixels, sizeof(pixels), 160));
    assert(!lg_finish(&guard));
    assert(lg_finish(&guard));
    assert(!lg_check(&guard, pixels, sizeof(pixels), 160, &matched, &informative));
    assert(matched == informative && informative >= 16);
    for (unsigned i = 0; i < sizeof(pixels); ++i) changed[i] = pixels[i] + 25U;
    assert(!lg_check(&guard, changed, sizeof(changed), 160, &matched, &informative));
    /* Different product in ROI cannot affect exterior anchors. */
    for (unsigned y = 60; y < 100; ++y)
        for (unsigned x = 60; x < 100; ++x) changed[y * 160 + x] = 250;
    assert(!lg_check(&guard, changed, sizeof(changed), 160, &matched, &informative));
    for (unsigned i = 0; i < sizeof(pixels); ++i) changed[i] = 200U - pixels[i];
    assert(lg_check(&guard, changed, sizeof(changed), 160, &matched, &informative));
    assert(lg_check(&guard, pixels, 1, 160, &matched, &informative));
    assert(!lg_init(&guard, 160, 160, 60, 60, 40, 40));
    for (unsigned i = 0; i < sizeof(pixels); ++i) pixels[i] = 80;
    assert(!lg_add(&guard, pixels, sizeof(pixels), 160));
    assert(!lg_finish(&guard));
    assert(lg_check(&guard, pixels, sizeof(pixels), 160, &matched, &informative));
    assert(informative == 0);
    puts("local_guard: brightness, ROI exclusion, weak texture, invalid frame, state OK");
    return 0;
}
