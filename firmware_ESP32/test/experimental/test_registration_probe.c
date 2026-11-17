#include "registration_probe.c"
#include <assert.h>
#include <stdio.h>

static uint8_t a[160 * 160], b[160 * 160];

int main(void) {
    uint32_t seed = 20261113;
    uint8_t grid[40 * 40];
    for (unsigned i = 0; i < sizeof(grid); ++i) {
        seed = seed * 1664525U + 1013904223U;
        grid[i] = (uint8_t)(30U + seed % 130U);
    }
    for (int y = 0; y < 160; ++y)
        for (int x = 0; x < 160; ++x)
            a[y * 160 + x] = grid[(y / 4) * 40 + x / 4];
    registration_result_t result;
    assert(registration_probe(NULL, b, 160, 160, 60, 60, 40, 40, 1, &result));
    assert(registration_probe(a, b, 60, 60, 0, 0, 20, 20, 1, &result));
    assert(!registration_probe(a, a, 160, 160, 60, 60, 40, 40, 1, &result));
    assert(result.dx == 0 && result.dy == 0 && result.best_correlation > .999);
    for (unsigned i = 0; i < sizeof(a); ++i) b[i] = (uint8_t)(a[i] + 25U);
    assert(!registration_probe(a, b, 160, 160, 60, 60, 40, 40, 1, &result));
    assert(result.dx == 0 && result.dy == 0 && result.best_correlation > .999);
    for (int y = 0; y < 160; ++y)
        for (int x = 0; x < 160; ++x)
            b[y * 160 + x] = x + 4 < 160 && y + 8 < 160
                ? a[(y + 8) * 160 + x + 4] : 0U;
    assert(!registration_probe(a, b, 160, 160, 60, 60, 40, 40, 1, &result));
    assert(result.dx == -4 && result.dy == -8 && result.best_correlation > .999);
    puts("registration_probe: identity, brightness, known shift and invalid arguments OK");
    return 0;
}
