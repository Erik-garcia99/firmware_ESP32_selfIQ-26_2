#include "local_guard.h"
#include <string.h>

static int eligible(uint32_t px, uint32_t py, uint32_t x, uint32_t y,
                    uint32_t w, uint32_t h) {
    int a = (int)px, b = (int)py, rx = (int)x, ry = (int)y;
    int right = rx + (int)w, bottom = ry + (int)h;
    int near = a + 16 > rx - 64 && a < right + 64 &&
               b + 16 > ry - 64 && b < bottom + 64;
    int overlaps = a + 16 > rx - 8 && a < right + 8 &&
                   b + 16 > ry - 8 && b < bottom + 8;
    return near && !overlaps;
}
int lg_init(lg_context_t *c, uint32_t width, uint32_t height,
            uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    if (!c) return 1;
    memset(c, 0, sizeof(*c));
    if (width < 16 || height < 16 || width > 4096 || height > 4096 ||
        !w || !h || x >= width || y >= height || w > width - x || h > height - y)
        return 1;
    c->width = width; c->height = height;
    uint32_t total = 0;
    for (uint32_t py = 0; py + 16 <= height; py += 16)
        for (uint32_t px = 0; px + 16 <= width; px += 16)
            total += eligible(px, py, x, y, w, h);
    uint32_t step = (total + LG_MAX - 1U) / LG_MAX;
    if (!step) return 1;
    uint32_t index = 0;
    for (uint32_t py = 0; py + 16 <= height; py += 16)
        for (uint32_t px = 0; px + 16 <= width; px += 16) {
            if (!eligible(px, py, x, y, w, h)) continue;
            if (index++ % step) continue;
            c->x[c->count] = (uint16_t)px; c->y[c->count] = (uint16_t)py;
            ++c->count;
        }
    return c->count < 16;
}
static int valid(const lg_context_t *c, const uint8_t *p, size_t n, uint32_t stride) {
    return c && p && c->count && stride >= c->width &&
           (size_t)stride <= SIZE_MAX / c->height && n >= (size_t)stride * c->height;
}
static void features(const lg_context_t *c, const uint8_t *p, uint32_t stride,
                     uint32_t i, uint16_t out[16]) {
    for (uint32_t by = 0; by < 4; ++by)
        for (uint32_t bx = 0; bx < 4; ++bx) {
            uint32_t sum = 0;
            for (uint32_t y = 0; y < 4; ++y)
                for (uint32_t x = 0; x < 4; ++x)
                    sum += p[(size_t)(c->y[i] + by * 4 + y) * stride
                             + c->x[i] + bx * 4 + x];
            out[by * 4 + bx] = (uint16_t)((sum + 8U) / 16U);
        }
}
int lg_add(lg_context_t *c, const uint8_t *p, size_t n, uint32_t stride) {
    if (!valid(c, p, n, stride) || c->ready || c->samples >= 9) return 1;
    for (uint32_t i = 0; i < c->count; ++i) {
        uint16_t f[16];
        features(c, p, stride, i, f);
        for (uint32_t k = 0; k < 16; ++k) c->ref[i][k] += f[k];
    }
    ++c->samples;
    return 0;
}
int lg_finish(lg_context_t *c) {
    if (!c || !c->count || !c->samples || c->ready) return 1;
    for (uint32_t i = 0; i < c->count; ++i)
        for (uint32_t k = 0; k < 16; ++k)
            c->ref[i][k] = (uint16_t)((c->ref[i][k] + c->samples / 2) / c->samples);
    c->ready = 1;
    return 0;
}
int lg_check(const lg_context_t *c, const uint8_t *p, size_t n, uint32_t stride,
             uint32_t *matched, uint32_t *informative) {
    if (!matched || !informative) return 1;
    *matched = 0; *informative = 0;
    if (!valid(c, p, n, stride) || !c->ready) return 1;
    for (uint32_t i = 0; i < c->count; ++i) {
        uint16_t f[16];
        features(c, p, stride, i, f);
        int64_t sa = 0, sb = 0, aa = 0, bb = 0, ab = 0, residual = 0;
        for (uint32_t k = 0; k < 16; ++k) {
            int64_t a = c->ref[i][k], b = f[k];
            sa += a; sb += b; aa += a * a; bb += b * b; ab += a * b;
        }
        int64_t va = 16 * aa - sa * sa, vb = 16 * bb - sb * sb;
        if (va < 1024) continue;
        ++*informative;
        int64_t cov = 16 * ab - sa * sb;
        for (uint32_t k = 0; k < 16; ++k) {
            int64_t d = 16 * ((int64_t)c->ref[i][k] - f[k]) - sa + sb;
            residual += d < 0 ? -d : d;
        }
        if (residual <= 1280 || (cov > 0 && cov * cov * 100 >= va * vb * 9))
            ++*matched;
    }
    return *informative < 16 || *matched * 100 < *informative * 60;
}
