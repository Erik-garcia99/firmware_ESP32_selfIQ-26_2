/* Host diagnostic ONLY: exhaustive translation probe on high-pass texture.
 * Not a runtime alignment algorithm or quality gate. Uses double/libm.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
    int dx, dy, points;
    double unshifted_correlation, best_correlation;
} registration_result_t;

static void highpass(const uint8_t *src, int16_t *dst, int w, int h) {
    for (int y = 4; y < h - 4; ++y)
        for (int x = 4; x < w - 4; ++x) {
            int sum = 0;
            for (int j = -4; j <= 4; j += 4)
                for (int i = -4; i <= 4; i += 4)
                    sum += src[(y + j) * w + x + i];
            dst[y * w + x] = (int16_t)(9 * (int)src[y * w + x] - sum);
        }
}

static double correlation(const int16_t *a, const int16_t *b, int w, int h,
                          int rx, int ry, int rw, int rh, int dx, int dy,
                          int ring, int *points) {
    double sa = 0, sb = 0, aa = 0, bb = 0, ab = 0;
    int n = 0;
    for (int y = 32; y < h - 32; y += 4)
        for (int x = 32; x < w - 32; x += 4) {
            int inside = x >= rx && x < rx + rw && y >= ry && y < ry + rh;
            int near = x >= rx - 32 && x < rx + rw + 32 &&
                       y >= ry - 32 && y < ry + rh + 32;
            if (ring ? (!near || inside) : !inside) continue;
            double va = a[y * w + x], vb = b[(y + dy) * w + x + dx];
            sa += va; sb += vb; aa += va * va; bb += vb * vb; ab += va * vb;
            ++n;
        }
    *points = n;
    double variance = (n * aa - sa * sa) * (n * bb - sb * sb);
    return variance > 0 ? (n * ab - sa * sb) / sqrt(variance) : -1;
}

int registration_probe(const uint8_t *a, const uint8_t *b, int w, int h,
                       int rx, int ry, int rw, int rh, int ring,
                       registration_result_t *out) {
    if (!a || !b || !out || w < 80 || h < 80 || w > 1920 || h > 1080 ||
        rx < 0 || ry < 0 || rw <= 0 || rh <= 0 || rx > w - rw || ry > h - rh)
        return 1;
    int16_t *ha = calloc((size_t)w * (size_t)h, sizeof(*ha));
    int16_t *hb = calloc((size_t)w * (size_t)h, sizeof(*hb));
    if (!ha || !hb) { free(ha); free(hb); return 2; }
    highpass(a, ha, w, h); highpass(b, hb, w, h);
    out->dx = 0; out->dy = 0;
    out->unshifted_correlation = correlation(
        ha, hb, w, h, rx, ry, rw, rh, 0, 0, ring, &out->points);
    out->best_correlation = out->unshifted_correlation;
    for (int dy = -24; dy <= 24; dy += 2)
        for (int dx = -24; dx <= 24; dx += 2) {
            int n;
            double score = correlation(ha, hb, w, h, rx, ry, rw, rh, dx, dy, ring, &n);
            if (score > out->best_correlation) {
                out->best_correlation = score; out->dx = dx; out->dy = dy;
            }
        }
    int bx = out->dx, by = out->dy;
    for (int dy = by - 1; dy <= by + 1; ++dy)
        for (int dx = bx - 1; dx <= bx + 1; ++dx) {
            int n;
            double score = correlation(ha, hb, w, h, rx, ry, rw, rh, dx, dy, ring, &n);
            if (score > out->best_correlation) {
                out->best_correlation = score; out->dx = dx; out->dy = dy;
            }
        }
    free(ha); free(hb);
    return 0;
}
