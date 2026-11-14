#include "local_pattern.h"

#include <string.h>

size_t lp_context_bytes(void) { return sizeof(lp_context_t); }

int lp_init(lp_context_t *c, uint32_t width, uint32_t height,
            uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    if (!c) return LP_ARGUMENT;
    memset(c, 0, sizeof(*c));
    if (!width || !height || width > 4096U || height > 4096U ||
        w < 4U || h < 4U || x >= width || y >= height ||
        w > width - x || h > height - y) return LP_ARGUMENT;
    uint32_t cols = (w + 15U) / 16U, rows = (h + 15U) / 16U;
    if (cols * rows > LP_MAX_TILES) return LP_CAPACITY;
    c->width = width; c->height = height;
    c->x = x; c->y = y; c->w = w; c->h = h;
    c->cols = cols; c->rows = rows; c->count = cols * rows;
    return LP_OK;
}

static int frame_valid(const lp_context_t *c, const uint8_t *p,
                       size_t length, uint32_t stride) {
    return c && p && c->count && stride >= c->width &&
           (size_t)stride <= SIZE_MAX / c->height &&
           length >= (size_t)stride * c->height;
}

static uint32_t tile_area(const lp_context_t *c, uint32_t index) {
    uint32_t cx = index % c->cols, cy = index / c->cols;
    return ((cx + 1U) * c->w / c->cols - cx * c->w / c->cols) *
           ((cy + 1U) * c->h / c->rows - cy * c->h / c->rows);
}

static void features(const lp_context_t *c, const uint8_t *p, uint32_t stride,
                     uint32_t index, uint16_t out[LP_FEATURES]) {
    uint32_t cx = index % c->cols, cy = index / c->cols;
    uint32_t left = c->x + cx * c->w / c->cols;
    uint32_t top = c->y + cy * c->h / c->rows;
    uint32_t tw = (cx + 1U) * c->w / c->cols - cx * c->w / c->cols;
    uint32_t th = (cy + 1U) * c->h / c->rows - cy * c->h / c->rows;
    for (uint32_t by = 0; by < 4U; ++by) {
        for (uint32_t bx = 0; bx < 4U; ++bx) {
            uint32_t x0 = left + bx * tw / 4U;
            uint32_t x1 = left + (bx + 1U) * tw / 4U;
            uint32_t y0 = top + by * th / 4U;
            uint32_t y1 = top + (by + 1U) * th / 4U;
            uint32_t sum = 0, n = (x1 - x0) * (y1 - y0);
            for (uint32_t y = y0; y < y1; ++y)
                for (uint32_t x = x0; x < x1; ++x)
                    sum += p[(size_t)y * stride + x];
            out[by * 4U + bx] = (uint16_t)((sum + n / 2U) / n);
        }
    }
}

int lp_add_empty(lp_context_t *c, const uint8_t *p, size_t n, uint32_t stride) {
    if (!frame_valid(c, p, n, stride)) return LP_FRAME;
    if (c->ready || c->samples >= 9U) return LP_ARGUMENT;
    for (uint32_t i = 0; i < c->count; ++i) {
        uint16_t f[LP_FEATURES];
        features(c, p, stride, i, f);
        for (uint32_t k = 0; k < LP_FEATURES; ++k)
            c->reference[i][k] += f[k];
    }
    ++c->samples;
    return LP_OK;
}

int lp_finish_empty(lp_context_t *c) {
    if (!c || !c->count || !c->samples || c->ready) return LP_UNCALIBRATED;
    for (uint32_t i = 0; i < c->count; ++i)
        for (uint32_t k = 0; k < LP_FEATURES; ++k)
            c->reference[i][k] = (uint16_t)(
                (c->reference[i][k] + c->samples / 2U) / c->samples);
    c->ready = 1U;
    return LP_OK;
}

static int foreground(const uint16_t a[LP_FEATURES],
                      const uint16_t b[LP_FEATURES], int *weak) {
    int64_t sa = 0, sb = 0, aa = 0, bb = 0, ab = 0;
    for (uint32_t k = 0; k < LP_FEATURES; ++k) {
        sa += a[k]; sb += b[k];
        aa += (int64_t)a[k] * a[k];
        bb += (int64_t)b[k] * b[k];
        ab += (int64_t)a[k] * b[k];
    }
    int64_t va = 16 * aa - sa * sa, vb = 16 * bb - sb * sb;
    int64_t cov = 16 * ab - sa * sb;
    *weak = va < 16 * 16 * 4; /* reference stddev <2 */
    int64_t residual = 0;
    for (uint32_t k = 0; k < LP_FEATURES; ++k) {
        int64_t d = 16 * ((int64_t)a[k] - b[k]) - sa + sb;
        residual += d < 0 ? -d : d;
    }
    if (residual <= 16 * 16 * 5) return 0;
    if (!va || !vb) return 1;
    /* Bounds: variance <=4,161,600; multiplied squares fit signed int64.
     * NCC <0.70 or reversed contrast => foreground. No sqrt/libm/floats. */
    return cov <= 0 || cov * cov * 100 < va * vb * 49;
}

int lp_measure(lp_context_t *c, const uint8_t *p, size_t n, uint32_t stride,
               uint32_t full_pixels, lp_result_t *out) {
    if (!out) return LP_ARGUMENT;
    memset(out, 0, sizeof(*out));
    out->area_pct = 255U; out->relative_pct = 255U;
    if (!frame_valid(c, p, n, stride)) return LP_FRAME;
    if (!c->ready) return LP_UNCALIBRATED;
    out->total_pixels = c->w * c->h;
    for (uint32_t i = 0; i < c->count; ++i) {
        uint16_t f[LP_FEATURES];
        int weak = 0;
        features(c, p, stride, i, f);
        c->mask[i] = (uint8_t)foreground(c->reference[i], f, &weak);
        uint32_t area = tile_area(c, i);
        if (weak) out->weak_reference_pixels += area;
        if (c->mask[i]) out->raw_occupied_pixels += area;
    }
    for (uint32_t i = 0; i < c->count; ++i) {
        if (!c->mask[i]) continue;
        int cx = (int)(i % c->cols), cy = (int)(i / c->cols);
        int neighbors = 0, available = 0;
        for (int y = cy - 1; y <= cy + 1; ++y)
            for (int x = cx - 1; x <= cx + 1; ++x)
                if (x >= 0 && y >= 0 && x < (int)c->cols && y < (int)c->rows) {
                    ++available;
                    neighbors += c->mask[(uint32_t)y * c->cols + (uint32_t)x] != 0;
                }
        if (neighbors >= (available < 3 ? available : 3))
            out->occupied_pixels += tile_area(c, i);
    }
    out->area_pct = (uint8_t)(
        (100U * out->occupied_pixels + out->total_pixels / 2U) / out->total_pixels);
    if (full_pixels <= out->total_pixels && full_pixels &&
        (uint64_t)full_pixels * 25U >= out->total_pixels) {
        out->relative_unclamped_q8 = (uint32_t)(
            ((uint64_t)out->occupied_pixels * 25600U + full_pixels / 2U) / full_pixels);
        uint32_t pct = (out->relative_unclamped_q8 + 128U) / 256U;
        out->relative_pct = pct > 100U ? 100U : (uint8_t)pct;
    }
    return LP_OK;
}
