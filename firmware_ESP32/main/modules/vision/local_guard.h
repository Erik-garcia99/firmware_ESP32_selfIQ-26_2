#ifndef SELFIQ_LOCAL_GUARD_H
#define SELFIQ_LOCAL_GUARD_H
#include <stddef.h>
#include <stdint.h>
#define LG_MAX 128U
/* Experimental exterior-patch consistency guard, not camera registration. */
typedef struct {
    uint32_t width, height, count, samples;
    uint16_t x[LG_MAX], y[LG_MAX], ref[LG_MAX][16];
    uint8_t ready;
} lg_context_t;
int lg_init(lg_context_t *c, uint32_t width, uint32_t height,
            uint32_t x, uint32_t y, uint32_t w, uint32_t h);
int lg_add(lg_context_t *c, const uint8_t *p, size_t n, uint32_t stride);
int lg_finish(lg_context_t *c);
/* 0 consistent, 1 inconclusive/invalid; no automatic realignment. */
int lg_check(const lg_context_t *c, const uint8_t *p, size_t n, uint32_t stride,
             uint32_t *matched, uint32_t *informative);
#endif
