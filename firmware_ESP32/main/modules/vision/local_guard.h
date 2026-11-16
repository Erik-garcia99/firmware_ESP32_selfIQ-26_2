#ifndef SELFIQ_LOCAL_GUARD_H
#define SELFIQ_LOCAL_GUARD_H
#include <stddef.h>
#include <stdint.h>
#define LG_MAX 128U
/* Guarda de fondo por bandas (decisión del operador, 15/11/2026).
 * No es registro de cámara: son parches 16x16 exteriores al ROI.
 *   >= LG_GUARD_PASS_PCT                        -> mide normal
 *   [LG_GUARD_REJECT_PCT, LG_GUARD_PASS_PCT)    -> mide y registra guard_warn
 *   <  LG_GUARD_REJECT_PCT                      -> rechaza (movimiento grave,
 *                                                  cámara tapada o movida)
 * La banda intermedia evita abortar por un brazo o una persona cerca del ROI,
 * pero el piso duro conserva la detección de un desplazamiento real. */
#define LG_GUARD_REJECT_PCT 25U
#define LG_GUARD_PASS_PCT 60U
typedef enum {
    LG_GUARD_PASS = 0,   /* coincidencia >= 60 % */
    LG_GUARD_WARN = 1,   /* 25-60 %: medir y avisar */
    LG_GUARD_REJECT = 2  /* < 25 % o evidencia insuficiente */
} lg_verdict_t;
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
/* Veredicto por bandas; matched/informative se exponen al llamador.
 * Un frame inválido o una guarda sin preparar devuelve LG_GUARD_REJECT.
 * No hay realineamiento automático. */
lg_verdict_t lg_guard_verdict(const lg_context_t *c, const uint8_t *p, size_t n,
                              uint32_t stride, uint32_t *matched,
                              uint32_t *informative);
/* Misma regla de bandas aplicada a conteos ya obtenidos; no recalcula píxeles.
 * Útil para anotar el nivel en respuestas y logs sin duplicar umbrales. */
lg_verdict_t lg_guard_band(uint32_t matched, uint32_t informative);
const char *lg_verdict_name(lg_verdict_t verdict);
#endif
