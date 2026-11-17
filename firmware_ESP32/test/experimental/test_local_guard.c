#include "../../main/modules/vision/local_guard.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static lg_context_t guard;
static uint8_t pixels[160U * 160U], changed[160U * 160U];
/* Aplana las anclas [0, limit) a un gris uniforme: dejan de coincidir con su
 * referencia sin tocar la desviación de la referencia, así que siguen contando
 * como "informative" y bajan el porcentaje de coincidencia. */
static void flatten_anchors(const lg_context_t *c, uint32_t limit) {
    for (uint32_t i = 0; i < limit && i < c->count; ++i)
        for (unsigned y = 0; y < 16; ++y)
            for (unsigned x = 0; x < 16; ++x)
                changed[(size_t)(c->y[i] + y) * 160 + c->x[i] + x] = 80;
}
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
    /* Sin preparar: no hay veredicto posible. */
    assert(lg_guard_verdict(&guard, pixels, sizeof(pixels), 160,
                            &matched, &informative) == LG_GUARD_REJECT);
    for (unsigned i = 0; i < 3; ++i)
        assert(!lg_add(&guard, pixels, sizeof(pixels), 160));
    assert(!lg_finish(&guard));
    assert(lg_finish(&guard));
    /* Escena idéntica: coincide todo, banda alta. */
    assert(lg_guard_verdict(&guard, pixels, sizeof(pixels), 160,
                            &matched, &informative) == LG_GUARD_PASS);
    assert(matched == informative && informative >= 16);
    /* Brillo global moderado: sigue coincidiendo (comparación invariante). */
    for (unsigned i = 0; i < sizeof(pixels); ++i) changed[i] = pixels[i] + 25U;
    assert(lg_guard_verdict(&guard, changed, sizeof(changed), 160,
                            &matched, &informative) == LG_GUARD_PASS);
    /* Diferente producto en el ROI no afecta a las anclas exteriores. */
    for (unsigned y = 60; y < 100; ++y)
        for (unsigned x = 60; x < 100; ++x) changed[y * 160 + x] = 250;
    assert(lg_guard_verdict(&guard, changed, sizeof(changed), 160,
                            &matched, &informative) == LG_GUARD_PASS);
    /* Inversión total: nada coincide -> piso duro, rechazo. */
    for (unsigned i = 0; i < sizeof(pixels); ++i) changed[i] = 200U - pixels[i];
    assert(lg_guard_verdict(&guard, changed, sizeof(changed), 160,
                            &matched, &informative) == LG_GUARD_REJECT);
    assert(lg_guard_verdict(&guard, pixels, 1, 160,
                            &matched, &informative) == LG_GUARD_REJECT);
    /* Bandas 15/11/2026: media guarda alterada mide con aviso, no se rechaza. */
    for (unsigned i = 0; i < sizeof(pixels); ++i) changed[i] = pixels[i];
    flatten_anchors(&guard, guard.count / 2);
    assert(lg_guard_verdict(&guard, changed, sizeof(changed), 160,
                            &matched, &informative) == LG_GUARD_WARN);
    assert(informative >= 16);
    assert(matched * 100U < informative * LG_GUARD_PASS_PCT);
    assert(matched * 100U >= informative * LG_GUARD_REJECT_PCT);
    assert(!strcmp(lg_verdict_name(LG_GUARD_WARN), "warn"));
    /* Más de 3/4 de la guarda alterada: por debajo del piso -> rechazo. */
    for (unsigned i = 0; i < sizeof(pixels); ++i) changed[i] = pixels[i];
    flatten_anchors(&guard, guard.count - guard.count / 5);
    assert(lg_guard_verdict(&guard, changed, sizeof(changed), 160,
                            &matched, &informative) == LG_GUARD_REJECT);
    /* Guarda sin textura: sin evidencia espacial, no se mide a ciegas. */
    assert(!lg_init(&guard, 160, 160, 60, 60, 40, 40));
    for (unsigned i = 0; i < sizeof(pixels); ++i) pixels[i] = 80;
    assert(!lg_add(&guard, pixels, sizeof(pixels), 160));
    assert(!lg_finish(&guard));
    assert(lg_guard_verdict(&guard, pixels, sizeof(pixels), 160,
                            &matched, &informative) == LG_GUARD_REJECT);
    assert(informative == 0);
    assert(lg_guard_band(47, 47) == LG_GUARD_PASS);
    assert(lg_guard_band(29, 47) == LG_GUARD_PASS);  /* 61.7 %: sobre el umbral */
    assert(lg_guard_band(28, 47) == LG_GUARD_WARN);  /* 59.6 %: medía y ahora avisa */
    assert(lg_guard_band(26, 47) == LG_GUARD_WARN);  /* 55.3 %: medía y ahora avisa */
    assert(lg_guard_band(12, 47) == LG_GUARD_WARN);  /* 25.5 %: borde inferior */
    assert(lg_guard_band(11, 47) == LG_GUARD_REJECT); /* 23.4 %: bajo el piso */
    assert(lg_guard_band(0, 0) == LG_GUARD_REJECT);
    assert(!strcmp(lg_verdict_name(LG_GUARD_PASS), "pass"));
    assert(!strcmp(lg_verdict_name(LG_GUARD_REJECT), "reject"));
    puts("local_guard: brightness, ROI exclusion, weak texture, invalid frame, "
         "bandas 25/60 OK");
    return 0;
}
