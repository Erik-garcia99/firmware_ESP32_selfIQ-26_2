#include "../main/modules/vision/vision_core.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t image[640 * 480];
static uint8_t eb[VISION_DESCRIPTOR_BYTES], fb[VISION_DESCRIPTOR_BYTES];
static vision_input_t input;
static vision_reference_t empty, full;
static unsigned checks;

#define CHECK(x) do { ++checks; assert(x); } while (0)

/* Textura compartida +/-32; señal de ocupación independiente +/-16.
 * Las cuatro combinaciones están balanceadas: media=128, MAD=32. */
static uint8_t cell(unsigned x, unsigned y, unsigned pct, int shift, int gain) {
  int texture = x % 2U ? 32 : -32;
  int occupancy = y % 2U ? 16 : -16;
  int v = texture + occupancy * ((int)pct - 50) / 50;
  return (uint8_t)(128 + shift + v * gain / 100);
}

static void fill(unsigned pct, int shift, int gain) {
  for (unsigned y = 0; y < input.height; ++y)
    for (unsigned x = 0; x < input.width; ++x)
      image[y * input.stride + x] = cell(x * 8U / input.width,
                                        y * 8U / input.height, pct, shift, gain);
}

static void reference(vision_reference_t *ref, uint8_t *bytes) {
  cal_descriptor_t d;
  size_t written;
  CHECK(cal_descriptor_extract(image, input.length, input.width, input.height,
                              input.stride, input.roi, 8, &d) == CAL_OK);
  CHECK(cal_descriptor_encode(&d, bytes, VISION_DESCRIPTOR_BYTES, &written) == CAL_OK);
  CHECK(written == VISION_DESCRIPTOR_BYTES);
  ref->context = input.context;
  ref->bytes = bytes;
  ref->length = written;
  ref->crc32 = cal_descriptor_crc32(&d);
}

static void setup(void) {
  memset(&input, 0, sizeof input);
  input.gray = image;
  input.width = 640;
  input.height = 480;
  input.stride = 640;
  input.length = sizeof image;
  input.roi = (cal_roi_t){0, 0, CAL_COORD_SCALE, CAL_COORD_SCALE};
  memcpy(input.context.roi_id, "f71aaeca-d168-43e4-9eee-1001aa524f2f", 37);
  memset(input.context.context_hash, 'a', 64);
  input.context.config_version = 9;
  input.context.calibration_version = 2;
  input.measured_at_monotonic_ms = 123456;
  fill(0, 0, 100);
  reference(&empty, eb);
  fill(100, 0, 100);
  reference(&full, fb);
}

static void expect(vision_status_t status) {
  vision_result_t result;
  memset(&result, 0xff, sizeof result);
  vision_status_t actual = vision_measure(&input, &empty, &full, &result);
  if (actual != status)
    fprintf(stderr, "check %u: esperado %s, recibido %s (mean=%u, contrast=%u, clipped=%u, separation=%u)\n",
            checks + 1, vision_status_code(status), vision_status_code(actual),
            result.mean, result.contrast, result.clipped_pct, result.separation_q8);
  CHECK(actual == status);
  CHECK(result.status == status);
  CHECK(status == VISION_OK || result.occupancy_pct == VISION_INVALID_PERCENT);
  CHECK(status == VISION_OK || result.confidence == 0);
}

static void levels_and_determinism(void) {
  setup();
  vision_result_t a, b;
  uint8_t saved_empty[66], saved_full[66];
  memcpy(saved_empty, eb, 66);
  memcpy(saved_full, fb, 66);
  for (unsigned pct = 0; pct <= 100; pct += 25) {
    fill(pct, 0, 100);
    CHECK(vision_measure(&input, &empty, &full, &a) == VISION_OK);
    CHECK(a.occupancy_pct == pct);
    CHECK(a.confidence == 100); /* un 50% no significa incertidumbre binaria */
    CHECK(a.measured_at_monotonic_ms == 123456);
    CHECK(a.context.config_version == 9 && a.context.calibration_version == 2);
    CHECK(memcmp(a.context.roi_id, input.context.roi_id, 37) == 0);
    for (int repeat = 0; repeat < 20; ++repeat) {
      memset(&b, repeat, sizeof b);
      CHECK(vision_measure(&input, &empty, &full, &b) == VISION_OK);
      CHECK(memcmp(&a, &b, sizeof a) == 0);
    }
    for (int gain = 90; gain <= 110; gain += 10) {
      for (int shift = -8; shift <= 8; shift += 8) {
        fill(pct, shift, gain);
        CHECK(vision_measure(&input, &empty, &full, &b) == VISION_OK);
        CHECK((int)b.occupancy_pct >= (int)pct - 2 &&
              (int)b.occupancy_pct <= (int)pct + 2);
      }
    }
  }
  CHECK(memcmp(saved_empty, eb, 66) == 0 && memcmp(saved_full, fb, 66) == 0);
  /* Redondeo positivo: 1/32 => 3%, no truncamiento a 0. */
  fill(0, 0, 100);
  for (unsigned y = 0; y < input.height; ++y)
    for (unsigned x = 0; x < input.width; ++x)
      image[y * input.stride + x] += (y * 8U / input.height) % 2U ? 1 : -1;
  CHECK(vision_measure(&input, &empty, &full, &b) == VISION_OK);
  CHECK(b.occupancy_pct == 3);
}

static void invalid_inputs(void) {
  setup();
  vision_result_t r;
  CHECK(vision_measure(NULL, &empty, &full, &r) == VISION_BAD_ARGUMENT);
  CHECK(r.occupancy_pct == VISION_INVALID_PERCENT);
  CHECK(vision_measure(&input, &empty, &full, NULL) == VISION_BAD_ARGUMENT);
  CHECK(vision_measure(&input, NULL, &full, &r) == VISION_UNCALIBRATED);
  CHECK(r.occupancy_pct == VISION_INVALID_PERCENT);
  CHECK(vision_measure(&input, &empty, NULL, &r) == VISION_UNCALIBRATED);
  input.context.roi_id[36] = 'x'; expect(VISION_BAD_ARGUMENT);
  setup(); input.context.context_hash[0] = 'Z'; expect(VISION_BAD_ARGUMENT);
  setup(); input.context.config_version = 0; expect(VISION_BAD_ARGUMENT);
  setup(); empty.context.config_version++; expect(VISION_CONTEXT_MISMATCH);
  setup(); full.context.calibration_version++; expect(VISION_CONTEXT_MISMATCH);
  setup(); full.context.context_hash[0] = 'b'; expect(VISION_CONTEXT_MISMATCH);
  setup(); empty.context.roi_id[0] = '0'; expect(VISION_CONTEXT_MISMATCH);
  setup(); input.gray = NULL; expect(VISION_INVALID_FRAME);
  setup(); input.length--; expect(VISION_INVALID_FRAME);
  setup(); input.width = 0; expect(VISION_INVALID_FRAME);
  setup(); input.height = UINT32_MAX; expect(VISION_INVALID_FRAME);
  setup(); input.stride = 639; expect(VISION_INVALID_FRAME);
  setup(); input.roi.w = 0; expect(VISION_INVALID_ROI);
  setup(); input.roi.x = UINT32_MAX; expect(VISION_INVALID_ROI);
  setup(); input.roi = (cal_roi_t){999999, 0, 2, 1000000}; expect(VISION_INVALID_ROI);
  setup(); input.roi = (cal_roi_t){0, 0, 1000, 1000}; expect(VISION_INVALID_ROI);
  setup(); empty.bytes = NULL; expect(VISION_INVALID_REFERENCE);
  setup(); full.length--; expect(VISION_INVALID_REFERENCE);
  setup(); eb[0] = 2; expect(VISION_INVALID_REFERENCE);
  setup(); fb[1] = 16; expect(VISION_INVALID_REFERENCE);
  setup(); eb[2] ^= 1; expect(VISION_INVALID_REFERENCE);
  setup(); full.crc32 ^= 1; expect(VISION_INVALID_REFERENCE);
}

static void quality(void) {
  setup();
  full = empty; /* igualdad exacta / 0/0 potencial */
  expect(VISION_INDISTINGUISHABLE_REFERENCES);
  setup();
  /* Referencias distintas en brillo pero idénticas tras normalizar. */
  fill(0, 8, 100); reference(&full, fb);
  expect(VISION_INDISTINGUISHABLE_REFERENCES);
  setup();
  fill(0, 0, 100);
  image[0]++;
  reference(&full, fb); /* diferencia subcelda redondeada: no separación */
  expect(VISION_INDISTINGUISHABLE_REFERENCES);
  setup(); memset(image, 128, sizeof image); expect(VISION_LOW_CONTRAST);
  setup(); memset(image, 0, sizeof image); expect(VISION_EXCESSIVE_CLIPPING);
  setup(); memset(image, 255, sizeof image); expect(VISION_EXCESSIVE_CLIPPING);
  setup(); fill(50, 25, 100); expect(VISION_NORMALIZATION_REJECTED);
  setup(); fill(50, 0, 40); expect(VISION_OK); /* ganancia limitada, no rechazo por MAD baja */
  setup(); fill(50, 0, 210); expect(VISION_NORMALIZATION_REJECTED);
  setup(); memset(image, 128, sizeof image); reference(&empty, eb);
  expect(VISION_INVALID_REFERENCE);
  setup();
  /* 25% de píxeles saturados, pero medias aún utilizables: rechazo real. */
  fill(50, 0, 100);
  for (unsigned y = 0; y < input.height; ++y)
    for (unsigned x = 0; x < input.width; x += 4) image[y * input.stride + x] = 255;
  expect(VISION_EXCESSIVE_CLIPPING);
  setup();
  /* Patrón ajeno fuera del segmento empty/full: menor confianza, no precisión. */
  fill(50, 0, 100);
  for (unsigned y = 0; y < input.height; ++y)
    for (unsigned x = 0; x < input.width; ++x)
      image[y * input.stride + x] += (x * 8U / input.width) % 4U < 2U ? 24 : -24;
  vision_result_t r;
  CHECK(vision_measure(&input, &empty, &full, &r) == VISION_OK);
  CHECK(r.confidence < 100 && r.occupancy_pct <= 100);
}

static void geometry_and_stride(void) {
  setup();
  input.width = 8; input.height = 8; input.stride = 11; input.length = 88;
  memset(image, 255, 88); /* padding no debe entrar en métricas */
  fill(0, 0, 100); reference(&empty, eb);
  fill(100, 0, 100); reference(&full, fb);
  fill(50, 0, 100);
  vision_result_t r;
  CHECK(vision_measure(&input, &empty, &full, &r) == VISION_OK);
  CHECK(r.occupancy_pct == 50 && r.clipped_pct == 0);
  setup();
  input.width = 16; input.height = 16; input.stride = 16; input.length = 256;
  input.roi = (cal_roi_t){500000, 500000, 500000, 500000};
  /* Toca right/bottom exactamente; misma conversión usada por calibración. */
  for (unsigned pct = 0; pct <= 100; pct += 50) {
    memset(image, 255, 256);
    for (unsigned y = 8; y < 16; ++y)
      for (unsigned x = 8; x < 16; ++x) image[y * 16 + x] = cell(x-8, y-8, pct, 0, 100);
    if (pct == 0) reference(&empty, eb);
    if (pct == 100) reference(&full, fb);
  }
  CHECK(vision_measure(&input, &empty, &full, &r) == VISION_OK);
  CHECK(r.occupancy_pct == 100 && r.clipped_pct == 0);
  CHECK(strcmp(vision_status_code(VISION_OK), "ok") == 0);
  CHECK(strcmp(vision_status_code((vision_status_t)999), "unknown") == 0);
}

static void boundaries(void) {
  setup(); fill(50, 24, 100); expect(VISION_OK);
  setup(); fill(50, -24, 100); expect(VISION_OK);
  setup(); fill(50, -25, 100); expect(VISION_NORMALIZATION_REJECTED);
  setup(); fill(50, 0, 50); expect(VISION_OK); /* MAD=16, gain=2 */
  setup(); fill(50, 0, 200); expect(VISION_OK); /* MAD=64, gain=0.5 */
  for (int increment = 3; increment <= 4; ++increment) {
    setup(); fill(0, 0, 100);
    for (unsigned y = 0; y < input.height; ++y)
      for (unsigned x = 0; x < input.width; ++x)
        image[y * input.stride + x] += (y * 8U / input.height) % 2U ? increment : -increment;
    reference(&full, fb);
    expect(increment == 3 ? VISION_INDISTINGUISHABLE_REFERENCES : VISION_OK);
  }
  /* Saturación se evalúa con el porcentaje entero redondeado compartido. */
  for (unsigned clipping = 20; clipping <= 21; ++clipping) {
    setup(); fill(50, -8, 100);
    for (unsigned y = 0; y < input.height; ++y)
      for (unsigned x = 0; x < input.width; ++x)
        if ((y * input.width + x) % 100U < clipping) image[y * input.stride + x] = 255;
    expect(clipping == 20 ? VISION_OK : VISION_EXCESSIVE_CLIPPING);
  }
}

static void bounded_low_texture(void) {
  setup();
  input.width = 8; input.height = 8; input.stride = 8; input.length = 64;
  fill(0, 0, 25); reference(&empty, eb);
  fill(100, 0, 25); reference(&full, fb);
  vision_result_t a, b;
  /* MAD=8: ganancia 2, no 4. Señal normalizada empty/full separada 16. */
  for (unsigned pct = 0; pct <= 100; pct += 25) {
    fill(pct, 0, 25);
    CHECK(vision_measure(&input, &empty, &full, &a) == VISION_OK);
    CHECK(a.occupancy_pct == pct);
    CHECK(a.separation_q8 == 16U * 256U);
    fill(pct, 8, 25);
    CHECK(vision_measure(&input, &empty, &full, &b) == VISION_OK);
    CHECK(b.occupancy_pct == pct && a.d_empty_q8 == b.d_empty_q8);
  }
  /* Igualdad y debilidad no quedan autorizadas por acotar ganancia. */
  full = empty; expect(VISION_INDISTINGUISHABLE_REFERENCES);
  fill(100, 0, 25); reference(&full, fb);
  fill(0, 0, 10); reference(&empty, eb); expect(VISION_INVALID_REFERENCE);
  CHECK(vision_algorithm_version() == 2U);
}

static void persisted_pair_regression(void) {
  /* Descriptores reales config15/cal1 del operador. Reconstruir 8x8 desde
   * medias SOLO prueba extremos y normalización, no precisión en fotos VGA. */
  static const uint8_t real_empty[] =
    "\x01\x08\x5b\x6b\x6b\x68\x69\x6a\x6e\x72\x5c\x6f\x6d\x6c\x6d\x69"
    "\x6d\x72\x5c\x7e\x75\x6b\x6b\x6b\x74\x79\x59\x89\x7b\x6e\x6a\x6d"
    "\x7a\x7b\x5f\x86\x78\x6d\x6a\x73\x7c\x7f\x60\x81\x72\x6c\x6b\x6e"
    "\x72\x7c\x5d\x67\x6e\x6d\x6e\x71\x70\x7a\x56\x65\x69\x73\x99\xa6\x86\x77";
  static const uint8_t real_full[] =
    "\x01\x08\x1c\x23\x3e\x65\x6d\x71\x74\x77\x1c\x27\x43\x71\x71\x70"
    "\x73\x78\x1e\x28\x42\x6c\x72\x5c\xa9\x85\x1f\x39\x34\x7d\xb0\x62"
    "\xb7\xa3\x23\x49\x3e\x85\xb6\x60\xaa\x97\x49\x40\x44\x51\x77\x40"
    "\x69\x85\x3f\x38\x4c\x4f\x63\x69\x5a\x74\x39\x54\x59\x56\x69\x61\x4e\x6d";
  _Static_assert(sizeof real_empty == 67 && sizeof real_full == 67,
                 "66 bytes de descriptor mas terminador del literal C");
  setup();
  input.width = 8; input.height = 8; input.stride = 8; input.length = 64;
  memcpy(input.context.roi_id, "cd469f2c-da53-441f-af11-2520ae1d1bc6", 37);
  memcpy(input.context.context_hash,
         "db77020592b6d57e17f1cbe68638f55ac6f8b968771d58bf9b36108e33668d1a", 65);
  input.context.config_version = 15; input.context.calibration_version = 1;
  empty.context = input.context; full.context = input.context;
  empty.bytes = real_empty; full.bytes = real_full;
  empty.crc32 = 0xbb358bebU; full.crc32 = 0x6bc7e1a2U;
  cal_descriptor_t d;
  CHECK(cal_descriptor_decode(real_empty, 66, &d) == CAL_OK);
  CHECK(cal_descriptor_crc32(&d) == empty.crc32);
  uint32_t sum_abs = 0;
  for (unsigned i = 0; i < 64; ++i) {
    int diff = (int)d.cells[i] - d.mean;
    sum_abs += (uint32_t)(diff < 0 ? -diff : diff);
  }
  CHECK(sum_abs == 620U && sum_abs < 64U * 16U);
  CHECK(cal_descriptor_decode(real_full, 66, &d) == CAL_OK);
  CHECK(cal_descriptor_crc32(&d) == full.crc32);
  vision_result_t a, b;
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    const uint8_t *ref = endpoint ? real_full : real_empty;
    memcpy(image, ref + 2, 64);
    CHECK(vision_measure(&input, &empty, &full, &a) == VISION_OK);
    CHECK(a.occupancy_pct == endpoint * 100U);
    CHECK(a.separation_q8 >= 1024U);
    CHECK(vision_measure(&input, &empty, &full, &b) == VISION_OK);
    CHECK(memcmp(&a, &b, sizeof a) == 0);
    for (unsigned i = 0; i < 64; ++i) image[i] = (uint8_t)(ref[i+2] + 4U);
    CHECK(vision_measure(&input, &empty, &full, &b) == VISION_OK);
    CHECK(b.occupancy_pct == endpoint * 100U);
  }
  full.crc32 ^= 1U; expect(VISION_INVALID_REFERENCE); full.crc32 ^= 1U;
  full.context.config_version++; expect(VISION_CONTEXT_MISMATCH);
}

int main(void) {
  levels_and_determinism();
  invalid_inputs();
  quality();
  geometry_and_stride();
  boundaries();
  bounded_low_texture();
  persisted_pair_regression();
  printf("Fase 5 E2 vision_core: %u verificaciones host OK\n", checks);
  return 0;
}
