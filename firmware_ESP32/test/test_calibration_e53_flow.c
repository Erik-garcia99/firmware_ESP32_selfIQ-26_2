/* E5.3: secuencia completa en host — filtro de frame -> descriptor 8x8 v1 ->
 * JPEG verificado -> metadatos correlacionados. No usa cámara ni ESP-IDF; la
 * ejecución real de la tarea de cámara se prueba en placa (E5.7). */
#include <stdio.h>
#include <string.h>

#include "../main/modules/calibration/calibration_capture.h"
#include "../main/modules/calibration/calibration_frame.h"

static int g_fail = 0, g_run = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    ++g_run;                                                           \
    if (!(c)) {                                                        \
      ++g_fail;                                                        \
      printf("FALLO %s:%d: %s\n", __FILE__, __LINE__, #c);             \
    }                                                                  \
  } while (0)

#define W 320U
#define H 240U
static uint8_t g_frame[W * H];

static size_t make_jpeg(uint8_t *out, size_t cap, uint16_t width,
                        uint16_t height) {
  static const uint8_t header[] = {
      0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00,
      0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
      0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x00, 0x00, 0x00,
      0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01,
      0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F, 0x00,
      0x10, 0x20, 0x30, 0x40, 0xFF, 0xD9};
  if (cap < sizeof(header)) return 0;
  memcpy(out, header, sizeof(header));
  out[25] = (uint8_t)(height >> 8);
  out[26] = (uint8_t)height;
  out[27] = (uint8_t)(width >> 8);
  out[28] = (uint8_t)width;
  return sizeof(header);
}

static void fill(uint8_t base, int object) {
  for (uint32_t y = 0; y < H; ++y)
    for (uint32_t x = 0; x < W; ++x) {
      uint8_t value = (uint8_t)(base + (x * 20U) / (W - 1U));
      if (object && x >= 80U && x < 190U && y >= 140U && y < 225U)
        value = 220U;
      g_frame[y * W + x] = value;
    }
}

int main(void) {
  const cal_roi_t roi = {79044U, 518627U, 599265U, 477941U};
  const int64_t request_us = 10000000LL;
  cal_frame_stats_t stats;
  memset(&stats, 0, sizeof(stats));

  fill(50, 0);

  /* 1) frame iniciado antes de la orden: se descarta y NO se usa. */
  cal_frame_info_t stale = {.is_grayscale = true, .width = W, .height = H,
                            .len = sizeof(g_frame), .captured_us = request_us - 1};
  CHECK(cal_frame_check(&stale, W, H, request_us, request_us + 400000,
                        CAL_FRAME_MAX_AGE_MS, &stats) == CAL_FRAME_E_STALE);
  CHECK(stats.rej_stale == 1);

  /* 2) el frame corto del log real (72960 != 76800) nunca llega al descriptor. */
  cal_frame_info_t short_frame = stale;
  short_frame.captured_us = request_us + 100000;
  short_frame.len = 72960;
  CHECK(cal_frame_check(&short_frame, W, H, request_us, request_us + 400000,
                        CAL_FRAME_MAX_AGE_MS, &stats) == CAL_FRAME_E_LENGTH);
  CHECK(stats.rej_length == 1);

  /* 3) frame posterior y reciente: se acepta. */
  cal_frame_info_t fresh = short_frame;
  fresh.len = sizeof(g_frame);
  CHECK(cal_frame_check(&fresh, W, H, request_us, request_us + 400000,
                        CAL_FRAME_MAX_AGE_MS, &stats) == CAL_FRAME_OK);
  CHECK(stats.accepted == 1);

  /* 4) descriptor 8x8 v1 del frame aceptado. */
  cal_capture_t capture;
  CHECK(cal_capture_build(g_frame, fresh.len, W, H, W, roi, &capture) == CAL_OK);
  CHECK(capture.descriptor_len == 66U);
  CHECK(capture.descriptor_bytes[0] == 1U);
  CHECK(capture.descriptor_bytes[1] == 8U);
  const uint32_t crc = capture.descriptor_crc;

  /* 5) JPEG del mismo frame: dimensiones y hash. */
  uint8_t jpeg[128];
  const size_t jpeg_len = make_jpeg(jpeg, sizeof(jpeg), W, H);
  CHECK(jpeg_len > 0);
  CHECK(cal_capture_attach_jpeg(&capture, jpeg, jpeg_len, W, H) == CAL_OK);
  CHECK(capture.image_bytes == jpeg_len);
  char sha[CAL_SHA256_HEX_SIZE];
  cal_sha256_hex(jpeg, jpeg_len, sha);
  CHECK(!strcmp(sha, capture.image_sha256));

  /* 6) el mismo frame es reproducible; una escena distinta cambia el CRC. */
  cal_capture_t repeat;
  CHECK(cal_capture_build(g_frame, fresh.len, W, H, W, roi, &repeat) == CAL_OK);
  CHECK(repeat.descriptor_crc == crc);
  CHECK(!strcmp(repeat.image_sha256, "")); /* sin JPEG todavía, no se inventa */
  fill(50, 1);
  cal_capture_t occupied;
  CHECK(cal_capture_build(g_frame, fresh.len, W, H, W, roi, &occupied) == CAL_OK);
  CHECK(occupied.descriptor_crc != crc);

  /* 7) recorte en el borde derecho/inferior: válido y acotado. */
  const cal_roi_t edge = {700000U, 500000U, 300000U, 500000U};
  cal_rect_t rect;
  CHECK(cal_roi_to_rect(W, H, edge, &rect) == CAL_OK);
  CHECK(rect.right == W && rect.bottom == H);
  cal_capture_t clipped;
  CHECK(cal_capture_build(g_frame, fresh.len, W, H, W, edge, &clipped) == CAL_OK);
  CHECK(clipped.descriptor_len == 66U);

  printf("calibration_e53_flow: %d comprobaciones, %d fallos (crc=%08lx)\n",
         g_run, g_fail, (unsigned long)crc);
  return g_fail ? 1 : 0;
}
