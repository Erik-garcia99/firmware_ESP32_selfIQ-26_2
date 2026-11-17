/* E5.3: pruebas host del paquete de captura (descriptor 8x8 v1 + JPEG).
 * No usa ESP-IDF ni cámara real: valida la lógica pura que camera_lib invoca
 * sobre un frame ya validado y sobre el JPEG generado del mismo frame. */
#include <stdio.h>
#include <string.h>

#include "../main/modules/calibration/calibration_capture.h"

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
#define ROI_X 79044U
#define ROI_Y 518627U
#define ROI_W 599265U
#define ROI_H 477941U

static uint8_t median3(uint8_t a, uint8_t b, uint8_t c) {
  if (a < b) return b < c ? b : (a < c ? c : a);
  return a < c ? a : (b < c ? c : b);
}

static uint8_t g_frame[W * H];

/* JPEG sintético mínimo: SOI + APP0 + SOF0(320x240) + SOS + datos + EOI.
 * El contenido no se decodifica; solo se ejerce el parser de cabeceras. */
static size_t make_jpeg(uint8_t *out, size_t cap, uint16_t width,
                        uint16_t height) {
  static const uint8_t header[] = {
      0xFF, 0xD8,                                            /* SOI */
      0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00,       /* APP0 */
      0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
      0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x00, 0x00, 0x00,  /* SOF0 */
      0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11,
      0x01,
      0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F,  /* SOS */
      0x00,
      0x11, 0x22, 0x33, 0x44,                                /* datos */
      0xFF, 0xD9};                                           /* EOI */
  const size_t total = sizeof(header);
  if (cap < total) return 0;
  memcpy(out, header, total);
  /* height/width en SOF0: offset 25/27 de este layout. */
  out[25] = (uint8_t)(height >> 8);
  out[26] = (uint8_t)height;
  out[27] = (uint8_t)(width >> 8);
  out[28] = (uint8_t)width;
  return total;
}

static void fill_frame(int with_object) {
  for (uint32_t y = 0; y < H; ++y) {
    for (uint32_t x = 0; x < W; ++x) {
      uint8_t value = (uint8_t)(40U + (x * 40U) / (W - 1U));
      if (with_object && x >= 70U && x < 180U && y >= 150U && y < 220U)
        value = 210U;
      g_frame[y * W + x] = value;
    }
  }
}

static void check_sha_vectors(void) {
  struct {
    size_t len;
    const char *hex;
  } cases[] = {
      {0, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
      {1, "ca978112ca1bbdcafac231b39a23dc4da786eff8147c4e72b9807785afee48bb"},
      {55, "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
      {56, "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
      {63, "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34"},
      {64, "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
      {65, "635361c48bb9eab14198e76ea8ab7f1a41685d6ad62aa9146d301d4f17eb0ae0"},
  };
  static const char abc[] = "abc";
  char hex[CAL_SHA256_HEX_SIZE];

  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    uint8_t data[65];
    memset(data, 'a', sizeof(data));
    cal_sha256_hex(data, cases[i].len, hex);
    CHECK(strlen(hex) == 64U);
    CHECK(!strcmp(hex, cases[i].hex));
  }

  cal_sha256_hex((const uint8_t *)abc, 3, hex);
  CHECK(!strcmp(hex,
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));

  uint8_t digest[CAL_SHA256_DIGEST_BYTES];
  cal_sha256((const uint8_t *)abc, 3, digest);
  CHECK(digest[0] == 0xba && digest[31] == 0xad);

  /* NULL con longitud 0 es el vector vacío; NULL con datos no escribe. */
  cal_sha256_hex(NULL, 0, hex);
  CHECK(!strcmp(hex, cases[0].hex));
  memset(digest, 0xAA, sizeof(digest));
  cal_sha256(NULL, 4, digest);
  for (size_t i = 0; i < sizeof(digest); ++i) CHECK(digest[i] == 0);
}

static void check_jpeg_parser(void) {
  uint8_t jpeg[256];
  const size_t len = make_jpeg(jpeg, sizeof(jpeg), W, H);
  CHECK(len > 0);
  uint16_t w = 0, h = 0;
  CHECK(cal_jpeg_dimensions(jpeg, len, &w, &h) == CAL_OK);
  CHECK(w == W && h == H);

  jpeg[len - 1] = 0x00; /* sin EOI */
  CHECK(cal_jpeg_dimensions(jpeg, len, &w, &h) == CAL_BAD_FRAME);
  jpeg[len - 1] = 0xD9;

  jpeg[0] = 0x00; /* sin SOI */
  CHECK(cal_jpeg_dimensions(jpeg, len, &w, &h) == CAL_BAD_FRAME);
  jpeg[0] = 0xFF;

  CHECK(cal_jpeg_dimensions(jpeg, 3, &w, &h) == CAL_BAD_FRAME);
  CHECK(cal_jpeg_dimensions(NULL, len, &w, &h) == CAL_BAD_ARGUMENT);
  CHECK(cal_jpeg_dimensions(jpeg, len, NULL, &h) == CAL_BAD_ARGUMENT);

  /* un solo marcador SOF con longitud truncada */
  const uint8_t truncated[] = {0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x11, 0x08,
                               0x00, 0xF0, 0xFF, 0xD9};
  CHECK(cal_jpeg_dimensions(truncated, sizeof(truncated), &w, &h) ==
        CAL_BAD_FRAME);

  /* SOF con alto cero */
  make_jpeg(jpeg, sizeof(jpeg), W, 0);
  CHECK(cal_jpeg_dimensions(jpeg, len, &w, &h) == CAL_BAD_FRAME);
}

static void check_descriptor_and_jpeg(void) {
  const cal_roi_t roi = {ROI_X, ROI_Y, ROI_W, ROI_H};
  cal_capture_t capture;
  fill_frame(0);
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, roi, &capture) ==
        CAL_OK);
  CHECK(capture.descriptor_len == CAL_CAPTURE_DESCRIPTOR_BYTES);
  CHECK(CAL_CAPTURE_DESCRIPTOR_BYTES == 66U);
  CHECK(capture.descriptor_bytes[0] == CAL_DESCRIPTOR_FORMAT_VERSION);
  CHECK(capture.descriptor_bytes[1] == CAL_DESCRIPTOR_GRID_WIDTH);
  CHECK(capture.descriptor.grid_side == 8U);
  CHECK(capture.width == W && capture.height == H);

  const uint32_t crc = capture.descriptor_crc;
  CHECK(crc != 0U);
  CHECK(cal_descriptor_crc32(&capture.descriptor) == crc);

  /* Recalcular sobre el mismo frame debe ser determinista. */
  cal_capture_t again;
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, roi, &again) ==
        CAL_OK);
  CHECK(again.descriptor_crc == crc);
  CHECK(!memcmp(again.descriptor_bytes, capture.descriptor_bytes,
                CAL_CAPTURE_DESCRIPTOR_BYTES));

  /* El CRC no depende de métricas auxiliares (padding/struct). */
  CHECK(cal_descriptor_crc32(&again.descriptor) == crc);

  /* Un frame con objeto en la ROI debe producir otro descriptor. */
  fill_frame(1);
  cal_capture_t occupied;
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, roi, &occupied) ==
        CAL_OK);
  CHECK(occupied.descriptor_crc != crc);
  float mae = 0.0f;
  CHECK(cal_descriptor_mae(&capture.descriptor, &occupied.descriptor, &mae) ==
        CAL_OK);
  CHECK(mae > 20.0f); /* separación sintética; no es umbral de producción */

  /* JPEG del mismo frame: dimensiones y SHA. */
  uint8_t jpeg[256];
  const size_t jpeg_len = make_jpeg(jpeg, sizeof(jpeg), W, H);
  CHECK(jpeg_len > 0);
  CHECK(cal_capture_attach_jpeg(&capture, jpeg, jpeg_len, W, H) == CAL_OK);
  CHECK(capture.image_bytes == jpeg_len);
  char expected[CAL_SHA256_HEX_SIZE];
  cal_sha256_hex(jpeg, jpeg_len, expected);
  CHECK(!strcmp(capture.image_sha256, expected));

  /* Mismo frame descriptivamente, pero JPEG de otra resolución: rechazo. */
  cal_capture_t wrong;
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, roi, &wrong) ==
        CAL_OK);
  uint8_t other_jpeg[256];
  const size_t other_len = make_jpeg(other_jpeg, sizeof(other_jpeg), 640, 480);
  CHECK(cal_capture_attach_jpeg(&wrong, other_jpeg, other_len, W, H) ==
        CAL_BAD_FRAME);
  CHECK(cal_capture_attach_jpeg(&wrong, other_jpeg, other_len, 640, 480) ==
        CAL_BAD_FRAME); /* tampoco coincide con el frame 320x240 */

  /* JPEG truncado: no se calcula SHA ni se acepta. */
  CHECK(cal_capture_attach_jpeg(&wrong, jpeg, jpeg_len - 1, W, H) ==
        CAL_BAD_FRAME);
  wrong.image_sha256[0] = '\0';
  CHECK(cal_capture_attach_jpeg(&wrong, jpeg, 0, W, H) == CAL_BAD_ARGUMENT);
  CHECK(wrong.image_sha256[0] == '\0');
}

static void check_rejections(void) {
  const cal_roi_t roi = {ROI_X, ROI_Y, ROI_W, ROI_H};
  cal_capture_t capture;
  fill_frame(0);
  CHECK(cal_capture_build(g_frame, W * H - 1U, W, H, W, roi, &capture) ==
        CAL_BAD_FRAME); /* longitud 76799, como el caso 72960 del log */
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W - 1U, roi,
                          &capture) == CAL_BAD_FRAME);
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, roi, NULL) ==
        CAL_BAD_ARGUMENT);

  const cal_roi_t outside = {900000U, 100000U, 200000U, 100000U};
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, outside,
                          &capture) == CAL_BAD_ROI);
  const cal_roi_t empty = {ROI_X, ROI_Y, 0U, ROI_H};
  CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, empty, &capture) ==
        CAL_BAD_ROI);
}

static void check_base64(void) {
  char out[256];
  size_t written = 0;
  CHECK(cal_base64_encode((const uint8_t *)"", 0, out, sizeof(out), &written) ==
        CAL_OK);
  CHECK(written == 0U && out[0] == '\0');
  CHECK(cal_base64_encode((const uint8_t *)"f", 1, out, sizeof(out), &written) ==
        CAL_OK);
  CHECK(!strcmp(out, "Zg=="));
  CHECK(cal_base64_encode((const uint8_t *)"fo", 2, out, sizeof(out),
                          &written) == CAL_OK);
  CHECK(!strcmp(out, "Zm8="));
  CHECK(cal_base64_encode((const uint8_t *)"foo", 3, out, sizeof(out),
                          &written) == CAL_OK);
  CHECK(!strcmp(out, "Zm9v"));

  uint8_t descriptor[CAL_CAPTURE_DESCRIPTOR_BYTES];
  memset(descriptor, 0xAB, sizeof(descriptor));
  CHECK(cal_base64_size(sizeof(descriptor)) == 89U);
  CHECK(cal_base64_encode(descriptor, sizeof(descriptor), out, sizeof(out),
                          &written) == CAL_OK);
  CHECK(written == 88U && strlen(out) == 88U);
  CHECK(cal_base64_encode(descriptor, sizeof(descriptor), out, 88U,
                          &written) == CAL_BUFFER_SMALL);
}

static void check_median_reference(void) {
  const cal_roi_t roi = {ROI_X, ROI_Y, ROI_W, ROI_H};
  cal_descriptor_t samples[3];
  cal_capture_t capture;

  /* Tres tomas del mismo frame base con brillo +0/+1/+2. */
  for (unsigned s = 0; s < 3; ++s) {
    fill_frame(0);
    for (size_t i = 0; i < sizeof(g_frame); ++i)
      g_frame[i] = (uint8_t)(g_frame[i] + s);
    CHECK(cal_capture_build(g_frame, sizeof(g_frame), W, H, W, roi, &capture) ==
          CAL_OK);
    samples[s] = capture.descriptor;
  }

  cal_capture_t median;
  CHECK(cal_capture_build_median(samples, 3, W, H, &median) == CAL_OK);
  CHECK(median.descriptor_len == CAL_CAPTURE_DESCRIPTOR_BYTES);
  CHECK(median.descriptor.grid_side == 8U && median.descriptor.cell_count == 64U);
  CHECK(median.width == W && median.height == H);
  CHECK(median.descriptor_crc == cal_descriptor_crc32(&median.descriptor));
  CHECK(!memcmp(median.descriptor_bytes + CAL_DESCRIPTOR_HEADER_BYTES,
                median.descriptor.cells, CAL_DESCRIPTOR_CELLS));
  CHECK(median.image_bytes == 0U && median.image_sha256[0] == '\0');

  /* Mediana por celda calculada aparte, sin usar la función bajo prueba. */
  for (int i = 0; i < 64; ++i) {
    const uint8_t expected =
        median3(samples[0].cells[i], samples[1].cells[i], samples[2].cells[i]);
    CHECK(median.descriptor.cells[i] == expected);
  }

  /* Orden de las muestras y repetición no cambian el resultado. */
  cal_descriptor_t reversed[3] = {samples[2], samples[1], samples[0]};
  cal_capture_t again;
  CHECK(cal_capture_build_median(reversed, 3, W, H, &again) == CAL_OK);
  CHECK(!memcmp(&again.descriptor, &median.descriptor, sizeof(median.descriptor)));
  CHECK(again.descriptor_crc == median.descriptor_crc);

  /* Una sola muestra: mismas celdas que esa toma. */
  cal_capture_t single;
  CHECK(cal_capture_build_median(samples, 1, W, H, &single) == CAL_OK);
  CHECK(!memcmp(single.descriptor.cells, samples[0].cells, CAL_DESCRIPTOR_CELLS));
  CHECK(single.descriptor_crc == cal_descriptor_crc32(&samples[0]));

  /* Rechazos: cantidad, argumentos, dimensiones y grid mezclado. */
  CHECK(cal_capture_build_median(samples, 0, W, H, &median) == CAL_BAD_ARGUMENT);
  CHECK(cal_capture_build_median(samples, CAL_DESCRIPTOR_MEDIAN_MAX + 1U, W, H,
                                 &median) == CAL_BAD_ARGUMENT);
  CHECK(cal_capture_build_median(NULL, 3, W, H, &median) == CAL_BAD_ARGUMENT);
  CHECK(cal_capture_build_median(samples, 3, W, H, NULL) == CAL_BAD_ARGUMENT);
  CHECK(cal_capture_build_median(samples, 3, 0U, H, &median) == CAL_BAD_FRAME);
  CHECK(cal_capture_build_median(samples, 3, W, 65536U, &median) == CAL_BAD_FRAME);
  cal_descriptor_t mixed[2] = {samples[0], samples[0]};
  mixed[1].grid_side = 16;
  mixed[1].cell_count = 256;
  CHECK(cal_capture_build_median(mixed, 2, W, H, &median) == CAL_BAD_GRID);

  /* El JPEG de revisión se adjunta igual que en la captura simple. */
  uint8_t jpeg[256];
  const size_t len = make_jpeg(jpeg, sizeof(jpeg), W, H);
  CHECK(len > 0);
  CHECK(cal_capture_attach_jpeg(&median, jpeg, len, W, H) == CAL_OK);
  CHECK(median.image_bytes == len);
}

int main(void) {
  check_sha_vectors();
  check_jpeg_parser();
  check_descriptor_and_jpeg();
  check_rejections();
  check_base64();
  check_median_reference();
  printf("calibration_capture: %d comprobaciones, %d fallos\n", g_run, g_fail);
  return g_fail ? 1 : 0;
}
