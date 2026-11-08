/* Prueba del nucleo de ROI en PC, sin ESP-IDF.
 *
 * Compilar y ejecutar desde firmware_ESP32:
 *   gcc -std=c11 -Wall -Wextra -I main/modules/roi \
 *       main/modules/roi/roi_core.c test/roi_core_test.c \
 *       -o roi_core_test -lm && ./roi_core_test
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "roi_core.h"

static int checks = 0;

#define CHECK(condition)                                                       \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(condition)) {                                                        \
      fprintf(stderr, "FALLO linea %d: %s\n", __LINE__, #condition);           \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)

static roi_def_t make_roi(int index, float x, float y, float w, float h) {
  roi_def_t roi;
  memset(&roi, 0, sizeof(roi));
  snprintf(roi.id, sizeof(roi.id), "00000000-0000-4000-8000-%012d", index);
  roi.x = x;
  roi.y = y;
  roi.w = w;
  roi.h = h;
  return roi;
}

static void valid_grid(roi_def_t *rois) {
  for (int i = 0; i < 8; ++i) {
    float x = (float)(i % 4) * 0.25f;
    float y = (float)(i / 4) * 0.5f;
    rois[i] = make_roi(i, x, y, 0.2f, 0.2f);
  }
}

int main(void) {
  roi_def_t valid[ROI_MAX];
  valid_grid(valid);
  CHECK(roi_validate_set(valid, 8) == ROI_OK);

  roi_def_t too_many[ROI_MAX + 1];
  for (int i = 0; i < 9; ++i)
    too_many[i] = make_roi(i, 0.0f, 0.0f, 0.1f, 0.1f);
  CHECK(roi_validate_set(too_many, 9) == ROI_ERR_TOO_MANY);

  CHECK(roi_validate_set(valid, 0) == ROI_OK);

  roi_def_t outside = make_roi(20, 0.9f, 0.1f, 0.3f, 0.2f);
  CHECK(roi_validate_set(&outside, 1) == ROI_ERR_INVALID_BBOX);

  roi_def_t narrow = make_roi(21, 0.1f, 0.1f, 0.02f, 0.2f);
  CHECK(roi_validate_set(&narrow, 1) == ROI_ERR_INVALID_BBOX);

  roi_def_t negative = make_roi(22, -0.1f, 0.1f, 0.2f, 0.2f);
  CHECK(roi_validate_set(&negative, 1) == ROI_ERR_INVALID_BBOX);

  roi_def_t overlap[2] = {make_roi(30, 0.0f, 0.0f, 0.5f, 0.5f),
                          make_roi(31, 0.1f, 0.1f, 0.5f, 0.5f)};
  CHECK(roi_validate_set(overlap, 2) == ROI_ERR_OVERLAP);

  roi_def_t border[2] = {make_roi(32, 0.0f, 0.0f, 0.5f, 0.5f),
                         make_roi(33, 0.5f, 0.0f, 0.5f, 0.5f)};
  CHECK(roi_validate_set(border, 2) == ROI_OK);

  roi_def_t tiny[2] = {make_roi(34, 0.0f, 0.0f, 0.5f, 0.5f),
                       make_roi(35, 0.495f, 0.0f, 0.5f, 0.5f)};
  CHECK(roi_validate_set(tiny, 2) == ROI_OK);

  roi_def_t duplicate[2] = {make_roi(36, 0.0f, 0.0f, 0.2f, 0.2f),
                            make_roi(36, 0.4f, 0.0f, 0.2f, 0.2f)};
  CHECK(roi_validate_set(duplicate, 2) == ROI_ERR_INVALID_JSON);

  CHECK(roi_uuid_valid("5c485efa-cc20-4f9a-91eb-ceeba0f99289"));
  CHECK(roi_uuid_valid("5C485EFA-CC20-4F9A-91EB-CEEBA0F99289"));
  CHECK(!roi_uuid_valid("5c485efacc20-4f9a-91eb-ceeba0f99289"));
  CHECK(!roi_uuid_valid("no-es-uuid"));

  CHECK(roi_crc32("123456789", 9) == 0xCBF43926U);

  roi_blob_t blob;
  memset(&blob, 0, sizeof(blob));
  blob.count = 2;
  blob.config_version = 7;
  memcpy(blob.rois, valid, 2 * sizeof(roi_def_t));
  roi_blob_seal(&blob);
  CHECK(blob.magic == ROI_BLOB_MAGIC);
  CHECK(blob.format_version == ROI_BLOB_FORMAT_VERSION);
  CHECK(blob.crc32 != 0);
  CHECK(roi_blob_verify(&blob));

  roi_blob_t corrupt = blob;
  corrupt.rois[0].x += 0.01f;
  CHECK(!roi_blob_verify(&corrupt));

  corrupt = blob;
  corrupt.magic = 0;
  CHECK(!roi_blob_verify(&corrupt));

  corrupt = blob;
  corrupt.format_version = 99;
  CHECK(!roi_blob_verify(&corrupt));

  corrupt = blob;
  corrupt.crc32 ^= 1U;
  CHECK(!roi_blob_verify(&corrupt));

  printf("roi_core_test: OK (%d verificaciones)\n", checks);
  return 0;
}
