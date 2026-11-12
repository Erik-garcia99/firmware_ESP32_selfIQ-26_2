#include "../main/modules/calibration/calibration_core.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  uint8_t img[32 * 24];
  memset(img, 100, sizeof img);
  cal_roi_t full = {0, 0, 1000000, 1000000};
  cal_roi_t edge = {500000, 500000, 500000, 500000};
  cal_rect_t rect;
  assert(cal_roi_to_rect(32, 24, full, &rect) == CAL_OK);
  assert(rect.left == 0 && rect.top == 0 && rect.right == 32 && rect.bottom == 24);
  assert(cal_roi_to_rect(32, 24, edge, &rect) == CAL_OK);
  assert(rect.left == 16 && rect.top == 12 && rect.right == 32 && rect.bottom == 24);
  assert(cal_roi_to_rect(32, 24, (cal_roi_t){999999,0,2,50000}, &rect) == CAL_BAD_ROI);
  cal_descriptor_t d, f;
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32, full, 8, &d) == CAL_OK);
  assert(d.cell_count == 64 && d.mean == 100 && d.contrast == 0 && d.clipped_pct == 0);
  for (int i = 0; i < 64; ++i) assert(d.cells[i] == 100);
  uint32_t crc = cal_descriptor_crc32(&d);
  assert(crc != 0);
  assert(cal_descriptor_extract(img, sizeof img - 1, 32, 24, 32, full, 8, &f) == CAL_BAD_FRAME);
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32, full, 12, &f) == CAL_BAD_GRID);
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32,
                                (cal_roi_t){0,0,10000,10000}, 16, &f) == CAL_BAD_ROI);
  memset(img, 110, sizeof img);
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32, full, 8, &f) == CAL_OK);
  float mae = -1;
  assert(cal_descriptor_mae(&d, &f, &mae) == CAL_OK && fabsf(mae - 10.0f) < .001f);
  assert(cal_descriptor_crc32(&f) != crc);
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32, full, 16, &f) == CAL_OK);
  assert(f.cell_count == 256);
  assert(cal_descriptor_mae(&d, &f, &mae) == CAL_BAD_GRID);
  uint8_t encoded[CAL_DESCRIPTOR_MAX_BYTES];
  size_t written = 0;
  assert(cal_descriptor_encode(&d, encoded, sizeof encoded, &written) == CAL_OK);
  assert(written == 66 && encoded[0] == 1 && encoded[1] == 8);
  assert(cal_descriptor_encode(&d, encoded, 65, &written) == CAL_BUFFER_SMALL);
  cal_descriptor_t recovered;
  assert(cal_descriptor_decode(encoded, 66, &recovered) == CAL_OK);
  assert(recovered.mean == 100 && recovered.cell_count == 64);
  assert(cal_descriptor_crc32(&recovered) == crc);
  encoded[0] = 2;
  assert(cal_descriptor_decode(encoded, 66, &recovered) == CAL_BAD_GRID);
  encoded[0] = 1;
  assert(cal_descriptor_decode(encoded, 65, &recovered) == CAL_BAD_FRAME);
  encoded[2] ^= 0xFF;
  assert(cal_descriptor_decode(encoded, 66, &recovered) == CAL_OK);
  assert(cal_descriptor_crc32(&recovered) != crc);
  /* El CRC detecta corrupción si el lector compara el valor almacenado. */
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32, full, 8, &f) == CAL_OK);
  assert(cal_descriptor_encode(&f, encoded, sizeof encoded, &written) == CAL_OK);
  assert(written == 66);
  assert(cal_descriptor_extract(img, sizeof img, 32, 24, 32, full, 16, &f) == CAL_OK);
  assert(cal_descriptor_encode(&f, encoded, sizeof encoded, &written) == CAL_OK);
  assert(written == 258);
  assert(cal_descriptor_decode(encoded, written, &recovered) == CAL_OK);
  assert(recovered.grid_side == 16 && recovered.cell_count == 256);
  puts("E2 calibration_core: pruebas host OK");
  return 0;
}
