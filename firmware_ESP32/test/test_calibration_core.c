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

  /* Mediana por celda: varias tomas de una referencia. */
  cal_descriptor_t samples[4];
  for (int s = 0; s < 4; ++s) {
    memset(&samples[s], 0, sizeof samples[s]);
    samples[s].grid_side = 8;
    samples[s].cell_count = 64;
    samples[s].clipped_pct = (uint8_t)(s * 3);
  }
  for (int i = 0; i < 64; ++i) {
    samples[0].cells[i] = 10;
    samples[1].cells[i] = 20;
    samples[2].cells[i] = 30;
    samples[3].cells[i] = 41;
  }
  cal_descriptor_t median;
  assert(cal_descriptor_median(samples, 3, &median) == CAL_OK);
  assert(median.grid_side == 8 && median.cell_count == 64);
  assert(median.cells[0] == 20 && median.cells[63] == 20);
  assert(median.mean == 20 && median.contrast == 0 && median.clipped_pct == 3);

  /* Par: media de centrales 20 y 31, .5 hacia arriba -> 26. */
  samples[2].cells[0] = 31;
  assert(cal_descriptor_median(samples, 4, &median) == CAL_OK);
  assert(median.cells[0] == 26);
  samples[2].cells[0] = 30;

  /* El orden de las muestras no altera el resultado. */
  assert(cal_descriptor_median(samples, 3, &median) == CAL_OK);
  cal_descriptor_t permuted[3] = {samples[2], samples[0], samples[1]};
  cal_descriptor_t median2;
  assert(cal_descriptor_median(permuted, 3, &median2) == CAL_OK);
  assert(!memcmp(median.cells, median2.cells, 64));
  assert(median.mean == median2.mean && median.contrast == median2.contrast);

  /* Una muestra recalcula las métricas desde sus celdas, conserva saturación. */
  samples[0].clipped_pct = 7;
  assert(cal_descriptor_median(samples, 1, &median) == CAL_OK);
  assert(!memcmp(median.cells, samples[0].cells, 64));
  assert(median.mean == 10 && median.contrast == 0 && median.clipped_pct == 7);

  /* Solapamiento con la salida permitido. */
  assert(cal_descriptor_median(samples, 3, &samples[0]) == CAL_OK);
  assert(samples[0].cells[0] == 20 && samples[0].mean == 20);

  /* Grid 16 también, y rechazos de longitud/grid/argumentos. */
  cal_descriptor_t big[2];
  for (int s = 0; s < 2; ++s) {
    memset(&big[s], 0, sizeof big[s]);
    big[s].grid_side = 16;
    big[s].cell_count = 256;
  }
  for (int i = 0; i < 256; ++i) {
    big[0].cells[i] = 40;
    big[1].cells[i] = 60;
  }
  assert(cal_descriptor_median(big, 2, &median) == CAL_OK);
  assert(median.grid_side == 16 && median.cell_count == 256);
  assert(median.cells[255] == 50 && median.mean == 50 && median.contrast == 0);
  assert(cal_descriptor_median(samples, 0, &median) == CAL_BAD_ARGUMENT);
  assert(cal_descriptor_median(samples, CAL_DESCRIPTOR_MEDIAN_MAX + 1, &median) ==
         CAL_BAD_ARGUMENT);
  assert(cal_descriptor_median(NULL, 3, &median) == CAL_BAD_ARGUMENT);
  assert(cal_descriptor_median(samples, 3, NULL) == CAL_BAD_ARGUMENT);
  cal_descriptor_t mismatched[2] = {big[0], samples[0]};
  assert(cal_descriptor_median(mismatched, 2, &median) == CAL_BAD_GRID);
  mismatched[1].grid_side = 8;
  mismatched[1].cell_count = 63;
  assert(cal_descriptor_median(mismatched, 2, &median) == CAL_BAD_GRID);
  puts("E2 calibration_core: pruebas host OK");
  return 0;
}
