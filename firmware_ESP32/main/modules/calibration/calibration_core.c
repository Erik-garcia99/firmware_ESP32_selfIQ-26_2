#include "calibration_core.h"

#include <limits.h>
#include <stdbool.h>
#include <string.h>

static uint32_t ceil_div_u64(uint64_t value, uint32_t den) {
  return (uint32_t)((value + den - 1U) / den);
}

cal_err_t cal_roi_to_rect(uint32_t fw, uint32_t fh, cal_roi_t r,
                          cal_rect_t *out) {
  if (!out) return CAL_BAD_ARGUMENT;
  if (!fw || !fh || fw > UINT16_MAX || fh > UINT16_MAX) return CAL_BAD_FRAME;
  if (!r.w || !r.h || r.x > CAL_COORD_SCALE || r.y > CAL_COORD_SCALE ||
      r.w > CAL_COORD_SCALE - r.x || r.h > CAL_COORD_SCALE - r.y)
    return CAL_BAD_ROI;
  uint32_t x0 = (uint32_t)((uint64_t)r.x * fw / CAL_COORD_SCALE);
  uint32_t y0 = (uint32_t)((uint64_t)r.y * fh / CAL_COORD_SCALE);
  uint32_t x1 = ceil_div_u64((uint64_t)(r.x + r.w) * fw, CAL_COORD_SCALE);
  uint32_t y1 = ceil_div_u64((uint64_t)(r.y + r.h) * fh, CAL_COORD_SCALE);
  if (x0 >= x1 || y0 >= y1 || x1 > fw || y1 > fh) return CAL_BAD_ROI;
  out->left = (uint16_t)x0;
  out->top = (uint16_t)y0;
  out->right = (uint16_t)x1;
  out->bottom = (uint16_t)y1;
  return CAL_OK;
}

cal_err_t cal_descriptor_extract(const uint8_t *gray, size_t length,
                                 uint32_t width, uint32_t height,
                                 uint32_t stride, cal_roi_t roi,
                                 uint8_t grid, cal_descriptor_t *out) {
  if (!gray || !out) return CAL_BAD_ARGUMENT;
  if (grid != 8U && grid != 16U) return CAL_BAD_GRID;
  if (!width || !height || stride < width ||
      (size_t)stride > SIZE_MAX / height || length < (size_t)stride * height)
    return CAL_BAD_FRAME;
  cal_rect_t r;
  cal_err_t err = cal_roi_to_rect(width, height, roi, &r);
  if (err != CAL_OK) return err;
  const uint32_t rw = (uint32_t)(r.right - r.left);
  const uint32_t rh = (uint32_t)(r.bottom - r.top);
  if (rw < grid || rh < grid) return CAL_BAD_ROI;
  cal_descriptor_t next;
  memset(&next, 0, sizeof next);
  next.grid_side = grid;
  next.cell_count = (uint16_t)(grid * grid);
  uint8_t min_cell = UINT8_MAX, max_cell = 0;
  uint32_t mean_sum = 0;
  uint64_t clipped = 0;
  for (uint32_t cy = 0; cy < grid; ++cy) {
    const uint32_t y0 = r.top + cy * rh / grid;
    const uint32_t y1 = r.top + (cy + 1U) * rh / grid;
    for (uint32_t cx = 0; cx < grid; ++cx) {
      const uint32_t x0 = r.left + cx * rw / grid;
      const uint32_t x1 = r.left + (cx + 1U) * rw / grid;
      uint64_t sum = 0;
      uint32_t n = 0;
      for (uint32_t y = y0; y < y1; ++y) {
        for (uint32_t x = x0; x < x1; ++x) {
          uint8_t pixel = gray[(size_t)y * stride + x];
          sum += pixel;
          clipped += (pixel <= 5U || pixel >= 250U) ? 1U : 0U;
          ++n;
        }
      }
      if (!n) return CAL_BAD_ROI;
      uint8_t value = (uint8_t)((sum + n / 2U) / n);
      next.cells[cy * grid + cx] = value;
      mean_sum += value;
      if (value < min_cell) min_cell = value;
      if (value > max_cell) max_cell = value;
    }
  }
  next.mean = (uint8_t)((mean_sum + next.cell_count / 2U) / next.cell_count);
  next.contrast = (uint8_t)(max_cell - min_cell);
  next.clipped_pct = (uint8_t)(((uint64_t)clipped * 100U + (uint64_t)rw * rh / 2U) /
                                ((uint64_t)rw * rh));
  *out = next;
  return CAL_OK;
}

cal_err_t cal_descriptor_mae(const cal_descriptor_t *a,
                             const cal_descriptor_t *b, float *out_mae) {
  if (!a || !b || !out_mae) return CAL_BAD_ARGUMENT;
  if ((a->grid_side != 8U && a->grid_side != 16U) ||
      a->grid_side != b->grid_side ||
      a->cell_count != a->grid_side * a->grid_side ||
      b->cell_count != b->grid_side * b->grid_side) return CAL_BAD_GRID;
  uint32_t total = 0;
  for (uint16_t i = 0; i < a->cell_count; ++i) {
    int diff = (int)a->cells[i] - (int)b->cells[i];
    total += (uint32_t)(diff < 0 ? -diff : diff);
  }
  *out_mae = (float)total / a->cell_count;
  return CAL_OK;
}

static bool descriptor_valid(const cal_descriptor_t *d) {
  return d && (d->grid_side == 8U || d->grid_side == 16U) &&
         d->cell_count == (uint16_t)(d->grid_side * d->grid_side);
}

/* Inserción sobre un arreglo pequeño (<= CAL_DESCRIPTOR_MEDIAN_MAX). */
static uint8_t median_byte(uint8_t *values, size_t count) {
  for (size_t i = 1; i < count; ++i) {
    const uint8_t value = values[i];
    size_t j = i;
    while (j > 0 && values[j - 1U] > value) {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = value;
  }
  if (count % 2U) return values[count / 2U];
  /* Par: media de los dos centrales, .5 hacia arriba. */
  return (uint8_t)(((unsigned)values[count / 2U - 1U] +
                    (unsigned)values[count / 2U] + 1U) / 2U);
}

cal_err_t cal_descriptor_median(const cal_descriptor_t *samples, size_t count,
                                cal_descriptor_t *out) {
  if (!samples || !out) return CAL_BAD_ARGUMENT;
  if (count == 0U || count > CAL_DESCRIPTOR_MEDIAN_MAX) return CAL_BAD_ARGUMENT;
  if (!descriptor_valid(&samples[0])) return CAL_BAD_GRID;
  const uint8_t grid = samples[0].grid_side;
  for (size_t i = 1; i < count; ++i)
    if (!descriptor_valid(&samples[i]) || samples[i].grid_side != grid)
      return CAL_BAD_GRID;

  cal_descriptor_t next;
  memset(&next, 0, sizeof next);
  next.grid_side = grid;
  next.cell_count = samples[0].cell_count;
  uint32_t sum = 0;
  uint8_t min_cell = UINT8_MAX, max_cell = 0;
  uint8_t clipped[CAL_DESCRIPTOR_MEDIAN_MAX];
  for (size_t s = 0; s < count; ++s) clipped[s] = samples[s].clipped_pct;
  next.clipped_pct = median_byte(clipped, count);
  for (uint16_t i = 0; i < next.cell_count; ++i) {
    uint8_t column[CAL_DESCRIPTOR_MEDIAN_MAX];
    for (size_t s = 0; s < count; ++s) column[s] = samples[s].cells[i];
    const uint8_t value = median_byte(column, count);
    next.cells[i] = value;
    sum += value;
    if (value < min_cell) min_cell = value;
    if (value > max_cell) max_cell = value;
  }
  next.mean = (uint8_t)((sum + next.cell_count / 2U) / next.cell_count);
  next.contrast = (uint8_t)(max_cell - min_cell);
  *out = next;
  return CAL_OK;
}

cal_err_t cal_descriptor_encode(const cal_descriptor_t *d,
                                uint8_t *dst, size_t capacity,
                                size_t *written) {
  if (!d || !dst || !written) return CAL_BAD_ARGUMENT;
  if (!descriptor_valid(d)) return CAL_BAD_GRID;
  size_t required = CAL_DESCRIPTOR_HEADER_BYTES + d->cell_count;
  if (capacity < required) return CAL_BUFFER_SMALL;
  dst[0] = CAL_DESCRIPTOR_FORMAT_VERSION;
  dst[1] = d->grid_side;
  memcpy(dst + CAL_DESCRIPTOR_HEADER_BYTES, d->cells, d->cell_count);
  *written = required;
  return CAL_OK;
}

cal_err_t cal_descriptor_decode(const uint8_t *bytes, size_t length,
                                cal_descriptor_t *out) {
  if (!bytes || !out) return CAL_BAD_ARGUMENT;
  if (length < CAL_DESCRIPTOR_HEADER_BYTES ||
      bytes[0] != CAL_DESCRIPTOR_FORMAT_VERSION ||
      (bytes[1] != 8U && bytes[1] != 16U)) return CAL_BAD_GRID;
  const uint8_t grid = bytes[1];
  const size_t count = (size_t)grid * grid;
  if (length != CAL_DESCRIPTOR_HEADER_BYTES + count) return CAL_BAD_FRAME;
  cal_descriptor_t next;
  memset(&next, 0, sizeof(next));
  next.grid_side = grid;
  next.cell_count = (uint16_t)count;
  uint32_t sum = 0;
  uint8_t minv = UINT8_MAX, maxv = 0;
  for (size_t i = 0; i < count; ++i) {
    uint8_t v = bytes[CAL_DESCRIPTOR_HEADER_BYTES + i];
    next.cells[i] = v;
    sum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }
  next.mean = (uint8_t)((sum + count / 2U) / count);
  next.contrast = (uint8_t)(maxv - minv);
  /* La saturación de píxeles no puede reconstruirse de las medias. */
  next.clipped_pct = 0U;
  *out = next;
  return CAL_OK;
}

uint32_t cal_descriptor_crc32(const cal_descriptor_t *d) {
  if (!descriptor_valid(d)) return 0U;
  uint32_t crc = 0xFFFFFFFFU;
  /* Version y lado forman parte del CRC; descriptor luego celdas. */
  for (size_t i = 0; i < d->cell_count + 2U; ++i) {
    uint8_t value = i == 0 ? CAL_DESCRIPTOR_FORMAT_VERSION :
                    i == 1 ? d->grid_side : d->cells[i - 2U];
    crc ^= value;
    for (int j = 0; j < 8; ++j)
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return crc ^ 0xFFFFFFFFU;
}

const char *cal_error_code(cal_err_t e) {
  switch (e) {
    case CAL_OK: return "ok";
    case CAL_BAD_ARGUMENT: return "invalid_argument";
    case CAL_BAD_FRAME: return "invalid_frame";
    case CAL_BAD_ROI: return "invalid_roi";
    case CAL_BAD_GRID: return "unsupported_grid";
    case CAL_BUFFER_SMALL: return "buffer_small";
    default: return "invalid_argument";
  }
}
