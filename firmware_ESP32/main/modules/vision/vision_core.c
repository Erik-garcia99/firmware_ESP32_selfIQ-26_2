#include "vision_core.h"

#include <stdbool.h>
#include <string.h>

/* Candidato E2: ganancia acotada; aún NO aprobado para el piloto. */
#define CELLS 64U
#define MIN_CONTRAST 16U
#define MAX_CLIPPED_PCT 20U
#define GAIN_FLOOR_MAD 16U
#define MAX_MAD 64U
#define MAX_MEAN_DRIFT 24
#define MIN_SEPARATION_Q8 (4U * 256U)

_Static_assert(CAL_DESCRIPTOR_FORMAT_VERSION == 1U,
               "vision_core solo admite descriptor v1");
_Static_assert(CAL_DESCRIPTOR_GRID_WIDTH == 8U && CAL_DESCRIPTOR_GRID_HEIGHT == 8U,
               "vision_core requiere grilla 8x8");

static bool hex_lower(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

static bool context_valid(const vision_context_t *c) {
  if (!c->config_version || !c->calibration_version ||
      c->roi_id[36] != '\0' || c->context_hash[64] != '\0') return false;
  for (size_t i = 0; i < 36; ++i) {
    bool dash = i == 8 || i == 13 || i == 18 || i == 23;
    if (dash ? c->roi_id[i] != '-' : !hex_lower(c->roi_id[i])) return false;
  }
  for (size_t i = 0; i < 64; ++i)
    if (!hex_lower(c->context_hash[i])) return false;
  return true;
}

static bool context_equal(const vision_context_t *a, const vision_context_t *b) {
  return context_valid(b) && a->config_version == b->config_version &&
         a->calibration_version == b->calibration_version &&
         memcmp(a->roi_id, b->roi_id, sizeof a->roi_id) == 0 &&
         memcmp(a->context_hash, b->context_hash, sizeof a->context_hash) == 0;
}

static bool decode(const vision_reference_t *r, cal_descriptor_t *d) {
  return r->bytes && r->length == VISION_DESCRIPTOR_BYTES &&
         r->bytes[0] == CAL_DESCRIPTOR_FORMAT_VERSION && r->bytes[1] == 8U &&
         cal_descriptor_decode(r->bytes, r->length, d) == CAL_OK &&
         cal_descriptor_crc32(d) == r->crc32;
}

/* Centrado por media entera de calibration_core.
 * Ganancia = 32/max(MAD,16): nunca supera 2, incluso con poca textura.
 * No forzar MAD=32 amplificando ruido ni rechazar solo por MAD<16.
 * Contraste mínimo, MAD máximo y separación de pareja siguen obligatorios.
 * Q8 conserva fracciones sin floats ni dependencia de libm. */
static vision_status_t normalize(const cal_descriptor_t *d, int32_t dst[CELLS]) {
  if (d->contrast < MIN_CONTRAST) return VISION_LOW_CONTRAST;
  uint32_t sum_abs = 0;
  for (size_t i = 0; i < CELLS; ++i) {
    int v = (int)d->cells[i] - d->mean;
    sum_abs += (uint32_t)(v < 0 ? -v : v);
  }
  if (sum_abs > CELLS * MAX_MAD)
    return VISION_NORMALIZATION_REJECTED;
  uint32_t denominator = sum_abs < CELLS * GAIN_FLOOR_MAD ?
                         CELLS * GAIN_FLOOR_MAD : sum_abs;
  for (size_t i = 0; i < CELLS; ++i) {
    int64_t numerator = ((int64_t)d->cells[i] - d->mean) * 32 * 256 * CELLS;
    int64_t magnitude = numerator < 0 ? -numerator : numerator;
    int32_t rounded = (int32_t)((magnitude + denominator / 2U) / denominator);
    dst[i] = numerator < 0 ? -rounded : rounded;
  }
  return VISION_OK;
}

static uint32_t mae(const int32_t a[CELLS], const int32_t b[CELLS]) {
  uint32_t total = 0;
  for (size_t i = 0; i < CELLS; ++i) {
    int32_t d = a[i] - b[i];
    total += (uint32_t)(d < 0 ? -d : d);
  }
  return (total + CELLS / 2U) / CELLS;
}

vision_status_t vision_measure(const vision_input_t *in,
                               const vision_reference_t *empty,
                               const vision_reference_t *full,
                               vision_result_t *out) {
  if (!out) return VISION_BAD_ARGUMENT;
  memset(out, 0, sizeof *out);
  out->occupancy_pct = VISION_INVALID_PERCENT;
  out->status = VISION_BAD_ARGUMENT;
  if (!in || !context_valid(&in->context)) return out->status;
  /* Copia explícita: padding del resultado siempre cero para determinismo. */
  memcpy(out->context.roi_id, in->context.roi_id, sizeof out->context.roi_id);
  memcpy(out->context.context_hash, in->context.context_hash,
         sizeof out->context.context_hash);
  out->context.config_version = in->context.config_version;
  out->context.calibration_version = in->context.calibration_version;
  out->measured_at_monotonic_ms = in->measured_at_monotonic_ms;
  if (!empty || !full) {
    out->status = VISION_UNCALIBRATED;
    return out->status;
  }
  if (!context_equal(&in->context, &empty->context) ||
      !context_equal(&in->context, &full->context)) {
    out->status = VISION_CONTEXT_MISMATCH;
    return out->status;
  }
  cal_descriptor_t e, f, current;
  if (!decode(empty, &e) || !decode(full, &f)) {
    out->status = VISION_INVALID_REFERENCE;
    return out->status;
  }
  cal_err_t err = cal_descriptor_extract(in->gray, in->length, in->width,
                                        in->height, in->stride, in->roi, 8,
                                        &current);
  if (err != CAL_OK) {
    out->status = err == CAL_BAD_ROI ? VISION_INVALID_ROI : VISION_INVALID_FRAME;
    return out->status;
  }
  out->mean = current.mean;
  out->contrast = current.contrast;
  out->clipped_pct = current.clipped_pct;
  if (current.clipped_pct > MAX_CLIPPED_PCT) {
    out->status = VISION_EXCESSIVE_CLIPPING;
    return out->status;
  }
  int32_t ne[CELLS], nf[CELLS], nc[CELLS];
  if (normalize(&e, ne) != VISION_OK || normalize(&f, nf) != VISION_OK) {
    out->status = VISION_INVALID_REFERENCE;
    return out->status;
  }
  out->separation_q8 = mae(ne, nf);
  if (out->separation_q8 < MIN_SEPARATION_Q8) {
    out->status = VISION_INDISTINGUISHABLE_REFERENCES;
    return out->status;
  }
  int min_mean = e.mean < f.mean ? e.mean : f.mean;
  int max_mean = e.mean > f.mean ? e.mean : f.mean;
  if ((int)current.mean < min_mean - MAX_MEAN_DRIFT ||
      (int)current.mean > max_mean + MAX_MEAN_DRIFT) {
    out->status = VISION_NORMALIZATION_REJECTED;
    return out->status;
  }
  out->status = normalize(&current, nc);
  if (out->status != VISION_OK) return out->status;
  out->d_empty_q8 = mae(nc, ne);
  out->d_full_q8 = mae(nc, nf);
  uint32_t denominator = out->d_empty_q8 + out->d_full_q8;
  /* Defensa explícita: nunca transformar 0/0 en vacío o lleno. */
  if (denominator < MIN_SEPARATION_Q8) {
    out->status = VISION_INDISTINGUISHABLE_REFERENCES;
    return out->status;
  }
  uint32_t pct = (100U * out->d_empty_q8 + denominator / 2U) / denominator;
  out->occupancy_pct = (uint8_t)(pct > 100U ? 100U : pct);
  uint32_t consistency = (100U * out->separation_q8 + denominator / 2U) / denominator;
  if (consistency > 100U) consistency = 100U;
  out->confidence = (uint8_t)((consistency * (100U - current.clipped_pct) + 50U) / 100U);
  return out->status;
}

const char *vision_status_code(vision_status_t status) {
  switch (status) {
    case VISION_OK: return "ok";
    case VISION_BAD_ARGUMENT: return "bad_argument";
    case VISION_INVALID_FRAME: return "invalid_frame";
    case VISION_INVALID_ROI: return "invalid_roi";
    case VISION_UNCALIBRATED: return "uncalibrated";
    case VISION_CONTEXT_MISMATCH: return "context_mismatch";
    case VISION_INVALID_REFERENCE: return "invalid_reference";
    case VISION_LOW_CONTRAST: return "low_contrast";
    case VISION_EXCESSIVE_CLIPPING: return "excessive_clipping";
    case VISION_NORMALIZATION_REJECTED: return "normalization_rejected";
    case VISION_INDISTINGUISHABLE_REFERENCES: return "indistinguishable_references";
    default: return "unknown";
  }
}

uint32_t vision_algorithm_version(void) {
  return VISION_ALGORITHM_VERSION;
}
