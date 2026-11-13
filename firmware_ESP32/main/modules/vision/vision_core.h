#ifndef SELFIQ_VISION_CORE_H
#define SELFIQ_VISION_CORE_H

#include "../calibration/calibration_core.h"

#define VISION_DESCRIPTOR_BYTES 66U
#define VISION_INVALID_PERCENT 255U
/* Versión del cálculo host, NO cambia el formato persistido de calibración. */
#define VISION_ALGORITHM_VERSION 2U

typedef enum {
  VISION_OK = 0,
  VISION_BAD_ARGUMENT,
  VISION_INVALID_FRAME,
  VISION_INVALID_ROI,
  VISION_UNCALIBRATED,
  VISION_CONTEXT_MISMATCH,
  VISION_INVALID_REFERENCE,
  VISION_LOW_CONTRAST,
  VISION_EXCESSIVE_CLIPPING,
  VISION_NORMALIZATION_REJECTED,
  VISION_INDISTINGUISHABLE_REFERENCES
} vision_status_t;

/* Contexto canónico del snapshot aplicado: UUID/hash hex en minúsculas.
 * El hash vincula geometría, producto y cámara según el contrato de Fase 4. */
typedef struct {
  char roi_id[37];
  char context_hash[65];
  uint32_t config_version;
  uint32_t calibration_version;
} vision_context_t;

typedef struct {
  vision_context_t context;
  const uint8_t *bytes; /* prestados, inmutables durante la llamada */
  size_t length;
  uint32_t crc32; /* valor esperado del almacenamiento validado */
} vision_reference_t;

typedef struct {
  const uint8_t *gray;
  size_t length;
  uint32_t width, height, stride;
  cal_roi_t roi;
  vision_context_t context;
  uint64_t measured_at_monotonic_ms; /* provisto por llamador, no reloj interno */
} vision_input_t;

typedef struct {
  vision_status_t status;
  vision_context_t context;
  uint64_t measured_at_monotonic_ms;
  uint8_t occupancy_pct; /* 255 en TODO rechazo; no publicar */
  uint8_t confidence; /* 0..100, consistencia geométrica, NO precisión */
  uint8_t mean, contrast, clipped_pct; /* frame actual, no referencias */
  uint32_t d_empty_q8, d_full_q8, separation_q8; /* MAE, gris normalizado Q8 */
} vision_result_t;

/* C11 puro, sin malloc ni estado global. out obligatorio y no solapado con
 * entradas. Dos referencias obligatorias, v1/8x8 exacto; NULL => uncalibrated.
 * Solo VISION_OK habilita occupancy_pct. Contrato E2 vigente en
 * docs_compartido/fotografias_calibracion/VISION_CORE_CONTRATO_E2.md;
 * README de E1 describe la normalización anterior, no la revisión E2. */
vision_status_t vision_measure(const vision_input_t *input,
                               const vision_reference_t *empty,
                               const vision_reference_t *full,
                               vision_result_t *out);
const char *vision_status_code(vision_status_t status);
uint32_t vision_algorithm_version(void);

#endif
