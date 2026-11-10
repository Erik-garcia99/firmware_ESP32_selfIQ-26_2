#ifndef SELFIQ_CALIBRATION_CORE_H
#define SELFIQ_CALIBRATION_CORE_H

#include <stddef.h>
#include <stdint.h>

#define CAL_DESCRIPTOR_GRID_WIDTH 8U
#define CAL_DESCRIPTOR_GRID_HEIGHT 8U

#define CAL_DESCRIPTOR_CELLS                                                   \
  (CAL_DESCRIPTOR_GRID_WIDTH * CAL_DESCRIPTOR_GRID_HEIGHT)

#define CAL_DESCRIPTOR_FORMAT_VERSION 1U

/* E2: algoritmo puro, sin ESP-IDF. Formato candidato; NO se persiste aún en
 * NVS. */
#define CAL_COORD_SCALE 1000000U
#define CAL_GRID_MIN 8U
#define CAL_GRID_MAX 16U
#define CAL_MAX_CELLS (CAL_GRID_MAX * CAL_GRID_MAX)
#define CAL_DESCRIPTOR_HEADER_BYTES 2U
#define CAL_DESCRIPTOR_MAX_BYTES (CAL_DESCRIPTOR_HEADER_BYTES + CAL_MAX_CELLS)

typedef enum {
  CAL_OK = 0,
  CAL_BAD_ARGUMENT,
  CAL_BAD_FRAME,
  CAL_BAD_ROI,
  CAL_BAD_GRID,
  CAL_BUFFER_SMALL
} cal_err_t;

typedef struct {
  uint32_t x; /* coordenadas normalizadas 0..1000000 */
  uint32_t y;
  uint32_t w;
  uint32_t h;
} cal_roi_t;

typedef struct {
  uint16_t left, top, right, bottom; /* [left,right) y [top,bottom) */
} cal_rect_t;

typedef struct {
  uint8_t grid_side;
  uint16_t cell_count;
  uint8_t cells[CAL_MAX_CELLS];
  uint8_t mean;        /* media de celdas; métricas exploratorias */
  uint8_t clipped_pct; /* píxeles cercanos a extremos, 0..100 */
  uint8_t contrast;    /* max celda - min celda */
} cal_descriptor_t;

/* uint64 evita desbordamiento x * ancho. left/top=floor; right/bottom=ceil.
 * Valida ROI no vacía y límites; el frame debe tener stride>=width y
 * length>=stride*height. Las celdas usan límites enteros [inicio,fin).
 * Cada media de celda redondea al entero más cercano: (suma+n/2)/n. */
cal_err_t cal_roi_to_rect(uint32_t frame_width, uint32_t frame_height,
                          cal_roi_t roi, cal_rect_t *out);
cal_err_t cal_descriptor_extract(const uint8_t *gray, size_t length,
                                 uint32_t width, uint32_t height,
                                 uint32_t stride, cal_roi_t roi,
                                 uint8_t grid_side, cal_descriptor_t *out);
/* Diferencia media absoluta de los bytes de celda, para investigación E2;
 * NO es una medida calibrada de porcentaje de ocupación. */
cal_err_t cal_descriptor_mae(const cal_descriptor_t *a,
                             const cal_descriptor_t *b, float *out_mae);
/* Contrato bytes v1: [format_version=1][grid_side=8|16][grid*grid bytes].
 * Sin padding, longitud exacta, sin métricas auxiliares. El mismo arreglo
 * se envía como descriptor base64 al backend para su pair_hash SHA-256.
 * Destino debe tener al menos 2+grid*grid bytes. */
cal_err_t cal_descriptor_encode(const cal_descriptor_t *descriptor,
                                uint8_t *dst, size_t capacity, size_t *written);
cal_err_t cal_descriptor_decode(const uint8_t *bytes, size_t length,
                                cal_descriptor_t *out);
/* CRC32 IEEE sobre los bytes explícitos del descriptor, no struct/padding. */
uint32_t cal_descriptor_crc32(const cal_descriptor_t *descriptor);
const char *cal_error_code(cal_err_t err);
#endif
