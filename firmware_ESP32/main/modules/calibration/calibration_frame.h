#ifndef SELFIQ_CALIBRATION_FRAME_H
#define SELFIQ_CALIBRATION_FRAME_H

/* E5.0.4: filtro de frame válido para calibración (módulo PURO, sin ESP-IDF).
 *
 * Motivo: el driver emite `FB-SIZE: 72960 != 76800`, `EV-EOF-OVF` y
 * `EV-VSYNC-OVF`, y el contador `CAPTURE_OK ... fail=0` no cuenta esos
 * descartes. Para calibrar solo se acepta un frame que cumpla TODO lo
 * siguiente. Los rechazos se cuentan aparte (cal_frame_stats_t) para no
 * confundirlos con `fail=0`. camera_lib (E5.3) rellena cal_frame_info_t desde
 * camera_fb_t y llama a cal_frame_check(). */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Resolución esperada hoy (log: 320x240). Se pasa por parámetro; estas macros
 * son solo el valor por defecto de la integración. */
#define CAL_FRAME_DEFAULT_WIDTH 320U
#define CAL_FRAME_DEFAULT_HEIGHT 240U
#define CAL_FRAME_MAX_AGE_MS 1500U

typedef enum {
  CAL_FRAME_OK = 0,
  CAL_FRAME_E_ARGUMENT,   /* puntero nulo o parámetros incoherentes */
  CAL_FRAME_E_FORMAT,     /* no es PIXFORMAT_GRAYSCALE */
  CAL_FRAME_E_DIMENSIONS, /* ancho/alto distintos de los esperados */
  CAL_FRAME_E_LENGTH,     /* len != ancho*alto (frame incompleto/desbordado) */
  CAL_FRAME_E_STALE,      /* capturado ANTES de la solicitud de calibración */
  CAL_FRAME_E_TOO_OLD,    /* demasiado viejo respecto a "ahora" */
  CAL_FRAME_E_FUTURE,     /* marca de tiempo posterior a "ahora" (reloj roto) */
} cal_frame_err_t;

typedef struct {
  bool is_grayscale;    /* fb->format == PIXFORMAT_GRAYSCALE */
  uint32_t width;       /* fb->width */
  uint32_t height;      /* fb->height */
  size_t len;           /* fb->len */
  int64_t captured_us;  /* fb->timestamp en µs (monotónico, mismo reloj que now) */
} cal_frame_info_t;

/* Contadores de rechazo propios de calibración. No sustituyen a `fail=` de la
 * captura ordinaria. */
typedef struct {
  uint32_t accepted;
  uint32_t rej_argument;
  uint32_t rej_format;
  uint32_t rej_dimensions;
  uint32_t rej_length;
  uint32_t rej_stale;
  uint32_t rej_too_old;
  uint32_t rej_future;
} cal_frame_stats_t;

/* Valida un frame. `request_us` es el instante en que llegó la orden
 * `calibrate`; `now_us` el instante actual (ambos del mismo reloj que
 * captured_us). `stats` es opcional. */
cal_frame_err_t cal_frame_check(const cal_frame_info_t *frame,
                                uint32_t expected_width,
                                uint32_t expected_height, int64_t request_us,
                                int64_t now_us, uint32_t max_age_ms,
                                cal_frame_stats_t *stats);

/* Código estable para el ACK (`invalid_frame`). */
const char *cal_frame_error_code(cal_frame_err_t err);
const char *cal_frame_error_reason(cal_frame_err_t err);
uint32_t cal_frame_rejected_total(const cal_frame_stats_t *stats);

#endif
