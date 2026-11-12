/* Pruebas host del filtro de frame (E5.0.4). */
#include <stdio.h>
#include <string.h>

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

#define W CAL_FRAME_DEFAULT_WIDTH
#define H CAL_FRAME_DEFAULT_HEIGHT
#define MS(x) ((int64_t)(x) * 1000)

static cal_frame_info_t good(int64_t at_us) {
  cal_frame_info_t f = {.is_grayscale = true, .width = W, .height = H,
                        .len = (size_t)W * H, .captured_us = at_us};
  return f;
}

int main(void) {
  cal_frame_stats_t s;
  memset(&s, 0, sizeof s);
  const int64_t req = MS(10000), now = MS(10400);

  /* frame válido: posterior a la solicitud y reciente */
  cal_frame_info_t f = good(MS(10200));
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_OK);
  CHECK(s.accepted == 1 && cal_frame_rejected_total(&s) == 0);

  /* el caso real del log: 72960 != 76800 */
  f = good(MS(10200));
  f.len = 72960;
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_LENGTH);
  CHECK(s.rej_length == 1);
  f.len = 76800 + 1; /* desbordado también se rechaza */
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_LENGTH);
  f.len = 0;
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_LENGTH);
  CHECK(s.rej_length == 3);

  /* formato (p. ej. JPEG/RGB565) */
  f = good(MS(10200));
  f.is_grayscale = false;
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_FORMAT);

  /* dimensiones distintas aunque len coincida con otro tamaño */
  f = good(MS(10200));
  f.width = 640;
  f.height = 120; /* 640*120 = 76800 = 320*240: aún así es otra geometría */
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_DIMENSIONS);

  /* frame viejo: capturado ANTES de la solicitud (buffer anterior reutilizado) */
  f = good(MS(9999));
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_STALE);
  f = good(req); /* exactamente en el instante de la solicitud: válido */
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_OK);

  /* demasiado viejo respecto a ahora */
  f = good(MS(10000));
  CHECK(cal_frame_check(&f, W, H, req, MS(10000) + MS(CAL_FRAME_MAX_AGE_MS), CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_OK);
  CHECK(cal_frame_check(&f, W, H, req, MS(10000) + MS(CAL_FRAME_MAX_AGE_MS) + 1, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_TOO_OLD);

  /* marca del futuro */
  f = good(MS(10500));
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_FUTURE);

  /* argumentos inválidos */
  f = good(MS(10200));
  CHECK(cal_frame_check(NULL, W, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_ARGUMENT);
  CHECK(cal_frame_check(&f, 0, H, req, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_ARGUMENT);
  CHECK(cal_frame_check(&f, W, H, req, now, 0, &s) == CAL_FRAME_E_ARGUMENT);
  CHECK(cal_frame_check(&f, W, H, now, req, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_ARGUMENT); /* now < request */
  CHECK(cal_frame_check(&f, W, H, -1, now, CAL_FRAME_MAX_AGE_MS, &s) == CAL_FRAME_E_ARGUMENT);

  /* stats opcionales */
  CHECK(cal_frame_check(&f, W, H, req, now, CAL_FRAME_MAX_AGE_MS, NULL) == CAL_FRAME_OK);

  /* contadores separados y código estable para el ACK */
  CHECK(s.accepted == 3);
  CHECK(s.rej_format == 1 && s.rej_dimensions == 1 && s.rej_stale == 1);
  CHECK(s.rej_too_old == 1 && s.rej_future == 1 && s.rej_argument == 5);
  CHECK(cal_frame_rejected_total(&s) == 3 + 1 + 1 + 1 + 1 + 1 + 5);
  CHECK(!strcmp(cal_frame_error_code(CAL_FRAME_E_LENGTH), "invalid_frame"));
  CHECK(!strcmp(cal_frame_error_code(CAL_FRAME_OK), "ok"));
  CHECK(!strcmp(cal_frame_error_reason(CAL_FRAME_E_STALE), "stale"));
  CHECK(cal_frame_rejected_total(NULL) == 0);

  printf("calibration_frame: %d comprobaciones, %d fallos\n", g_run, g_fail);
  return g_fail ? 1 : 0;
}
