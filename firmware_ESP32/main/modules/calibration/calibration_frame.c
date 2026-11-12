#include "calibration_frame.h"

#include <string.h>

static void count(cal_frame_stats_t *s, cal_frame_err_t err) {
  if (!s)
    return;
  switch (err) {
  case CAL_FRAME_OK:
    ++s->accepted;
    break;
  case CAL_FRAME_E_ARGUMENT:
    ++s->rej_argument;
    break;
  case CAL_FRAME_E_FORMAT:
    ++s->rej_format;
    break;
  case CAL_FRAME_E_DIMENSIONS:
    ++s->rej_dimensions;
    break;
  case CAL_FRAME_E_LENGTH:
    ++s->rej_length;
    break;
  case CAL_FRAME_E_STALE:
    ++s->rej_stale;
    break;
  case CAL_FRAME_E_TOO_OLD:
    ++s->rej_too_old;
    break;
  case CAL_FRAME_E_FUTURE:
    ++s->rej_future;
    break;
  }
}

cal_frame_err_t cal_frame_check(const cal_frame_info_t *f, uint32_t exp_w,
                                uint32_t exp_h, int64_t request_us,
                                int64_t now_us, uint32_t max_age_ms,
                                cal_frame_stats_t *stats) {
  cal_frame_err_t err = CAL_FRAME_OK;
  if (!f || !exp_w || !exp_h || max_age_ms == 0 || request_us < 0 ||
      now_us < request_us)
    err = CAL_FRAME_E_ARGUMENT;
  else if (!f->is_grayscale)
    err = CAL_FRAME_E_FORMAT;
  else if (f->width != exp_w || f->height != exp_h)
    err = CAL_FRAME_E_DIMENSIONS;
  /* uint64: 320*240 cabe de sobra, pero así no hay desbordes con otros tamaños */
  else if ((uint64_t)f->len != (uint64_t)exp_w * (uint64_t)exp_h)
    err = CAL_FRAME_E_LENGTH;
  else if (f->captured_us > now_us)
    err = CAL_FRAME_E_FUTURE;
  else if (f->captured_us < request_us)
    err = CAL_FRAME_E_STALE;
  else if ((uint64_t)(now_us - f->captured_us) >
           (uint64_t)max_age_ms * 1000ULL)
    err = CAL_FRAME_E_TOO_OLD;
  count(stats, err);
  return err;
}

const char *cal_frame_error_code(cal_frame_err_t err) {
  return err == CAL_FRAME_OK ? "ok" : "invalid_frame";
}

const char *cal_frame_error_reason(cal_frame_err_t err) {
  switch (err) {
  case CAL_FRAME_OK:
    return "ok";
  case CAL_FRAME_E_FORMAT:
    return "format";
  case CAL_FRAME_E_DIMENSIONS:
    return "dimensions";
  case CAL_FRAME_E_LENGTH:
    return "length";
  case CAL_FRAME_E_STALE:
    return "stale";
  case CAL_FRAME_E_TOO_OLD:
    return "too_old";
  case CAL_FRAME_E_FUTURE:
    return "future";
  default:
    return "argument";
  }
}

uint32_t cal_frame_rejected_total(const cal_frame_stats_t *s) {
  if (!s)
    return 0;
  return s->rej_argument + s->rej_format + s->rej_dimensions + s->rej_length +
         s->rej_stale + s->rej_too_old + s->rej_future;
}
