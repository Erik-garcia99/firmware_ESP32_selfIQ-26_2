/* Adaptador SOLO host: las identidades sintéticas permiten medir imágenes
 * históricas sin atribuirles una pareja activa. Python valida la procedencia;
 * este puente no demuestra contexto/calibración aplicados ni acepta telemetría. */
#include "../main/modules/vision/vision_core.h"
#include <string.h>

int offline_descriptor(const uint8_t *gray, size_t length, uint32_t width,
                       uint32_t height, const uint32_t roi[4], uint8_t bytes[66]) {
  cal_descriptor_t d;
  cal_roi_t r = {roi[0], roi[1], roi[2], roi[3]};
  cal_err_t err = cal_descriptor_extract(gray, length, width, height, width, r, 8, &d);
  if (err != CAL_OK) return (int)err;
  size_t written;
  return (int)cal_descriptor_encode(&d, bytes, 66, &written);
}

int offline_measure(const uint8_t *gray, size_t length, uint32_t width,
                    uint32_t height, const uint32_t roi[4],
                    const uint8_t empty_bytes[66], const uint8_t full_bytes[66],
                    uint32_t values[9]) {  vision_input_t in = {0};
  in.gray = gray; in.length = length;
  in.width = width; in.height = height; in.stride = width;
  in.roi = (cal_roi_t){roi[0], roi[1], roi[2], roi[3]};
  memcpy(in.context.roi_id, "00000000-0000-0000-0000-000000000001", 37);
  memset(in.context.context_hash, '0', 64);
  in.context.config_version = 1; in.context.calibration_version = 1;
  vision_reference_t refs[2] = {0};
  const uint8_t *bytes[2] = {empty_bytes, full_bytes};
  for (unsigned i = 0; i < 2; ++i) {
    cal_descriptor_t d;
    if (cal_descriptor_decode(bytes[i], 66, &d) != CAL_OK) return -1;
    refs[i].context = in.context; refs[i].bytes = bytes[i]; refs[i].length = 66;
    refs[i].crc32 = cal_descriptor_crc32(&d);
  }
  vision_result_t out;
  int status = (int)vision_measure(&in, &refs[0], &refs[1], &out);
  values[0] = out.occupancy_pct; values[1] = out.confidence;
  values[2] = out.mean; values[3] = out.contrast; values[4] = out.clipped_pct;
  values[5] = out.d_empty_q8; values[6] = out.d_full_q8;
  values[7] = out.separation_q8; values[8] = (uint32_t)status;
  return status;
}

/* Mediana por celda de N descriptores v1/8x8 ya persistidos o extraídos.
 * Solo host: valida el formato, delega en calibration_core y devuelve los 66 B
 * que se usarían como referencia. No captura ni persiste nada. */
int offline_median(const uint8_t *samples, size_t count, uint8_t out[66]) {
  if (!samples || !out || count == 0 || count > CAL_DESCRIPTOR_MEDIAN_MAX)
    return -1;
  cal_descriptor_t parsed[CAL_DESCRIPTOR_MEDIAN_MAX];
  for (size_t i = 0; i < count; ++i) {
    if (cal_descriptor_decode(samples + i * 66, 66, &parsed[i]) != CAL_OK)
      return -1;
  }
  cal_descriptor_t merged;
  if (cal_descriptor_median(parsed, count, &merged) != CAL_OK) return -1;
  size_t written = 0;
  if (cal_descriptor_encode(&merged, out, 66, &written) != CAL_OK ||
      written != 66)
    return -1;
  return 0;
}
