#ifndef SELFIQ_CALIBRATION_CAPTURE_H
#define SELFIQ_CALIBRATION_CAPTURE_H

/* E5.3: empaquetado puro de una captura de calibración.
 *
 * Este módulo NO toca la cámara ni ESP-IDF: recibe el frame gris que la tarea
 * propietaria de la cámara ya validó con calibration_frame, extrae el
 * descriptor 8x8 v1, lo serializa, calcula su CRC32 y, cuando la misma tarea
 * genera el JPEG de revisión a partir del MISMO camera_fb_t, verifica que el
 * JPEG sea válido, tenga las dimensiones esperadas y calcula su SHA-256.
 *
 * La correlación frame/JPEG la garantiza camera_lib: ambas operaciones usan el
 * mismo `camera_fb_t` antes de liberarlo. Este módulo no recibe dos frames
 * distintos, por lo que no puede "emparejar" una imagen con otro descriptor. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "calibration_core.h"
#include "calibration_sha256.h"

#define CAL_CAPTURE_UUID_CHARS 36U
#define CAL_CAPTURE_UUID_SIZE (CAL_CAPTURE_UUID_CHARS + 1U)
#define CAL_CAPTURE_HASH_HEX_CHARS 64U
#define CAL_CAPTURE_HASH_HEX_SIZE (CAL_CAPTURE_HASH_HEX_CHARS + 1U)
#define CAL_CAPTURE_CRC_HEX_SIZE 9U
#define CAL_CAPTURE_TIMESTAMP_SIZE 33U
#define CAL_CAPTURE_SHA256_HEX_SIZE CAL_SHA256_HEX_SIZE

/* Contrato v1 congelado: [1][8][64 celdas] = 66 B. */
#define CAL_CAPTURE_DESCRIPTOR_BYTES                                        \
  (CAL_DESCRIPTOR_HEADER_BYTES + CAL_DESCRIPTOR_CELLS)

/* Calidad del JPEG de revisión. El encoder de esp32-camera usa 0..100
 * (mayor = mejor); no confundir con sensor->set_quality (0..63, menor = mejor). */
#define CAL_CAPTURE_JPEG_QUALITY 30U

typedef struct {
  cal_descriptor_t descriptor; /* incluye mean/contrast/clipped_pct */
  uint8_t descriptor_bytes[CAL_CAPTURE_DESCRIPTOR_BYTES];
  size_t descriptor_len;
  uint32_t descriptor_crc;
  char image_sha256[CAL_CAPTURE_SHA256_HEX_SIZE];
  uint32_t image_bytes;
  uint16_t width;
  uint16_t height;
} cal_capture_t;

/* Extrae el descriptor 8x8 del frame y lo serializa en formato v1. No estima
 * ocupación porcentual. Devuelve CAL_OK solo si el descriptor mide 66 B y su
 * CRC32 se calculó sobre los mismos bytes que se enviarán. */
cal_err_t cal_capture_build(const uint8_t *gray, size_t length, uint32_t width,
                            uint32_t height, uint32_t stride, cal_roi_t roi,
                            cal_capture_t *out);

/* Igual que cal_capture_build, pero a partir de N descriptores ya extraídos de
 * frames del mismo tamaño/ROI (1..CAL_DESCRIPTOR_MEDIAN_MAX). Usa mediana por
 * celda para reducir el ruido/instante de una sola toma. No captura frames ni
 * toca la cámara: la tarea propietaria decide cuántos frames acumular. El
 * formato persistido sigue siendo v1/8x8/66 B. */
cal_err_t cal_capture_build_median(const cal_descriptor_t *samples, size_t count,
                                   uint32_t width, uint32_t height,
                                   cal_capture_t *out);

/* Añade la identidad del JPEG de revisión: valida SOI/SOF/EOI, exige ancho y
 * alto esperados (y coherentes con el frame), y calcula SHA-256 de los bytes
 * exactos. El SHA cubre el JPEG completo, no el frame gris. */
cal_err_t cal_capture_attach_jpeg(cal_capture_t *capture, const uint8_t *jpeg,
                                  size_t jpeg_length, uint16_t expected_width,
                                  uint16_t expected_height);

/* Lee ancho/alto del primer marcador SOF. Acepta SOF0..SOF15 de trama
 * (excepto DHT/JPG/DAC). Exige que el buffer termine en FF D9. */
cal_err_t cal_jpeg_dimensions(const uint8_t *jpeg, size_t length,
                              uint16_t *width, uint16_t *height);

/* Base64 estándar con relleno '='. Se usa para el descriptor opcional del
 * multipart de E5.6 (66 B -> 88 caracteres + '\0'). */
size_t cal_base64_size(size_t input_length);
cal_err_t cal_base64_encode(const uint8_t *src, size_t length, char *dst,
                            size_t capacity, size_t *written);

#endif
