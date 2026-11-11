#include "calibration_capture.h"

#include <string.h>

static bool jpeg_sof_marker(uint8_t marker) {
  switch (marker) {
    case 0xC0: /* SOF0 baseline */
    case 0xC1: /* SOF1 extended sequential */
    case 0xC2: /* SOF2 progressive */
    case 0xC3:
    case 0xC5:
    case 0xC6:
    case 0xC7:
    case 0xC9:
    case 0xCA:
    case 0xCB:
    case 0xCD:
    case 0xCE:
    case 0xCF:
      return true;
    default:
      return false;
  }
}

cal_err_t cal_jpeg_dimensions(const uint8_t *jpeg, size_t length,
                              uint16_t *width, uint16_t *height) {
  if (!jpeg || !width || !height) return CAL_BAD_ARGUMENT;
  if (length < 4U || jpeg[0] != 0xFFU || jpeg[1] != 0xD8U)
    return CAL_BAD_FRAME;
  if (jpeg[length - 2U] != 0xFFU || jpeg[length - 1U] != 0xD9U)
    return CAL_BAD_FRAME;

  size_t pos = 2U;
  while (pos + 2U <= length) {
    if (jpeg[pos] != 0xFFU) return CAL_BAD_FRAME;
    while (pos < length && jpeg[pos] == 0xFFU) ++pos;
    if (pos >= length) return CAL_BAD_FRAME;
    const uint8_t marker = jpeg[pos++];

    if (marker == 0xD9U || marker == 0xDAU) break; /* EOI o SOS sin SOF */
    if (marker == 0x01U || (marker >= 0xD0U && marker <= 0xD7U))
      continue; /* marcadores sin longitud */

    if (pos + 2U > length) return CAL_BAD_FRAME;
    const uint16_t segment = (uint16_t)(((uint16_t)jpeg[pos] << 8) | jpeg[pos + 1U]);
    if (segment < 2U || pos + segment > length) return CAL_BAD_FRAME;

    if (jpeg_sof_marker(marker)) {
      if (segment < 7U) return CAL_BAD_FRAME;
      const uint16_t h =
          (uint16_t)(((uint16_t)jpeg[pos + 3U] << 8) | jpeg[pos + 4U]);
      const uint16_t w =
          (uint16_t)(((uint16_t)jpeg[pos + 5U] << 8) | jpeg[pos + 6U]);
      if (!w || !h) return CAL_BAD_FRAME;
      *width = w;
      *height = h;
      return CAL_OK;
    }
    pos += segment;
  }
  return CAL_BAD_FRAME;
}

cal_err_t cal_capture_build(const uint8_t *gray, size_t length, uint32_t width,
                            uint32_t height, uint32_t stride, cal_roi_t roi,
                            cal_capture_t *out) {
  if (!out) return CAL_BAD_ARGUMENT;
  cal_descriptor_t descriptor;
  cal_err_t err =
      cal_descriptor_extract(gray, length, width, height, stride, roi,
                             CAL_DESCRIPTOR_GRID_WIDTH, &descriptor);
  if (err != CAL_OK) return err;

  memset(out, 0, sizeof(*out));
  size_t written = 0;
  err = cal_descriptor_encode(&descriptor, out->descriptor_bytes,
                              sizeof(out->descriptor_bytes), &written);
  if (err != CAL_OK) return err;
  if (written != CAL_CAPTURE_DESCRIPTOR_BYTES ||
      descriptor.grid_side != CAL_DESCRIPTOR_GRID_WIDTH)
    return CAL_BAD_FRAME;

  out->descriptor = descriptor;
  out->descriptor_len = written;
  out->descriptor_crc = cal_descriptor_crc32(&descriptor);
  out->width = (uint16_t)width;
  out->height = (uint16_t)height;
  return CAL_OK;
}

cal_err_t cal_capture_attach_jpeg(cal_capture_t *capture, const uint8_t *jpeg,
                                  size_t jpeg_length, uint16_t expected_width,
                                  uint16_t expected_height) {
  if (!capture || !jpeg || jpeg_length == 0U ||
      jpeg_length > (size_t)UINT32_MAX)
    return CAL_BAD_ARGUMENT;
  if (!expected_width || !expected_height) return CAL_BAD_FRAME;

  uint16_t width = 0, height = 0;
  cal_err_t err = cal_jpeg_dimensions(jpeg, jpeg_length, &width, &height);
  if (err != CAL_OK) return err;
  if (width != expected_width || height != expected_height ||
      width != capture->width || height != capture->height)
    return CAL_BAD_FRAME;

  cal_sha256_hex(jpeg, jpeg_length, capture->image_sha256);
  capture->image_bytes = (uint32_t)jpeg_length;
  return CAL_OK;
}

size_t cal_base64_size(size_t input_length) {
  return ((input_length + 2U) / 3U) * 4U + 1U;
}

cal_err_t cal_base64_encode(const uint8_t *src, size_t length, char *dst,
                            size_t capacity, size_t *written) {
  if ((!src && length) || !dst || !written) return CAL_BAD_ARGUMENT;
  const size_t required = cal_base64_size(length);
  if (capacity < required) return CAL_BUFFER_SMALL;
  static const char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t in = 0, out = 0;
  while (in + 3U <= length) {
    const uint32_t value = ((uint32_t)src[in] << 16) |
                           ((uint32_t)src[in + 1U] << 8) | src[in + 2U];
    dst[out++] = alphabet[(value >> 18) & 0x3FU];
    dst[out++] = alphabet[(value >> 12) & 0x3FU];
    dst[out++] = alphabet[(value >> 6) & 0x3FU];
    dst[out++] = alphabet[value & 0x3FU];
    in += 3U;
  }
  const size_t remaining = length - in;
  if (remaining) {
    uint32_t value = (uint32_t)src[in] << 16;
    if (remaining == 2U) value |= (uint32_t)src[in + 1U] << 8;
    dst[out++] = alphabet[(value >> 18) & 0x3FU];
    dst[out++] = alphabet[(value >> 12) & 0x3FU];
    dst[out++] = remaining == 2U ? alphabet[(value >> 6) & 0x3FU] : '=';
    dst[out++] = '=';
  }
  dst[out] = '\0';
  *written = out;
  return CAL_OK;
}
