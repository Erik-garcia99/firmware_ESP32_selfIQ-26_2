#include "roi_core.h"

#include <math.h>
#include <string.h>

static bool hex_digit(char value) {
  return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f') ||
         (value >= 'A' && value <= 'F');
}

bool roi_uuid_valid(const char *value) {
  if (!value || strlen(value) != ROI_ID_LEN)
    return false;
  for (size_t i = 0; i < ROI_ID_LEN; ++i) {
    if (i == 8U || i == 13U || i == 18U || i == 23U) {
      if (value[i] != '-')
        return false;
    } else if (!hex_digit(value[i])) {
      return false;
    }
  }
  return true;
}

static bool bbox_valid(const roi_def_t *roi) {
  if (roi->x < 0.0f || roi->y < 0.0f || roi->w < ROI_MIN_SIDE ||
      roi->h < ROI_MIN_SIDE)
    return false;
  if (!isfinite((double)roi->x) || !isfinite((double)roi->y) ||
      !isfinite((double)roi->w) || !isfinite((double)roi->h))
    return false;
  return roi->x + roi->w <= 1.0f + ROI_GEOMETRY_EPSILON &&
         roi->y + roi->h <= 1.0f + ROI_GEOMETRY_EPSILON;
}

static float overlap_ratio(const roi_def_t *first, const roi_def_t *second) {
  float left = fmaxf(first->x, second->x);
  float top = fmaxf(first->y, second->y);
  float right = fminf(first->x + first->w, second->x + second->w);
  float bottom = fminf(first->y + first->h, second->y + second->h);
  if (right <= left || bottom <= top)
    return 0.0f;
  float smaller = fminf(first->w * first->h, second->w * second->h);
  if (smaller <= 0.0f)
    return 0.0f;
  return ((right - left) * (bottom - top)) / smaller;
}

roi_err_t roi_validate_set(const roi_def_t *rois, size_t count) {
  if (!rois && count)
    return ROI_ERR_INVALID_JSON;
  if (count > ROI_MAX)
    return ROI_ERR_TOO_MANY;

  for (size_t i = 0; i < count; ++i) {
    if (!roi_uuid_valid(rois[i].id))
      return ROI_ERR_INVALID_JSON;
    if (!bbox_valid(&rois[i]))
      return ROI_ERR_INVALID_BBOX;
    for (size_t j = 0; j < i; ++j) {
      if (!strcmp(rois[i].id, rois[j].id))
        return ROI_ERR_INVALID_JSON;
    }
  }

  for (size_t i = 0; i < count; ++i) {
    for (size_t j = i + 1; j < count; ++j) {
      if (overlap_ratio(&rois[i], &rois[j]) >
          ROI_MAX_OVERLAP + ROI_GEOMETRY_EPSILON)
        return ROI_ERR_OVERLAP;
    }
  }
  return ROI_OK;
}

const char *roi_error_code(roi_err_t err) {
  switch (err) {
  case ROI_OK:
    return "ok";
  case ROI_ERR_INVALID_JSON:
    return "invalid_json";
  case ROI_ERR_TOO_MANY:
    return "too_many_rois";
  case ROI_ERR_INVALID_BBOX:
    return "invalid_bbox";
  case ROI_ERR_OVERLAP:
    return "overlap";
  default:
    return "invalid_json";
  }
}

uint32_t roi_crc32(const void *data, size_t len) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFU;
  while (len--) {
    crc ^= *bytes++;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return crc ^ 0xFFFFFFFFU;
}

uint32_t roi_blob_crc(const roi_blob_t *blob) {
  roi_blob_t copy = *blob;
  copy.crc32 = 0;
  return roi_crc32(&copy, sizeof(copy));
}

void roi_blob_seal(roi_blob_t *blob) {
  blob->magic = ROI_BLOB_MAGIC;
  blob->format_version = ROI_BLOB_FORMAT_VERSION;
  blob->crc32 = 0;
  blob->crc32 = roi_blob_crc(blob);
}

bool roi_blob_verify(const roi_blob_t *blob) {
  if (!blob || blob->magic != ROI_BLOB_MAGIC ||
      blob->format_version != ROI_BLOB_FORMAT_VERSION ||
      blob->count > ROI_MAX)
    return false;
  return blob->crc32 == roi_blob_crc(blob);
}
