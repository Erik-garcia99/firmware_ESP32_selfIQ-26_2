#include "roi_cmd.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static bool req_id_valid(const char *value) {
  if (!value)
    return false;
  size_t length = strlen(value);
  if (length < 1U || length > ROI_REQ_ID_MAX)
    return false;
  for (size_t i = 0; i < length; ++i) {
    char c = value[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '.' || c == '_' || c == ':' ||
          c == '-'))
      return false;
  }
  return true;
}

void roi_cmd_extract_req_id(const cJSON *root, char *out, size_t out_size) {
  if (!out || out_size == 0)
    return;
  out[0] = '\0';
  if (!root)
    return;
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, "req_id");
  if (!cJSON_IsString(item) || !req_id_valid(item->valuestring))
    return;
  size_t length = strlen(item->valuestring);
  if (length + 1U > out_size)
    return;
  memcpy(out, item->valuestring, length + 1U);
}

void roi_cmd_extract_req_id_raw(const char *data, int len, char *out,
                                size_t out_size) {
  if (!out || out_size == 0)
    return;
  out[0] = '\0';
  if (!data || len <= 0)
    return;

  const char *end = data + len;
  for (const char *p = data; p + 8 <= end; ++p) {
    if (memcmp(p, "\"req_id\"", 8) != 0)
      continue;
    const char *cursor = p + 8;
    while (cursor < end && (*cursor == ' ' || *cursor == '\t'))
      ++cursor;
    if (cursor >= end || *cursor != ':')
      continue;
    ++cursor;
    while (cursor < end && (*cursor == ' ' || *cursor == '\t'))
      ++cursor;
    if (cursor >= end || *cursor != '"')
      continue;
    ++cursor;
    const char *start = cursor;
    while (cursor < end && *cursor != '"')
      ++cursor;
    size_t length = (size_t)(cursor - start);
    if (length < 1U || length > ROI_REQ_ID_MAX)
      return;
    char candidate[ROI_REQ_ID_SIZE];
    memcpy(candidate, start, length);
    candidate[length] = '\0';
    if (!req_id_valid(candidate))
      return;
    if (length + 1U <= out_size)
      memcpy(out, candidate, length + 1U);
    return;
  }
}

static bool json_number(const cJSON *item, double *value) {
  if (!cJSON_IsNumber(item))
    return false;
  double number = item->valuedouble;
  if (!isfinite(number))
    return false;
  *value = number;
  return true;
}

roi_err_t roi_cmd_parse_set(const cJSON *root, roi_def_t *rois,
                            size_t *count, uint32_t *version) {
  if (!root || !rois || !count || !version)
    return ROI_ERR_INVALID_JSON;

  const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
  if (!cJSON_IsString(type) || strcmp(type->valuestring, "roi.set") != 0)
    return ROI_ERR_INVALID_JSON;

  /* req_id es obligatorio: sin el no se puede correlacionar el ACK. */
  const cJSON *req_id = cJSON_GetObjectItemCaseSensitive(root, "req_id");
  if (!cJSON_IsString(req_id) || !req_id_valid(req_id->valuestring))
    return ROI_ERR_INVALID_JSON;

  const cJSON *version_item =
      cJSON_GetObjectItemCaseSensitive(root, "version");
  double version_value = 0;
  if (!json_number(version_item, &version_value) || version_value < 0 ||
      version_value > 2147483647.0 || floor(version_value) != version_value)
    return ROI_ERR_INVALID_JSON;

  const cJSON *array = cJSON_GetObjectItemCaseSensitive(root, "rois");
  if (!cJSON_IsArray(array))
    return ROI_ERR_INVALID_JSON;

  int length = cJSON_GetArraySize(array);
  if (length < 0 || length > (int)ROI_MAX)
    return ROI_ERR_TOO_MANY;

  memset(rois, 0, ROI_MAX * sizeof(roi_def_t));
  for (int i = 0; i < length; ++i) {
    const cJSON *item = cJSON_GetArrayItem(array, i);
    if (!cJSON_IsObject(item))
      return ROI_ERR_INVALID_JSON;

    const cJSON *id = cJSON_GetObjectItemCaseSensitive(item, "id");
    if (!cJSON_IsString(id) || !roi_uuid_valid(id->valuestring))
      return ROI_ERR_INVALID_JSON;
    memcpy(rois[i].id, id->valuestring, ROI_ID_SIZE);

    double x, y, w, h;
    if (!json_number(cJSON_GetObjectItemCaseSensitive(item, "x"), &x) ||
        !json_number(cJSON_GetObjectItemCaseSensitive(item, "y"), &y) ||
        !json_number(cJSON_GetObjectItemCaseSensitive(item, "w"), &w) ||
        !json_number(cJSON_GetObjectItemCaseSensitive(item, "h"), &h))
      return ROI_ERR_INVALID_JSON;
    rois[i].x = (float)x;
    rois[i].y = (float)y;
    rois[i].w = (float)w;
    rois[i].h = (float)h;
  }

  *count = (size_t)length;
  *version = (uint32_t)version_value;
  return roi_validate_set(rois, *count);
}

bool roi_cmd_ack_ok(char *out, size_t out_size, const char *req_id,
                    uint32_t version, size_t count) {
  if (!out || !out_size || !req_id)
    return false;
  int written = snprintf(
      out, out_size,
      "{\"req_id\":\"%s\",\"type\":\"roi_ack\",\"ok\":true,"
      "\"version\":%lu,\"count\":%u}",
      req_id, (unsigned long)version, (unsigned)count);
  return written > 0 && (size_t)written < out_size;
}

bool roi_cmd_ack_error(char *out, size_t out_size, const char *req_id,
                       const char *error_code) {
  if (!out || !out_size || !req_id || !error_code)
    return false;
  int written = snprintf(
      out, out_size,
      "{\"req_id\":\"%s\",\"type\":\"roi_ack\",\"ok\":false,"
      "\"error\":\"%s\"}",
      req_id, error_code);
  return written > 0 && (size_t)written < out_size;
}
