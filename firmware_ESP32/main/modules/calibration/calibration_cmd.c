#include "calibration_cmd.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Utilidades de validación                                            */
/* ------------------------------------------------------------------ */

#define CAL_TS_LEN 24U /* YYYY-MM-DDTHH:MM:SS.mmmZ */
#define CAL_INT32_MAX 2147483647.0

static bool is_hex(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}
static bool is_lower_hex(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}
static bool is_digit(char c) { return c >= '0' && c <= '9'; }

bool cal_uuid_valid(const char *value) {
  if (!value || strlen(value) != CAL_UUID_LEN)
    return false;
  for (size_t i = 0; i < CAL_UUID_LEN; ++i) {
    if (i == 8U || i == 13U || i == 18U || i == 23U) {
      if (value[i] != '-')
        return false;
    } else if (!is_hex(value[i])) {
      return false;
    }
  }
  return true;
}

static bool lower_hex_n(const char *value, size_t n) {
  if (!value || strlen(value) != n)
    return false;
  for (size_t i = 0; i < n; ++i)
    if (!is_lower_hex(value[i]))
      return false;
  return true;
}
bool cal_hash_hex_valid(const char *value) {
  return lower_hex_n(value, CAL_HASH_HEX_LEN);
}
bool cal_crc_hex_valid(const char *value) {
  return lower_hex_n(value, CAL_CRC_HEX_LEN);
}

static int two(const char *p) { return (p[0] - '0') * 10 + (p[1] - '0'); }

bool cal_timestamp_valid(const char *v) {
  if (!v || strlen(v) != CAL_TS_LEN)
    return false;
  static const char shape[] = "dddd-dd-ddTdd:dd:dd.dddZ";
  for (size_t i = 0; i < CAL_TS_LEN; ++i) {
    if (shape[i] == 'd' ? !is_digit(v[i]) : v[i] != shape[i])
      return false;
  }
  int month = two(v + 5), day = two(v + 8), hour = two(v + 11),
      minute = two(v + 14), second = two(v + 17);
  return month >= 1 && month <= 12 && day >= 1 && day <= 31 && hour <= 23 &&
         minute <= 59 && second <= 60;
}

const char *cal_cmd_error_code(cal_cmd_err_t err) {
  switch (err) {
  case CAL_CMD_OK:
    return "ok";
  case CAL_CMD_E_INVALID_JSON:
  case CAL_CMD_E_TOO_LARGE:
  case CAL_CMD_E_UNKNOWN_TYPE:
  case CAL_CMD_E_INVALID_REQUEST:
    return "invalid_request";
  case CAL_CMD_E_UNSUPPORTED_SCHEMA:
  case CAL_CMD_E_UNSUPPORTED_FORMAT:
    return "unsupported_format";
  case CAL_CMD_E_SESSION_EXPIRED:
    return "session_expired";
  case CAL_CMD_E_BUSY:
    return "busy";
  case CAL_CMD_E_OPERATION_CONFLICT:
    return "operation_conflict";
  case CAL_CMD_E_ROI_NOT_FOUND:
    return "roi_not_found";
  case CAL_CMD_E_CONFIG_MISMATCH:
    return "config_mismatch";
  case CAL_CMD_E_CONTEXT_MISMATCH:
    return "context_mismatch";
  default:
    return "invalid_request";
  }
}

const char *cal_action_name(cal_action_t a) {
  switch (a) {
  case CAL_ACTION_CAPTURE:
    return "capture";
  case CAL_ACTION_COMMIT:
    return "commit";
  case CAL_ACTION_CANCEL:
    return "cancel";
  case CAL_ACTION_STATUS:
    return "status";
  default:
    return "unknown";
  }
}

const char *cal_kind_name(cal_kind_t k) {
  return k == CAL_KIND_EMPTY ? "empty" : k == CAL_KIND_FULL ? "full" : "none";
}

/* Etapa que falló, para el ACK de error. */
static const char *error_stage(cal_cmd_err_t err) {
  switch (err) {
  case CAL_CMD_E_BUSY:
    return "queue";
  case CAL_CMD_E_OPERATION_CONFLICT:
    return "replay";
  case CAL_CMD_E_ROI_NOT_FOUND:
  case CAL_CMD_E_CONFIG_MISMATCH:
  case CAL_CMD_E_CONTEXT_MISMATCH:
  case CAL_CMD_E_SESSION_EXPIRED:
    return "context";
  default:
    return "parse";
  }
}

/* ------------------------------------------------------------------ */
/* Extracción de identidad con JSON malformado                          */
/* ------------------------------------------------------------------ */

/* Busca "key":"<uuid>" sin cJSON, para poder responder un ACK de error aunque
 * el JSON no se pueda parsear. Solo acepta un UUID completo y válido. */
static void raw_uuid(const char *data, size_t len, const char *key, char *out) {
  out[0] = '\0';
  if (!data)
    return;
  char needle[24];
  int n = snprintf(needle, sizeof(needle), "\"%s\"", key);
  if (n <= 0 || (size_t)n >= sizeof(needle))
    return;
  size_t nl = (size_t)n;
  for (size_t i = 0; i + nl <= len; ++i) {
    if (memcmp(data + i, needle, nl) != 0)
      continue;
    size_t p = i + nl;
    while (p < len && (data[p] == ' ' || data[p] == '\t'))
      ++p;
    if (p >= len || data[p] != ':')
      continue;
    ++p;
    while (p < len && (data[p] == ' ' || data[p] == '\t'))
      ++p;
    if (p >= len || data[p] != '"')
      continue;
    ++p;
    if (p + CAL_UUID_LEN + 1U > len || data[p + CAL_UUID_LEN] != '"')
      return;
    char tmp[CAL_UUID_SIZE];
    memcpy(tmp, data + p, CAL_UUID_LEN);
    tmp[CAL_UUID_LEN] = '\0';
    if (cal_uuid_valid(tmp))
      memcpy(out, tmp, CAL_UUID_SIZE);
    return;
  }
}

static cal_action_t action_from_type(const char *type) {
  if (!type)
    return CAL_ACTION_NONE;
  if (!strcmp(type, "calibrate"))
    return CAL_ACTION_CAPTURE;
  if (!strcmp(type, "calibration.commit"))
    return CAL_ACTION_COMMIT;
  if (!strcmp(type, "calibration.cancel"))
    return CAL_ACTION_CANCEL;
  if (!strcmp(type, "calibration.status"))
    return CAL_ACTION_STATUS;
  return CAL_ACTION_NONE;
}

static void raw_action_hint(const char *data, size_t len, cal_cmd_status_t *st) {
  static const char *const types[] = {"calibrate", "calibration.commit",
                                      "calibration.cancel",
                                      "calibration.status"};
  static const cal_action_t acts[] = {CAL_ACTION_CAPTURE, CAL_ACTION_COMMIT,
                                      CAL_ACTION_CANCEL, CAL_ACTION_STATUS};
  for (size_t t = 0; t < 4; ++t) {
    char needle[40];
    int n = snprintf(needle, sizeof(needle), "\"%s\"", types[t]);
    if (n <= 0)
      continue;
    for (size_t i = 0; i + (size_t)n <= len; ++i) {
      if (memcmp(data + i, needle, (size_t)n) == 0) {
        st->action_hint = acts[t];
        return;
      }
    }
  }
}

static void status_reset(cal_cmd_status_t *st) {
  memset(st, 0, sizeof(*st));
  st->err = CAL_CMD_OK;
}

/* ------------------------------------------------------------------ */
/* Lectura estricta de campos cJSON                                     */
/* ------------------------------------------------------------------ */

static const cJSON *field(const cJSON *root, const char *name) {
  return cJSON_GetObjectItemCaseSensitive(root, name);
}

/* Copia un string obligatorio con validador. Falla si falta/no es string. */
typedef bool (*validator_t)(const char *);
static bool take_str(const cJSON *root, const char *name, validator_t ok,
                     char *out, size_t out_size) {
  const cJSON *item = field(root, name);
  if (!cJSON_IsString(item) || !item->valuestring)
    return false;
  size_t n = strlen(item->valuestring);
  if (n + 1U > out_size || (ok && !ok(item->valuestring)))
    return false;
  memcpy(out, item->valuestring, n + 1U);
  return true;
}

static bool take_uint(const cJSON *root, const char *name, double min_value,
                      uint32_t *out) {
  const cJSON *item = field(root, name);
  if (!cJSON_IsNumber(item))
    return false;
  double v = item->valuedouble;
  if (!isfinite(v) || floor(v) != v || v < min_value || v > CAL_INT32_MAX)
    return false;
  *out = (uint32_t)v;
  return true;
}

/* Identidad común a las cuatro acciones. */
static bool take_identity(const cJSON *root, cal_cmd_t *cmd) {
  return take_str(root, "id", cal_uuid_valid, cmd->operation_id,
                  sizeof(cmd->operation_id)) &&
         take_str(root, "req_id", cal_uuid_valid, cmd->req_id,
                  sizeof(cmd->req_id)) &&
         take_str(root, "calibration_id", cal_uuid_valid, cmd->calibration_id,
                  sizeof(cmd->calibration_id)) &&
         take_str(root, "roi_id", cal_uuid_valid, cmd->roi_id,
                  sizeof(cmd->roi_id));
}

/* Destino opcional: si viene, debe ser un UUID válido. */
static bool take_optional_target(const cJSON *root, cal_cmd_t *cmd) {
  const cJSON *branch = field(root, "branch_id");
  const cJSON *node = field(root, "node_id");
  if (branch && !take_str(root, "branch_id", cal_uuid_valid, cmd->branch_id,
                          sizeof(cmd->branch_id)))
    return false;
  if (node && !take_str(root, "node_id", cal_uuid_valid, cmd->node_id,
                        sizeof(cmd->node_id)))
    return false;
  return true;
}

/* ------------------------------------------------------------------ */
/* Huella semántica (FNV-1a 32)                                         */
/* ------------------------------------------------------------------ */

static uint32_t fnv_bytes(uint32_t h, const void *data, size_t n) {
  const uint8_t *p = (const uint8_t *)data;
  while (n--) {
    h ^= *p++;
    h *= 16777619U;
  }
  return h;
}
static uint32_t fnv_str(uint32_t h, const char *s) {
  h = fnv_bytes(h, s, strlen(s));
  return fnv_bytes(h, "\x1f", 1); /* separador: evita ambigüedad entre campos */
}
static uint32_t fnv_u32(uint32_t h, uint32_t v) {
  uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16),
                  (uint8_t)(v >> 24)};
  return fnv_bytes(h, b, sizeof b);
}

uint32_t cal_cmd_fingerprint(const cal_cmd_t *c) {
  uint32_t h = 2166136261U;
  h = fnv_u32(h, (uint32_t)c->action);
  h = fnv_str(h, c->operation_id);
  h = fnv_str(h, c->calibration_id);
  h = fnv_str(h, c->roi_id);
  h = fnv_u32(h, c->calibration_version);
  h = fnv_u32(h, c->config_version);
  h = fnv_str(h, c->context_hash);
  h = fnv_u32(h, c->format_version);
  h = fnv_str(h, c->capture_id);
  h = fnv_u32(h, (uint32_t)c->kind);
  h = fnv_str(h, c->empty_capture_id);
  h = fnv_str(h, c->full_capture_id);
  h = fnv_str(h, c->pair_hash);
  /* req_id, issued_at y expires_at pertenecen al intento: NO entran. */
  return h;
}

/* ------------------------------------------------------------------ */
/* Parseo                                                               */
/* ------------------------------------------------------------------ */

static void remember_identity(const cJSON *root, cal_cmd_status_t *st) {
  static const struct {
    const char *key;
    size_t off;
  } keys[] = {{"id", offsetof(cal_cmd_status_t, operation_id)},
              {"req_id", offsetof(cal_cmd_status_t, req_id)},
              {"calibration_id", offsetof(cal_cmd_status_t, calibration_id)},
              {"roi_id", offsetof(cal_cmd_status_t, roi_id)}};
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    const cJSON *item = field(root, keys[i].key);
    if (cJSON_IsString(item) && cal_uuid_valid(item->valuestring))
      memcpy((char *)st + keys[i].off, item->valuestring, CAL_UUID_SIZE);
  }
}

static cal_cmd_err_t fail(cal_cmd_status_t *st, cal_cmd_err_t err) {
  if (st)
    st->err = err;
  return err;
}

cal_cmd_err_t cal_cmd_parse_json(const cJSON *root, cal_cmd_t *cmd,
                                 cal_cmd_status_t *status) {
  cal_cmd_status_t local;
  cal_cmd_status_t *st = status ? status : &local;
  if (!status)
    status_reset(&local);
  if (!root || !cmd || !cJSON_IsObject(root))
    return fail(st, CAL_CMD_E_INVALID_JSON);

  remember_identity(root, st);
  const cJSON *type = field(root, "type");
  if (!cJSON_IsString(type) || !type->valuestring)
    return fail(st, CAL_CMD_E_INVALID_REQUEST);
  cal_action_t action = action_from_type(type->valuestring);
  st->action_hint = action;
  if (action == CAL_ACTION_NONE)
    return fail(st, CAL_CMD_E_UNKNOWN_TYPE);

  /* schema_version: ausente = inválido (no se asume 1); otro valor = no
   * soportado. */
  uint32_t schema = 0;
  if (!take_uint(root, "schema_version", 0, &schema))
    return fail(st, CAL_CMD_E_INVALID_REQUEST);
  if (schema != CAL_CMD_SCHEMA_VERSION)
    return fail(st, CAL_CMD_E_UNSUPPORTED_SCHEMA);

  cal_cmd_t next;
  memset(&next, 0, sizeof(next));
  next.action = action;
  if (!take_identity(root, &next) || !take_optional_target(root, &next) ||
      !take_uint(root, "calibration_version", 1, &next.calibration_version))
    return fail(st, CAL_CMD_E_INVALID_REQUEST);

  if (action == CAL_ACTION_CAPTURE || action == CAL_ACTION_COMMIT) {
    uint32_t format = 0;
    if (!take_uint(root, "config_version", 0, &next.config_version) ||
        !take_str(root, "context_hash", cal_hash_hex_valid, next.context_hash,
                  sizeof(next.context_hash)) ||
        !take_uint(root, "format_version", 0, &format) ||
        !take_str(root, "issued_at", cal_timestamp_valid, next.issued_at,
                  sizeof(next.issued_at)) ||
        !take_str(root, "expires_at", cal_timestamp_valid, next.expires_at,
                  sizeof(next.expires_at)))
      return fail(st, CAL_CMD_E_INVALID_REQUEST);
    if (format != CAL_CMD_FORMAT_VERSION)
      return fail(st, CAL_CMD_E_UNSUPPORTED_FORMAT);
    next.format_version = (uint8_t)format;

    if (action == CAL_ACTION_CAPTURE) {
      const cJSON *kind = field(root, "kind");
      if (!take_str(root, "capture_id", cal_uuid_valid, next.capture_id,
                    sizeof(next.capture_id)) ||
          !cJSON_IsString(kind) || !kind->valuestring)
        return fail(st, CAL_CMD_E_INVALID_REQUEST);
      if (!strcmp(kind->valuestring, "empty"))
        next.kind = CAL_KIND_EMPTY;
      else if (!strcmp(kind->valuestring, "full"))
        next.kind = CAL_KIND_FULL;
      else
        return fail(st, CAL_CMD_E_INVALID_REQUEST);
    } else {
      if (!take_str(root, "empty_capture_id", cal_uuid_valid,
                    next.empty_capture_id, sizeof(next.empty_capture_id)) ||
          !take_str(root, "full_capture_id", cal_uuid_valid,
                    next.full_capture_id, sizeof(next.full_capture_id)) ||
          !take_str(root, "pair_hash", cal_hash_hex_valid, next.pair_hash,
                    sizeof(next.pair_hash)))
        return fail(st, CAL_CMD_E_INVALID_REQUEST);
      /* Una pareja con la misma captura en ambos lados nunca es válida. */
      if (!strcmp(next.empty_capture_id, next.full_capture_id))
        return fail(st, CAL_CMD_E_INVALID_REQUEST);
    }
  } else if (action == CAL_ACTION_CANCEL) {
    if (!take_str(root, "context_hash", cal_hash_hex_valid, next.context_hash,
                  sizeof(next.context_hash)))
      return fail(st, CAL_CMD_E_INVALID_REQUEST);
  }
  /* calibration.status lleva identidad y versión, también obligatoria en ACK. */

  next.semantic_fingerprint = cal_cmd_fingerprint(&next);
  *cmd = next;
  st->err = CAL_CMD_OK;
  return CAL_CMD_OK;
}

cal_cmd_err_t cal_cmd_parse(const char *payload, size_t length, cal_cmd_t *cmd,
                            cal_cmd_status_t *status) {
  cal_cmd_status_t local;
  cal_cmd_status_t *st = status ? status : &local;
  status_reset(st);
  if (!payload || !length || !cmd)
    return fail(st, CAL_CMD_E_INVALID_JSON);

  /* Identidad "a la mala": útil si el resto falla. */
  size_t scan = length > CAL_CMD_MAX_BYTES ? CAL_CMD_MAX_BYTES : length;
  raw_uuid(payload, scan, "id", st->operation_id);
  raw_uuid(payload, scan, "req_id", st->req_id);
  raw_uuid(payload, scan, "calibration_id", st->calibration_id);
  raw_uuid(payload, scan, "roi_id", st->roi_id);
  raw_action_hint(payload, scan, st);

  if (length > CAL_CMD_MAX_BYTES)
    return fail(st, CAL_CMD_E_TOO_LARGE);

  cJSON *root = cJSON_ParseWithLength(payload, length);
  if (!root)
    return fail(st, CAL_CMD_E_INVALID_JSON);
  cal_cmd_err_t err = cal_cmd_parse_json(root, cmd, st);
  cJSON_Delete(root);
  return err;
}

/* ------------------------------------------------------------------ */
/* Contexto, destino y repetición                                       */
/* ------------------------------------------------------------------ */

cal_cmd_err_t cal_cmd_check_context(const cal_cmd_t *cmd, bool known_roi,
                                    uint32_t applied_version) {
  if (!cmd)
    return CAL_CMD_E_INVALID_REQUEST;
  if (!known_roi)
    return CAL_CMD_E_ROI_NOT_FOUND;
  if ((cmd->action == CAL_ACTION_CAPTURE || cmd->action == CAL_ACTION_COMMIT) &&
      cmd->config_version != applied_version)
    return CAL_CMD_E_CONFIG_MISMATCH;
  return CAL_CMD_OK;
}

cal_cmd_err_t cal_cmd_check_target(const cal_cmd_t *cmd, const char *my_branch,
                                   const char *my_node) {
  if (!cmd)
    return CAL_CMD_E_INVALID_REQUEST;
  if (cmd->branch_id[0] && (!my_branch || strcmp(cmd->branch_id, my_branch)))
    return CAL_CMD_E_CONTEXT_MISMATCH;
  if (cmd->node_id[0] && (!my_node || strcmp(cmd->node_id, my_node)))
    return CAL_CMD_E_CONTEXT_MISMATCH;
  return CAL_CMD_OK;
}

cal_cmd_err_t cal_cmd_check_replay(uint32_t stored_fingerprint,
                                   const cal_cmd_t *cmd) {
  if (!cmd)
    return CAL_CMD_E_INVALID_REQUEST;
  return stored_fingerprint == cmd->semantic_fingerprint
             ? CAL_CMD_OK
             : CAL_CMD_E_OPERATION_CONFLICT;
}

/* ------------------------------------------------------------------ */
/* ACK                                                                  */
/* ------------------------------------------------------------------ */

/* Acumulador que nunca desborda ni trunca: si no cabe, marca fallo. */
typedef struct {
  char *buf;
  size_t cap;
  size_t len;
  bool ok;
} out_t;

static void out_init(out_t *o, char *buf, size_t size) {
  size_t limit = size < CAL_ACK_JSON_SIZE ? size : CAL_ACK_JSON_SIZE;
  o->buf = buf;
  o->cap = limit;
  o->len = 0;
  o->ok = buf && limit > 1U;
  if (o->ok)
    buf[0] = '\0';
}

static void out_fmt(out_t *o, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void out_fmt(out_t *o, const char *fmt, ...) {
  if (!o->ok)
    return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(o->buf + o->len, o->cap - o->len, fmt, ap);
  va_end(ap);
  if (n < 0 || (size_t)n >= o->cap - o->len) {
    o->ok = false;
    o->buf[0] = '\0';
    return;
  }
  o->len += (size_t)n;
}

static void out_head(out_t *o, const cal_cmd_t *c, bool ok, const char *status) {
  out_fmt(o,
          "{\"schema_version\":%u,\"type\":\"cal_result\",\"id\":\"%s\","
          "\"req_id\":\"%s\",\"ok\":%s,\"action\":\"%s\",\"status\":\"%s\","
          "\"calibration_id\":\"%s\",\"roi_id\":\"%s\"",
          (unsigned)CAL_CMD_SCHEMA_VERSION, c->operation_id, c->req_id,
          ok ? "true" : "false", cal_action_name(c->action), status,
          c->calibration_id, c->roi_id);
}

static void out_versions(out_t *o, const cal_cmd_t *c) {
  out_fmt(o,
          ",\"calibration_version\":%lu,\"config_version\":%lu,"
          "\"context_hash\":\"%s\",\"format_version\":%u",
          (unsigned long)c->calibration_version,
          (unsigned long)c->config_version, c->context_hash,
          (unsigned)c->format_version);
}

bool cal_ack_capture(char *out, size_t out_size, const cal_cmd_t *cmd,
                     const cal_capture_ack_t *cap) {
  if (!cmd || !cap || cmd->action != CAL_ACTION_CAPTURE || !cap->clock_quality)
    return false;
  if (!cal_hash_hex_valid(cap->image_sha256))
    return false;
  out_t o;
  out_init(&o, out, out_size);
  out_head(&o, cmd, true, "stored");
  out_versions(&o, cmd);
  out_fmt(&o,
          ",\"capture_id\":\"%s\",\"kind\":\"%s\",\"descriptor_crc\":\"%08lx\","
          "\"image_sha256\":\"%s\",\"image_bytes\":%lu,\"width\":%u,"
          "\"height\":%u,\"quality\":{\"mean\":%u,\"contrast\":%u,"
          "\"clipped_pct\":%u},\"clock_quality\":\"%s\",\"uptime_ms\":%lu,"
          "\"persisted\":%s}",
          cmd->capture_id, cal_kind_name(cmd->kind),
          (unsigned long)cap->descriptor_crc, cap->image_sha256,
          (unsigned long)cap->image_bytes, (unsigned)cap->width,
          (unsigned)cap->height, (unsigned)cap->quality.mean,
          (unsigned)cap->quality.contrast, (unsigned)cap->quality.clipped_pct,
          cap->clock_quality, (unsigned long)cap->uptime_ms,
          cap->persisted ? "true" : "false");
  return o.ok;
}

bool cal_ack_commit(char *out, size_t out_size, const cal_cmd_t *cmd,
                    uint32_t active_version, bool persisted) {
  if (!cmd || cmd->action != CAL_ACTION_COMMIT)
    return false;
  out_t o;
  out_init(&o, out, out_size);
  out_head(&o, cmd, true, "applied");
  out_versions(&o, cmd);
  out_fmt(&o,
          ",\"empty_capture_id\":\"%s\",\"full_capture_id\":\"%s\","
          "\"pair_hash\":\"%s\",\"persisted\":%s,"
          "\"active_calibration_version\":%lu}",
          cmd->empty_capture_id, cmd->full_capture_id, cmd->pair_hash,
          persisted ? "true" : "false", (unsigned long)active_version);
  return o.ok;
}

bool cal_ack_cancel(char *out, size_t out_size, const cal_cmd_t *cmd) {
  if (!cmd || cmd->action != CAL_ACTION_CANCEL)
    return false;
  out_t o;
  out_init(&o, out, out_size);
  out_head(&o, cmd, true, "cancelled");
  out_fmt(&o, ",\"calibration_version\":%lu,\"context_hash\":\"%s\"}",
          (unsigned long)cmd->calibration_version, cmd->context_hash);
  return o.ok;
}

bool cal_ack_status(char *out, size_t out_size, const cal_cmd_t *cmd,
                    const char *status_body) {
  if (!cmd || cmd->action != CAL_ACTION_STATUS || !status_body)
    return false;
  out_t o;
  out_init(&o, out, out_size);
  out_head(&o, cmd, true, "reported");
  out_fmt(&o, ",\"calibration_version\":%lu",
          (unsigned long)cmd->calibration_version);
  out_fmt(&o, "%s%s}", status_body[0] ? "," : "", status_body);
  return o.ok;
}

bool cal_ack_error(char *out, size_t out_size, const cal_cmd_status_t *st,
                   cal_cmd_err_t err) {
  if (!st || err == CAL_CMD_OK)
    return false;
  out_t o;
  out_init(&o, out, out_size);
  out_fmt(&o, "{\"schema_version\":%u,\"type\":\"cal_result\"",
          (unsigned)CAL_CMD_SCHEMA_VERSION);
  /* Solo identidad realmente extraída: nada se completa con valores supuestos. */
  if (st->operation_id[0])
    out_fmt(&o, ",\"id\":\"%s\"", st->operation_id);
  if (st->req_id[0])
    out_fmt(&o, ",\"req_id\":\"%s\"", st->req_id);
  out_fmt(&o, ",\"ok\":false,\"action\":\"%s\"", cal_action_name(st->action_hint));
  if (st->calibration_id[0])
    out_fmt(&o, ",\"calibration_id\":\"%s\"", st->calibration_id);
  if (st->roi_id[0])
    out_fmt(&o, ",\"roi_id\":\"%s\"", st->roi_id);
  out_fmt(&o, ",\"status\":\"error\",\"code\":\"%s\",\"stage\":\"%s\"}",
          cal_cmd_error_code(err), error_stage(err));
  return o.ok;
}
