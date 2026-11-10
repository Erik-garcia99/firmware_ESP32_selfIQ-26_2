/* Pruebas host de calibration_cmd (E5.1). Sin ESP-IDF. */
#include <stdio.h>
#include <string.h>

#include "../main/modules/calibration/calibration_cmd.h"

static int g_fail = 0, g_run = 0;
#define CHECK(c)                                                              \
  do {                                                                        \
    ++g_run;                                                                  \
    if (!(c)) {                                                               \
      ++g_fail;                                                               \
      printf("FALLO %s:%d: %s\n", __FILE__, __LINE__, #c);                    \
    }                                                                         \
  } while (0)

#define OP "11111111-2222-4333-8444-555555555555"
#define REQ "aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee"
#define REQ2 "99999999-bbbb-4ccc-8ddd-eeeeeeeeeeee"
#define CAL "9f1c6a4e-0f6d-4a63-9a1b-2a5c7d1e0b11"
#define ROI "3a3e7d54-9b9f-4d4e-8c6b-1f0a2b3c4d5e"
#define CAP "c0ffee00-0000-4000-8000-000000000001"
#define CAP2 "c0ffee00-0000-4000-8000-000000000002"
#define H64 "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
#define H64B "fedcba9876543210fedcba9876543210fedcba9876543210fedcba9876543210"

static const char *CAPTURE_FMT =
    "{\"schema_version\":1,\"type\":\"calibrate\",\"id\":\"" OP
    "\",\"req_id\":\"%s\",\"calibration_id\":\"" CAL
    "\",\"calibration_version\":12,\"roi_id\":\"" ROI
    "\",\"config_version\":7,\"context_hash\":\"" H64
    "\",\"capture_id\":\"" CAP
    "\",\"kind\":\"%s\",\"format_version\":1,"
    "\"issued_at\":\"2026-11-08T18:04:00.000Z\","
    "\"expires_at\":\"2026-11-08T18:04:10.000Z\"}";

static const char *COMMIT =
    "{\"schema_version\":1,\"type\":\"calibration.commit\",\"id\":\"" OP
    "\",\"req_id\":\"" REQ "\",\"calibration_id\":\"" CAL
    "\",\"calibration_version\":12,\"roi_id\":\"" ROI
    "\",\"config_version\":7,\"context_hash\":\"" H64
    "\",\"empty_capture_id\":\"" CAP "\",\"full_capture_id\":\"" CAP2
    "\",\"pair_hash\":\"" H64B
    "\",\"format_version\":1,\"issued_at\":\"2026-11-08T18:10:00.000Z\","
    "\"expires_at\":\"2026-11-08T18:10:10.000Z\"}";

static const char *CANCEL =
    "{\"schema_version\":1,\"type\":\"calibration.cancel\",\"id\":\"" OP
    "\",\"req_id\":\"" REQ "\",\"calibration_id\":\"" CAL
    "\",\"roi_id\":\"" ROI "\",\"context_hash\":\"" H64 "\"}";

static const char *STATUS =
    "{\"schema_version\":1,\"type\":\"calibration.status\",\"id\":\"" OP
    "\",\"req_id\":\"" REQ "\",\"calibration_id\":\"" CAL
    "\",\"roi_id\":\"" ROI "\"}";

static cal_cmd_err_t parse(const char *s, cal_cmd_t *c, cal_cmd_status_t *st) {
  return cal_cmd_parse(s, strlen(s), c, st);
}

static void capture_json(char *out, size_t n, const char *req, const char *kind) {
  snprintf(out, n, CAPTURE_FMT, req, kind);
}

/* Reemplaza la primera aparición de `from` por `to` (misma longitud o no). */
static void swap(char *buf, size_t cap, const char *from, const char *to) {
  char *p = strstr(buf, from);
  if (!p) {
    printf("swap: no se encontro '%s'\n", from);
    ++g_fail;
    return;
  }
  char tail[2048];
  snprintf(tail, sizeof tail, "%s", p + strlen(from));
  snprintf(p, cap - (size_t)(p - buf), "%s%s", to, tail);
}

static void test_utils(void) {
  CHECK(cal_uuid_valid(OP));
  CHECK(!cal_uuid_valid("no-es-uuid"));
  CHECK(!cal_uuid_valid("11111111-2222-4333-8444-55555555555"));   /* 35 */
  CHECK(!cal_uuid_valid("11111111-2222-4333-8444-5555555555555")); /* 37 */
  CHECK(!cal_uuid_valid("11111111_2222-4333-8444-555555555555"));
  CHECK(!cal_uuid_valid("1111111g-2222-4333-8444-555555555555"));
  CHECK(!cal_uuid_valid(NULL));
  CHECK(cal_hash_hex_valid(H64));
  CHECK(!cal_hash_hex_valid("ABCDEF"));
  CHECK(!cal_hash_hex_valid("0123456789ABCDEF0123456789abcdef0123456789abcdef0123456789abcdef"));
  CHECK(cal_crc_hex_valid("deadbeef"));
  CHECK(!cal_crc_hex_valid("DEADBEEF"));
  CHECK(!cal_crc_hex_valid("deadbee"));
  CHECK(cal_timestamp_valid("2026-11-08T18:04:00.000Z"));
  CHECK(!cal_timestamp_valid("2026-11-08T18:04:00Z"));
  CHECK(!cal_timestamp_valid("2026-13-08T18:04:00.000Z"));
  CHECK(!cal_timestamp_valid("2026-11-08 18:04:00.000Z"));
  CHECK(!cal_timestamp_valid("2026-11-08T25:04:00.000Z"));
}

static void test_parse_valid(void) {
  char js[CAL_CMD_MAX_BYTES];
  cal_cmd_t c;
  cal_cmd_status_t st;

  capture_json(js, sizeof js, REQ, "empty");
  CHECK(parse(js, &c, &st) == CAL_CMD_OK);
  CHECK(c.action == CAL_ACTION_CAPTURE && c.kind == CAL_KIND_EMPTY);
  CHECK(!strcmp(c.operation_id, OP) && !strcmp(c.req_id, REQ));
  CHECK(!strcmp(c.capture_id, CAP) && c.calibration_version == 12);
  CHECK(c.config_version == 7 && c.format_version == 1);
  CHECK(!strcmp(c.context_hash, H64));

  capture_json(js, sizeof js, REQ, "full");
  CHECK(parse(js, &c, &st) == CAL_CMD_OK && c.kind == CAL_KIND_FULL);

  CHECK(parse(COMMIT, &c, &st) == CAL_CMD_OK && c.action == CAL_ACTION_COMMIT);
  CHECK(!strcmp(c.empty_capture_id, CAP) && !strcmp(c.full_capture_id, CAP2));
  CHECK(!strcmp(c.pair_hash, H64B));

  CHECK(parse(CANCEL, &c, &st) == CAL_CMD_OK && c.action == CAL_ACTION_CANCEL);
  CHECK(!strcmp(c.context_hash, H64) && c.calibration_version == 0);

  CHECK(parse(STATUS, &c, &st) == CAL_CMD_OK && c.action == CAL_ACTION_STATUS);
  CHECK(c.context_hash[0] == '\0' && c.config_version == 0);
}

static void test_parse_rejects(void) {
  char js[CAL_CMD_MAX_BYTES], tmp[CAL_CMD_MAX_BYTES];
  cal_cmd_t c;
  cal_cmd_status_t st;

  /* accion desconocida */
  capture_json(js, sizeof js, REQ, "empty");
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"calibrate\"", "\"calibrate.now\"");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_UNKNOWN_TYPE);
  CHECK(!strcmp(st.operation_id, OP) && !strcmp(st.req_id, REQ));

  /* schema desconocido y ausente */
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"schema_version\":1", "\"schema_version\":2");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_UNSUPPORTED_SCHEMA);
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"schema_version\":1,", "");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* format_version distinto */
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"format_version\":1", "\"format_version\":2");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_UNSUPPORTED_FORMAT);

  /* UUID malformado en cada campo de identidad */
  const char *uuids[] = {OP, CAL, ROI, CAP};
  for (size_t i = 0; i < 4; ++i) {
    snprintf(tmp, sizeof tmp, "%s", js);
    swap(tmp, sizeof tmp, uuids[i], "xxxxxxxx-2222-4333-8444-555555555555");
    CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  }
  /* req_id malformado */
  capture_json(tmp, sizeof tmp, "no-uuid", "empty");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  CHECK(st.req_id[0] == '\0'); /* no se inventa un req_id */

  /* kind invalido */
  capture_json(tmp, sizeof tmp, REQ, "half");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  capture_json(tmp, sizeof tmp, REQ, "lleno"); /* "llena/lleno" no se usa */
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* context_hash en mayusculas / corto */
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, H64, "ABCDEF");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* version fraccionaria, negativa o cero (calibration_version >= 1) */
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"calibration_version\":12", "\"calibration_version\":0");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"config_version\":7", "\"config_version\":7.5");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"config_version\":7", "\"config_version\":-1");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* timestamps malformados */
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "2026-11-08T18:04:00.000Z", "ayer");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* faltan campos obligatorios: no se completan */
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"context_hash\":\"" H64 "\",", "");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  snprintf(tmp, sizeof tmp, "%s", js);
  swap(tmp, sizeof tmp, "\"capture_id\":\"" CAP "\",", "");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* commit con la misma captura en ambos lados */
  snprintf(tmp, sizeof tmp, "%s", COMMIT);
  swap(tmp, sizeof tmp, CAP2, CAP);
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  /* commit sin pair_hash */
  snprintf(tmp, sizeof tmp, "%s", COMMIT);
  swap(tmp, sizeof tmp, "\"pair_hash\":\"" H64B "\",", "");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  /* cancel sin context_hash */
  snprintf(tmp, sizeof tmp, "%s", CANCEL);
  swap(tmp, sizeof tmp, ",\"context_hash\":\"" H64 "\"", "");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  /* status sin roi_id */
  snprintf(tmp, sizeof tmp, "%s", STATUS);
  swap(tmp, sizeof tmp, ",\"roi_id\":\"" ROI "\"", "");
  CHECK(parse(tmp, &c, &st) == CAL_CMD_E_INVALID_REQUEST);

  /* basura / vacio / no-objeto */
  CHECK(parse("{no es json", &c, &st) == CAL_CMD_E_INVALID_JSON);
  CHECK(cal_cmd_parse("", 0, &c, &st) == CAL_CMD_E_INVALID_JSON);
  CHECK(cal_cmd_parse(NULL, 5, &c, &st) == CAL_CMD_E_INVALID_JSON);
  CHECK(parse("[1,2,3]", &c, &st) == CAL_CMD_E_INVALID_JSON);
  CHECK(parse("{\"type\":7}", &c, &st) == CAL_CMD_E_INVALID_REQUEST);
}

static void test_json_malformed_keeps_identity(void) {
  cal_cmd_t c;
  cal_cmd_status_t st;
  /* JSON roto, pero con id y req_id legibles: el ACK de error debe poder
   * correlacionarse. */
  const char *bad = "{\"type\":\"calibrate\",\"id\":\"" OP
                    "\",\"req_id\":\"" REQ "\",\"calibration_id\":\"" CAL
                    "\",\"roi_id\":\"" ROI "\", cortado";
  CHECK(parse(bad, &c, &st) == CAL_CMD_E_INVALID_JSON);
  CHECK(!strcmp(st.operation_id, OP) && !strcmp(st.req_id, REQ));
  CHECK(!strcmp(st.calibration_id, CAL) && !strcmp(st.roi_id, ROI));
  CHECK(st.action_hint == CAL_ACTION_CAPTURE);
  char ack[CAL_ACK_JSON_SIZE];
  CHECK(cal_ack_error(ack, sizeof ack, &st, st.err));
  CHECK(strstr(ack, "\"ok\":false") && strstr(ack, REQ));
  CHECK(strstr(ack, "\"code\":\"invalid_request\""));
}

static void test_too_large(void) {
  cal_cmd_t c;
  cal_cmd_status_t st;
  static char big[CAL_CMD_MAX_BYTES + 64];
  memset(big, ' ', sizeof big);
  size_t head = (size_t)snprintf(big, sizeof big,
      "{\"type\":\"calibrate\",\"id\":\"" OP "\",\"req_id\":\"" REQ "\"}");
  big[head] = ' ';
  CHECK(cal_cmd_parse(big, CAL_CMD_MAX_BYTES + 1U, &c, &st) == CAL_CMD_E_TOO_LARGE);
  CHECK(!strcmp(st.req_id, REQ)); /* aun asi se puede correlacionar */
}

static void test_target_and_context(void) {
  cal_cmd_t c;
  cal_cmd_status_t st;
  char js[CAL_CMD_MAX_BYTES];
  capture_json(js, sizeof js, REQ, "empty");

  /* sin branch/node en el cuerpo: el topico ya fijo el destino */
  CHECK(parse(js, &c, &st) == CAL_CMD_OK);
  CHECK(cal_cmd_check_target(&c, "b", "n") == CAL_CMD_OK);

  /* con destino en el cuerpo (como lo publica el gateway) */
  static char with[CAL_CMD_MAX_BYTES];
  snprintf(with, sizeof with, "%s", js);
  swap(with, sizeof with, "{\"schema_version\":1,",
       "{\"branch_id\":\"bbbbbbbb-0000-4000-8000-000000000000\","
       "\"node_id\":\"nnnnnnnn-0000-4000-8000-000000000000\",\"schema_version\":1,");
  /* 'nnnnnnnn' no es hex: debe rechazarse por UUID invalido */
  CHECK(parse(with, &c, &st) == CAL_CMD_E_INVALID_REQUEST);
  snprintf(with, sizeof with, "%s", js);
  swap(with, sizeof with, "{\"schema_version\":1,",
       "{\"branch_id\":\"b0000000-0000-4000-8000-000000000000\","
       "\"node_id\":\"a0000000-0000-4000-8000-000000000000\",\"schema_version\":1,");
  CHECK(parse(with, &c, &st) == CAL_CMD_OK);
  CHECK(cal_cmd_check_target(&c, "b0000000-0000-4000-8000-000000000000",
                             "a0000000-0000-4000-8000-000000000000") == CAL_CMD_OK);
  CHECK(cal_cmd_check_target(&c, "b0000000-0000-4000-8000-000000000000",
                             "a0000000-0000-4000-8000-000000000001") ==
        CAL_CMD_E_CONTEXT_MISMATCH);
  CHECK(cal_cmd_check_target(&c, "b0000000-0000-4000-8000-0000000000ff",
                             "a0000000-0000-4000-8000-000000000000") ==
        CAL_CMD_E_CONTEXT_MISMATCH);

  /* contexto ROI/version */
  CHECK(parse(js, &c, &st) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&c, true, 7) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&c, true, 8) == CAL_CMD_E_CONFIG_MISMATCH);
  CHECK(cal_cmd_check_context(&c, true, 6) == CAL_CMD_E_CONFIG_MISMATCH);
  CHECK(cal_cmd_check_context(&c, false, 7) == CAL_CMD_E_ROI_NOT_FOUND);
  /* cancel/status no llevan config_version: solo se comprueba el ROI */
  CHECK(parse(STATUS, &c, &st) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&c, true, 99) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&c, false, 99) == CAL_CMD_E_ROI_NOT_FOUND);
  CHECK(parse(CANCEL, &c, &st) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&c, true, 99) == CAL_CMD_OK);
}

static void test_replay_semantics(void) {
  char a[CAL_CMD_MAX_BYTES], b[CAL_CMD_MAX_BYTES];
  cal_cmd_t c1, c2, c3;
  cal_cmd_status_t st;

  capture_json(a, sizeof a, REQ, "empty");
  /* mismo comando, otro req_id y otras fechas: duplicado legitimo */
  capture_json(b, sizeof b, REQ2, "empty");
  swap(b, sizeof b, "2026-11-08T18:04:10.000Z", "2026-11-08T18:09:59.000Z");
  CHECK(parse(a, &c1, &st) == CAL_CMD_OK && parse(b, &c2, &st) == CAL_CMD_OK);
  CHECK(strcmp(c1.req_id, c2.req_id) != 0);
  CHECK(c1.semantic_fingerprint == c2.semantic_fingerprint);
  CHECK(cal_cmd_check_replay(c1.semantic_fingerprint, &c2) == CAL_CMD_OK);

  /* mismo operation_id con contenido logico distinto -> conflicto */
  capture_json(b, sizeof b, REQ, "full");
  CHECK(parse(b, &c3, &st) == CAL_CMD_OK);
  CHECK(c3.semantic_fingerprint != c1.semantic_fingerprint);
  CHECK(cal_cmd_check_replay(c1.semantic_fingerprint, &c3) == CAL_CMD_E_OPERATION_CONFLICT);

  /* otro capture_id, otra version o otro contexto tambien cambian la huella */
  capture_json(b, sizeof b, REQ, "empty");
  swap(b, sizeof b, CAP, CAP2);
  CHECK(parse(b, &c3, &st) == CAL_CMD_OK);
  CHECK(c3.semantic_fingerprint != c1.semantic_fingerprint);
  capture_json(b, sizeof b, REQ, "empty");
  swap(b, sizeof b, "\"config_version\":7", "\"config_version\":8");
  CHECK(parse(b, &c3, &st) == CAL_CMD_OK);
  CHECK(c3.semantic_fingerprint != c1.semantic_fingerprint);
  capture_json(b, sizeof b, REQ, "empty");
  swap(b, sizeof b, H64, H64B);
  CHECK(parse(b, &c3, &st) == CAL_CMD_OK);
  CHECK(c3.semantic_fingerprint != c1.semantic_fingerprint);
}

static void test_acks(void) {
  char js[CAL_CMD_MAX_BYTES], ack[CAL_ACK_JSON_SIZE];
  cal_cmd_t c;
  cal_cmd_status_t st;

  capture_json(js, sizeof js, REQ, "empty");
  CHECK(parse(js, &c, &st) == CAL_CMD_OK);
  cal_capture_ack_t cap = {.descriptor_crc = 0xdeadbeefU,
                           .image_bytes = 34211,
                           .width = 320,
                           .height = 240,
                           .quality = {121, 38, 1},
                           .clock_quality = "unknown",
                           .uptime_ms = 1234567,
                           .persisted = true};
  memcpy(cap.image_sha256, H64B, sizeof(H64B));
  CHECK(cal_ack_capture(ack, sizeof ack, &c, &cap));
  CHECK(strlen(ack) < CAL_ACK_JSON_SIZE);
  CHECK(strstr(ack, "\"type\":\"cal_result\""));
  CHECK(strstr(ack, "\"id\":\"" OP "\"") && strstr(ack, "\"req_id\":\"" REQ "\""));
  CHECK(strstr(ack, "\"action\":\"capture\"") && strstr(ack, "\"status\":\"stored\""));
  CHECK(strstr(ack, "\"descriptor_crc\":\"deadbeef\""));
  CHECK(strstr(ack, "\"persisted\":true"));
  CHECK(strstr(ack, "\"kind\":\"empty\""));
  CHECK(strstr(ack, "\"calibration_version\":12") && strstr(ack, "\"config_version\":7"));
  CHECK(strstr(ack, "\"context_hash\":\"" H64 "\""));
  /* el ACK se puede volver a parsear como JSON valido */
  cJSON *root = cJSON_Parse(ack);
  CHECK(root != NULL);
  cJSON_Delete(root);
  /* el JSON entrante de ejemplo mide menos que el limite */
  CHECK(strlen(js) < CAL_CMD_MAX_BYTES);
  printf("  tamano: calibrate=%zu B, ACK captura=%zu B\n", strlen(js), strlen(ack));

  /* el req_id del intento vigente se refleja en el ACK */
  capture_json(js, sizeof js, REQ2, "empty");
  CHECK(parse(js, &c, &st) == CAL_CMD_OK);
  CHECK(cal_ack_capture(ack, sizeof ack, &c, &cap));
  CHECK(strstr(ack, "\"req_id\":\"" REQ2 "\""));

  /* no cabe -> false, sin truncar */
  char tiny[100];
  CHECK(!cal_ack_capture(tiny, sizeof tiny, &c, &cap));
  CHECK(tiny[0] == '\0');
  /* sha invalido o clock nulo -> false */
  cal_capture_ack_t bad = cap;
  strcpy(bad.image_sha256, "xyz");
  CHECK(!cal_ack_capture(ack, sizeof ack, &c, &bad));
  bad = cap;
  bad.clock_quality = NULL;
  CHECK(!cal_ack_capture(ack, sizeof ack, &c, &bad));
  /* accion equivocada */
  CHECK(!cal_ack_commit(ack, sizeof ack, &c, 12, true));

  CHECK(parse(COMMIT, &c, &st) == CAL_CMD_OK);
  CHECK(cal_ack_commit(ack, sizeof ack, &c, 12, true));
  CHECK(strstr(ack, "\"action\":\"commit\"") && strstr(ack, "\"status\":\"applied\""));
  CHECK(strstr(ack, "\"persisted\":true") && strstr(ack, "\"active_calibration_version\":12"));
  CHECK(strstr(ack, "\"pair_hash\":\"" H64B "\""));
  root = cJSON_Parse(ack);
  CHECK(root != NULL);
  cJSON_Delete(root);
  printf("  tamano: commit=%zu B, ACK commit=%zu B\n", strlen(COMMIT), strlen(ack));

  CHECK(parse(CANCEL, &c, &st) == CAL_CMD_OK);
  CHECK(cal_ack_cancel(ack, sizeof ack, &c));
  CHECK(strstr(ack, "\"action\":\"cancel\"") && strstr(ack, "\"status\":\"cancelled\""));
  root = cJSON_Parse(ack);
  CHECK(root != NULL);
  cJSON_Delete(root);

  CHECK(parse(STATUS, &c, &st) == CAL_CMD_OK);
  CHECK(cal_ack_status(ack, sizeof ack, &c,
                       "\"active\":null,\"draft\":null,\"last_results\":[]"));
  CHECK(strstr(ack, "\"action\":\"status\"") && strstr(ack, "\"status\":\"reported\""));
  root = cJSON_Parse(ack);
  CHECK(root != NULL);
  cJSON_Delete(root);
  CHECK(cal_ack_status(ack, sizeof ack, &c, ""));
  root = cJSON_Parse(ack);
  CHECK(root != NULL);
  cJSON_Delete(root);
  /* un cuerpo de status demasiado grande no se trunca */
  char huge[CAL_ACK_JSON_SIZE];
  memset(huge, 'a', sizeof huge - 1);
  huge[sizeof huge - 1] = '\0';
  CHECK(!cal_ack_status(ack, sizeof ack, &c, huge));
}

static void test_error_acks(void) {
  char js[CAL_CMD_MAX_BYTES], ack[CAL_ACK_JSON_SIZE];
  cal_cmd_t c;
  cal_cmd_status_t st;
  capture_json(js, sizeof js, REQ, "empty");
  CHECK(parse(js, &c, &st) == CAL_CMD_OK);

  static const cal_cmd_err_t errs[] = {
      CAL_CMD_E_INVALID_JSON,   CAL_CMD_E_TOO_LARGE,
      CAL_CMD_E_UNKNOWN_TYPE,   CAL_CMD_E_UNSUPPORTED_SCHEMA,
      CAL_CMD_E_UNSUPPORTED_FORMAT, CAL_CMD_E_INVALID_REQUEST,
      CAL_CMD_E_SESSION_EXPIRED, CAL_CMD_E_BUSY,
      CAL_CMD_E_OPERATION_CONFLICT, CAL_CMD_E_ROI_NOT_FOUND,
      CAL_CMD_E_CONFIG_MISMATCH, CAL_CMD_E_CONTEXT_MISMATCH};
  size_t max_len = 0;
  for (size_t i = 0; i < sizeof errs / sizeof errs[0]; ++i) {
    st.err = errs[i];
    CHECK(cal_ack_error(ack, sizeof ack, &st, errs[i]));
    CHECK(strstr(ack, "\"ok\":false") && strstr(ack, "\"code\":\""));
    CHECK(strstr(ack, "\"req_id\":\"" REQ "\""));
    cJSON *root = cJSON_Parse(ack);
    CHECK(root != NULL);
    cJSON_Delete(root);
    if (strlen(ack) > max_len)
      max_len = strlen(ack);
  }
  CHECK(max_len < CAL_ACK_JSON_SIZE);
  printf("  tamano: ACK error maximo=%zu B\n", max_len);

  /* codigos estables */
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_BUSY), "busy"));
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_CONFIG_MISMATCH), "config_mismatch"));
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_CONTEXT_MISMATCH), "context_mismatch"));
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_ROI_NOT_FOUND), "roi_not_found"));
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_OPERATION_CONFLICT), "operation_conflict"));
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_UNSUPPORTED_FORMAT), "unsupported_format"));
  CHECK(!strcmp(cal_cmd_error_code(CAL_CMD_E_SESSION_EXPIRED), "session_expired"));
  CHECK(!cal_ack_error(ack, sizeof ack, &st, CAL_CMD_OK));

  /* identidad ausente: no se inventa */
  cal_cmd_status_t empty;
  memset(&empty, 0, sizeof empty);
  CHECK(cal_ack_error(ack, sizeof ack, &empty, CAL_CMD_E_INVALID_JSON));
  CHECK(!strstr(ack, "\"req_id\"") && !strstr(ack, "\"id\""));
  CHECK(!strstr(ack, "calibration_id") && !strstr(ack, "roi_id"));
}

// /* E5.2 — contexto: captura/commit exigen config_version */
// static cal_cmd_t mk(cal_action_t a, uint32_t cfg) {
//   cal_cmd_t c; memset(&c, 0, sizeof c);
//   c.action = a; c.config_version = cfg;
//   return c;
// }
/* =================================================================== */
/* E5.2 — contexto: qué exige cada acción respecto a ROI/config_version */
/*                                                                     */
/* Matriz: capture/commit exigen ROI conocido Y config_version exacta.  */
/*         cancel/status solo exigen ROI (no llevan config_version).    */
/* =================================================================== */

static cal_cmd_t ctx_cmd(cal_action_t a, uint32_t cfg) {
  cal_cmd_t c;
  memset(&c, 0, sizeof c);
  c.action = a;
  c.config_version = cfg;
  return c;
}

static void test_e52_context_matrix(void) {
  /* --- capture --- */
  cal_cmd_t cap8 = ctx_cmd(CAL_ACTION_CAPTURE, 8);
  CHECK(cal_cmd_check_context(&cap8, /*known_roi=*/true,  /*applied=*/8) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&cap8, true,  9) == CAL_CMD_E_CONFIG_MISMATCH);
  CHECK(cal_cmd_check_context(&cap8, true,  7) == CAL_CMD_E_CONFIG_MISMATCH);
  CHECK(cal_cmd_check_context(&cap8, false, 8) == CAL_CMD_E_ROI_NOT_FOUND);

  /* --- commit ---
   * Escenario central de E5.2: durante la calibración llegó un roi.set
   * que aplicó version 9; el commit original trae 8 → config_mismatch. */
  cal_cmd_t com8 = ctx_cmd(CAL_ACTION_COMMIT, 8);
  CHECK(cal_cmd_check_context(&com8, true,  8) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&com8, true,  9) == CAL_CMD_E_CONFIG_MISMATCH);
  CHECK(cal_cmd_check_context(&com8, false, 8) == CAL_CMD_E_ROI_NOT_FOUND);

  /* --- cancel: no lleva config_version; solo importa el ROI --- */
  cal_cmd_t can = ctx_cmd(CAL_ACTION_CANCEL, 0);
  CHECK(cal_cmd_check_context(&can, true,   0) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&can, true,  99) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&can, false,  0) == CAL_CMD_E_ROI_NOT_FOUND);

  /* --- status: igual que cancel --- */
  cal_cmd_t sta = ctx_cmd(CAL_ACTION_STATUS, 0);
  CHECK(cal_cmd_check_context(&sta, true,   0) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&sta, true,  99) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&sta, false,  0) == CAL_CMD_E_ROI_NOT_FOUND);

  /* --- defensivo: NULL --- */
  CHECK(cal_cmd_check_context(NULL, true, 0) == CAL_CMD_E_INVALID_REQUEST);
}


int main(void) {
  test_utils();
  test_parse_valid();
  test_parse_rejects();
  test_json_malformed_keeps_identity();
  test_too_large();
  test_target_and_context();
  test_replay_semantics();
  test_acks();
  test_error_acks();
  test_e52_context_matrix();

  printf("calibration_cmd: %d comprobaciones, %d fallos\n", g_run, g_fail);
  return g_fail ? 1 : 0;
}
