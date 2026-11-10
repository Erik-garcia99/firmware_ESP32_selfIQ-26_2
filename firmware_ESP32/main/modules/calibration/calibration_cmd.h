#ifndef SELFIQ_CALIBRATION_CMD_H
#define SELFIQ_CALIBRATION_CMD_H

/* E5.1: contrato de comandos de calibración (Anexo A.7).
 *
 * Módulo PURO: solo depende de cJSON y de la biblioteca estándar, para poder
 * compilarlo y probarlo en PC. No toca NVS, cámara ni MQTT. El callback MQTT
 * únicamente llama a cal_cmd_parse() (rápido, sin E/S) y encola la estructura
 * copiada; el trabajo pesado lo hace el consumidor único de calibración. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"

#define CAL_CMD_SCHEMA_VERSION 1U
#define CAL_CMD_FORMAT_VERSION 1U

#define CAL_UUID_LEN 36U
#define CAL_UUID_SIZE (CAL_UUID_LEN + 1U)
#define CAL_HASH_HEX_LEN 64U
#define CAL_HASH_HEX_SIZE (CAL_HASH_HEX_LEN + 1U)
#define CAL_CRC_HEX_LEN 8U
#define CAL_CRC_HEX_SIZE (CAL_CRC_HEX_LEN + 1U)
#define CAL_TIMESTAMP_MAX 32U
#define CAL_TIMESTAMP_SIZE (CAL_TIMESTAMP_MAX + 1U)

/* Límites del Anexo A.7.6 (medidos con UUID/hash máximos). */
#define CAL_CMD_MAX_BYTES 1536U /* comando entrante */
#define CAL_ACK_JSON_SIZE 1024U /* ACK admitido por el gateway */

/* Cola FreeRTOS de calibración: un solo consumidor, profundidad acotada. */
#define CAL_CMD_QUEUE_DEPTH 4U

typedef enum {
  CAL_ACTION_NONE = 0,
  CAL_ACTION_CAPTURE,
  CAL_ACTION_COMMIT,
  CAL_ACTION_CANCEL,
  CAL_ACTION_STATUS,
} cal_action_t;

typedef enum {
  CAL_KIND_NONE = 0,
  CAL_KIND_EMPTY,
  CAL_KIND_FULL,
} cal_kind_t;

/* Códigos estables de error (§6.2 + Anexo A). Nunca cambian de texto. */
typedef enum {
  CAL_CMD_OK = 0,
  CAL_CMD_E_INVALID_JSON,       /* JSON malformado o tipo incorrecto */
  CAL_CMD_E_TOO_LARGE,          /* comando > CAL_CMD_MAX_BYTES */
  CAL_CMD_E_UNKNOWN_TYPE,       /* acción no reconocida */
  CAL_CMD_E_UNSUPPORTED_SCHEMA, /* schema_version distinto de 1 */
  CAL_CMD_E_UNSUPPORTED_FORMAT, /* format_version distinto de 1 */
  CAL_CMD_E_INVALID_REQUEST,    /* campo obligatorio ausente o malformado */
  CAL_CMD_E_SESSION_EXPIRED,    /* expires_at ya vencido (reloj válido) */
  CAL_CMD_E_BUSY,               /* otra operación física en curso/cola llena */
  CAL_CMD_E_OPERATION_CONFLICT, /* mismo operation_id, otro contenido */
  CAL_CMD_E_ROI_NOT_FOUND,
  CAL_CMD_E_CONFIG_MISMATCH,
  CAL_CMD_E_CONTEXT_MISMATCH,
} cal_cmd_err_t;

typedef struct {
  cal_action_t action;
  /* Destino opcional: el gateway puede incluirlos en el cuerpo. Si vienen, el
   * llamador los compara con la identidad del nodo (cal_cmd_check_target). */
  char branch_id[CAL_UUID_SIZE];
  char node_id[CAL_UUID_SIZE];
  char operation_id[CAL_UUID_SIZE]; /* "id": identidad semántica */
  char req_id[CAL_UUID_SIZE];       /* intento de transporte */
  char calibration_id[CAL_UUID_SIZE];
  char roi_id[CAL_UUID_SIZE];
  uint32_t calibration_version; /* 0 si la acción no la lleva (cancel/status) */
  uint32_t config_version;      /* idem */
  char context_hash[CAL_HASH_HEX_SIZE]; /* vacío en calibration.status */
  uint8_t format_version;               /* 0 si no aplica */
  /* calibrate */
  char capture_id[CAL_UUID_SIZE];
  cal_kind_t kind;
  /* calibration.commit */
  char empty_capture_id[CAL_UUID_SIZE];
  char full_capture_id[CAL_UUID_SIZE];
  char pair_hash[CAL_HASH_HEX_SIZE];
  /* calibrate / commit: informativos, RFC 3339 con milisegundos */
  char issued_at[CAL_TIMESTAMP_SIZE];
  char expires_at[CAL_TIMESTAMP_SIZE];
  /* Huella SEMÁNTICA: ignora req_id/issued_at/expires_at (son del intento). */
  uint32_t semantic_fingerprint;
} cal_cmd_t;

/* Resultado de validar una carga útil. Siempre llena req_id/operation_id si
 * pudo extraerlos, para poder responder un ACK de error correlacionable. */
typedef struct {
  cal_cmd_err_t err;
  cal_action_t action_hint; /* mejor esfuerzo, para el ACK de error */
  char operation_id[CAL_UUID_SIZE];
  char req_id[CAL_UUID_SIZE];
  char calibration_id[CAL_UUID_SIZE];
  char roi_id[CAL_UUID_SIZE];
} cal_cmd_status_t;

/* ---- utilidades ------------------------------------------------------- */
bool cal_uuid_valid(const char *value);
bool cal_hash_hex_valid(const char *value); /* 64 hex minúsculas */
bool cal_crc_hex_valid(const char *value);  /* 8 hex minúsculas */
bool cal_timestamp_valid(const char *value);

const char *cal_cmd_error_code(cal_cmd_err_t err);
const char *cal_action_name(cal_action_t action);
const char *cal_kind_name(cal_kind_t kind);

/* ---- entrada ---------------------------------------------------------- */

/* Parsea y valida un comando. NO completa campos obligatorios faltantes y NO
 * consulta ROI/NVS: la comprobación de contexto es de cal_cmd_check_context().
 * Devuelve CAL_CMD_OK y llena `cmd`; en error deja `status` con lo que se pudo
 * extraer para responder un ACK de error. `payload` no necesita terminador. */
cal_cmd_err_t cal_cmd_parse(const char *payload, size_t length, cal_cmd_t *cmd,
                            cal_cmd_status_t *status);

/* Versión sobre un árbol cJSON ya parseado (la usan las pruebas y E5.2). */
cal_cmd_err_t cal_cmd_parse_json(const cJSON *root, cal_cmd_t *cmd,
                                 cal_cmd_status_t *status);

/* Verifica que el comando se refiera al ROI/versión vigentes en el nodo.
 * `known_roi` / `applied_version` los entrega la capa que posee roi_store (E5.2)
 * — así este módulo sigue sin depender de ESP-IDF.
 *  - ROI inexistente            -> CAL_CMD_E_ROI_NOT_FOUND
 *  - config_version distinta    -> CAL_CMD_E_CONFIG_MISMATCH
 * (calibration.cancel/status no llevan config_version y solo comprueban ROI.) */
cal_cmd_err_t cal_cmd_check_context(const cal_cmd_t *cmd, bool known_roi,
                                    uint32_t applied_version);

/* Si el cuerpo trae branch_id/node_id deben coincidir con los del nodo; si no
 * vienen, el tópico MQTT ya fijó el destino y no se exige. */
cal_cmd_err_t cal_cmd_check_target(const cal_cmd_t *cmd, const char *my_branch,
                                   const char *my_node);

/* ---- idempotencia ----------------------------------------------------- */

/* Compara contra el registro previo de la misma operation_id:
 *   misma huella semántica  -> CAL_CMD_OK (duplicado legítimo)
 *   huella distinta         -> CAL_CMD_E_OPERATION_CONFLICT */
cal_cmd_err_t cal_cmd_check_replay(uint32_t stored_fingerprint,
                                   const cal_cmd_t *cmd);

/* ---- salida: ACK `cal_result` ----------------------------------------- */

/* Calidad resumida de una captura (A.7.1). */
typedef struct {
  uint8_t mean;
  uint8_t contrast;
  uint8_t clipped_pct;
} cal_quality_t;

/* Campos específicos del ACK de captura. */
typedef struct {
  uint32_t descriptor_crc;
  char image_sha256[CAL_HASH_HEX_SIZE];
  uint32_t image_bytes;
  uint16_t width;
  uint16_t height;
  cal_quality_t quality;
  const char *clock_quality; /* "unknown" mientras no haya reloj válido */
  uint32_t uptime_ms;
  bool persisted;
} cal_capture_ack_t;

/* Todas devuelven true solo si el JSON cabe COMPLETO en `out` y en
 * CAL_ACK_JSON_SIZE. Nunca truncan. */
bool cal_ack_capture(char *out, size_t out_size, const cal_cmd_t *cmd,
                     const cal_capture_ack_t *capture);
bool cal_ack_commit(char *out, size_t out_size, const cal_cmd_t *cmd,
                    uint32_t active_calibration_version, bool persisted);
bool cal_ack_cancel(char *out, size_t out_size, const cal_cmd_t *cmd);
/* `status_body` es el fragmento JSON ya serializado por quien posee el almacén
 * (`"active":...,"draft":...,"last_results":[...]`), sin llaves exteriores. */
bool cal_ack_status(char *out, size_t out_size, const cal_cmd_t *cmd,
                    const char *status_body);
/* ACK de error: ok=false, código estable y etapa que falló. Usa solo la
 * identidad que se pudo extraer (puede faltar calibration_id/roi_id). */
bool cal_ack_error(char *out, size_t out_size, const cal_cmd_status_t *status,
                   cal_cmd_err_t err);

/* Huella FNV-1a/32 de los campos semánticos (ver cal_cmd_t). */
uint32_t cal_cmd_fingerprint(const cal_cmd_t *cmd);

#endif
