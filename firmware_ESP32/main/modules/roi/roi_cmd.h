#ifndef SELFIQ_ROI_CMD_H
#define SELFIQ_ROI_CMD_H

#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"
#include "roi_core.h"

#define ROI_ACK_JSON_SIZE 224U

/* Extrae req_id valido de un objeto ya parseado. Deja out vacio si falta o
 * tiene caracteres no permitidos. */
void roi_cmd_extract_req_id(const cJSON *root, char *out, size_t out_size);

/* Respaldo para JSON malformado: busca "req_id" sin depender de cJSON. */
void roi_cmd_extract_req_id_raw(const char *data, int len, char *out,
                                size_t out_size);

/* Valida type, req_id, version y rois[]. No toca NVS ni publica. */
roi_err_t roi_cmd_parse_set(const cJSON *root, roi_def_t *rois,
                            size_t *count, uint32_t *version);

/* Devuelve true si el JSON cabe en out. */
bool roi_cmd_ack_ok(char *out, size_t out_size, const char *req_id,
                    uint32_t version, size_t count);

bool roi_cmd_ack_error(char *out, size_t out_size, const char *req_id,
                       const char *error_code);

#endif
