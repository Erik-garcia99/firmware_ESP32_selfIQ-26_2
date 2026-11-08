#ifndef SELFIQ_ROI_CORE_H
#define SELFIQ_ROI_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Nucleo puro: sin dependencias de ESP-IDF para poder compilarlo en PC. */

#define ROI_MAX 8U
#define ROI_ID_LEN 36U
#define ROI_ID_SIZE (ROI_ID_LEN + 1U)
#define ROI_REQ_ID_MAX 64U
#define ROI_REQ_ID_SIZE (ROI_REQ_ID_MAX + 1U)

#define ROI_MIN_SIDE 0.03f
#define ROI_MAX_OVERLAP 0.02f
#define ROI_GEOMETRY_EPSILON 0.000001f

#define ROI_BLOB_MAGIC 0x494F5253U /* 'SROI' en little endian */
#define ROI_BLOB_FORMAT_VERSION 1U

typedef struct {
  char id[ROI_ID_SIZE];
  float x;
  float y;
  float w;
  float h;
} roi_def_t;

typedef struct {
  uint32_t magic;
  uint16_t format_version;
  uint16_t count;
  uint32_t config_version;
  uint32_t crc32;
  roi_def_t rois[ROI_MAX];
} roi_blob_t;

typedef enum {
  ROI_OK = 0,
  ROI_ERR_INVALID_JSON = 1,
  ROI_ERR_TOO_MANY = 2,
  ROI_ERR_INVALID_BBOX = 3,
  ROI_ERR_OVERLAP = 4,
} roi_err_t;

/* Codigo estable que viaja en roi_ack.error. */
const char *roi_error_code(roi_err_t err);

bool roi_uuid_valid(const char *value);

/* Valida cantidad, IDs unicos, limites 0..1, lado minimo y solapes.
 * Un conjunto vacio es valido: significa borrar todas las zonas. */
roi_err_t roi_validate_set(const roi_def_t *rois, size_t count);

/* CRC32 IEEE 802.3, mismo polinomio que usa el firmware para credenciales. */
uint32_t roi_crc32(const void *data, size_t len);

/* CRC del blob con el campo crc32 en cero. */
uint32_t roi_blob_crc(const roi_blob_t *blob);

/* Ajusta magic/version de formato y sella el CRC. */
void roi_blob_seal(roi_blob_t *blob);

/* true si magic, formato, count y CRC coinciden. */
bool roi_blob_verify(const roi_blob_t *blob);

#endif
