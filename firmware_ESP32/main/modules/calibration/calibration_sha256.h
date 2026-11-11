#ifndef SELFIQ_CALIBRATION_SHA256_H
#define SELFIQ_CALIBRATION_SHA256_H

/* SHA-256 mínimo y puro (sin ESP-IDF ni mbedTLS) para E5.3.
 *
 * La referencia de revisión viaja como JPEG y el backend exige
 * `image_sha256` (Anexo A.8). Se calcula sobre los bytes exactos que se
 * envían, no sobre el frame gris. Al ser puro se puede validar en PC con los
 * vectores conocidos sin enlazar la criptografía de ESP-IDF. */

#include <stddef.h>
#include <stdint.h>

#define CAL_SHA256_DIGEST_BYTES 32U
#define CAL_SHA256_HEX_CHARS 64U
#define CAL_SHA256_HEX_SIZE (CAL_SHA256_HEX_CHARS + 1U)

/* `data` puede ser NULL solo cuando `length` es 0. `out` nunca se escribe
 * parcialmente: si los argumentos son inválidos queda en cero. */
void cal_sha256(const uint8_t *data, size_t length,
                uint8_t out[CAL_SHA256_DIGEST_BYTES]);

/* Igual que cal_sha256() pero en hexadecimal minúsculas (64 caracteres + '\0'),
 * que es el formato que valida el gateway/backend. */
void cal_sha256_hex(const uint8_t *data, size_t length,
                    char out[CAL_SHA256_HEX_SIZE]);

#endif
