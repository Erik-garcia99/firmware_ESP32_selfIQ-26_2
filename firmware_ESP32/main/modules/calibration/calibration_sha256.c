#include "calibration_sha256.h"

#include <string.h>

/* FIPS 180-4. Implementación compacta de un solo bloque, sin estado global:
 * dos tareas pueden calcular hashes concurrentemente sin sincronización. */

#define SHA256_BLOCK_BYTES 64U
#define SHA256_WORDS 8U
#define SHA256_ROUNDS 64U

static const uint32_t K[SHA256_ROUNDS] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

static uint32_t rotr(uint32_t value, unsigned bits) {
  return (value >> bits) | (value << (32U - bits));
}

static void transform(uint32_t state[SHA256_WORDS],
                      const uint8_t block[SHA256_BLOCK_BYTES]) {
  uint32_t w[SHA256_ROUNDS];
  for (unsigned i = 0; i < 16U; ++i) {
    w[i] = ((uint32_t)block[i * 4U] << 24) |
           ((uint32_t)block[i * 4U + 1U] << 16) |
           ((uint32_t)block[i * 4U + 2U] << 8) | (uint32_t)block[i * 4U + 3U];
  }
  for (unsigned i = 16U; i < SHA256_ROUNDS; ++i) {
    const uint32_t s0 =
        rotr(w[i - 15U], 7U) ^ rotr(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
    const uint32_t s1 =
        rotr(w[i - 2U], 17U) ^ rotr(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
    w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
  }

  uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
  uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
  for (unsigned i = 0; i < SHA256_ROUNDS; ++i) {
    const uint32_t s1 = rotr(e, 6U) ^ rotr(e, 11U) ^ rotr(e, 25U);
    const uint32_t ch = (e & f) ^ ((~e) & g);
    const uint32_t temp1 = h + s1 + ch + K[i] + w[i];
    const uint32_t s0 = rotr(a, 2U) ^ rotr(a, 13U) ^ rotr(a, 22U);
    const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t temp2 = s0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + temp1;
    d = c;
    c = b;
    b = a;
    a = temp1 + temp2;
  }
  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;
}

void cal_sha256(const uint8_t *data, size_t length,
                uint8_t out[CAL_SHA256_DIGEST_BYTES]) {
  if (!out) return;
  memset(out, 0, CAL_SHA256_DIGEST_BYTES);
  if (!data && length) return;

  uint32_t state[SHA256_WORDS] = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                  0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                  0x1f83d9abU, 0x5be0cd19U};

  size_t offset = 0;
  while (length - offset >= SHA256_BLOCK_BYTES) {
    transform(state, data + offset);
    offset += SHA256_BLOCK_BYTES;
  }

  uint8_t tail[SHA256_BLOCK_BYTES];
  memset(tail, 0, sizeof(tail));
  const size_t remaining = length - offset;
  if (remaining) memcpy(tail, data + offset, remaining);
  tail[remaining] = 0x80U;
  if (remaining >= 56U) {
    transform(state, tail);
    memset(tail, 0, sizeof(tail));
  }
  const uint64_t bits = (uint64_t)length * 8U;
  for (unsigned i = 0; i < 8U; ++i)
    tail[SHA256_BLOCK_BYTES - 1U - i] = (uint8_t)(bits >> (i * 8U));
  transform(state, tail);

  for (unsigned i = 0; i < SHA256_WORDS; ++i) {
    out[i * 4U] = (uint8_t)(state[i] >> 24);
    out[i * 4U + 1U] = (uint8_t)(state[i] >> 16);
    out[i * 4U + 2U] = (uint8_t)(state[i] >> 8);
    out[i * 4U + 3U] = (uint8_t)state[i];
  }
}

void cal_sha256_hex(const uint8_t *data, size_t length,
                    char out[CAL_SHA256_HEX_SIZE]) {
  static const char hex[] = "0123456789abcdef";
  uint8_t digest[CAL_SHA256_DIGEST_BYTES];
  cal_sha256(data, length, digest);
  if (!out) return;
  for (unsigned i = 0; i < CAL_SHA256_DIGEST_BYTES; ++i) {
    out[i * 2U] = hex[digest[i] >> 4];
    out[i * 2U + 1U] = hex[digest[i] & 0x0FU];
  }
  out[CAL_SHA256_HEX_CHARS] = '\0';
}
