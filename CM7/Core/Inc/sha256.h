/**
  ******************************************************************************
  * @file    sha256.h
  * @brief   Portable SHA-256 (FIPS 180-4) and HMAC-SHA256 (RFC 2104).
  *
  * No external dependencies. Self-contained. Plain C99. Constant amount of
  * stack (~256 B). Suitable for use from main-loop code on a Cortex-M7.
  ******************************************************************************
  */

#ifndef SHA256_H
#define SHA256_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define SHA256_DIGEST_SIZE   32U
#define SHA256_BLOCK_SIZE    64U

typedef struct
{
  uint32_t state[8];
  uint64_t bitlen;
  uint32_t datalen;
  uint8_t  data[SHA256_BLOCK_SIZE];
} Sha256Ctx;

void Sha256_Init(Sha256Ctx *ctx);
void Sha256_Update(Sha256Ctx *ctx, const uint8_t *data, size_t len);
void Sha256_Final(Sha256Ctx *ctx, uint8_t out[SHA256_DIGEST_SIZE]);
void Sha256(const uint8_t *data, size_t len, uint8_t out[SHA256_DIGEST_SIZE]);

/* HMAC-SHA256 over `data` with the given key. `out` receives 32 bytes. */
void HmacSha256(const uint8_t *key, size_t keyLen,
                const uint8_t *data, size_t dataLen,
                uint8_t out[SHA256_DIGEST_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* SHA256_H */
