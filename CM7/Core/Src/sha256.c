/**
  ******************************************************************************
  * @file    sha256.c
  * @brief   Portable SHA-256 (FIPS 180-4) and HMAC-SHA256 (RFC 2104).
  *
  * Reference test vectors verified:
  *   SHA256("abc") = ba7816bf 8f01cfea 414140de 5dae2223 b00361a3 96177a9c
  *                   b410ff61 f20015ad
  *   HMAC-SHA256(key="key", data="The quick brown fox jumps over the lazy dog")
  *     = f7bc83f4 30538424 b13298e6 aa6fb143 ef4d59a1 49461759 97479dbc 2d1a3cd8
  ******************************************************************************
  */

#include "sha256.h"
#include <string.h>

static const uint32_t SHA256_K[64] =
{
  0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
  0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
  0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
  0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
  0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
  0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
  0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
  0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
  0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
  0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
  0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
  0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
  0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
  0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
  0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
  0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U
};

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32U - (n))))
#define CH(x,y,z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x)     (ROTR(x, 2)  ^ ROTR(x, 13) ^ ROTR(x, 22))
#define EP1(x)     (ROTR(x, 6)  ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SIG0(x)    (ROTR(x, 7)  ^ ROTR(x, 18) ^ ((x) >> 3))
#define SIG1(x)    (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

static void Sha256_Transform(Sha256Ctx *ctx, const uint8_t *data)
{
  uint32_t a, b, c, d, e, f, g, h, t1, t2;
  uint32_t m[64];
  uint32_t i;

  for (i = 0U; i < 16U; i++)
  {
    m[i] = ((uint32_t)data[i * 4U + 0U] << 24) |
           ((uint32_t)data[i * 4U + 1U] << 16) |
           ((uint32_t)data[i * 4U + 2U] <<  8) |
           ((uint32_t)data[i * 4U + 3U]);
  }
  for (i = 16U; i < 64U; i++)
  {
    m[i] = SIG1(m[i - 2U]) + m[i - 7U] + SIG0(m[i - 15U]) + m[i - 16U];
  }

  a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
  e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

  for (i = 0U; i < 64U; i++)
  {
    t1 = h + EP1(e) + CH(e, f, g) + SHA256_K[i] + m[i];
    t2 = EP0(a) + MAJ(a, b, c);
    h = g; g = f; f = e;
    e = d + t1;
    d = c; c = b; b = a;
    a = t1 + t2;
  }

  ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
  ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

void Sha256_Init(Sha256Ctx *ctx)
{
  ctx->datalen  = 0U;
  ctx->bitlen   = 0U;
  ctx->state[0] = 0x6a09e667U;
  ctx->state[1] = 0xbb67ae85U;
  ctx->state[2] = 0x3c6ef372U;
  ctx->state[3] = 0xa54ff53aU;
  ctx->state[4] = 0x510e527fU;
  ctx->state[5] = 0x9b05688cU;
  ctx->state[6] = 0x1f83d9abU;
  ctx->state[7] = 0x5be0cd19U;
}

void Sha256_Update(Sha256Ctx *ctx, const uint8_t *data, size_t len)
{
  size_t i;

  for (i = 0U; i < len; i++)
  {
    ctx->data[ctx->datalen] = data[i];
    ctx->datalen++;
    if (ctx->datalen == SHA256_BLOCK_SIZE)
    {
      Sha256_Transform(ctx, ctx->data);
      ctx->bitlen  += 512U;
      ctx->datalen  = 0U;
    }
  }
}

void Sha256_Final(Sha256Ctx *ctx, uint8_t out[SHA256_DIGEST_SIZE])
{
  uint32_t i = ctx->datalen;

  /* Pad */
  if (ctx->datalen < 56U)
  {
    ctx->data[i++] = 0x80U;
    while (i < 56U)
    {
      ctx->data[i++] = 0x00U;
    }
  }
  else
  {
    ctx->data[i++] = 0x80U;
    while (i < SHA256_BLOCK_SIZE)
    {
      ctx->data[i++] = 0x00U;
    }
    Sha256_Transform(ctx, ctx->data);
    (void)memset(ctx->data, 0, 56U);
  }

  ctx->bitlen += (uint64_t)ctx->datalen * 8U;
  ctx->data[63] = (uint8_t)(ctx->bitlen);
  ctx->data[62] = (uint8_t)(ctx->bitlen >> 8);
  ctx->data[61] = (uint8_t)(ctx->bitlen >> 16);
  ctx->data[60] = (uint8_t)(ctx->bitlen >> 24);
  ctx->data[59] = (uint8_t)(ctx->bitlen >> 32);
  ctx->data[58] = (uint8_t)(ctx->bitlen >> 40);
  ctx->data[57] = (uint8_t)(ctx->bitlen >> 48);
  ctx->data[56] = (uint8_t)(ctx->bitlen >> 56);
  Sha256_Transform(ctx, ctx->data);

  for (i = 0U; i < 8U; i++)
  {
    out[i * 4U + 0U] = (uint8_t)(ctx->state[i] >> 24);
    out[i * 4U + 1U] = (uint8_t)(ctx->state[i] >> 16);
    out[i * 4U + 2U] = (uint8_t)(ctx->state[i] >>  8);
    out[i * 4U + 3U] = (uint8_t)(ctx->state[i]);
  }
}

void Sha256(const uint8_t *data, size_t len, uint8_t out[SHA256_DIGEST_SIZE])
{
  Sha256Ctx ctx;
  Sha256_Init(&ctx);
  Sha256_Update(&ctx, data, len);
  Sha256_Final(&ctx, out);
}

void HmacSha256(const uint8_t *key, size_t keyLen,
                const uint8_t *data, size_t dataLen,
                uint8_t out[SHA256_DIGEST_SIZE])
{
  uint8_t   k0[SHA256_BLOCK_SIZE];
  uint8_t   ipad[SHA256_BLOCK_SIZE];
  uint8_t   opad[SHA256_BLOCK_SIZE];
  uint8_t   inner[SHA256_DIGEST_SIZE];
  Sha256Ctx ctx;
  size_t    i;

  /* RFC 2104 §2: derive K0. If key is longer than block size, hash it. */
  (void)memset(k0, 0, sizeof(k0));
  if (keyLen > SHA256_BLOCK_SIZE)
  {
    Sha256(key, keyLen, k0);
  }
  else
  {
    (void)memcpy(k0, key, keyLen);
  }

  for (i = 0U; i < SHA256_BLOCK_SIZE; i++)
  {
    ipad[i] = k0[i] ^ 0x36U;
    opad[i] = k0[i] ^ 0x5cU;
  }

  /* inner = SHA256( (K0 ^ ipad) || data ) */
  Sha256_Init(&ctx);
  Sha256_Update(&ctx, ipad, SHA256_BLOCK_SIZE);
  Sha256_Update(&ctx, data, dataLen);
  Sha256_Final(&ctx, inner);

  /* out = SHA256( (K0 ^ opad) || inner ) */
  Sha256_Init(&ctx);
  Sha256_Update(&ctx, opad, SHA256_BLOCK_SIZE);
  Sha256_Update(&ctx, inner, SHA256_DIGEST_SIZE);
  Sha256_Final(&ctx, out);
}
