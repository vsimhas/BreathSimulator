/**
  ******************************************************************************
  * @file    nfc_secure_key.h
  * @brief   Per-deployment HMAC-SHA256 secret used to authenticate NTAG21x
  *          stickers attached to critical components.
  *
  *  IMPORTANT
  *  ---------
  *  The bytes below are a development-only placeholder. Generate your own
  *  32 random bytes for each production line / fleet, and keep them out of
  *  source control (e.g. include this header from a build-time generated
  *  file in your CI). Anyone with this key can forge valid stickers.
  *
  *  Generation example (any one of):
  *     openssl rand -hex 32
  *     python -c "import secrets; print(secrets.token_hex(32))"
  *
  *  Rotation: change the key, bump NFC_SECURE_KEY_VERSION, and re-provision
  *  every sticker. The verifier will reject records signed with an older key.
  ******************************************************************************
  */

#ifndef NFC_SECURE_KEY_H
#define NFC_SECURE_KEY_H

#include <stdint.h>

#define NFC_SECURE_KEY_VERSION   0x01U

/* 32-byte (256-bit) HMAC-SHA256 key. CHANGE BEFORE PRODUCTION. */
static const uint8_t NFC_SECURE_KEY[32] =
{
  0x4F, 0x6E, 0x65, 0x4B, 0x65, 0x79, 0x54, 0x6F,
  0x52, 0x75, 0x6C, 0x65, 0x54, 0x68, 0x65, 0x6D,
  0x41, 0x6C, 0x6C, 0x2D, 0x44, 0x45, 0x56, 0x45,
  0x4C, 0x4F, 0x50, 0x4D, 0x45, 0x4E, 0x54, 0x21,
};

#endif /* NFC_SECURE_KEY_H */
