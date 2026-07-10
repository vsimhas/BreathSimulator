/**
  ******************************************************************************
  * @file    nfc_secure.c
  * @brief   HMAC-protected NTAG21x record - read / verify / provision.
  *
  *  Threat model
  *  ------------
  *  Goal: prove that the NTAG sticker on a critical CPAP component is one we
  *        provisioned, and not a counterfeit.
  *
  *    1. Each tag is uniquely identified by its 7-byte factory UID
  *       (NTAG21x silicon, byte 0 = 0x04 NXP).
  *    2. We store a small payload (component-type + ASCII serial) on the tag.
  *    3. We append HMAC-SHA256 over (UID || header) using a 32-byte secret
  *       compiled into firmware. Only firmware that knows the secret can
  *       compute a valid HMAC.
  *    4. On every detection we re-read the record, recompute the HMAC over
  *       the *actual* tag UID, and compare. Any one of: wrong tag, wrong
  *       payload, byte-flipped HMAC, or copy of legitimate bytes onto a
  *       different NTAG fails.
  *
  *  What this does NOT defend against
  *  ----------------------------------
  *    - Reading the secret out of the MCU's flash via JTAG/SWD without RDP.
  *    - Replay onto the *exact same physical tag* by an attacker who
  *      desolders it and moves it (the tag is the secret in that case).
  *    - Side-channel extraction of the HMAC key from the MCU.
  *
  *  Mitigations available later (not implemented here):
  *    - STM32H7 RDP Level 2.
  *    - NTAG213 PWD_AUTH so re-writing the tag in the field requires the PWD.
  *    - Move the secret to OTP / secure storage.
  ******************************************************************************
  */

#include "nfc_secure.h"
#include "nfc_secure_key.h"
#include "sha256.h"
#include "rfal_t2t.h"
#include "rfal_nfca.h"
#include "dbg_log.h"
#include <string.h>

/*
 ******************************************************************************
 *  Globals (debugger-visible)
 ******************************************************************************
 */

volatile NfcSecureResult g_nfc_secure_result               = NFC_SECURE_RESULT_NONE;
volatile uint32_t        g_nfc_secure_valid_count          = 0U;
volatile uint32_t        g_nfc_secure_invalid_count        = 0U;
volatile uint32_t        g_nfc_secure_provisioned_count    = 0U;
volatile uint8_t         g_nfc_secure_uid[7]               = {0U};
volatile uint8_t         g_nfc_secure_uid_len              = 0U;
volatile uint8_t         g_nfc_secure_part_kind            = 0U;
volatile uint8_t         g_nfc_secure_part_id[NFC_SECURE_PART_ID_SIZE + 1U] = {0U};
volatile uint8_t         g_nfc_secure_record[NFC_SECURE_RECORD_SIZE]        = {0U};
volatile uint8_t         g_nfc_secure_hmac_expected[NFC_SECURE_HMAC_SIZE]   = {0U};

volatile ReturnCode      g_nfc_secure_read_rc1            = RFAL_ERR_NONE;
volatile uint16_t        g_nfc_secure_read_len1           = 0U;
volatile ReturnCode      g_nfc_secure_read_rc2            = RFAL_ERR_NONE;
volatile uint16_t        g_nfc_secure_read_len2           = 0U;

volatile uint8_t  g_nfc_secure_provision_request                = 0U;
volatile uint8_t  g_nfc_secure_provision_kind                   = 0U;
volatile uint8_t  g_nfc_secure_provision_part_id[NFC_SECURE_PART_ID_SIZE] = {0U};

/*
 ******************************************************************************
 *  Helpers
 ******************************************************************************
 */

/*
 * Build the 32-byte tag record from header fields and UID.
 *
 *   record[ 0.. 3] : magic 'STAG'
 *   record[ 4]    : version
 *   record[ 5]    : part_kind
 *   record[ 6..15]: part_id (10 bytes ASCII, null padded)
 *   record[16..31]: HMAC-SHA256(key, UID || record[0..15])[0..15]
 */
static void NfcSecure_BuildRecord(const uint8_t *uid, uint8_t uidLen,
                                  uint8_t partKind,
                                  const uint8_t partId[NFC_SECURE_PART_ID_SIZE],
                                  uint8_t out[NFC_SECURE_RECORD_SIZE])
{
  uint8_t hmac[SHA256_DIGEST_SIZE];
  uint8_t macInput[16U + NFC_SECURE_HEADER_SIZE]; /* up to 10-byte UID + 16 hdr */
  uint8_t macInputLen;

  (void)memset(out, 0, NFC_SECURE_RECORD_SIZE);

  out[0] = NFC_SECURE_MAGIC0;
  out[1] = NFC_SECURE_MAGIC1;
  out[2] = NFC_SECURE_MAGIC2;
  out[3] = NFC_SECURE_MAGIC3;
  out[4] = NFC_SECURE_KEY_VERSION;
  out[5] = partKind;
  (void)memcpy(&out[6], partId, NFC_SECURE_PART_ID_SIZE);

  /* HMAC input = UID || header[0..15] */
  if (uidLen > 16U)
  {
    uidLen = 16U;
  }
  (void)memcpy(macInput, uid, uidLen);
  (void)memcpy(&macInput[uidLen], out, NFC_SECURE_HEADER_SIZE);
  macInputLen = (uint8_t)(uidLen + NFC_SECURE_HEADER_SIZE);

  HmacSha256(NFC_SECURE_KEY, sizeof(NFC_SECURE_KEY),
             macInput, macInputLen, hmac);

  (void)memcpy(&out[NFC_SECURE_HEADER_SIZE], hmac, NFC_SECURE_HMAC_SIZE);
}

/* Constant-time compare so an adversary timing the verify can't learn HMAC. */
static int NfcSecure_ConstTimeMemEq(const uint8_t *a, const uint8_t *b, size_t n)
{
  uint8_t diff = 0U;
  size_t i;

  for (i = 0U; i < n; i++)
  {
    diff |= (uint8_t)(a[i] ^ b[i]);
  }
  return (diff == 0U) ? 1 : 0;
}

/*
 *  Read pages 4..11 from the activated T2T into `out`. Returns 1 on success.
 *  Uses two T2T READ commands (each reads 4 pages = 16 bytes).
 */
static int NfcSecure_ReadRecord(uint8_t out[NFC_SECURE_RECORD_SIZE])
{
  uint8_t  buf[RFAL_T2T_READ_DATA_LEN];   /* 16 bytes */
  uint16_t rxLen = 0U;

  g_nfc_secure_read_rc1  = RFAL_ERR_IO;
  g_nfc_secure_read_len1 = 0U;
  g_nfc_secure_read_rc2  = RFAL_ERR_IO;
  g_nfc_secure_read_len2 = 0U;

  /* Pages 4..7 */
  (void)memset(buf, 0, sizeof(buf));
  g_nfc_secure_read_rc1 = rfalT2TPollerRead(
      (uint8_t)NFC_SECURE_RECORD_PAGE_BASE,
      buf, (uint16_t)sizeof(buf), &rxLen);
  g_nfc_secure_read_len1 = rxLen;
  if ((g_nfc_secure_read_rc1 != RFAL_ERR_NONE) || (rxLen < 16U))
  {
    return 0;
  }
  (void)memcpy(out, buf, 16U);

  /* Pages 8..11 */
  (void)memset(buf, 0, sizeof(buf));
  rxLen = 0U;
  g_nfc_secure_read_rc2 = rfalT2TPollerRead(
      (uint8_t)(NFC_SECURE_RECORD_PAGE_BASE + 4U),
      buf, (uint16_t)sizeof(buf), &rxLen);
  g_nfc_secure_read_len2 = rxLen;
  if ((g_nfc_secure_read_rc2 != RFAL_ERR_NONE) || (rxLen < 16U))
  {
    return 0;
  }
  (void)memcpy(&out[16], buf, 16U);

  return 1;
}

/*
 *  Write 32 bytes (8 pages) starting at page 4. Each T2T WRITE writes one
 *  4-byte page, so this is 8 individual command exchanges.
 */
static int NfcSecure_WriteRecord(const uint8_t record[NFC_SECURE_RECORD_SIZE])
{
  uint32_t i;
  ReturnCode rc;

  for (i = 0U; i < 8U; i++)
  {
    rc = rfalT2TPollerWrite((uint8_t)(NFC_SECURE_RECORD_PAGE_BASE + i),
                            &record[i * 4U]);
    if (rc != RFAL_ERR_NONE)
    {
      return 0;
    }
  }
  return 1;
}

/*
 *  Verify the on-tag record against the given UID. Populates the
 *  debugger-visible globals as a side effect.
 */
static NfcSecureResult NfcSecure_VerifyRecord(const uint8_t *uid, uint8_t uidLen,
                                              const uint8_t record[NFC_SECURE_RECORD_SIZE])
{
  uint8_t expected[NFC_SECURE_RECORD_SIZE];
  uint8_t partKind;
  uint8_t partId[NFC_SECURE_PART_ID_SIZE];
  uint32_t i;
  int allFF = 1;
  int all00 = 1;

  /* Detect a blank/factory tag - all 0xFF or all 0x00 is "no record". */
  for (i = 0U; i < NFC_SECURE_RECORD_SIZE; i++)
  {
    if (record[i] != 0xFFU) { allFF = 0; }
    if (record[i] != 0x00U) { all00 = 0; }
  }
  if ((allFF != 0) || (all00 != 0))
  {
    return NFC_SECURE_RESULT_BLANK;
  }

  if ((record[0] != NFC_SECURE_MAGIC0) ||
      (record[1] != NFC_SECURE_MAGIC1) ||
      (record[2] != NFC_SECURE_MAGIC2) ||
      (record[3] != NFC_SECURE_MAGIC3))
  {
    return NFC_SECURE_RESULT_BAD_MAGIC;
  }

  if (record[4] != NFC_SECURE_KEY_VERSION)
  {
    return NFC_SECURE_RESULT_BAD_VERSION;
  }

  partKind = record[5];
  (void)memcpy(partId, &record[6], NFC_SECURE_PART_ID_SIZE);

  /* Recompute the HMAC the way it would have been made at provisioning. */
  NfcSecure_BuildRecord(uid, uidLen, partKind, partId, expected);

  /* Stash the expected HMAC for debugger inspection regardless of result. */
  (void)memcpy((void *)g_nfc_secure_hmac_expected,
               &expected[NFC_SECURE_HEADER_SIZE], NFC_SECURE_HMAC_SIZE);

  if (NfcSecure_ConstTimeMemEq(&record[NFC_SECURE_HEADER_SIZE],
                               &expected[NFC_SECURE_HEADER_SIZE],
                               NFC_SECURE_HMAC_SIZE) == 0)
  {
    return NFC_SECURE_RESULT_BAD_HMAC;
  }

  /* Valid - publish part info. */
  g_nfc_secure_part_kind = partKind;
  (void)memcpy((void *)g_nfc_secure_part_id, partId, NFC_SECURE_PART_ID_SIZE);
  g_nfc_secure_part_id[NFC_SECURE_PART_ID_SIZE] = 0U;

  return NFC_SECURE_RESULT_VALID;
}

/*
 ******************************************************************************
 *  Public API
 ******************************************************************************
 */

void NfcSecure_RequestProvision(uint8_t partKind, const char *partId)
{
  size_t len;

  g_nfc_secure_provision_kind = partKind;
  (void)memset((void *)g_nfc_secure_provision_part_id, 0,
               sizeof(g_nfc_secure_provision_part_id));

  if (partId != NULL)
  {
    len = strnlen(partId, NFC_SECURE_PART_ID_SIZE + 1U);
    if (len > NFC_SECURE_PART_ID_SIZE)
    {
      len = NFC_SECURE_PART_ID_SIZE;
    }
    (void)memcpy((void *)g_nfc_secure_provision_part_id, partId, len);
  }

  g_nfc_secure_provision_request = 1U;
}

static const char *NfcSecure_ResultStr(NfcSecureResult r)
{
  switch (r)
  {
    case NFC_SECURE_RESULT_NONE:                 return "NONE";
    case NFC_SECURE_RESULT_NOT_T2T:               return "NOT_T2T";
    case NFC_SECURE_RESULT_READ_ERR:              return "READ_ERR";
    case NFC_SECURE_RESULT_BLANK:                 return "BLANK";
    case NFC_SECURE_RESULT_BAD_MAGIC:             return "BAD_MAGIC";
    case NFC_SECURE_RESULT_BAD_VERSION:           return "BAD_VERSION";
    case NFC_SECURE_RESULT_BAD_HMAC:              return "BAD_HMAC";
    case NFC_SECURE_RESULT_VALID:                 return "VALID";
    case NFC_SECURE_RESULT_WRITE_ERR:             return "WRITE_ERR";
    case NFC_SECURE_RESULT_PROVISION_VERIFY_FAIL: return "PROVISION_VFAIL";
    case NFC_SECURE_RESULT_PROVISIONED:           return "PROVISIONED";
    default:                                      return "?";
  }
}

NfcSecureResult NfcSecure_OnTagActivated(rfalNfcDevice *dev)
{
  uint8_t  uid[7];
  uint8_t  uidLen;
  uint8_t  record[NFC_SECURE_RECORD_SIZE];
  NfcSecureResult result;

  g_nfc_secure_result = NFC_SECURE_RESULT_NONE;

  if ((dev == NULL) ||
      (dev->type != RFAL_NFC_LISTEN_TYPE_NFCA) ||
      (dev->dev.nfca.type != RFAL_NFCA_T2T))
  {
    g_nfc_secure_result = NFC_SECURE_RESULT_NOT_T2T;
    return g_nfc_secure_result;
  }

  /* Snapshot UID. */
  uidLen = dev->nfcidLen;
  if (uidLen > sizeof(uid))
  {
    uidLen = (uint8_t)sizeof(uid);
  }
  (void)memcpy(uid, dev->nfcid, uidLen);
  (void)memcpy((void *)g_nfc_secure_uid, uid, uidLen);
  g_nfc_secure_uid_len = uidLen;

  /* Read the record. */
  if (NfcSecure_ReadRecord(record) == 0)
  {
    g_nfc_secure_result = NFC_SECURE_RESULT_READ_ERR;
    g_nfc_secure_invalid_count++;
    DBG_W("NFC_SEC", "T2T READ failed rc1=%d/%u rc2=%d/%u",
          (int)g_nfc_secure_read_rc1, (unsigned)g_nfc_secure_read_len1,
          (int)g_nfc_secure_read_rc2, (unsigned)g_nfc_secure_read_len2);
    return g_nfc_secure_result;
  }
  (void)memcpy((void *)g_nfc_secure_record, record, NFC_SECURE_RECORD_SIZE);

  result = NfcSecure_VerifyRecord(uid, uidLen, record);

  /*
   * Provisioning policy: only when the operator explicitly requests it AND
   * the tag is blank or invalid. We never overwrite a tag whose record
   * already verifies.
   */
  if ((g_nfc_secure_provision_request != 0U) &&
      (result != NFC_SECURE_RESULT_VALID))
  {
    uint8_t newRecord[NFC_SECURE_RECORD_SIZE];

    NfcSecure_BuildRecord(uid, uidLen,
                          g_nfc_secure_provision_kind,
                          (const uint8_t *)g_nfc_secure_provision_part_id,
                          newRecord);

    DBG_I("NFC_SEC", "provisioning kind=%u",
          (unsigned)g_nfc_secure_provision_kind);

    if (NfcSecure_WriteRecord(newRecord) == 0)
    {
      g_nfc_secure_result = NFC_SECURE_RESULT_WRITE_ERR;
      g_nfc_secure_invalid_count++;
      DBG_E("NFC_SEC", "provision WRITE failed");
      return g_nfc_secure_result;
    }

    /* Verify-after-write. */
    if (NfcSecure_ReadRecord(record) == 0)
    {
      g_nfc_secure_result = NFC_SECURE_RESULT_PROVISION_VERIFY_FAIL;
      g_nfc_secure_invalid_count++;
      return g_nfc_secure_result;
    }
    (void)memcpy((void *)g_nfc_secure_record, record, NFC_SECURE_RECORD_SIZE);

    result = NfcSecure_VerifyRecord(uid, uidLen, record);
    if (result == NFC_SECURE_RESULT_VALID)
    {
      g_nfc_secure_provision_request = 0U;
      g_nfc_secure_provisioned_count++;
      g_nfc_secure_valid_count++;
      g_nfc_secure_result = NFC_SECURE_RESULT_PROVISIONED;
      DBG_I("NFC_SEC", "provisioned ok (count=%lu)",
            (unsigned long)g_nfc_secure_provisioned_count);
      return g_nfc_secure_result;
    }

    g_nfc_secure_result = NFC_SECURE_RESULT_PROVISION_VERIFY_FAIL;
    g_nfc_secure_invalid_count++;
    DBG_E("NFC_SEC", "provision verify fail: %s",
          NfcSecure_ResultStr(result));
    return g_nfc_secure_result;
  }

  if (result == NFC_SECURE_RESULT_VALID)
  {
    g_nfc_secure_valid_count++;
    DBG_I("NFC_SEC", "verify ok (valid_count=%lu)",
          (unsigned long)g_nfc_secure_valid_count);
  }
  else
  {
    g_nfc_secure_invalid_count++;
    DBG_W("NFC_SEC", "verify fail: %s (invalid_count=%lu)",
          NfcSecure_ResultStr(result),
          (unsigned long)g_nfc_secure_invalid_count);
  }
  g_nfc_secure_result = result;
  return result;
}
