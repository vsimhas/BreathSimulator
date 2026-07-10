/**
  ******************************************************************************
  * @file    nfc_secure.h
  * @brief   HMAC-protected NTAG21x record for component-authentication on
  *          critical CPAP components.
  *
  *  Wire format on the NTAG21x (32 bytes, pages 4..11):
  *
  *    Off  Size  Field
  *    ---  ----  -----------------------------------------------
  *      0    4   magic       = "STAG"  (0x53 0x54 0x41 0x47)
  *      4    1   version     = NFC_SECURE_KEY_VERSION
  *      5    1   part_kind   = caller-defined component type code
  *      6   10   part_id     = ASCII serial / lot, null-padded
  *     16   16   hmac        = HMAC-SHA256(key, UID || header[0..15])
  *                             truncated to 16 bytes
  *
  *  HMAC binds the payload to the *specific* tag UID, so cloning the bytes
  *  to another NTAG21x fails verification.
  ******************************************************************************
  */

#ifndef NFC_SECURE_H
#define NFC_SECURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "rfal_utils.h"
#include "rfal_nfc.h"

#define NFC_SECURE_RECORD_SIZE          32U
#define NFC_SECURE_HEADER_SIZE          16U
#define NFC_SECURE_HMAC_SIZE            16U
#define NFC_SECURE_PART_ID_SIZE         10U
#define NFC_SECURE_RECORD_PAGE_BASE     4U   /* T2T page 4 .. 11 */
#define NFC_SECURE_MAGIC0               0x53U  /* 'S' */
#define NFC_SECURE_MAGIC1               0x54U  /* 'T' */
#define NFC_SECURE_MAGIC2               0x41U  /* 'A' */
#define NFC_SECURE_MAGIC3               0x47U  /* 'G' */

typedef enum
{
  NFC_SECURE_RESULT_NONE         = 0,  /*!< No tap yet                                  */
  NFC_SECURE_RESULT_VALID        = 1,  /*!< HMAC matched - genuine component            */
  NFC_SECURE_RESULT_BLANK        = 2,  /*!< Tag has no STAG record (factory-fresh)      */
  NFC_SECURE_RESULT_BAD_MAGIC    = 3,  /*!< 4-byte magic mismatch                       */
  NFC_SECURE_RESULT_BAD_VERSION  = 4,  /*!< Key/format version mismatch                 */
  NFC_SECURE_RESULT_BAD_HMAC     = 5,  /*!< HMAC mismatch - tampered or cloned          */
  NFC_SECURE_RESULT_READ_ERR     = 6,  /*!< T2T READ failed                             */
  NFC_SECURE_RESULT_WRITE_ERR    = 7,  /*!< T2T WRITE failed during provisioning        */
  NFC_SECURE_RESULT_NOT_T2T      = 8,  /*!< Activated tag isn't NFC-A T2T               */
  NFC_SECURE_RESULT_PROVISIONED  = 9,  /*!< Tag was just provisioned (write succeeded
                                            and verify-after-write passed)             */
  NFC_SECURE_RESULT_PROVISION_VERIFY_FAIL = 10 /*!< Wrote OK but read-back didn't match */
} NfcSecureResult;

/*
 *  Debugger-visible state — watch these in Keil to see verification outcome.
 */
extern volatile NfcSecureResult g_nfc_secure_result;
extern volatile uint32_t        g_nfc_secure_valid_count;
extern volatile uint32_t        g_nfc_secure_invalid_count;
extern volatile uint32_t        g_nfc_secure_provisioned_count;
extern volatile uint8_t         g_nfc_secure_uid[7];
extern volatile uint8_t         g_nfc_secure_uid_len;
extern volatile uint8_t         g_nfc_secure_part_kind;
extern volatile uint8_t         g_nfc_secure_part_id[NFC_SECURE_PART_ID_SIZE + 1U]; /* +NUL */
extern volatile uint8_t         g_nfc_secure_record[NFC_SECURE_RECORD_SIZE];
extern volatile uint8_t         g_nfc_secure_hmac_expected[NFC_SECURE_HMAC_SIZE];

/*
 *  Read diagnostics — useful when g_nfc_secure_result is READ_ERR.
 *
 *    g_nfc_secure_read_rc1/len1 : status of T2T READ at page 4 (pages 4..7)
 *    g_nfc_secure_read_rc2/len2 : status of T2T READ at page 8 (pages 8..11)
 *
 *  RFAL ReturnCode values you'll commonly see:
 *      0x00 = RFAL_ERR_NONE (success, len should be 16)
 *      0x10 = RFAL_ERR_TIMEOUT  (tag didn't reply - left field, halted, or
 *                                doesn't support T2T READ)
 *      0x11 = RFAL_ERR_FRAMING  (collision / signal integrity)
 *      0x33 = RFAL_ERR_PROTO    (NACK returned by tag - not a Type-2 tag)
 */
extern volatile ReturnCode      g_nfc_secure_read_rc1;
extern volatile uint16_t        g_nfc_secure_read_len1;
extern volatile ReturnCode      g_nfc_secure_read_rc2;
extern volatile uint16_t        g_nfc_secure_read_len2;

/*
 *  Provisioning controls — write from the debugger to provision the next tag.
 *
 *    g_nfc_secure_provision_request  : Set to 1 to provision the *next*
 *                                      blank/invalid tag; auto-clears after
 *                                      a successful provisioning attempt.
 *    g_nfc_secure_provision_kind     : Component-type code burned into the tag.
 *    g_nfc_secure_provision_part_id  : ASCII serial/lot to burn (10 chars max,
 *                                      will be null-padded).
 */
extern volatile uint8_t  g_nfc_secure_provision_request;
extern volatile uint8_t  g_nfc_secure_provision_kind;
extern volatile uint8_t  g_nfc_secure_provision_part_id[NFC_SECURE_PART_ID_SIZE];

/*
 *  Hook called from NfcTest_Poll() once the activated device is in hand.
 *  Reads the on-tag record, verifies HMAC, and (if requested) writes a fresh
 *  record onto a blank/invalid tag.
 *
 *  Returns the same status that's posted to g_nfc_secure_result.
 */
NfcSecureResult NfcSecure_OnTagActivated(rfalNfcDevice *dev);

/*
 *  Optional: programmatic provisioning trigger. Equivalent to setting
 *  g_nfc_secure_provision_request = 1 with the given parameters.
 */
void NfcSecure_RequestProvision(uint8_t partKind,
                                const char *partId);

#ifdef __cplusplus
}
#endif

#endif /* NFC_SECURE_H */
