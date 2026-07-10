/**
  ******************************************************************************
  * @file    nfc_test.h
  * @brief   Minimal NFC bring-up / tag polling test (ST25R3920B via RFAL)
  ******************************************************************************
  */

#ifndef NFC_TEST_H
#define NFC_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "rfal_utils.h"

typedef enum
{
  NFC_TEST_OK = 0,
  NFC_TEST_ERR_SPI,
  NFC_TEST_ERR_CHIP_ID,
  NFC_TEST_ERR_RFAL_INIT,
  NFC_TEST_ERR_DISCOVER
} NfcTestStatus;

/* Debugger-visible status (watch in Keil) */
extern volatile NfcTestStatus g_nfc_test_status;
extern volatile ReturnCode   g_nfc_init_rc;
extern volatile uint8_t        g_nfc_chip_id;
extern volatile uint8_t        g_nfc_chip_rev;
extern volatile uint8_t        g_nfc_state;
extern volatile uint8_t        g_nfc_tag_count;
extern volatile uint8_t        g_nfc_tag_uid_len;
extern volatile uint8_t        g_nfc_tag_uid[10];
extern volatile uint32_t       g_nfc_poll_count;
extern volatile uint32_t       g_nfc_tag_detect_count;
extern volatile uint8_t        g_nfc_raw_chip_id;
extern volatile uint8_t        g_nfc_raw_rx0;
extern volatile uint8_t        g_nfc_raw_rx1;
extern volatile uint32_t       g_nfc_spi_hal_status;
extern volatile uint8_t        g_nfc_miso_idle;
extern volatile uint8_t        g_nfc_irq_idle;
extern volatile uint32_t       g_nfc_spi_prescaler;

/*
 * Type-2 Tag (NFC-A T2T) probe results, populated when an NFC-A T2T listener
 * is activated. Watch these in the debugger to confirm an NTAG21x.
 *
 *   g_nfc_t2t_cc[]            : Capability Container, page 3 of T2T memory
 *                               NTAG213 -> {0xE1, 0x10, 0x12, 0x00}
 *                               NTAG215 -> {0xE1, 0x10, 0x3E, 0x00}
 *                               NTAG216 -> {0xE1, 0x10, 0x6D, 0x00}
 *   g_nfc_t2t_pages_4_6[]     : First 12 bytes of NDEF area (pages 4..6)
 *   g_nfc_t2t_read_rc/len     : Result of the T2T READ from page 3
 *   g_nfc_t2t_version[]       : NTAG21x GET_VERSION (0x60) response (8 bytes)
 *                               NTAG213 -> {00, 04, 04, 02, 01, 00, 0F, 03}
 *                               NTAG215 -> {00, 04, 04, 02, 01, 00, 11, 03}
 *                               NTAG216 -> {00, 04, 04, 02, 01, 00, 13, 03}
 *   g_nfc_t2t_version_rc/len  : Result of the GET_VERSION exchange
 *                               (RFAL_ERR_TIMEOUT / PROTO if the tag is not
 *                                an NTAG21x — perfectly normal)
 *   g_nfc_t2t_family          : 213 / 215 / 216 if a known NTAG, else 0
 *   g_nfc_t2t_is_ntag213      : 1 only when both CC and GET_VERSION agree
 *                               that the tag is an NTAG213
 */
extern volatile uint8_t        g_nfc_t2t_cc[4];
extern volatile uint8_t        g_nfc_t2t_pages_4_6[12];
extern volatile ReturnCode     g_nfc_t2t_read_rc;
extern volatile uint16_t       g_nfc_t2t_read_len;
extern volatile uint8_t        g_nfc_t2t_version[8];
extern volatile ReturnCode     g_nfc_t2t_version_rc;
extern volatile uint16_t       g_nfc_t2t_version_len;
extern volatile uint16_t       g_nfc_t2t_family;
extern volatile uint8_t        g_nfc_t2t_is_ntag213;

ReturnCode NfcTest_Init(void);
void       NfcTest_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* NFC_TEST_H */
