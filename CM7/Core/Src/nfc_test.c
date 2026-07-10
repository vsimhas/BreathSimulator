/**
  ******************************************************************************
  * @file    nfc_test.c
  * @brief   Minimal NFC bring-up / tag polling test (ST25R3920B via RFAL)
  ******************************************************************************
  */

#include "nfc_test.h"
#include "rfal_nfc.h"
#include "rfal_nfca.h"
#include "rfal_t2t.h"
#include "rfal_rf.h"
#include "st25r3916.h"
#include "st25r3916_com.h"
#include "rfal_platform.h"
#include "nfc_secure.h"
#include "dbg_log.h"
#include "main.h"

volatile NfcTestStatus g_nfc_test_status = NFC_TEST_ERR_SPI;
volatile ReturnCode    g_nfc_init_rc = RFAL_ERR_IO;
volatile uint8_t       g_nfc_chip_id = 0U;
volatile uint8_t       g_nfc_chip_rev = 0U;
volatile uint8_t       g_nfc_state = 0U;
volatile uint8_t       g_nfc_tag_count = 0U;
volatile uint8_t       g_nfc_tag_uid_len = 0U;
volatile uint8_t       g_nfc_tag_uid[10] = {0U};
volatile uint32_t      g_nfc_poll_count = 0U;
volatile uint32_t      g_nfc_tag_detect_count = 0U;
volatile uint8_t       g_nfc_raw_chip_id = 0U;
volatile uint8_t       g_nfc_raw_rx0 = 0U;
volatile uint8_t       g_nfc_raw_rx1 = 0U;
volatile uint8_t       g_nfc_miso_idle = 0U;
volatile uint8_t       g_nfc_irq_idle = 0U;
volatile uint32_t      g_nfc_spi_prescaler = 0U;

volatile uint8_t       g_nfc_t2t_cc[4]            = {0U};
volatile uint8_t       g_nfc_t2t_pages_4_6[12]    = {0U};
volatile ReturnCode    g_nfc_t2t_read_rc          = RFAL_ERR_NONE;
volatile uint16_t      g_nfc_t2t_read_len         = 0U;
volatile uint8_t       g_nfc_t2t_version[8]       = {0U};
volatile ReturnCode    g_nfc_t2t_version_rc       = RFAL_ERR_NONE;
volatile uint16_t      g_nfc_t2t_version_len      = 0U;
volatile uint16_t      g_nfc_t2t_family           = 0U;
volatile uint8_t       g_nfc_t2t_is_ntag213       = 0U;

static bool            s_nfc_ready = false;
static bool            s_discovery_active = false;

#define NFC_SPI_READ_MODE (0x40U)

/* SPI1 kernel clock is ~129 MHz; prescaler N gives ~129/N MHz bit rate */
static const uint32_t s_nfcSpiPrescalers[] = {
  SPI_BAUDRATEPRESCALER_256,  /* ~503 kHz */
  SPI_BAUDRATEPRESCALER_128,  /* ~1.0 MHz */
  SPI_BAUDRATEPRESCALER_64,   /* ~2.0 MHz (CubeMX default) */
};

static ReturnCode NfcTest_ConfigSpiPrescaler(uint32_t prescaler)
{
  if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
  {
    return RFAL_ERR_IO;
  }

  hspi1.Init.BaudRatePrescaler = prescaler;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return RFAL_ERR_IO;
  }

  g_nfc_spi_prescaler = prescaler;
  return RFAL_ERR_NONE;
}

static ReturnCode NfcTest_RawReadChipIdOnce(void)
{
  uint8_t cmd;
  uint8_t tx[2];
  uint8_t rx[2];
  uint8_t data;

  g_nfc_miso_idle = (uint8_t)HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9);
  g_nfc_irq_idle = (uint8_t)HAL_GPIO_ReadPin(NFC_INT_GPIO_Port, NFC_INT_Pin);

  /* Method A: single 2-byte full-duplex transfer */
  tx[0] = (uint8_t)(ST25R3916_REG_IC_IDENTITY | NFC_SPI_READ_MODE);
  tx[1] = 0x00U;
  rx[0] = 0x00U;
  rx[1] = 0x00U;

  platformSpiSelect();
  platformSpiTxRx(tx, rx, 2U);
  platformSpiDeselect();

  g_nfc_raw_rx0 = rx[0];
  g_nfc_raw_rx1 = rx[1];
  g_nfc_raw_chip_id = rx[1];

  if (g_nfc_spi_hal_status != (uint32_t)HAL_OK)
  {
    g_nfc_test_status = NFC_TEST_ERR_SPI;
    return RFAL_ERR_IO;
  }

  /* Method B: RFAL-style command byte then data byte (same CS assertion) */
  cmd = (uint8_t)(ST25R3916_REG_IC_IDENTITY | NFC_SPI_READ_MODE);
  data = 0x00U;

  platformSpiSelect();
  platformSpiTxRx(&cmd, NULL, 1U);
  if (g_nfc_spi_hal_status != (uint32_t)HAL_OK)
  {
    platformSpiDeselect();
    g_nfc_test_status = NFC_TEST_ERR_SPI;
    return RFAL_ERR_IO;
  }

  platformSpiTxRx(&data, &data, 1U);
  platformSpiDeselect();

  if (g_nfc_spi_hal_status != (uint32_t)HAL_OK)
  {
    g_nfc_test_status = NFC_TEST_ERR_SPI;
    return RFAL_ERR_IO;
  }

  /* Prefer RFAL-style result if method A did not get a valid ID */
  if ((g_nfc_raw_chip_id == 0x00U) || (g_nfc_raw_chip_id == 0xFFU))
  {
    g_nfc_raw_chip_id = data;
  }

  if ((g_nfc_raw_chip_id == 0x00U) || (g_nfc_raw_chip_id == 0xFFU))
  {
    g_nfc_test_status = NFC_TEST_ERR_CHIP_ID;
    return RFAL_ERR_HW_MISMATCH;
  }

  return RFAL_ERR_NONE;
}

static ReturnCode NfcTest_RawReadChipId(void)
{
  ReturnCode err;
  uint32_t i;

  for (i = 0U; i < (uint32_t)(sizeof(s_nfcSpiPrescalers) / sizeof(s_nfcSpiPrescalers[0])); i++)
  {
    err = NfcTest_ConfigSpiPrescaler(s_nfcSpiPrescalers[i]);
    if (err != RFAL_ERR_NONE)
    {
      g_nfc_test_status = NFC_TEST_ERR_SPI;
      return err;
    }

    platformDelay(2U);
    err = NfcTest_RawReadChipIdOnce();
    if (err == RFAL_ERR_NONE)
    {
      return RFAL_ERR_NONE;
    }
  }

  return RFAL_ERR_HW_MISMATCH;
}

static ReturnCode NfcTest_ReadChipId(void)
{
  uint8_t id = 0U;
  uint8_t chip_rev = 0U;

  platformSpiDeselect();
  st25r3916ReadRegister(ST25R3916_REG_IC_IDENTITY, &id);

  g_nfc_chip_id = id;

  if (!st25r3916CheckChipID(&chip_rev))
  {
    g_nfc_test_status = NFC_TEST_ERR_CHIP_ID;
    return RFAL_ERR_HW_MISMATCH;
  }

  g_nfc_chip_rev = chip_rev;
  return RFAL_ERR_NONE;
}

static ReturnCode NfcTest_StartDiscovery(void)
{
  rfalNfcDiscoverParam discParam;

  RFAL_MEMSET(&discParam, 0, sizeof(discParam));
  discParam.compMode = RFAL_COMPLIANCE_MODE_NFC;
  discParam.techs2Find = RFAL_NFC_POLL_TECH_A;
  discParam.totalDuration = 1000U;
  discParam.devLimit = 1U;

  return rfalNfcDiscover(&discParam);
}

ReturnCode NfcTest_Init(void)
{
  ReturnCode err;

  g_nfc_test_status = NFC_TEST_ERR_SPI;
  g_nfc_init_rc = RFAL_ERR_IO;
  s_nfc_ready = false;
  s_discovery_active = false;

  platformSpiDeselect();
  platformDelay(10U);

  err = NfcTest_RawReadChipId();
  if (err != RFAL_ERR_NONE)
  {
    g_nfc_init_rc = err;
    return err;
  }

  err = NfcTest_ReadChipId();
  if (err != RFAL_ERR_NONE)
  {
    g_nfc_init_rc = err;
    return err;
  }

  err = rfalNfcInitialize();
  g_nfc_init_rc = err;
  if (err != RFAL_ERR_NONE)
  {
    g_nfc_test_status = NFC_TEST_ERR_RFAL_INIT;
    return err;
  }

  err = NfcTest_StartDiscovery();
  if (err != RFAL_ERR_NONE)
  {
    g_nfc_test_status = NFC_TEST_ERR_DISCOVER;
    return err;
  }

  s_nfc_ready = true;
  s_discovery_active = true;
  g_nfc_test_status = NFC_TEST_OK;
  return RFAL_ERR_NONE;
}

/* NTAG21x GET_VERSION storage size byte (response[6]) → user-memory family. */
#define NFC_NTAG_GET_VERSION_CMD       0x60U
#define NFC_NTAG_VERSION_LEN           8U
#define NFC_NTAG213_STORAGE_SIZE       0x0FU
#define NFC_NTAG215_STORAGE_SIZE       0x11U
#define NFC_NTAG216_STORAGE_SIZE       0x13U

/* Type-2 Tag Capability Container live in page 3. NDEF magic = 0xE1, 0x10. */
#define NFC_T2T_CC_MAGIC0              0xE1U
#define NFC_T2T_CC_MAGIC1              0x10U
#define NFC_T2T_CC_NTAG213_SIZE        0x12U  /* 0x12 * 8 = 144 B usable */

static void NfcTest_DecodeT2TFamily(void)
{
  g_nfc_t2t_family = 0U;
  g_nfc_t2t_is_ntag213 = 0U;

  if ((g_nfc_t2t_version_rc == RFAL_ERR_NONE) &&
      (g_nfc_t2t_version_len >= NFC_NTAG_VERSION_LEN) &&
      (g_nfc_t2t_version[1] == 0x04U) &&  /* NXP vendor ID */
      (g_nfc_t2t_version[2] == 0x04U))    /* NTAG product type */
  {
    switch (g_nfc_t2t_version[6])
    {
      case NFC_NTAG213_STORAGE_SIZE:
        g_nfc_t2t_family = 213U;
        break;
      case NFC_NTAG215_STORAGE_SIZE:
        g_nfc_t2t_family = 215U;
        break;
      case NFC_NTAG216_STORAGE_SIZE:
        g_nfc_t2t_family = 216U;
        break;
      default:
        g_nfc_t2t_family = 0U;
        break;
    }
  }

  if ((g_nfc_t2t_family == 213U) &&
      (g_nfc_t2t_read_rc == RFAL_ERR_NONE) &&
      (g_nfc_t2t_cc[0] == NFC_T2T_CC_MAGIC0) &&
      (g_nfc_t2t_cc[1] == NFC_T2T_CC_MAGIC1) &&
      (g_nfc_t2t_cc[2] == NFC_T2T_CC_NTAG213_SIZE))
  {
    g_nfc_t2t_is_ntag213 = 1U;
  }
}

/* Issue a T2T READ at page 3 plus an NTAG21x GET_VERSION on the activated tag. */
static void NfcTest_ProbeT2T(rfalNfcDevice *dev)
{
  uint8_t  rxBuf[RFAL_T2T_READ_DATA_LEN];   /* 16 bytes = 4 pages */
  uint16_t rxLen = 0U;
  uint8_t  versionCmd = (uint8_t)NFC_NTAG_GET_VERSION_CMD;
  uint8_t  versionRx[NFC_NTAG_VERSION_LEN] = {0U};
  uint16_t versionRxLen = 0U;

  g_nfc_t2t_read_rc      = RFAL_ERR_IO;
  g_nfc_t2t_read_len     = 0U;
  g_nfc_t2t_version_rc   = RFAL_ERR_IO;
  g_nfc_t2t_version_len  = 0U;
  g_nfc_t2t_family       = 0U;
  g_nfc_t2t_is_ntag213   = 0U;
  RFAL_MEMSET((void *)g_nfc_t2t_cc, 0, sizeof(g_nfc_t2t_cc));
  RFAL_MEMSET((void *)g_nfc_t2t_pages_4_6, 0, sizeof(g_nfc_t2t_pages_4_6));
  RFAL_MEMSET((void *)g_nfc_t2t_version, 0, sizeof(g_nfc_t2t_version));

  if ((dev == NULL) ||
      (dev->type != RFAL_NFC_LISTEN_TYPE_NFCA) ||
      (dev->dev.nfca.type != RFAL_NFCA_T2T))
  {
    /* Not a Type-2 tag — leave probe results zeroed and bail out. */
    return;
  }

  /* Read pages 3..6 (16 bytes). Page 3 is the Capability Container. */
  RFAL_MEMSET(rxBuf, 0, sizeof(rxBuf));
  g_nfc_t2t_read_rc = rfalT2TPollerRead(3U, rxBuf, (uint16_t)sizeof(rxBuf), &rxLen);
  g_nfc_t2t_read_len = rxLen;

  if ((g_nfc_t2t_read_rc == RFAL_ERR_NONE) && (rxLen >= 4U))
  {
    g_nfc_t2t_cc[0] = rxBuf[0];
    g_nfc_t2t_cc[1] = rxBuf[1];
    g_nfc_t2t_cc[2] = rxBuf[2];
    g_nfc_t2t_cc[3] = rxBuf[3];

    {
      uint16_t copyLen = (uint16_t)(rxLen - 4U);
      if (copyLen > (uint16_t)sizeof(g_nfc_t2t_pages_4_6))
      {
        copyLen = (uint16_t)sizeof(g_nfc_t2t_pages_4_6);
      }
      RFAL_MEMCPY((void *)g_nfc_t2t_pages_4_6, &rxBuf[4], copyLen);
    }
  }

  /*
   * NTAG21x GET_VERSION (0x60). Non-NTAG2x Type-2 tags either return a
   * 4-bit NACK or no response — that is *not* a bus failure, just a feature
   * mismatch, so we tolerate timeout / proto errors here.
   */
  g_nfc_t2t_version_rc = rfalTransceiveBlockingTxRx(
      &versionCmd, 1U,
      versionRx, (uint16_t)sizeof(versionRx), &versionRxLen,
      RFAL_TXRX_FLAGS_DEFAULT,
      rfalConvMsTo1fc(20U));
  g_nfc_t2t_version_len = versionRxLen;

  if ((g_nfc_t2t_version_rc == RFAL_ERR_NONE) && (versionRxLen >= NFC_NTAG_VERSION_LEN))
  {
    RFAL_MEMCPY((void *)g_nfc_t2t_version, versionRx, NFC_NTAG_VERSION_LEN);
  }

  NfcTest_DecodeT2TFamily();
}

void NfcTest_Poll(void)
{
  rfalNfcState state;
  rfalNfcDevice *devList = NULL;
  uint8_t devCnt = 0U;

  if (!s_nfc_ready)
  {
    return;
  }

  g_nfc_poll_count++;
  rfalNfcWorker();

  state = rfalNfcGetState();
  g_nfc_state = (uint8_t)state;

  if (state == RFAL_NFC_STATE_ACTIVATED)
  {
    if (rfalNfcGetDevicesFound(&devList, &devCnt) == RFAL_ERR_NONE)
    {
      g_nfc_tag_count = devCnt;

      if ((devCnt > 0U) && (devList != NULL) && (devList[0].nfcid != NULL))
      {
        uint8_t copyLen = devList[0].nfcidLen;

        if (copyLen > sizeof(g_nfc_tag_uid))
        {
          copyLen = (uint8_t)sizeof(g_nfc_tag_uid);
        }

        g_nfc_tag_uid_len = copyLen;
        RFAL_MEMCPY((void *)g_nfc_tag_uid, devList[0].nfcid, copyLen);
        g_nfc_tag_detect_count++;

        DBG_I("NFC", "tag activated #%lu len=%u",
              (unsigned long)g_nfc_tag_detect_count, (unsigned)copyLen);
        DBG_HEX_I("NFC", "uid=", (const void *)g_nfc_tag_uid, copyLen);

        /*
         * Authenticate the component sticker FIRST: the secure read uses
         * standard T2T READ which any Type-2 tag supports. The GET_VERSION
         * sent by NfcTest_ProbeT2T is NTAG21x-specific and can put non-NTAG
         * tags into a halt/error state, so we run it after.
         */
        (void)NfcSecure_OnTagActivated(&devList[0]);

        /* Probe T2T memory and GET_VERSION while the tag is still selected. */
        NfcTest_ProbeT2T(&devList[0]);

        if (g_nfc_t2t_is_ntag213 != 0U)
        {
          DBG_D("NFC", "NTAG213 confirmed (cc=E1 10 12 ..)");
        }
        else if (g_nfc_t2t_family != 0U)
        {
          DBG_D("NFC", "NTAG family=%u (non-213)", (unsigned)g_nfc_t2t_family);
        }
        else
        {
          DBG_D("NFC", "T2T probe rc=%d len=%u ver_rc=%d",
                (int)g_nfc_t2t_read_rc, (unsigned)g_nfc_t2t_read_len,
                (int)g_nfc_t2t_version_rc);
        }
      }
    }

    (void)rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
    s_discovery_active = false;
  }

  if (!s_discovery_active && (rfalNfcGetState() == RFAL_NFC_STATE_IDLE))
  {
    if (NfcTest_StartDiscovery() == RFAL_ERR_NONE)
    {
      s_discovery_active = true;
    }
  }
}
