/**
  ******************************************************************************
  * @file    rfal_platform.c
  * @brief   Platform glue for ST25RFAL on STM32H747 (SPI1 + NFC_INT)
  ******************************************************************************
  */

#include "rfal_platform.h"

static volatile uint8_t s_commProtectCnt = 0U;
static void (*s_st25rIrqCallback)(void) = NULL;
volatile uint32_t g_nfc_spi_hal_status = 0U;

void platformDelay(uint32_t ms)
{
  HAL_Delay(ms);
}

uint32_t platformTimerCreate(uint16_t timeMs)
{
  return (HAL_GetTick() + (uint32_t)timeMs);
}

bool platformTimerIsExpired(uint32_t timer)
{
  return ((int32_t)(HAL_GetTick() - timer) >= 0);
}

void platformTimerDestroy_func(uint32_t timer)
{
  (void)timer;
}

void platformProtectST25RComm(void)
{
  s_commProtectCnt++;
  __DSB();
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  __DSB();
  __ISB();
}

void platformUnprotectST25RComm(void)
{
  if (s_commProtectCnt > 0U)
  {
    s_commProtectCnt--;
  }

  if (s_commProtectCnt == 0U)
  {
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
}

void platformSpiSelect(void)
{
  HAL_GPIO_WritePin(ST25R_SS_PORT, ST25R_SS_PIN, GPIO_PIN_RESET);
}

void platformSpiDeselect(void)
{
  HAL_GPIO_WritePin(ST25R_SS_PORT, ST25R_SS_PIN, GPIO_PIN_SET);
}

void platformSpiTxRx(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len)
{
  HAL_StatusTypeDef halRc = HAL_OK;

  if (len == 0U)
  {
    return;
  }

  if ((txBuf != NULL) && (rxBuf != NULL))
  {
    halRc = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)txBuf, rxBuf, len, 1000U);
  }
  else if (txBuf != NULL)
  {
    halRc = HAL_SPI_Transmit(&hspi1, (uint8_t *)txBuf, len, 1000U);
  }
  else if (rxBuf != NULL)
  {
    halRc = HAL_SPI_Receive(&hspi1, rxBuf, len, 1000U);
  }
  else
  {
    /* Nothing to transfer */
  }

  g_nfc_spi_hal_status = (uint32_t)halRc;
}

void platformIrqST25RPinInitialize_func(void)
{
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void platformIrqST25RSetCallback_func(void (*cb)(void))
{
  s_st25rIrqCallback = cb;
}

void platformNfcIrqHandler(void)
{
  if (s_st25rIrqCallback != NULL)
  {
    s_st25rIrqCallback();
  }
}
