/**
  ******************************************************************************
  * @file    rfal_platform.h
  * @brief   Platform glue for ST25RFAL on STM32H747 (SPI1 + NFC_INT)
  ******************************************************************************
  */

#ifndef RFAL_PLATFORM_H
#define RFAL_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* ST25R3916B is defined in the Keil CM7 target compiler options */

#define ST25R_SS_PIN                    SPI1_CS_Pin
#define ST25R_SS_PORT                   SPI1_CS_GPIO_Port
#define ST25R_INT_PIN                   NFC_INT_Pin
#define ST25R_INT_PORT                  NFC_INT_GPIO_Port

extern SPI_HandleTypeDef hspi1;
extern volatile uint32_t g_nfc_spi_hal_status;

void platformNfcIrqHandler(void);

#define platformProtectST25RIrqStatus()     __disable_irq()
#define platformUnprotectST25RIrqStatus()   __enable_irq()

#define platformProtectWorker()
#define platformUnprotectWorker()

#define platformGpioSet(port, pin)          HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define platformGpioClear(port, pin)        HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define platformGpioToggle(port, pin)       HAL_GPIO_TogglePin((port), (pin))
#define platformGpioIsHigh(port, pin)       (HAL_GPIO_ReadPin((port), (pin)) == GPIO_PIN_SET)
#define platformGpioIsLow(port, pin)        (!platformGpioIsHigh((port), (pin)))

#define platformLedOff(port, pin)           platformGpioClear((port), (pin))
#define platformLedOn(port, pin)            platformGpioSet((port), (pin))
#define platformLedToggle(port, pin)        platformGpioToggle((port), (pin))

#define platformGetSysTick()                HAL_GetTick()

void     platformDelay(uint32_t ms);
uint32_t platformTimerCreate(uint16_t timeMs);
bool     platformTimerIsExpired(uint32_t timer);
#define platformTimerDestroy(timer)         platformTimerDestroy_func(timer)
void     platformTimerDestroy_func(uint32_t timer);

void platformProtectST25RComm(void);
void platformUnprotectST25RComm(void);

void platformSpiSelect(void);
void platformSpiDeselect(void);
void platformSpiTxRx(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len);

#define platformIrqST25RPinInitialize()     platformIrqST25RPinInitialize_func()
#define platformIrqST25RSetCallback(cb)     platformIrqST25RSetCallback_func(cb)
void platformIrqST25RPinInitialize_func(void);
void platformIrqST25RSetCallback_func(void (*cb)(void));

#define platformLedsInitialize()
#define platformLog(...)
#define platformAssert(exp)
#define platformErrorHandle()               __BKPT(0)

/*
 * STM32H747 CMSIS defines DSI as the Display Serial Interface peripheral.
 * RFAL uses DSI as an NFC bit-rate identifier in struct members and parameters.
 */
#ifdef DSI
#undef DSI
#endif

#include "rfal_defConfig.h"

#ifdef __cplusplus
}
#endif

#endif /* RFAL_PLATFORM_H */
