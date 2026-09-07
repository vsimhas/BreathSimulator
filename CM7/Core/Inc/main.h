/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile uint32_t sd_test_step;
extern volatile uint32_t sd_test_fail_step;
extern volatile HAL_StatusTypeDef sd_test_result;
extern volatile uint8_t sd_card_present;
extern volatile uint32_t sd_hal_error;
extern volatile uint32_t sd_sdmmc_ker_clk_hz;
extern volatile uint32_t sd_sdmmc_clkcr;
extern volatile uint32_t sd_sdmmc_power;
extern volatile uint32_t sd_sdmmc_sta;
extern volatile uint32_t sd_sdmmc_resp1;
extern volatile uint32_t sd_cmd_idle;
extern volatile uint32_t sd_d0_idle;
extern volatile uint32_t sd_init_attempt;
extern HAL_SD_CardInfoTypeDef sd_card_info;
extern volatile uint32_t qspi_test_step;
extern volatile HAL_StatusTypeDef qspi_test_result;
extern volatile uint8_t qspi_jedec_id[3];
extern volatile uint32_t qspi_hal_error;
extern volatile uint32_t qspi_read_substep;
extern volatile uint32_t sdram_test_step;
extern volatile HAL_StatusTypeDef sdram_test_result;
extern volatile uint32_t sdram_test_fail_index;
extern volatile uint32_t sdram_test_fail_expected;
extern volatile uint32_t sdram_test_fail_actual;
extern volatile uint32_t clock_config_step;
extern volatile uint32_t boot_stage;
extern volatile uint32_t boot_sync_step;
/* Prefer boot_trace_stage — survives __main; use BOOT_TRACE_RTC_STAGE if still 0. */
extern volatile uint32_t boot_trace_stage;
extern volatile uint32_t boot_trace_sync;
#define BOOT_TRACE_STAGE   boot_trace_stage
#define BOOT_TRACE_SYNC    boot_trace_sync
extern volatile uint32_t ltdc_cfbbar;
extern volatile uint32_t ltdc_gcr;
extern volatile uint32_t hf_boot_stage;
extern volatile uint32_t hf_cfsr;
extern volatile uint32_t hf_hfsr;
extern volatile uint32_t hf_bfar;
extern volatile uint32_t hf_mmfar;
extern volatile uint32_t hf_ufsr;
extern volatile uint32_t hf_pc;
extern volatile uint32_t hf_lr;
extern volatile uint32_t hf_cpacr;
extern volatile uint32_t hf_ccr;
extern volatile uint32_t hf_this;
extern volatile uint32_t hf_sp;
extern volatile uint32_t hf_fault_addr;
extern volatile uint32_t hf_unalign_recoveries;
extern LTDC_HandleTypeDef hltdc;
extern volatile uint32_t lcd_test_step;
extern volatile HAL_StatusTypeDef lcd_test_result;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SDIO_DET_Pin GPIO_PIN_3
#define SDIO_DET_GPIO_Port GPIOE
#define SPI4_CS1_Pin GPIO_PIN_4
#define SPI4_CS1_GPIO_Port GPIOE
#define NFC_INT_Pin GPIO_PIN_8
#define NFC_INT_GPIO_Port GPIOI
#define NFC_INT_EXTI_IRQn EXTI9_5_IRQn
#define OTG_FS_PWR_ON_Pin GPIO_PIN_13
#define OTG_FS_PWR_ON_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define BL_CTL_Pin GPIO_PIN_0
#define BL_CTL_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI4_CS2_Pin GPIO_PIN_3
#define SPI4_CS2_GPIO_Port GPIOG
#define ESP32_TX_Pin GPIO_PIN_5
#define ESP32_TX_GPIO_Port GPIOD
#define ESP32_RX_Pin GPIO_PIN_6
#define ESP32_RX_GPIO_Port GPIOD
#define ROT_SW_Pin GPIO_PIN_5
#define ROT_SW_GPIO_Port GPIOB
#define ROT_SW_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/**
  * Climate sensors (Sensirion SHT4x on I2C2, STS4x on I2C1) and the
  * humidifier control loop that depends on them.
  *
  * Set to 0 for breath-simulator work: the rig does not need humidity or
  * tube temperature, and the STS4x read blocks I2C1 for ~10 ms every
  * 250 ms, which is the only thing competing with the SFM3300 flow sensor
  * for the bus. With this at 0 the flow sensor has I2C1 to itself and
  * neither I2C peripheral carries any other traffic.
  *
  * Set back to 1 to restore both sensors and the humidifier loop; nothing
  * else has to change.
  */
#ifndef CLIMATE_SENSORS_ENABLED
#define CLIMATE_SENSORS_ENABLED   0
#endif

/**
  * Barometric pressure sensor U11 (AMS5935-1200, SPI4 CS1 = PE4).
  *
  * Set to 0 while the part is not fitted. With it off the working pressure
  * sensor U24 gets every SPI4 slot (a true 100 Hz instead of 95 Hz), and
  * the log is not cluttered with probe failures for a device that is not
  * there. U11's chip select is still driven inactive at startup either way
  * - MX_GPIO_Init leaves both SPI4 chip selects asserted, so deasserting is
  * not optional even for an unpopulated footprint.
  *
  * Set to 1 when U11 is populated; it is only needed for density/BTPS
  * correction of the flow reading, which matters once flow is used
  * quantitatively rather than just observed.
  */
#ifndef PRESSURE_AS_ENABLED
#define PRESSURE_AS_ENABLED       0
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
