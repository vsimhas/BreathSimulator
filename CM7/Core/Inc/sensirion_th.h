/**
  ******************************************************************************
  * @file    sensirion_th.h
  * @brief   Driver for Sensirion SHT4x (RH+T) and STS4x (T-only) I2C sensors.
  *
  *  - SHT4x : Relative Humidity + Temperature, default 7-bit address 0x44
  *            Wired on I2C2 in this project.
  *  - STS4x : Temperature only, default 7-bit address 0x44
  *            Wired on I2C1 in this project.
  *
  *  Both devices use the same protocol family: a 1-byte command followed
  *  by a delayed read of 16-bit words, each followed by a CRC-8 byte
  *  (polynomial 0x31, init 0xFF).
  ******************************************************************************
  */

#ifndef SENSIRION_TH_H
#define SENSIRION_TH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 7-bit I2C addresses (HAL expects the address shifted left by 1). */
#define SENSIRION_I2C_ADDR_44      (0x44U << 1)
#define SENSIRION_I2C_ADDR_45      (0x45U << 1)
#define SENSIRION_I2C_ADDR_46      (0x46U << 1)

/* Single-shot measurement commands (same on SHT4x and STS4x). */
#define SENSIRION_CMD_MEAS_HIGH    0xFDU  /* high precision   (~8.3 ms max) */
#define SENSIRION_CMD_MEAS_MED     0xF6U  /* medium precision (~4.5 ms max) */
#define SENSIRION_CMD_MEAS_LOW     0xE0U  /* low precision    (~1.6 ms max) */
#define SENSIRION_CMD_READ_SERIAL  0x89U
#define SENSIRION_CMD_SOFT_RESET   0x94U

/* SHT4x heater commands (do not use on STS4x — no heater). */
#define SHT4X_CMD_HEATER_200MW_1S  0x39U
#define SHT4X_CMD_HEATER_200MW_01S 0x32U
#define SHT4X_CMD_HEATER_110MW_1S  0x2FU
#define SHT4X_CMD_HEATER_110MW_01S 0x24U
#define SHT4X_CMD_HEATER_20MW_1S   0x1EU
#define SHT4X_CMD_HEATER_20MW_01S  0x15U

typedef struct
{
    I2C_HandleTypeDef *hi2c;    /* I2C bus the sensor sits on */
    uint16_t           addr;    /* shifted 7-bit address (use SENSIRION_I2C_ADDR_xx) */
    uint8_t            has_rh;  /* 1 = SHT4x (read T+RH), 0 = STS4x (read T only)   */

    /* Last successful conversion (also valid debug values). */
    float              temperature_c;
    float              humidity_pct;  /* always 0 for STS4x */
    uint16_t           raw_t_ticks;
    uint16_t           raw_rh_ticks;
    uint32_t           read_count;
    uint32_t           error_count;
    HAL_StatusTypeDef  last_status;
    uint8_t            last_crc_ok;   /* 1 = both CRCs OK, 0 = CRC mismatch */
} SensirionTH_Handle_t;

/**
  * @brief Initialise a handle and issue a soft reset to the device.
  * @retval HAL_OK if the device acknowledged the reset.
  */
HAL_StatusTypeDef SensirionTH_Init(SensirionTH_Handle_t *h,
                                   I2C_HandleTypeDef *hi2c,
                                   uint16_t addr_shifted,
                                   uint8_t has_rh);

/**
  * @brief Probe device by issuing a soft reset (cheap "are you there?" check).
  */
HAL_StatusTypeDef SensirionTH_Probe(SensirionTH_Handle_t *h);

/**
  * @brief Run one high-precision blocking measurement.
  *        Updates h->temperature_c (and h->humidity_pct on SHT4x).
  *
  * Total worst-case wall-clock cost: ~10 ms (write 1 byte, wait, read 3/6).
  */
HAL_StatusTypeDef SensirionTH_ReadMeasurement(SensirionTH_Handle_t *h);

/**
  * @brief Read the unique 32-bit serial number (works on both parts).
  */
HAL_StatusTypeDef SensirionTH_ReadSerial(SensirionTH_Handle_t *h, uint32_t *serial);

#ifdef __cplusplus
}
#endif

#endif /* SENSIRION_TH_H */
