/**
  ******************************************************************************
  * @file    pressure_sensors.h
  * @brief   AMS5935 SPI pressure sensors.
  *
  * Ported from the CPAP_InitialCode firmware, which runs on this same board
  * with the same pinout. The analog (AMS5105 / ADC3) backup channel from
  * that project is deliberately not carried over - it is not populated on
  * the breath simulator schematic. Everything else is kept byte-compatible
  * so fixes can be moved between the two projects.
  *
  * Wiring on this board (both on SPI4, software chip select):
  *
  *   U11  AMS5935-1200   CS = SPI4_CS1 = PE4    barometric, 700..1200 mbar
  *   U24  AMS5935-0050   CS = SPI4_CS2 = PG3    gauge/diff, -50..+50 mbar
  *
  * SPI4 is already configured by CubeMX as master, 8-bit, CPOL=0, CPHA=0,
  * prescaler 32 (3.125 Mbit/s) - the same settings the CPAP firmware uses,
  * so no SPI changes are needed.
  *
  * Protocol: 0xAA starts a single conversion (~4 ms), 0xAD an oversampled
  * one (~14.5 ms), 0xF0 reads a 7-byte frame of status + 24-bit pressure +
  * 24-bit temperature. The EOC pin is not wired on this board, so the host
  * must wait out the conversion time rather than watch a ready line.
  *
  * Units: the driver reports mbar, which is what the part is calibrated in.
  * For CPAP work multiply by AMS5935_MBAR_TO_CMH2O.
  ******************************************************************************
  */

#ifndef PRESSURE_SENSORS_H
#define PRESSURE_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/** 1 mbar = 1.01972 cmH2O. CPAP pressures are quoted in cmH2O. */
#define AMS5935_MBAR_TO_CMH2O   1.01972f

typedef struct
{
    uint8_t  status;
    uint32_t pressure_raw;
    uint32_t temperature_raw;
    float    pressure;          /**< mbar */
    float    temperature_c;
} AMS5935_Data_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;

    GPIO_TypeDef *eoc_port;
    uint16_t      eoc_pin;
    uint8_t       use_eoc;

    float p_min;                /**< mbar at 10% of full digital span */
    float p_max;                /**< mbar at 90% of full digital span */

    /* Last conversion that was not busy / memory-error / overflow.
     * ReadRaw substitutes this when the sensor reports a bad sample so a
     * caller never sees a one-tick garbage pressure. */
    uint8_t        last_valid;
    AMS5935_Data_t last_good;

    /* 1 = the sample just returned by ReadRaw is a repeat of last_good,
     * not a finished conversion. Callers must not treat it as new data. */
    uint8_t  held_last;
    uint32_t held_count;

    /* Bring-up bookkeeping, not present in the CPAP version. The breath
     * simulator has to be able to say "is this sensor actually there?"
     * without a debugger attached. */
    uint32_t read_count;
    uint32_t error_count;
    uint8_t  present;           /**< 1 once a clean conversion has landed. */
    float    zero_offset_mbar;  /**< subtracted from every reading */
} AMS5935_HandleTypeDef;

HAL_StatusTypeDef AMS5935_Init(AMS5935_HandleTypeDef *dev);
HAL_StatusTypeDef AMS5935_StartSingleMeasurement(AMS5935_HandleTypeDef *dev);
HAL_StatusTypeDef AMS5935_StartOversamplingMeasurement(AMS5935_HandleTypeDef *dev);
HAL_StatusTypeDef AMS5935_ReadRaw(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data);
HAL_StatusTypeDef AMS5935_ReadMeasurement(AMS5935_HandleTypeDef *dev,
                                          AMS5935_Data_t *data,
                                          uint8_t oversampling);

float AMS5935_ConvertPressure(uint32_t raw_p, float p_min, float p_max);
float AMS5935_ConvertTemperature(uint32_t raw_t);

/** 1 = last ReadRaw returned a repeated (held) sample, not fresh data. */
uint8_t AMS5935_WasSampleHeld(const AMS5935_HandleTypeDef *dev);

uint8_t AMS5935_StatusBusy(uint8_t status);
uint8_t AMS5935_StatusMemoryError(uint8_t status);
uint8_t AMS5935_StatusOverflow(uint8_t status);

/** Tare: take @p current_mbar as the new zero. Only valid at zero pressure. */
void AMS5935_ZeroNow(AMS5935_HandleTypeDef *dev, float current_mbar);
void AMS5935_ClearZero(AMS5935_HandleTypeDef *dev);

extern volatile HAL_StatusTypeDef g_pressure_spi_last_status;

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_SENSORS_H */
