
#ifndef __PRESSURE_SENSORS_H
#define __PRESSURE_SENSORS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    GPIO_TypeDef *eoc_port;
    uint16_t eoc_pin;
    uint8_t use_eoc;

    float p_min;
    float p_max;
} AMS5935_HandleTypeDef;

typedef struct
{
    uint8_t status;
    uint32_t pressure_raw;
    uint32_t temperature_raw;
    float pressure;
    float temperature_c;
} AMS5935_Data_t;

HAL_StatusTypeDef AMS5935_Init(AMS5935_HandleTypeDef *dev);
HAL_StatusTypeDef AMS5935_StartSingleMeasurement(AMS5935_HandleTypeDef *dev);
HAL_StatusTypeDef AMS5935_StartOversamplingMeasurement(AMS5935_HandleTypeDef *dev);
HAL_StatusTypeDef AMS5935_ReadRaw(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data);
HAL_StatusTypeDef AMS5935_ReadMeasurement(AMS5935_HandleTypeDef *dev,
                                          AMS5935_Data_t *data,
                                          uint8_t oversampling);

float AMS5935_ConvertPressure(uint32_t raw_p, float p_min, float p_max);
float AMS5935_ConvertTemperature(uint32_t raw_t);

uint8_t AMS5935_StatusBusy(uint8_t status);
uint8_t AMS5935_StatusMemoryError(uint8_t status);
uint8_t AMS5935_StatusOverflow(uint8_t status);

extern volatile HAL_StatusTypeDef g_pressure_spi_last_status;
extern volatile uint16_t g_pressure_analog_adc_raw;
extern float g_pressure_analog_mbar;

uint16_t pressure_sensor_read_analog_adc_raw(void);

#endif
