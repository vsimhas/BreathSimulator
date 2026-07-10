/**
  ******************************************************************************
  * @file    pressure_sensors.c
  * @brief   AMS5935 SPI pressure sensors and analog backup pressure ADC.
  ******************************************************************************
  */

#include "pressure_sensors.h"

#define AMS5935_CMD_SINGLE_MEAS       0xAAU
#define AMS5935_CMD_OVERSAMPLE_MEAS   0xADU
#define AMS5935_CMD_READ              0xF0U

#define AMS5935_SPI_TIMEOUT_MS        100U
#define AMS5935_SPI_CS_HOLD_NOP_COUNT 200U

#define AMS5935_DIGOUT_MIN            (0.1f * 16777216.0f)
#define AMS5935_DIGOUT_MAX            (0.9f * 16777216.0f)

#define PRESSURE_ANALOG_ADC_FULL_SCALE  65535.0f
#define PRESSURE_ANALOG_REF_VOLTAGE_V   3.3f
#define PRESSURE_ANALOG_DIVIDER_GAIN    3.0f
#define PRESSURE_ANALOG_DIVIDER_ATTEN   2.0f
#define PRESSURE_ANALOG_V_MIN_V         0.5f
#define PRESSURE_ANALOG_V_SPAN_V        4.0f

static void ams5935_cs_low(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void ams5935_cs_high(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

volatile HAL_StatusTypeDef g_pressure_spi_last_status = HAL_OK;

static HAL_StatusTypeDef ams5935_tx_rx(AMS5935_HandleTypeDef *dev,
                                       uint8_t *tx,
                                       uint8_t *rx,
                                       uint16_t len)
{
    HAL_StatusTypeDef ret;

    ams5935_cs_low(dev);
    ret = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len, AMS5935_SPI_TIMEOUT_MS);
    g_pressure_spi_last_status = ret;

    /* Datasheet: short wait before deasserting CS. */
    for (volatile uint32_t i = 0U; i < AMS5935_SPI_CS_HOLD_NOP_COUNT; i++)
    {
        __NOP();
    }

    ams5935_cs_high(dev);
    return ret;
}

HAL_StatusTypeDef AMS5935_Init(AMS5935_HandleTypeDef *dev)
{
    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    ams5935_cs_high(dev);
    /* Datasheet startup: 2.5 ms, no SCLK during this window. */
    HAL_Delay(5U);
    return HAL_OK;
}

HAL_StatusTypeDef AMS5935_StartSingleMeasurement(AMS5935_HandleTypeDef *dev)
{
    uint8_t tx[3] = { AMS5935_CMD_SINGLE_MEAS, 0x00U, 0x00U };
    uint8_t rx[3] = { 0U };

    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    return ams5935_tx_rx(dev, tx, rx, 3U);
}

HAL_StatusTypeDef AMS5935_StartOversamplingMeasurement(AMS5935_HandleTypeDef *dev)
{
    uint8_t tx[3] = { AMS5935_CMD_OVERSAMPLE_MEAS, 0x00U, 0x00U };
    uint8_t rx[3] = { 0U };

    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    return ams5935_tx_rx(dev, tx, rx, 3U);
}

HAL_StatusTypeDef AMS5935_ReadRaw(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data)
{
    uint8_t tx[7] = { AMS5935_CMD_READ, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U };
    uint8_t rx[7] = { 0U };
    HAL_StatusTypeDef ret;

    if ((dev == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    ret = ams5935_tx_rx(dev, tx, rx, 7U);
    if (ret != HAL_OK)
    {
        return ret;
    }

    data->status = rx[0];
    data->pressure_raw =
        ((uint32_t)rx[1] << 16) |
        ((uint32_t)rx[2] << 8)  |
        ((uint32_t)rx[3]);

    data->temperature_raw =
        ((uint32_t)rx[4] << 16) |
        ((uint32_t)rx[5] << 8)  |
        ((uint32_t)rx[6]);

    data->pressure = AMS5935_ConvertPressure(data->pressure_raw, dev->p_min, dev->p_max);
    data->temperature_c = AMS5935_ConvertTemperature(data->temperature_raw);

    return HAL_OK;
}

HAL_StatusTypeDef AMS5935_ReadMeasurement(AMS5935_HandleTypeDef *dev,
                                          AMS5935_Data_t *data,
                                          uint8_t oversampling)
{
    HAL_StatusTypeDef ret;

    if ((dev == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    if (oversampling != 0U)
    {
        ret = AMS5935_StartOversamplingMeasurement(dev);
    }
    else
    {
        ret = AMS5935_StartSingleMeasurement(dev);
    }

    if (ret != HAL_OK)
    {
        return ret;
    }

    if (dev->use_eoc != 0U)
    {
        uint32_t t0_ms = HAL_GetTick();
        while (HAL_GPIO_ReadPin(dev->eoc_port, dev->eoc_pin) == GPIO_PIN_RESET)
        {
            if ((HAL_GetTick() - t0_ms) > 50U)
            {
                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        if (oversampling != 0U)
        {
            HAL_Delay(15U); /* datasheet: 14.5 ms */
        }
        else
        {
            HAL_Delay(6U);  /* datasheet: 4.0 ms */
        }
    }

    return AMS5935_ReadRaw(dev, data);
}

float AMS5935_ConvertPressure(uint32_t raw_p, float p_min, float p_max)
{
    const float sensp = (AMS5935_DIGOUT_MAX - AMS5935_DIGOUT_MIN) / (p_max - p_min);

    return (((float)raw_p - AMS5935_DIGOUT_MIN) / sensp) + p_min;
}

float AMS5935_ConvertTemperature(uint32_t raw_t)
{
    return (((float)raw_t * 165.0f) / 16777216.0f) - 40.0f;
}

uint8_t AMS5935_StatusBusy(uint8_t status)
{
    return (status >> 5) & 0x01U;
}

uint8_t AMS5935_StatusMemoryError(uint8_t status)
{
    return (status >> 2) & 0x01U;
}

uint8_t AMS5935_StatusOverflow(uint8_t status)
{
    return status & 0x01U;
}

extern ADC_HandleTypeDef hadc3;

volatile uint16_t g_pressure_analog_adc_raw = 0U;
volatile HAL_StatusTypeDef g_pressure_analog_adc_start_status = HAL_OK;
volatile HAL_StatusTypeDef g_pressure_analog_adc_poll_status = HAL_OK;
float g_pressure_analog_voltage_v = 0.0f;
float g_pressure_analog_mbar = 0.0f;
float g_pressure_analog_min_mbar = -50.0f;
float g_pressure_analog_max_mbar = 50.0f;

uint16_t pressure_sensor_read_analog_adc_raw(void)
{
    g_pressure_analog_adc_start_status = HAL_ADC_Start(&hadc3);
    if (g_pressure_analog_adc_start_status != HAL_OK)
    {
        return g_pressure_analog_adc_raw;
    }

    g_pressure_analog_adc_poll_status = HAL_ADC_PollForConversion(&hadc3, 10U);
    if (g_pressure_analog_adc_poll_status == HAL_OK)
    {
        g_pressure_analog_adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc3);
    }

    g_pressure_analog_voltage_v =
        ((((float)g_pressure_analog_adc_raw) / PRESSURE_ANALOG_ADC_FULL_SCALE)
         * PRESSURE_ANALOG_REF_VOLTAGE_V * PRESSURE_ANALOG_DIVIDER_GAIN)
        / PRESSURE_ANALOG_DIVIDER_ATTEN;

    /* AMS5105-0050-D-B: 0.5 V @ -50 mbar, 2.5 V @ 0, 4.5 V @ +50 mbar */
    g_pressure_analog_mbar = g_pressure_analog_min_mbar
        + ((g_pressure_analog_voltage_v - PRESSURE_ANALOG_V_MIN_V) / PRESSURE_ANALOG_V_SPAN_V)
          * (g_pressure_analog_max_mbar - g_pressure_analog_min_mbar);

    (void)HAL_ADC_Stop(&hadc3);
    return g_pressure_analog_adc_raw;
}
