/**
  ******************************************************************************
  * @file    pressure_sensors.c
  * @brief   AMS5935 SPI pressure sensors.
  *
  * Ported from CPAP_InitialCode/CM7/Core/Src/pressure_sensors.c. The read
  * path (status validation, span check, torn-register detection, hold-last)
  * is unchanged from that file: it was tuned against this exact sensor on
  * this exact board and the failure modes it guards are real. Keep the two
  * copies in step.
  ******************************************************************************
  */

#include "pressure_sensors.h"

#define AMS5935_CMD_SINGLE_MEAS       0xAAU
#define AMS5935_CMD_OVERSAMPLE_MEAS   0xADU
#define AMS5935_CMD_READ              0xF0U

#define AMS5935_SPI_TIMEOUT_MS        100U
#define AMS5935_SPI_CS_HOLD_NOP_COUNT 200U

/* The part maps its calibrated span onto 10%..90% of the 24-bit range. */
#define AMS5935_DIGOUT_MIN            (0.1f * 16777216.0f)
#define AMS5935_DIGOUT_MAX            (0.9f * 16777216.0f)
#define AMS5935_DIGOUT_MIN_U32        1677722UL
#define AMS5935_DIGOUT_MAX_U32        15099494UL

/* Two back-to-back F0 reads of a finished conversion must match; a torn
 * 24-bit update disagrees by thousands of counts. */
#define AMS5935_TEAR_MAX_COUNTS       1024UL

volatile HAL_StatusTypeDef g_pressure_spi_last_status = HAL_OK;

static void ams5935_cs_low(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void ams5935_cs_high(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

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

    /* Deassert first. CubeMX's MX_GPIO_Init leaves both SPI4 chip selects
     * LOW at boot, so until every device on the bus has been through here
     * two of them can be driving MISO at once. */
    ams5935_cs_high(dev);
    dev->last_valid  = 0U;
    dev->held_last   = 0U;
    dev->held_count  = 0U;
    dev->read_count  = 0U;
    dev->error_count = 0U;
    dev->present     = 0U;

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

static void ams5935_parse_frame(const uint8_t *rx, AMS5935_Data_t *data,
                                float p_min, float p_max)
{
    data->status = rx[0];
    data->pressure_raw =
        ((uint32_t)rx[1] << 16) |
        ((uint32_t)rx[2] << 8)  |
        ((uint32_t)rx[3]);
    data->temperature_raw =
        ((uint32_t)rx[4] << 16) |
        ((uint32_t)rx[5] << 8)  |
        ((uint32_t)rx[6]);
    data->pressure = AMS5935_ConvertPressure(data->pressure_raw, p_min, p_max);
    data->temperature_c = AMS5935_ConvertTemperature(data->temperature_raw);
}

static uint8_t ams5935_payload_unreliable(const AMS5935_Data_t *data)
{
    if ((AMS5935_StatusBusy(data->status) != 0U)
        || (AMS5935_StatusMemoryError(data->status) != 0U)
        || (AMS5935_StatusOverflow(data->status) != 0U))
    {
        return 1U;
    }
    if ((data->pressure_raw < AMS5935_DIGOUT_MIN_U32)
        || (data->pressure_raw > AMS5935_DIGOUT_MAX_U32))
    {
        return 1U;
    }
    return 0U;
}

static HAL_StatusTypeDef ams5935_read_frame(AMS5935_HandleTypeDef *dev,
                                            AMS5935_Data_t *data)
{
    uint8_t tx[7] = { AMS5935_CMD_READ, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U };
    uint8_t rx[7] = { 0U };
    const HAL_StatusTypeDef ret = ams5935_tx_rx(dev, tx, rx, 7U);
    if (ret != HAL_OK)
    {
        return ret;
    }
    ams5935_parse_frame(rx, data, dev->p_min, dev->p_max);
    return HAL_OK;
}

static HAL_StatusTypeDef ams5935_hold_last(AMS5935_HandleTypeDef *dev,
                                           AMS5935_Data_t *data)
{
    dev->held_last = 1U;
    dev->held_count++;
    if (dev->last_valid != 0U)
    {
        const uint8_t bad_status = data->status;
        *data = dev->last_good;
        data->status = bad_status;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef AMS5935_ReadRaw(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data)
{
    AMS5935_Data_t second;
    uint32_t tear;
    HAL_StatusTypeDef ret;

    if ((dev == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    ret = ams5935_read_frame(dev, data);
    if (ret != HAL_OK)
    {
        dev->error_count++;
        return ret;
    }

    /* Busy / memory-error / analog-overflow / out-of-span: the 24-bit
     * payload is not a finished conversion. Hold the last good sample. */
    if (ams5935_payload_unreliable(data) != 0U)
    {
        return ams5935_hold_last(dev, data);
    }

    /* Status can read "ready" while the 24-bit pressure register is still
     * updating (no EOC pin wired). A second F0 must match; a torn byte
     * does not. */
    ret = ams5935_read_frame(dev, &second);
    if (ret != HAL_OK)
    {
        dev->error_count++;
        return ams5935_hold_last(dev, data);
    }
    if (ams5935_payload_unreliable(&second) != 0U)
    {
        return ams5935_hold_last(dev, data);
    }

    tear = (data->pressure_raw > second.pressure_raw)
               ? (data->pressure_raw - second.pressure_raw)
               : (second.pressure_raw - data->pressure_raw);
    if (tear > AMS5935_TEAR_MAX_COUNTS)
    {
        return ams5935_hold_last(dev, data);
    }

    dev->last_good  = *data;
    dev->last_valid = 1U;
    dev->held_last  = 0U;
    dev->present    = 1U;
    dev->read_count++;
    return HAL_OK;
}

uint8_t AMS5935_WasSampleHeld(const AMS5935_HandleTypeDef *dev)
{
    return (dev != NULL) ? dev->held_last : 1U;
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

void AMS5935_ZeroNow(AMS5935_HandleTypeDef *dev, float current_mbar)
{
    if (dev == NULL) { return; }
    dev->zero_offset_mbar += current_mbar;
}

void AMS5935_ClearZero(AMS5935_HandleTypeDef *dev)
{
    if (dev == NULL) { return; }
    dev->zero_offset_mbar = 0.0f;
}
