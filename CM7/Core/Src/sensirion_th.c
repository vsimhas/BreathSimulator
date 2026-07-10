/**
  ******************************************************************************
  * @file    sensirion_th.c
  * @brief   Driver for Sensirion SHT4x (RH+T) and STS4x (T-only) I2C sensors.
  ******************************************************************************
  */

#include "sensirion_th.h"
#include <string.h>

/* CRC-8: polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0xFF, no reflect, no XOR-out.
 * Matches table 7 of the SHT4x datasheet and table 5 of the STS4x datasheet.
 */
static uint8_t SensirionTH_Crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFFU;
    for (uint32_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint32_t b = 0U; b < 8U; b++)
        {
            crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0x31U) : (crc << 1));
        }
    }
    return crc;
}

static HAL_StatusTypeDef SensirionTH_WriteCmd(SensirionTH_Handle_t *h, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(h->hi2c, h->addr, &cmd, 1U, 100U);
}

HAL_StatusTypeDef SensirionTH_Probe(SensirionTH_Handle_t *h)
{
    HAL_StatusTypeDef st;

    if ((h == NULL) || (h->hi2c == NULL))
    {
        return HAL_ERROR;
    }

    /* Soft reset is the cheapest harmless command to confirm the device responds. */
    st = SensirionTH_WriteCmd(h, SENSIRION_CMD_SOFT_RESET);
    h->last_status = st;
    if (st == HAL_OK)
    {
        HAL_Delay(2U); /* tSR max = 1 ms — wait a bit longer to be safe */
    }
    return st;
}

HAL_StatusTypeDef SensirionTH_Init(SensirionTH_Handle_t *h,
                                   I2C_HandleTypeDef *hi2c,
                                   uint16_t addr_shifted,
                                   uint8_t has_rh)
{
    if ((h == NULL) || (hi2c == NULL))
    {
        return HAL_ERROR;
    }

    memset(h, 0, sizeof(*h));
    h->hi2c   = hi2c;
    h->addr   = addr_shifted;
    h->has_rh = has_rh ? 1U : 0U;

    /* Power-up time tPU max = 1 ms — caller has typically already waited far longer. */
    return SensirionTH_Probe(h);
}

HAL_StatusTypeDef SensirionTH_ReadMeasurement(SensirionTH_Handle_t *h)
{
    HAL_StatusTypeDef st;
    uint8_t rx[6];
    uint32_t rx_len;

    if ((h == NULL) || (h->hi2c == NULL))
    {
        return HAL_ERROR;
    }

    rx_len = h->has_rh ? 6U : 3U;

    /* 1. Send high-precision measurement command. */
    st = SensirionTH_WriteCmd(h, SENSIRION_CMD_MEAS_HIGH);
    if (st != HAL_OK)
    {
        h->last_status = st;
        h->error_count++;
        return st;
    }

    /* 2. Wait for the conversion. Datasheet tMEAS,h max = 8.3 ms. */
    HAL_Delay(10U);

    /* 3. Read result (3 bytes for STS4x, 6 bytes for SHT4x). */
    st = HAL_I2C_Master_Receive(h->hi2c, h->addr | 0x01U, rx, rx_len, 100U);
    h->last_status = st;
    if (st != HAL_OK)
    {
        h->error_count++;
        return st;
    }

    /* 4. CRC check on temperature word. */
    if (SensirionTH_Crc8(&rx[0], 2U) != rx[2])
    {
        h->last_crc_ok = 0U;
        h->error_count++;
        return HAL_ERROR;
    }

    /* 5. CRC check on humidity word (SHT4x only). */
    if (h->has_rh && (SensirionTH_Crc8(&rx[3], 2U) != rx[5]))
    {
        h->last_crc_ok = 0U;
        h->error_count++;
        return HAL_ERROR;
    }

    h->last_crc_ok = 1U;

    /* 6. Convert temperature: T = -45 + 175 * ticks / 65535 */
    h->raw_t_ticks  = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    h->temperature_c = -45.0f + 175.0f * ((float)h->raw_t_ticks / 65535.0f);

    /* 7. Convert humidity: RH = -6 + 125 * ticks / 65535, clamp to [0..100] */
    if (h->has_rh)
    {
        h->raw_rh_ticks = (uint16_t)(((uint16_t)rx[3] << 8) | rx[4]);
        float rh = -6.0f + 125.0f * ((float)h->raw_rh_ticks / 65535.0f);
        if (rh < 0.0f)   { rh = 0.0f; }
        if (rh > 100.0f) { rh = 100.0f; }
        h->humidity_pct = rh;
    }
    else
    {
        h->raw_rh_ticks = 0U;
        h->humidity_pct = 0.0f;
    }

    h->read_count++;
    return HAL_OK;
}

HAL_StatusTypeDef SensirionTH_ReadSerial(SensirionTH_Handle_t *h, uint32_t *serial)
{
    HAL_StatusTypeDef st;
    uint8_t rx[6];

    if ((h == NULL) || (h->hi2c == NULL) || (serial == NULL))
    {
        return HAL_ERROR;
    }

    st = SensirionTH_WriteCmd(h, SENSIRION_CMD_READ_SERIAL);
    if (st != HAL_OK)
    {
        return st;
    }

    HAL_Delay(1U);

    st = HAL_I2C_Master_Receive(h->hi2c, h->addr | 0x01U, rx, sizeof(rx), 100U);
    if (st != HAL_OK)
    {
        return st;
    }

    if ((SensirionTH_Crc8(&rx[0], 2U) != rx[2]) ||
        (SensirionTH_Crc8(&rx[3], 2U) != rx[5]))
    {
        return HAL_ERROR;
    }

    *serial = ((uint32_t)rx[0] << 24) |
              ((uint32_t)rx[1] << 16) |
              ((uint32_t)rx[3] << 8)  |
               (uint32_t)rx[4];
    return HAL_OK;
}
