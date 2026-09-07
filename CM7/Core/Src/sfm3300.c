/**
  ******************************************************************************
  * @file    sfm3300.c
  * @brief   Driver for the Sensirion SFM3300-250-D bidirectional flow sensor.
  ******************************************************************************
  */

#include "sfm3300.h"
#include <string.h>

#define SFM3300_I2C_TIMEOUT_MS      100U
/** Datasheet soft-reset time; be generous, this only runs at init. */
#define SFM3300_RESET_MS             50U
/** First measurement after the start command needs longer than the rest. */
#define SFM3300_FIRST_MEAS_MS        20U

/**
  * CRC-8: polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0x00, no reflection,
  * no final XOR. The SHT4x/STS4x driver uses the same polynomial with init
  * 0xFF, so the two are deliberately kept as separate functions.
  */
static uint8_t SFM3300_Crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00U;
    for (uint32_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint32_t b = 0U; b < 8U; b++)
        {
            uint32_t v = (uint32_t)crc << 1;
            if ((crc & 0x80U) != 0U) { v ^= 0x31U; }
            crc = (uint8_t)v;
        }
    }
    return crc;
}

static HAL_StatusTypeDef SFM3300_WriteCmd(SFM3300_Handle_t *h, uint16_t cmd)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    return HAL_I2C_Master_Transmit(h->hi2c, h->addr, tx, 2U,
                                   SFM3300_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef SFM3300_SoftReset(SFM3300_Handle_t *h)
{
    if ((h == NULL) || (h->hi2c == NULL)) { return HAL_ERROR; }

    const HAL_StatusTypeDef st = SFM3300_WriteCmd(h, SFM3300_CMD_SOFT_RESET);
    h->last_status = st;
    h->measuring = 0U;
    if (st == HAL_OK)
    {
        HAL_Delay(SFM3300_RESET_MS);
    }
    return st;
}

HAL_StatusTypeDef SFM3300_ReadSerial(SFM3300_Handle_t *h, uint32_t *serial)
{
    uint8_t rx[6];

    if ((h == NULL) || (h->hi2c == NULL)) { return HAL_ERROR; }

    /* The serial command is only accepted when the sensor is idle. */
    HAL_StatusTypeDef st = SFM3300_SoftReset(h);
    if (st != HAL_OK) { return st; }

    st = SFM3300_WriteCmd(h, SFM3300_CMD_READ_SERIAL);
    if (st != HAL_OK)
    {
        h->last_status = st;
        h->error_count++;
        return st;
    }

    HAL_Delay(2U);

    st = HAL_I2C_Master_Receive(h->hi2c, h->addr | 0x01U, rx, sizeof(rx),
                                SFM3300_I2C_TIMEOUT_MS);
    h->last_status = st;
    if (st != HAL_OK)
    {
        h->error_count++;
        return st;
    }

    /* Two 16-bit words, each followed by its own CRC byte. */
    if ((SFM3300_Crc8(&rx[0], 2U) != rx[2]) ||
        (SFM3300_Crc8(&rx[3], 2U) != rx[5]))
    {
        h->crc_error_count++;
        h->error_count++;
        return HAL_ERROR;
    }

    h->serial = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
                ((uint32_t)rx[3] << 8)  | (uint32_t)rx[4];
    h->present   = 1U;
    h->serial_ok = 1U;

    if (serial != NULL) { *serial = h->serial; }
    return HAL_OK;
}

HAL_StatusTypeDef SFM3300_StartContinuous(SFM3300_Handle_t *h)
{
    uint8_t rx[3];

    if ((h == NULL) || (h->hi2c == NULL)) { return HAL_ERROR; }

    const HAL_StatusTypeDef st = SFM3300_WriteCmd(h, SFM3300_CMD_START_MEAS);
    h->last_status = st;
    if (st != HAL_OK)
    {
        h->error_count++;
        h->start_fail_count++;
        h->measuring = 0U;
        return st;
    }

    HAL_Delay(SFM3300_FIRST_MEAS_MS);

    /* The first sample after the start command is specified as invalid.
     * Read and drop it so the caller's first real Read() is trustworthy;
     * ignore its status, the sensor may legitimately NACK this one. */
    (void)HAL_I2C_Master_Receive(h->hi2c, h->addr | 0x01U, rx, sizeof(rx),
                                 SFM3300_I2C_TIMEOUT_MS);

    h->measuring = 1U;
    h->consec_errors = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef SFM3300_Init(SFM3300_Handle_t *h,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t addr_shifted)
{
    if ((h == NULL) || (hi2c == NULL)) { return HAL_ERROR; }

    memset(h, 0, sizeof(*h));
    h->hi2c         = hi2c;
    h->addr         = addr_shifted;
    h->filter_alpha = 0.2f;   /* mild smoothing; caller may override */

    /* Reading the serial is a useful presence check - it exercises a write,
     * a read and two CRCs - but it is only identification. If the command
     * is unsupported on this part, or the sensor is fussy about it after a
     * reset, that must not stop us measuring flow: the whole point of the
     * driver is the flow reading. Record the outcome and carry on. */
    uint32_t serial = 0U;
    (void)SFM3300_ReadSerial(h, &serial);

    /* ReadSerial leaves the sensor idle (it soft-resets first), which is
     * exactly the state StartContinuous expects, whether it succeeded or
     * not. If it failed outright, reset explicitly so we start clean. */
    if (h->serial_ok == 0U)
    {
        (void)SFM3300_SoftReset(h);
    }

    return SFM3300_StartContinuous(h);
}

HAL_StatusTypeDef SFM3300_Read(SFM3300_Handle_t *h)
{
    uint8_t rx[3];

    if ((h == NULL) || (h->hi2c == NULL)) { return HAL_ERROR; }

    h->last_sample_ok = 0U;

    if (h->measuring == 0U)
    {
        return HAL_ERROR;
    }

    /* Continuous mode: no command, just clock out the next sample. */
    const HAL_StatusTypeDef st =
        HAL_I2C_Master_Receive(h->hi2c, h->addr | 0x01U, rx, sizeof(rx),
                               SFM3300_I2C_TIMEOUT_MS);
    h->last_status = st;

    if (st != HAL_OK)
    {
        h->error_count++;
        h->consec_errors++;
    }
    else if (SFM3300_Crc8(&rx[0], 2U) != rx[2])
    {
        h->crc_error_count++;
        h->error_count++;
        h->consec_errors++;
    }
    else
    {
        const uint16_t raw = (uint16_t)(((uint16_t)rx[0] << 8) | (uint16_t)rx[1]);

        if ((raw < SFM3300_RAW_MIN_VALID) || (raw > SFM3300_RAW_MAX_VALID))
        {
            /* CRC can pass on an all-zero frame, so the range check is not
             * redundant: it is what catches a sensor that has stopped
             * converting but is still acknowledging. */
            h->range_error_count++;
            h->error_count++;
            h->consec_errors++;
        }
        else
        {
            h->raw          = raw;
            h->flow_raw_slm = ((float)raw - SFM3300_FLOW_OFFSET) /
                              SFM3300_FLOW_SCALE_SLM;
            h->flow_slm     = h->flow_raw_slm - h->zero_offset_slm;

            if (h->filter_alpha > 0.0f)
            {
                h->flow_filt_slm += h->filter_alpha *
                                    (h->flow_slm - h->flow_filt_slm);
            }
            else
            {
                h->flow_filt_slm = h->flow_slm;
            }

            h->read_count++;
            h->consec_errors  = 0U;
            h->present        = 1U;
            h->last_sample_ok = 1U;
            return HAL_OK;
        }
    }

    /* Recovery: a sensor that browned out or was hot-plugged has forgotten
     * it was in continuous mode, and would otherwise never come back. */
    if (h->consec_errors >= SFM3300_RECOVER_AFTER_ERRORS)
    {
        h->consec_errors = 0U;
        h->recover_count++;
        (void)SFM3300_StartContinuous(h);
    }

    return (st != HAL_OK) ? st : HAL_ERROR;
}

void SFM3300_ZeroNow(SFM3300_Handle_t *h)
{
    if (h == NULL) { return; }
    h->zero_offset_slm += h->flow_filt_slm;
    h->flow_slm       = 0.0f;
    h->flow_filt_slm  = 0.0f;
}

void SFM3300_ClearZero(SFM3300_Handle_t *h)
{
    if (h == NULL) { return; }
    h->zero_offset_slm = 0.0f;
}
