/**
  ******************************************************************************
  * @file    sfm3300.h
  * @brief   Driver for the Sensirion SFM3300-250-D bidirectional flow sensor.
  *
  * Wired on I2C1 in this project, 7-bit address 0x40. Note that the STS4x
  * tube-temperature sensor also sits on I2C1 (address 0x44), so all bus
  * traffic must be serialised - see the I2C1 mutex in main.c.
  *
  * Protocol (SFM3xxx family):
  *   - 16-bit commands, MSB first.
  *   - 0x1000 starts continuous measurement; after that the host simply
  *     reads 3 bytes (data MSB, data LSB, CRC) whenever it wants a sample.
  *     The FIRST sample after the start command is invalid and is discarded.
  *   - CRC-8, polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0x00.
  *     Careful: this is the same polynomial as the SHT4x/STS4x but a
  *     DIFFERENT init value (those use 0xFF), so the CRC routines are not
  *     interchangeable with sensirion_th.c.
  *
  * Conversion (SFM3300-250 variant):
  *   flow [slm] = (raw - 32768) / 120
  *
  * Sign convention: positive is flow in the direction of the arrow on the
  * sensor housing. Mount the sensor so that positive means gas flowing from
  * the CPAP toward the test lung, i.e. positive = inspiration.
  ******************************************************************************
  */

#ifndef SFM3300_H
#define SFM3300_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 7-bit address shifted left by 1, as the HAL expects. */
#define SFM3300_I2C_ADDR            (0x40U << 1)

#define SFM3300_CMD_START_MEAS      0x1000U
#define SFM3300_CMD_SOFT_RESET      0x2000U
#define SFM3300_CMD_READ_SERIAL     0x31AEU

/* SFM3300-250 scaling. */
#define SFM3300_FLOW_OFFSET         32768.0f
#define SFM3300_FLOW_SCALE_SLM        120.0f
#define SFM3300_FLOW_RANGE_SLM        250.0f

/**
  * Raw plausibility window. A NACKed or floating bus reads back as all
  * zeroes or all ones, and both would decode to a huge but "valid looking"
  * flow. Zero flow is 32768 counts, and +/-250 slm only spans about
  * +/-30000 counts, so anything outside this window is not a real reading.
  */
#define SFM3300_RAW_MIN_VALID          2000U
#define SFM3300_RAW_MAX_VALID         63000U

/** Consecutive failures after which the driver re-issues the start command. */
#define SFM3300_RECOVER_AFTER_ERRORS      10U

typedef struct
{
    I2C_HandleTypeDef *hi2c;        /**< Bus the sensor sits on. */
    uint16_t           addr;        /**< Shifted 7-bit address. */

    /* Latest conversion. */
    float              flow_slm;        /**< Zero-corrected instantaneous flow. */
    float              flow_filt_slm;   /**< Single-pole IIR of flow_slm. */
    float              flow_raw_slm;    /**< Before zero offset is applied. */
    uint16_t           raw;             /**< Raw 16-bit sensor counts. */

    /* Configuration. */
    float              zero_offset_slm; /**< Subtracted from every reading. */
    float              filter_alpha;    /**< 0 = no filtering, ->1 = heavy. */

    /* Identification and health. */
    uint32_t           serial;
    uint32_t           read_count;
    uint32_t           error_count;
    uint32_t           crc_error_count;
    uint32_t           range_error_count;
    uint32_t           recover_count;
    uint16_t           consec_errors;
    HAL_StatusTypeDef  last_status;
    uint8_t            present;         /**< 1 once the device has answered. */
    uint8_t            measuring;       /**< 1 once continuous mode is running. */
    uint8_t            serial_ok;       /**< 1 if the serial number read succeeded. */
    uint8_t            last_sample_ok;
    uint32_t           start_fail_count;/**< Failed attempts to enter continuous mode. */
} SFM3300_Handle_t;

/**
  * @brief Soft reset, read the serial number, then start continuous mode.
  *
  * Blocking; costs roughly 60 ms of HAL_Delay. Call once from a task, not
  * from an ISR, and hold the I2C1 mutex across the call.
  *
  * A failed serial read is NOT fatal: identification is a convenience, flow
  * measurement does not depend on it, and letting an optional step gate the
  * whole driver turns one unsupported command into a dead sensor. The
  * failure is recorded in serial_ok and logged by the caller instead.
  *
  * @retval HAL_OK when continuous measurement is running.
  */
HAL_StatusTypeDef SFM3300_Init(SFM3300_Handle_t *h,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t addr_shifted);

/** @brief Issue the soft reset command and wait out the reset time. */
HAL_StatusTypeDef SFM3300_SoftReset(SFM3300_Handle_t *h);

/**
  * @brief Start continuous measurement and discard the first (invalid) sample.
  */
HAL_StatusTypeDef SFM3300_StartContinuous(SFM3300_Handle_t *h);

/**
  * @brief Read the 32-bit serial number.
  *
  * Only valid when continuous measurement is NOT running, so this performs
  * a soft reset first and leaves the sensor idle. Init() calls it before
  * starting continuous mode; calling it later will stop the measurement,
  * so follow it with SFM3300_StartContinuous().
  */
HAL_StatusTypeDef SFM3300_ReadSerial(SFM3300_Handle_t *h, uint32_t *serial);

/**
  * @brief Read one flow sample. Non-blocking apart from the I2C transfer
  *        itself (3 bytes, well under a millisecond at the bus speed
  *        MX_I2C1_Init configures).
  *
  * Updates flow_slm / flow_filt_slm / raw on success. On repeated failure
  * the continuous measurement is restarted automatically, which covers the
  * sensor being hot-plugged or browning out.
  */
HAL_StatusTypeDef SFM3300_Read(SFM3300_Handle_t *h);

/**
  * @brief Tare: take the current filtered reading as the new zero.
  *
  * Only meaningful with genuinely no flow through the sensor. The SFM3300
  * has a small offset drift with temperature, so a tare at the start of a
  * session measurably improves the integrated tidal volume later on.
  */
void SFM3300_ZeroNow(SFM3300_Handle_t *h);

/** @brief Clear the zero offset back to the factory calibration. */
void SFM3300_ClearZero(SFM3300_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* SFM3300_H */
