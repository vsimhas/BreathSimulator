/**
  ******************************************************************************
  * @file    diagnostics_data.h
  * @brief   C-side aggregator for the GUI Diagnostics page.
  *
  * The GUI cannot reach into main.c naming directly without coupling the
  * TouchGFX layer to internal driver state. Instead, this thin bridge
  * snapshots the relevant globals (peripheral self-test results + live
  * sensor readings) into a single struct on demand. The Diagnostics screen
  * polls Diagnostics_GetSnapshot() once a second.
  ******************************************************************************
  */

#ifndef DIAGNOSTICS_DATA_H
#define DIAGNOSTICS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Per-test outcome category. The Diagnostics screen uses this to pick the
 * label text and colour ("PASS" green, "FAIL" red, "RUN..." amber, "---"
 * dim, "NO CARD" amber). */
typedef enum
{
    DIAG_TEST_NOT_RUN = 0,    /* test pipeline hasn't started yet            */
    DIAG_TEST_RUNNING,        /* in progress                                 */
    DIAG_TEST_PASS,           /* completed OK                                */
    DIAG_TEST_FAIL,           /* completed with error                        */
    DIAG_TEST_NO_CARD         /* SD-specific: no card inserted               */
} DiagnosticsTestState;

typedef struct
{
    /* Self-test results --------------------------------------------------- */
    DiagnosticsTestState sd_state;
    uint32_t             sd_fail_step;     /* 0 if pass, else stage that broke */
    uint32_t             sd_hal_error;     /* hsd1.ErrorCode at failure        */

    DiagnosticsTestState nfc_state;
    uint8_t              nfc_chip_id;      /* ST25R3920B revision register     */

    DiagnosticsTestState sdram_state;
    DiagnosticsTestState qspi_state;

    /* Live sensors -------------------------------------------------------- */
    /* SHT4x (humidifier air %RH + air temperature). */
    float                humidity_pct;
    float                humidifier_temp_c;
    uint8_t              humidity_valid;   /* 1 if last read CRC ok            */

    /* STS4x (tube / outlet probe, temperature only). */
    float                tube_temp_c;
    uint8_t              tube_temp_valid;

    /* Pressure sensors. The board carries 3x AMS5935 (SPI) + 1x AMS5105
     * (analog through ADC3). All readings are in mbar. */
    float                pressure_flow_mbar;     /* hamsFS  -5..5 mbar         */
    float                pressure_atmos_mbar;    /* hamsAS  700..1200 mbar abs */
    float                pressure_mask_mbar;     /* hamsPS  -50..50 mbar       */
    float                pressure_analog_mbar;   /* AMS5105 analog            */
} DiagnosticsSnapshot_t;

/* Populates *out with the latest values. Safe to call from any task; reads
 * are non-blocking, no I/O is initiated here. */
void Diagnostics_GetSnapshot(DiagnosticsSnapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTICS_DATA_H */
