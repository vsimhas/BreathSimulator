/**
  ******************************************************************************
  * @file    diagnostics_data.h
  * @brief   C-side aggregator for the GUI Diagnostics page.
  ******************************************************************************
  */

#ifndef DIAGNOSTICS_DATA_H
#define DIAGNOSTICS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    DIAG_TEST_NOT_RUN = 0,
    DIAG_TEST_RUNNING,
    DIAG_TEST_PASS,
    DIAG_TEST_FAIL,
    DIAG_TEST_NO_CARD
} DiagnosticsTestState;

typedef struct
{
    DiagnosticsTestState sd_state;
    uint32_t             sd_fail_step;
    uint32_t             sd_hal_error;

    DiagnosticsTestState nfc_state;
    uint8_t              nfc_chip_id;

    DiagnosticsTestState sdram_state;
    DiagnosticsTestState qspi_state;

    float                humidity_pct;
    float                humidifier_temp_c;
    uint8_t              humidity_valid;

    float                tube_temp_c;
    uint8_t              tube_temp_valid;

    uint8_t              sim_running;
    uint8_t              sim_state;    /**< BreathSimState_t */
    uint8_t              sim_segment;  /**< BreathSegment_t */
    uint8_t              sim_event;    /**< BreathEventType_t */
    uint16_t             sim_flags;    /**< Sticky BREATH_FLAG_* for the run. */
    uint32_t             breath_index;
    int32_t              rpm_cmd;
    int32_t              rpm_act;
    float                phase;
} DiagnosticsSnapshot_t;

void Diagnostics_GetSnapshot(DiagnosticsSnapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTICS_DATA_H */
