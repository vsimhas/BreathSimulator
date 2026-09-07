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
    /** 0 when the climate sensors are compiled out (CLIMATE_SENSORS_ENABLED). */
    uint8_t              climate_enabled;

    float                flow_slm;
    uint16_t             flow_raw;
    uint8_t              flow_present;   /**< Sensor has answered at least once. */
    uint8_t              flow_valid;     /**< Last sample passed CRC + range. */
    uint32_t             flow_error_count;

    float                press_cmh2o;    /**< Working pressure, zero-corrected. */
    float                press_mbar;
    float                baro_mbar;
    uint8_t              press_present;  /**< AMS5935-0050 has answered. */
    uint8_t              press_held;     /**< Last sample was a repeat, not fresh. */

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
