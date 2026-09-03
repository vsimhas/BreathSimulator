/**
  ******************************************************************************
  * @file    diagnostics_data.c
  * @brief   Self-test + runtime snapshot for the Diagnostics screen.
  ******************************************************************************
  */

#include "diagnostics_data.h"

#include "stm32h7xx_hal.h"
#include "sensirion_th.h"
#include "nfc_test.h"
#include "breath_sim.h"
#include "blower_ipc.h"

#include <string.h>

extern volatile HAL_StatusTypeDef sd_test_result;
extern volatile uint32_t          sd_test_step;
extern volatile uint32_t          sd_test_fail_step;
extern volatile uint8_t           sd_card_present;
extern volatile uint32_t          sd_hal_error;

extern volatile HAL_StatusTypeDef sdram_test_result;
extern volatile uint32_t          sdram_test_step;

extern volatile HAL_StatusTypeDef qspi_test_result;
extern volatile uint32_t          qspi_test_step;

extern SensirionTH_Handle_t hSht4x;
extern SensirionTH_Handle_t hSts4x;

extern BlowerIpcStatus_t g_blower_status;

static DiagnosticsTestState halToState(HAL_StatusTypeDef rc, uint32_t step)
{
    if (rc == HAL_OK)
    {
        return DIAG_TEST_PASS;
    }
    if (rc == HAL_BUSY)
    {
        return (step == 0U) ? DIAG_TEST_NOT_RUN : DIAG_TEST_RUNNING;
    }
    return DIAG_TEST_FAIL;
}

void Diagnostics_GetSnapshot(DiagnosticsSnapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));

    if (sd_card_present == 0U)
    {
        out->sd_state = DIAG_TEST_NO_CARD;
    }
    else
    {
        out->sd_state = halToState(sd_test_result, sd_test_step);
    }
    out->sd_fail_step = sd_test_fail_step;
    out->sd_hal_error = sd_hal_error;

    out->nfc_state   = (g_nfc_test_status == NFC_TEST_OK)
                           ? DIAG_TEST_PASS : DIAG_TEST_FAIL;
    out->nfc_chip_id = (uint8_t)g_nfc_chip_id;

    out->sdram_state = halToState(sdram_test_result, sdram_test_step);
    out->qspi_state  = halToState(qspi_test_result,  qspi_test_step);

    out->humidity_pct       = hSht4x.humidity_pct;
    out->humidifier_temp_c  = hSht4x.temperature_c;
    out->humidity_valid     = hSht4x.last_crc_ok;

    out->tube_temp_c        = hSts4x.temperature_c;
    out->tube_temp_valid    = hSts4x.last_crc_ok;

    /* One coherent snapshot rather than several independent getters: the
     * breath task runs at a higher priority than this caller, so pulling
     * phase and RPM separately could mix values from different ticks. */
    BreathSimStatus_t sim;
    BreathSim_GetStatus(&sim);

    out->sim_running        = BreathSim_IsRunning() ? 1U : 0U;
    out->sim_state          = sim.state;
    out->sim_segment        = sim.segment;
    out->sim_event          = sim.event;
    out->sim_flags          = sim.flags;
    out->breath_index       = sim.breath_index;
    out->rpm_cmd            = sim.rpm_cmd;
    out->rpm_act            = sim.rpm_act;
    out->phase              = sim.phase;
}
