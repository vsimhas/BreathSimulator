/**
  ******************************************************************************
  * @file    diagnostics_data.c
  * @brief   Read-only snapshot of self-test results + live sensor values for
  *          the Diagnostics screen.
  *
  * The state lives in main.c (self-test state machines) and various drivers
  * (sensirion_th.c, pressure_sensors.c, nfc_test.c). We pull from each
  * module's debugger-visible globals via extern declarations so we don't
  * need to wire new accessors into hot paths.
  ******************************************************************************
  */

#include "diagnostics_data.h"

#include "stm32h7xx_hal.h"
#include "sensirion_th.h"
#include "pressure_sensors.h"
#include "nfc_test.h"

#include <string.h>

/* ---------- self-test globals owned by main.c ----------------------------- */
extern volatile HAL_StatusTypeDef sd_test_result;
extern volatile uint32_t          sd_test_step;
extern volatile uint32_t          sd_test_fail_step;
extern volatile uint8_t           sd_card_present;
extern volatile uint32_t          sd_hal_error;

extern volatile HAL_StatusTypeDef sdram_test_result;
extern volatile uint32_t          sdram_test_step;

extern volatile HAL_StatusTypeDef qspi_test_result;
extern volatile uint32_t          qspi_test_step;

/* ---------- driver globals ------------------------------------------------ */
extern SensirionTH_Handle_t hSht4x;
extern SensirionTH_Handle_t hSts4x;

extern AMS5935_Data_t       measFS;
extern AMS5935_Data_t       measAS;
extern AMS5935_Data_t       measPS;

/* Updated in pressure_sensors.c on each analog ADC read. */
extern float                g_pressure_analog_mbar;

/* Map a HAL_Status + step counter to one of our DiagnosticsTestState values.
 * The state machines in main.c follow a common shape: result starts as
 * HAL_BUSY with step=0, transitions through steps 1..N as work progresses,
 * lands at result=HAL_OK and step=100 on success, or result=HAL_ERROR with
 * step != 100 on failure. */
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

    /* SD: card-presence trumps the self-test result so the user gets a
     * "no card" indication instead of a misleading FAIL when they pull the
     * card mid-session. */
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

    /* NFC: NfcTest sets g_nfc_test_status once after init. NFC_TEST_OK = 0,
     * any other value is a failure mode. There's no "running" intermediate
     * state - init is synchronous. */
    out->nfc_state   = (g_nfc_test_status == NFC_TEST_OK)
                           ? DIAG_TEST_PASS : DIAG_TEST_FAIL;
    out->nfc_chip_id = (uint8_t)g_nfc_chip_id;

    out->sdram_state = halToState(sdram_test_result, sdram_test_step);
    out->qspi_state  = halToState(qspi_test_result,  qspi_test_step);

    /* Live values. The drivers update these whenever their owning task does
     * a successful read; we just sample whatever's there. last_crc_ok is
     * the cleanest "is this number trustworthy?" gate for the Sensirion
     * parts. */
    out->humidity_pct       = hSht4x.humidity_pct;
    out->humidifier_temp_c  = hSht4x.temperature_c;
    out->humidity_valid     = hSht4x.last_crc_ok;

    out->tube_temp_c        = hSts4x.temperature_c;
    out->tube_temp_valid    = hSts4x.last_crc_ok;

    out->pressure_flow_mbar   = measFS.pressure;
    out->pressure_atmos_mbar  = measAS.pressure;
    out->pressure_mask_mbar   = measPS.pressure;
    out->pressure_analog_mbar = g_pressure_analog_mbar;
}
