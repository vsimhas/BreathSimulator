/**
  ******************************************************************************
  * @file    tube_comp.c
  * @brief   Implementation: dP_hose = R_lam*|Q| + R_turb*Q*|Q|.
  ******************************************************************************
  */

#include "tube_comp.h"

#include <math.h>

/* === Tunables (debugger-writable volatiles) ================================ */
volatile uint8_t g_tubecomp_enabled       = 1U;
volatile float   g_tubecomp_R_lam_cmh2o_per_lpm  = 0.002f;     /* 15 mm slimline */
volatile float   g_tubecomp_R_turb_cmh2o_per_lpm2 = 0.00022f;  /* bench-tuned above 0.00015 */

/* === Live values (debugger-readable volatiles) === */
volatile float   g_tubecomp_p_sensor_cmh2o = 0.0f;
volatile float   g_tubecomp_p_mask_cmh2o   = 0.0f;
volatile float   g_tubecomp_q_slm          = 0.0f;
volatile float   g_tubecomp_drop_cmh2o     = 0.0f;
volatile uint32_t g_tubecomp_step_count    = 0U;

/* === Public API === */

void TubeComp_Init(void)
{
    g_tubecomp_enabled                = 1U;
    g_tubecomp_R_lam_cmh2o_per_lpm    = 0.002f;
    g_tubecomp_R_turb_cmh2o_per_lpm2  = 0.00022f;
    g_tubecomp_p_sensor_cmh2o         = 0.0f;
    g_tubecomp_p_mask_cmh2o           = 0.0f;
    g_tubecomp_q_slm                  = 0.0f;
    g_tubecomp_drop_cmh2o             = 0.0f;
    g_tubecomp_step_count             = 0U;
}

void  TubeComp_SetEnable(bool en) { g_tubecomp_enabled = en ? 1U : 0U; }
bool  TubeComp_IsEnabled(void)    { return (g_tubecomp_enabled != 0U); }

void TubeComp_SetCoefficients(float R_lam, float R_turb)
{
    /* Defensive clamping: physically sensible adult-CPAP hose has
     * R_turb in [0, 0.001] and R_lam in [0, 0.05]. Reject obviously
     * silly numbers - they'd swamp the loop. */
    if (R_lam  < 0.0f)     R_lam  = 0.0f;
    if (R_lam  > 0.05f)    R_lam  = 0.05f;
    if (R_turb < 0.0f)     R_turb = 0.0f;
    if (R_turb > 0.001f)   R_turb = 0.001f;
    g_tubecomp_R_lam_cmh2o_per_lpm    = R_lam;
    g_tubecomp_R_turb_cmh2o_per_lpm2  = R_turb;
}

float TubeComp_GetRLam(void)             { return g_tubecomp_R_lam_cmh2o_per_lpm; }
float TubeComp_GetRTurb(void)            { return g_tubecomp_R_turb_cmh2o_per_lpm2; }
float TubeComp_GetLastSensorCmh2o(void)  { return g_tubecomp_p_sensor_cmh2o; }
float TubeComp_GetLastMaskCmh2o(void)    { return g_tubecomp_p_mask_cmh2o; }
float TubeComp_GetLastDropCmh2o(void)    { return g_tubecomp_drop_cmh2o; }
float TubeComp_GetLastFlowSlm(void)      { return g_tubecomp_q_slm; }

float TubeComp_Apply(float p_sensor_cmh2o, float q_slm)
{
    g_tubecomp_p_sensor_cmh2o = p_sensor_cmh2o;
    g_tubecomp_q_slm          = q_slm;
    g_tubecomp_step_count++;

    if (g_tubecomp_enabled == 0U)
    {
        /* Pass-through: mask = sensor, no drop, no surprise. */
        g_tubecomp_drop_cmh2o   = 0.0f;
        g_tubecomp_p_mask_cmh2o = p_sensor_cmh2o;
        return p_sensor_cmh2o;
    }

    /* dP magnitude is always positive; sign comes from flow direction.
     * Convention: Q > 0 means flow from device toward patient (the usual
     * direction during inspiration). With positive Q, P_mask < P_sensor.
     * During exhalation Q is negative and P_mask > P_sensor for the same
     * reason - the air piles up against the closed inspiratory limb. */
    const float qabs    = fabsf(q_slm);
    const float drop_mag = g_tubecomp_R_lam_cmh2o_per_lpm   * qabs
                         + g_tubecomp_R_turb_cmh2o_per_lpm2 * qabs * qabs;
    const float drop    = (q_slm >= 0.0f) ? drop_mag : -drop_mag;

    g_tubecomp_drop_cmh2o   = drop;
    g_tubecomp_p_mask_cmh2o = p_sensor_cmh2o - drop;
    return g_tubecomp_p_mask_cmh2o;
}
