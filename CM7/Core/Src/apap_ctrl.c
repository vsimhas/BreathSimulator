/**
  ******************************************************************************
  * @file    apap_ctrl.c
  * @brief   Autoset (APAP) titration state machine.
  ******************************************************************************
  */

#include "apap_ctrl.h"
#include "therapy.h"
#include "epr_ctrl.h"
#include "blower_osc.h"
#include "blower_ctrl.h"
#include "flow_sensor.h"
#include "stm32h7xx_hal.h"

#include <math.h>

/* === Tunables (debugger-writable) === */
volatile float   g_apap_no_breath_timeout_s        = 10.0f;
volatile float   g_apap_breath_window_s            = 12.0f;
volatile float   g_apap_startup_grace_s            = 15.0f;
volatile float   g_apap_osc_amplitude_cmh2o        = 1.0f;   /* setpoint peak; ~2.0 p-p */
volatile float   g_apap_osc_amplitude_max_cmh2o    = 1.75f; /* setpoint peak clamp */
volatile float   g_apap_osc_freq_hz                = 4.0f;
volatile float   g_apap_fot_eval_period_s          = 2.0f;   /* ~8 cycles @ 4 Hz */
volatile float   g_apap_osc_min_p_pp_cmh2o         = 0.15f;
volatile float   g_apap_osa_flow_pp_thresh_slm     = 2.0f;
volatile float   g_apap_raise_rate_standard_cmh2o_per_s = 0.050f; /* ~3/min */
volatile float   g_apap_raise_rate_soft_cmh2o_per_s     = 0.020f; /* ~1.2/min */
volatile float   g_apap_raise_rate_her_scale     = 0.75f;
volatile uint8_t g_apap_response_soft              = 0U;

/* === Live state === */
volatile uint8_t  g_apap_state                     = (uint8_t)APAP_STATE_IDLE;
volatile float    g_apap_target_cmh2o              = 8.0f;
volatile uint32_t g_apap_no_breath_ms              = 0U;
volatile uint32_t g_apap_osc_eval_ms               = 0U;
volatile float    g_apap_osc_p_min_cmh2o           = 0.0f;
volatile float    g_apap_osc_p_max_cmh2o           = 0.0f;
volatile float    g_apap_osc_q_min_slm             = 0.0f;
volatile float    g_apap_osc_q_max_slm             = 0.0f;
volatile uint8_t  g_apap_last_osa_detected         = 0U;
volatile uint8_t  g_apap_osa_treating              = 0U;
volatile float    g_apap_fot_p_pp_cmh2o           = 0.0f;
volatile float    g_apap_fot_q_pp_slm             = 0.0f;
volatile float    g_apap_fot_z_cmh2o_per_slm      = 0.0f; /* dP_pp / dQ_pp     */
volatile uint8_t  g_apap_fot_osa_likely           = 0U;

#define FOT_Q_PP_EPS_SLM  0.15f
#define FOT_Z_BLOCKED     999.0f

/* === Internal === */
static float     s_min_cmh2o = 5.0f;
static float     s_max_cmh2o = 20.0f;
static bool      s_for_her = false;
static bool      s_active = false;
static bool      s_osa_treating = false;
static uint32_t  s_running_enter_ms = 0U;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool autosetMode(uint8_t mode)
{
    return (mode == (uint8_t)THERAPY_MODE_AUTOSET)
        || (mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
}

static float raiseRate(void)
{
    float rate = (g_apap_response_soft != 0U)
                     ? g_apap_raise_rate_soft_cmh2o_per_s
                     : g_apap_raise_rate_standard_cmh2o_per_s;
    if (s_for_her)
    {
        rate *= clampf(g_apap_raise_rate_her_scale, 0.25f, 1.0f);
    }
    return clampf(rate, 0.005f, 0.2f);
}

static void setState(ApapState_t st)
{
    g_apap_state = (uint8_t)st;
}

static void disableOsc(void)
{
    BlowerOsc_SetEnable(false);
}

static void configureOsc(void)
{
    /* Commanded on the PID setpoint. Mask p-p is often ~30-50% of this
     * (tube/mask compliance, leak). AirSense manual cites ~1 cmH2O p-p
     * at the mask; tune g_apap_osc_amplitude_cmh2o in the debugger. */
    const float ampl_max = clampf(g_apap_osc_amplitude_max_cmh2o, 0.5f, 2.5f);
    BlowerOsc_SetAmplitude(clampf(g_apap_osc_amplitude_cmh2o, 0.05f, ampl_max));
    BlowerOsc_SetFrequency(clampf(g_apap_osc_freq_hz, 2.0f, 8.0f));
    BlowerOsc_SetEnable(true);
}

static void resetOscStats(void)
{
    const float p = BlowerCtrl_GetMeasured();
    const float q = Flow_GetPatientSlm();
    g_apap_osc_p_min_cmh2o = p;
    g_apap_osc_p_max_cmh2o = p;
    g_apap_osc_q_min_slm = q;
    g_apap_osc_q_max_slm = q;
    g_apap_osc_eval_ms = 0U;
}

static void trackOscStats(void)
{
    const float p = BlowerCtrl_GetMeasured();
    const float q = Flow_GetPatientSlm();
    if (p < g_apap_osc_p_min_cmh2o) { g_apap_osc_p_min_cmh2o = p; }
    if (p > g_apap_osc_p_max_cmh2o) { g_apap_osc_p_max_cmh2o = p; }
    if (q < g_apap_osc_q_min_slm)   { g_apap_osc_q_min_slm = q; }
    if (q > g_apap_osc_q_max_slm)   { g_apap_osc_q_max_slm = q; }
}

static bool startupGraceActive(void)
{
    if (g_apap_startup_grace_s <= 0.0f)
    {
        return false;
    }
    const uint32_t grace_ms = (uint32_t)(g_apap_startup_grace_s * 1000.0f);
    return (HAL_GetTick() - s_running_enter_ms) < grace_ms;
}

static bool recentBreath(void)
{
    const uint32_t window_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                                  4.0f, 30.0f) * 1000.0f);
    return Epr_HasRecentBreath(window_ms);
}

static void refreshFotMetrics(void)
{
    const float p_pp = g_apap_osc_p_max_cmh2o - g_apap_osc_p_min_cmh2o;
    const float q_pp = g_apap_osc_q_max_slm - g_apap_osc_q_min_slm;
    const float p_min = clampf(g_apap_osc_min_p_pp_cmh2o, 0.05f, 0.8f);
    const float q_thr = clampf(g_apap_osa_flow_pp_thresh_slm, 0.5f, 10.0f);

    g_apap_fot_p_pp_cmh2o = p_pp;
    g_apap_fot_q_pp_slm = q_pp;
    if (q_pp > FOT_Q_PP_EPS_SLM)
    {
        g_apap_fot_z_cmh2o_per_slm = p_pp / q_pp;
    }
    else
    {
        g_apap_fot_z_cmh2o_per_slm = FOT_Z_BLOCKED;
    }

    const bool osa = (p_pp >= p_min) && (q_pp < q_thr);
    g_apap_fot_osa_likely = osa ? 1U : 0U;
}

static bool evaluateOscForOsa(void)
{
    refreshFotMetrics();
    return (g_apap_fot_osa_likely != 0U);
}

static void enterFotProbe(void)
{
    resetOscStats();
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_last_osa_detected = 0U;
    configureOsc();
    setState(APAP_STATE_OSCILLATE);
}

static void exitProbeToMonitor(void)
{
    disableOsc();
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_no_breath_ms = 0U;
    setState(APAP_STATE_MONITOR);
}

void Apap_Init(void)
{
    g_apap_no_breath_timeout_s = 10.0f;
    g_apap_breath_window_s = 12.0f;
    g_apap_startup_grace_s = 15.0f;
    g_apap_osc_amplitude_cmh2o = 1.0f;
    g_apap_osc_amplitude_max_cmh2o = 1.75f;
    g_apap_osc_freq_hz = 4.0f;
    g_apap_fot_eval_period_s = 2.0f;
    g_apap_osc_min_p_pp_cmh2o = 0.15f;
    g_apap_osa_flow_pp_thresh_slm = 2.0f;
    g_apap_raise_rate_standard_cmh2o_per_s = 0.050f;
    g_apap_raise_rate_soft_cmh2o_per_s = 0.020f;
    g_apap_raise_rate_her_scale = 0.75f;
    g_apap_response_soft = 0U;

    s_active = false;
    s_for_her = false;
    s_osa_treating = false;
    s_running_enter_ms = 0U;
    g_apap_target_cmh2o = 8.0f;
    g_apap_no_breath_ms = 0U;
    g_apap_osa_treating = 0U;
    g_apap_fot_p_pp_cmh2o = 0.0f;
    g_apap_fot_q_pp_slm = 0.0f;
    g_apap_fot_z_cmh2o_per_slm = 0.0f;
    g_apap_fot_osa_likely = 0U;
    setState(APAP_STATE_IDLE);
    disableOsc();
}

void Apap_SetResponseSoft(bool soft)
{
    g_apap_response_soft = soft ? 1U : 0U;
}

bool Apap_IsResponseSoft(void)
{
    return (g_apap_response_soft != 0U);
}

void Apap_OnTherapyStart(float min_cmh2o, float max_cmh2o, uint8_t therapy_mode)
{
    if (!autosetMode(therapy_mode))
    {
        s_active = false;
        setState(APAP_STATE_IDLE);
        disableOsc();
        return;
    }

    s_active = true;
    s_for_her = (therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
    if (s_for_her)
    {
        /* AirSense for Her: gentler pressure rises (like Response = Soft). */
        g_apap_response_soft = 1U;
    }
    s_min_cmh2o = clampf(min_cmh2o, THERAPY_PRESS_MIN_CMH2O, THERAPY_PRESS_MAX_CMH2O);
    s_max_cmh2o = clampf(max_cmh2o, s_min_cmh2o + 0.5f, THERAPY_PRESS_MAX_CMH2O);
    if (s_max_cmh2o < s_min_cmh2o)
    {
        s_max_cmh2o = s_min_cmh2o;
    }

    g_apap_target_cmh2o = s_min_cmh2o;
    g_apap_no_breath_ms = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_osa_treating = 0U;
    s_osa_treating = false;
    s_running_enter_ms = HAL_GetTick();
    disableOsc();
    setState(APAP_STATE_MONITOR);
}

void Apap_OnTherapyStop(void)
{
    s_active = false;
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_no_breath_ms = 0U;
    g_apap_last_osa_detected = 0U;
    setState(APAP_STATE_IDLE);
    disableOsc();
}

void Apap_OnMaskReconnect(void)
{
    if (!s_active)
    {
        return;
    }
    exitProbeToMonitor();
}

float Apap_GetTargetPressure(void)
{
    return g_apap_target_cmh2o;
}

ApapState_t Apap_GetState(void)
{
    return (ApapState_t)g_apap_state;
}

const char* Apap_GetStateName(void)
{
    switch ((ApapState_t)g_apap_state)
    {
    case APAP_STATE_MONITOR:    return "Monitor";
    case APAP_STATE_NO_BREATH:  return "No breath";
    case APAP_STATE_OSCILLATE:
        return s_osa_treating ? "FOT+Raise" : "FOT probe";
    case APAP_STATE_RESPOND:    return "Raise P";
    case APAP_STATE_IDLE:
    default:                    return "Idle";
    }
}

bool Apap_IsOscillating(void)
{
    return ((ApapState_t)g_apap_state == APAP_STATE_OSCILLATE)
        && BlowerOsc_IsEnabled();
}

float Apap_GetFotPressurePpCmh2o(void)     { return g_apap_fot_p_pp_cmh2o; }
float Apap_GetFotFlowPpSlm(void)           { return g_apap_fot_q_pp_slm; }
float Apap_GetFotImpedanceCmh2oPerSlm(void) { return g_apap_fot_z_cmh2o_per_slm; }
float Apap_GetOsaFlowPpThreshSlm(void)     { return g_apap_osa_flow_pp_thresh_slm; }
bool  Apap_IsOsaLikely(void)               { return (g_apap_fot_osa_likely != 0U); }
bool  Apap_IsOsaTreating(void)             { return (g_apap_osa_treating != 0U); }

float Apap_Update(float dt_s, uint8_t therapy_state, bool mask_off)
{
    if (!s_active
        || mask_off
        || (therapy_state != (uint8_t)THERAPY_STATE_RUNNING))
    {
        if (mask_off && s_active)
        {
            disableOsc();
            s_osa_treating = false;
            g_apap_osa_treating = 0U;
        }
        return g_apap_target_cmh2o;
    }

    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return g_apap_target_cmh2o;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    if (startupGraceActive())
    {
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        setState(APAP_STATE_MONITOR);
        return g_apap_target_cmh2o;
    }

    /* Breath ended the apnea / FOT episode. */
    if (recentBreath()
        && ((ApapState_t)g_apap_state == APAP_STATE_OSCILLATE
            || (ApapState_t)g_apap_state == APAP_STATE_NO_BREATH))
    {
        exitProbeToMonitor();
    }

    switch ((ApapState_t)g_apap_state)
    {
    case APAP_STATE_MONITOR:
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        if (!recentBreath())
        {
            setState(APAP_STATE_NO_BREATH);
            g_apap_no_breath_ms = 0U;
        }
        break;

    case APAP_STATE_NO_BREATH:
        disableOsc();
        g_apap_no_breath_ms += dt_ms;
        if (recentBreath())
        {
            exitProbeToMonitor();
            break;
        }
        {
            const uint32_t timeout_ms =
                (uint32_t)(clampf(g_apap_no_breath_timeout_s, 5.0f, 30.0f)
                           * 1000.0f);
            if (g_apap_no_breath_ms >= timeout_ms)
            {
                enterFotProbe();
            }
        }
        break;

    case APAP_STATE_OSCILLATE:
        /* AirSense: keep FOT running until breathing resumes. */
        configureOsc();
        trackOscStats();
        refreshFotMetrics();
        g_apap_osc_eval_ms += dt_ms;

        if (recentBreath())
        {
            exitProbeToMonitor();
            break;
        }

        {
            const uint32_t eval_ms =
                (uint32_t)(clampf(g_apap_fot_eval_period_s, 0.5f, 10.0f)
                           * 1000.0f);
            if (g_apap_osc_eval_ms >= eval_ms)
            {
                const bool osa = evaluateOscForOsa();
                g_apap_last_osa_detected = osa ? 1U : 0U;
                s_osa_treating = osa;
                g_apap_osa_treating = osa ? 1U : 0U;
                resetOscStats();
            }
        }

        if (s_osa_treating)
        {
            const float rate = raiseRate();
            g_apap_target_cmh2o += rate * dt_s;
            if (g_apap_target_cmh2o > s_max_cmh2o)
            {
                g_apap_target_cmh2o = s_max_cmh2o;
            }
        }
        break;

    case APAP_STATE_RESPOND:
        /* Legacy state: treat as continuous FOT probe. */
        enterFotProbe();
        break;

    case APAP_STATE_IDLE:
    default:
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        break;
    }

    g_apap_target_cmh2o = clampf(g_apap_target_cmh2o, s_min_cmh2o, s_max_cmh2o);
    return g_apap_target_cmh2o;
}
