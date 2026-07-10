/**
  ******************************************************************************
  * @file    epr_ctrl.c
  * @brief   EPR breath detection + setpoint relief.
  ******************************************************************************
  */

#include "epr_ctrl.h"
#include "therapy.h"
#include "stm32h7xx_hal.h"

#include <math.h>

/* === Tunables (debugger-writable volatiles) === */
volatile uint8_t g_epr_enabled                 = 0U;
volatile uint8_t g_epr_type                    = (uint8_t)EPR_TYPE_FULL_TIME;
volatile uint8_t g_epr_level                   = 1U;
volatile float   g_epr_relief_cmh2o            = 1.0f;
volatile float   g_epr_min_pressure_cmh2o      = 4.0f;

/* Relief restore (inhale rise): slewed for comfort. Expiratory apply: snap
 * for fast return to CPAP-level after inspiration ends. */
volatile float   g_epr_relief_apply_slew_cmh2o_per_s    = 12.0f;
volatile float   g_epr_relief_restore_slew_cmh2o_per_s  = 8.0f;

/* Breath detection thresholds (absolute minimums in slm). Adaptive logic
 * also scales thresholds as a fraction of the running flow baseline so a
 * high-leak bench rig and a masked patient both work without retuning. */
volatile float   g_epr_insp_thresh_slm         = 4.0f;
volatile float   g_epr_exp_enter_thresh_slm    = 3.0f;   /* enter exhale    */
volatile float   g_epr_exp_end_hyst_slm        = 0.5f;   /* end exhale      */

/* Fraction of flow baseline added to the absolute thresholds above.
 * With patient flow (~zero-mean) baseline is small; keep fractions low. */
volatile float   g_epr_insp_thresh_frac        = 0.02f;
volatile float   g_epr_exp_thresh_frac         = 0.02f;

volatile float   g_epr_flow_baseline_tau_s     = 2.5f;
volatile float   g_epr_baseline_track_deadband_slm = 5.0f;

volatile uint32_t g_epr_insp_confirm_ms        = 50U;
volatile uint32_t g_epr_exp_confirm_ms         = 50U;
volatile uint32_t g_epr_insp_min_hold_ms       = 120U;
/* Max confirmed-inspiration time before expiratory relief is forced
 * (breath detector missed exhale). 0 = disabled. */
volatile uint32_t g_epr_insp_max_hold_ms       = 1200U;
volatile float   g_epr_min_insp_peak_slm       = 4.0f;
/* Min flow fall from insp peak (slm) before exhale can be accepted. */
volatile float   g_epr_exp_min_peak_fall_slm   = 6.0f;
/* Min time (ms) after last insp-peak update before exhale qualify. */
volatile uint32_t g_epr_exp_post_peak_ms       = 80U;
/* |Q_patient| below this (slm) = expiratory flow ended. */
volatile float   g_epr_exp_end_flow_slm        = 4.0f;
volatile uint32_t g_epr_exp_end_confirm_ms     = 120U;
/* Min quiet time after exp relief before next insp arm (ms). */
volatile uint32_t g_epr_insp_refractory_ms     = 350U;
volatile uint32_t g_epr_insp_dropout_ms        = 80U;

/* Advanced expiratory bump / dual-peak mitigation in the breath FSM
 * (post-peak gate, peak-fall, refractory, relief latch). Default off —
 * set to 1 in debugger to re-enable. Blower-side twin:
 * g_blower_ctrl_epr_exhale_mit_enable. */
volatile uint8_t  g_epr_bump_mit_enable          = 0U;
/* Drop setpoint to expiratory level on first qualified exhale flow (after
 * insp_min_hold) without waiting for exp_confirm. FSM still debounces. */
volatile uint8_t  g_epr_exp_early_relief_enable  = 1U;
/* End inspiration when Q falls from peak AND therapy pressure is reached
 * (before Q goes strongly negative). Default on. */
volatile uint8_t  g_epr_peak_fall_detect_enable  = 1U;
volatile float    g_epr_insp_pressure_band_cmh2o = 0.5f;

/* === Live state === */
volatile uint8_t g_epr_phase                   = (uint8_t)EPR_PHASE_UNKNOWN;
volatile float   g_epr_active_relief_cmh2o     = 0.0f;
volatile float   g_epr_flow_baseline_slm       = 0.0f;
volatile uint32_t g_epr_step_count             = 0U;
volatile uint8_t  g_epr_insp_armed             = 0U;
volatile float    g_epr_insp_peak_delta_slm    = 0.0f;
volatile float    g_epr_flow_delta_slm         = 0.0f;  /* Q - baseline    */
volatile uint8_t  g_epr_breath_fsm             = 0U;    /* BreathFsm_t     */
volatile int8_t   g_epr_insp_polarity          = 0;     /* +1/-1 auto      */

/* === Internal breath FSM === */
typedef enum
{
    BREATH_IDLE = 0,
    BREATH_INSP_PENDING,
    BREATH_INSPIRATION,
    BREATH_EXP_PENDING,
    BREATH_EXPIRATION
} BreathFsm_t;

static BreathFsm_t s_breath_fsm = BREATH_IDLE;
static uint32_t    s_phase_timer_ms = 0U;
static float       s_cycle_insp_peak = 0.0f;
static float       s_cycle_insp_peak_q_slm = 0.0f;
static bool        s_baseline_seeded = false;
static int8_t      s_insp_polarity = 0;   /* +1: inhale = flow up; -1: inhale = flow down */
static uint32_t    s_startup_grace_ms = 0U;
static bool        s_pressurize_hold = false;
static float       s_hold_target_sp_cmh2o = 0.0f;
static uint32_t    s_hold_stable_ms = 0U;
static uint32_t    s_last_breath_ms = 0U;
static uint32_t    s_insp_enter_ms = 0U;
static uint32_t    s_insp_peak_ms = 0U;
static uint32_t    s_exp_commit_ms = 0U;
static uint32_t    s_dropout_timer_ms = 0U;
static bool        s_exp_relief_committed = false;
static bool        s_force_exp_relief = false;
static bool        s_awaiting_exp_complete = false;
static bool        s_early_exp_relief_latched = false;

#define EPR_STARTUP_GRACE_MS       5000U
#define EPR_PRESSURIZE_BAND_CMH2O  0.75f
#define EPR_PRESSURIZE_SETTLE_MS   1000U

static bool eprServiceActive(uint8_t therapy_state);
static bool eprInspirationActive(void);

static float effectiveInspThresh(void)
{
    const float base = (fabsf(g_epr_flow_baseline_slm) > 2.0f)
                         ? fabsf(g_epr_flow_baseline_slm) : 2.0f;
    return g_epr_insp_thresh_slm + base * g_epr_insp_thresh_frac;
}

static float effectiveExpEnterThresh(void)
{
    const float base = (fabsf(g_epr_flow_baseline_slm) > 2.0f)
                         ? fabsf(g_epr_flow_baseline_slm) : 2.0f;
    return g_epr_exp_enter_thresh_slm + base * g_epr_exp_thresh_frac;
}

static void syncFsmDebug(void)
{
    g_epr_breath_fsm = (uint8_t)s_breath_fsm;
    g_epr_insp_polarity = s_insp_polarity;
}

/* Map flow delta into "inspiration-positive" coordinates. Auto-detects
 * polarity on the first strong breath (handles bench test lungs where
 * inhale may appear as a flow drop instead of a rise). */
static float directedDelta(float delta)
{
    if (s_insp_polarity < 0)
    {
        return -delta;
    }
    return delta;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void lockPolarityFromDelta(float delta, float th)
{
    if (s_insp_polarity != 0)
    {
        return;
    }
    if (delta > th)
    {
        s_insp_polarity = 1;
    }
    else if (delta < -th)
    {
        s_insp_polarity = -1;
    }
}

static float patientDirectedFlow(float q_slm)
{
    if (s_insp_polarity < 0)
    {
        return -q_slm;
    }
    return q_slm;
}

static bool patientInspiratoryFlow(float q_slm, float insp_th)
{
    return patientDirectedFlow(q_slm) > insp_th;
}

static bool patientExpiratoryFlow(float q_slm, float exp_enter)
{
    return patientDirectedFlow(q_slm) < -exp_enter;
}

static bool patientFlowNearZero(float q_slm)
{
    const float th = clampf(g_epr_exp_end_flow_slm, 1.0f, 15.0f);
    return fabsf(q_slm) < th;
}

static void markExpiratoryReliefCommitted(uint32_t now_ms)
{
    s_exp_relief_committed = true;
    s_exp_commit_ms = now_ms;
}

static void slewToward(float *val, float target, float max_step)
{
    const float diff = target - *val;
    if (diff > max_step)
    {
        *val += max_step;
    }
    else if (diff < -max_step)
    {
        *val -= max_step;
    }
    else
    {
        *val = target;
    }
}

/* === Public API === */

void Epr_Init(void)
{
    g_epr_enabled             = 0U;
    g_epr_type                = (uint8_t)EPR_TYPE_FULL_TIME;
    g_epr_level               = 1U;
    g_epr_relief_cmh2o        = 1.0f;
    g_epr_min_pressure_cmh2o  = 4.0f;
    g_epr_relief_apply_slew_cmh2o_per_s    = 12.0f;
    g_epr_relief_restore_slew_cmh2o_per_s  = 8.0f;
    g_epr_flow_baseline_tau_s = 2.5f;
    g_epr_insp_thresh_slm     = 4.0f;
    g_epr_exp_enter_thresh_slm = 3.0f;
    g_epr_exp_end_hyst_slm    = 0.5f;
    g_epr_insp_thresh_frac    = 0.02f;
    g_epr_exp_thresh_frac     = 0.02f;
    g_epr_baseline_track_deadband_slm = 5.0f;
    g_epr_insp_confirm_ms     = 50U;
    g_epr_exp_confirm_ms      = 50U;
    g_epr_insp_min_hold_ms    = 120U;
    g_epr_insp_max_hold_ms    = 1200U;
    g_epr_min_insp_peak_slm   = 4.0f;
    g_epr_exp_min_peak_fall_slm = 6.0f;
    g_epr_exp_post_peak_ms    = 80U;
    g_epr_exp_end_flow_slm    = 4.0f;
    g_epr_exp_end_confirm_ms  = 120U;
    g_epr_insp_refractory_ms  = 350U;
    g_epr_insp_dropout_ms     = 80U;
    g_epr_bump_mit_enable     = 0U;
    g_epr_exp_early_relief_enable = 1U;
    g_epr_peak_fall_detect_enable = 1U;
    g_epr_insp_pressure_band_cmh2o = 0.5f;

    g_epr_phase               = (uint8_t)EPR_PHASE_UNKNOWN;
    g_epr_active_relief_cmh2o = 0.0f;
    g_epr_flow_baseline_slm   = 0.0f;
    g_epr_step_count          = 0U;
    g_epr_insp_armed          = 0U;
    g_epr_insp_peak_delta_slm = 0.0f;
    g_epr_flow_delta_slm      = 0.0f;
    g_epr_breath_fsm          = 0U;
    g_epr_insp_polarity      = 0;
    s_baseline_seeded         = false;
    s_breath_fsm              = BREATH_IDLE;
    s_phase_timer_ms          = 0U;
    s_cycle_insp_peak         = 0.0f;
    s_cycle_insp_peak_q_slm   = 0.0f;
    s_insp_polarity           = 0;
    s_insp_enter_ms           = 0U;
    s_insp_peak_ms            = 0U;
    s_exp_commit_ms           = 0U;
    s_dropout_timer_ms        = 0U;
    s_exp_relief_committed    = false;
    s_force_exp_relief        = false;
    s_awaiting_exp_complete   = false;
    s_early_exp_relief_latched = false;
}

void  Epr_SetEnable(bool en)              { g_epr_enabled = en ? 1U : 0U;
                                            if (!en) { s_pressurize_hold = false; } }
bool  Epr_IsEnabled(void)                 { return (g_epr_enabled != 0U); }

void Epr_SetType(EprType_t type)
{
    g_epr_type = (type == EPR_TYPE_RAMP_ONLY) ? (uint8_t)EPR_TYPE_RAMP_ONLY
                                              : (uint8_t)EPR_TYPE_FULL_TIME;
}

EprType_t Epr_GetType(void)
{
    return (g_epr_type == (uint8_t)EPR_TYPE_RAMP_ONLY) ? EPR_TYPE_RAMP_ONLY
                                                       : EPR_TYPE_FULL_TIME;
}

void Epr_SetLevel(uint8_t level)
{
    if (level < 1U) { level = 1U; }
    if (level > 3U) { level = 3U; }
    g_epr_level = level;
    g_epr_relief_cmh2o = (float)level;
    if (!eprInspirationActive())
    {
        g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
    }
}

uint8_t Epr_GetLevel(void)                { return g_epr_level; }

void Epr_SetReliefCmh2o(float cmh2o)
{
    const float clamped = clampf(cmh2o, 1.0f, 3.0f);
    g_epr_relief_cmh2o = clamped;
    g_epr_level = (uint8_t)lrintf(clamped);
    if (g_epr_level < 1U) { g_epr_level = 1U; }
    if (g_epr_level > 3U) { g_epr_level = 3U; }
    if (!eprInspirationActive())
    {
        g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
    }
}

float Epr_GetReliefCmh2o(void)            { return g_epr_relief_cmh2o; }

void Epr_SetMinPressureCmh2o(float cmh2o)
{
    g_epr_min_pressure_cmh2o = clampf(cmh2o, 4.0f, 10.0f);
}

float Epr_GetMinPressureCmh2o(void)       { return g_epr_min_pressure_cmh2o; }

EprPhase_t Epr_GetPhase(void)             { return (EprPhase_t)g_epr_phase; }

const char* Epr_GetPhaseName(void)
{
    switch ((EprPhase_t)g_epr_phase)
    {
    case EPR_PHASE_INSPIRATION:  return "Inhale";
    case EPR_PHASE_EXPIRATION:   return "Exhale";
    case EPR_PHASE_UNKNOWN:
    default:                     return "---";
    }
}

float Epr_GetActiveReliefCmh2o(void)      { return g_epr_active_relief_cmh2o; }
float Epr_GetFlowBaselineSlm(void)        { return g_epr_flow_baseline_slm; }

static void resetBreathFsmCore(void)
{
    g_epr_phase               = (uint8_t)EPR_PHASE_UNKNOWN;
    g_epr_flow_baseline_slm   = 0.0f;
    g_epr_insp_armed          = 0U;
    g_epr_insp_peak_delta_slm = 0.0f;
    g_epr_flow_delta_slm      = 0.0f;
    g_epr_breath_fsm          = 0U;
    g_epr_insp_polarity      = 0;
    s_baseline_seeded         = false;
    s_breath_fsm              = BREATH_IDLE;
    s_phase_timer_ms          = 0U;
    s_cycle_insp_peak         = 0.0f;
    s_cycle_insp_peak_q_slm   = 0.0f;
    s_insp_polarity           = 0;
    s_last_breath_ms          = 0U;
    s_insp_enter_ms           = 0U;
    s_insp_peak_ms            = 0U;
    s_exp_commit_ms           = 0U;
    s_dropout_timer_ms        = 0U;
    s_exp_relief_committed    = false;
    s_force_exp_relief        = false;
    s_awaiting_exp_complete   = false;
    s_early_exp_relief_latched = false;
}

void Epr_ResetBreathState(void)
{
    resetBreathFsmCore();
    g_epr_active_relief_cmh2o = 0.0f;
    s_startup_grace_ms = 0U;
    s_pressurize_hold = false;
    s_hold_stable_ms = 0U;
}

static float expiratorySetpointCmh2o(float base_setpoint_cmh2o)
{
    float sp = base_setpoint_cmh2o - g_epr_relief_cmh2o;
    if (sp < g_epr_min_pressure_cmh2o)
    {
        sp = g_epr_min_pressure_cmh2o;
    }
    return sp;
}

float Epr_GetExpiratorySetpointCmh2o(float base_setpoint_cmh2o,
                                     uint8_t therapy_state)
{
    if (!eprServiceActive(therapy_state))
    {
        return base_setpoint_cmh2o;
    }
    return expiratorySetpointCmh2o(base_setpoint_cmh2o);
}

void Epr_SetStartupHoldTarget(float expiratory_sp_cmh2o)
{
    if (g_epr_enabled == 0U)
    {
        s_pressurize_hold = false;
        s_hold_stable_ms = 0U;
        return;
    }

    s_pressurize_hold = true;
    s_hold_target_sp_cmh2o = expiratory_sp_cmh2o;
    s_hold_stable_ms = 0U;
}

void Epr_OnTherapyStart(void)
{
    resetBreathFsmCore();

    if (g_epr_enabled != 0U)
    {
        /* AirSense: therapy begins at expiratory (CPAP - level) pressure. */
        g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
        s_startup_grace_ms = EPR_STARTUP_GRACE_MS;
        s_hold_stable_ms = 0U;
    }
    else
    {
        g_epr_active_relief_cmh2o = 0.0f;
        s_startup_grace_ms = 0U;
        s_pressurize_hold = false;
        s_hold_stable_ms = 0U;
    }
}

static void mapPhaseForGui(BreathFsm_t fsm)
{
    switch (fsm)
    {
    case BREATH_INSP_PENDING:
    case BREATH_INSPIRATION:
        g_epr_phase = (uint8_t)EPR_PHASE_INSPIRATION;
        break;
    case BREATH_EXP_PENDING:
    case BREATH_EXPIRATION:
        g_epr_phase = (uint8_t)EPR_PHASE_EXPIRATION;
        break;
    case BREATH_IDLE:
    default:
        g_epr_phase = (uint8_t)EPR_PHASE_UNKNOWN;
        break;
    }
}

static bool eprServiceActive(uint8_t therapy_state)
{
    if (g_epr_enabled == 0U)
    {
        return false;
    }

    if (g_epr_type == (uint8_t)EPR_TYPE_RAMP_ONLY)
    {
        return (therapy_state == (uint8_t)THERAPY_STATE_RAMP);
    }

    return (therapy_state == (uint8_t)THERAPY_STATE_RAMP)
        || (therapy_state == (uint8_t)THERAPY_STATE_RUNNING);
}

static bool eprInspirationActive(void)
{
    if (s_startup_grace_ms > 0U)
    {
        return false;
    }

    if (s_pressurize_hold)
    {
        return false;
    }

    if (s_force_exp_relief)
    {
        return false;
    }

    /* Full CPAP only after inspiration is confirmed — pending state is
     * too sensitive to leak / spin-up flow and caused 8 cmH2O flapping. */
    if (s_breath_fsm != BREATH_INSPIRATION)
    {
        return false;
    }

    /* Latched after sustained exhale flow — do not toggle on brief flow
     * reversals (caused a double pressure peak during inspiration). */
    if ((g_epr_bump_mit_enable != 0U) && s_exp_relief_committed)
    {
        return false;
    }

    if ((g_epr_exp_early_relief_enable != 0U) && s_early_exp_relief_latched)
    {
        return false;
    }

    return true;
}

static bool expiratoryFlowQualified(float q_slm, float dir, float exp_enter,
                                    float min_peak, uint32_t now_ms)
{
    if (s_cycle_insp_peak < min_peak)
    {
        return false;
    }

    if (!patientExpiratoryFlow(q_slm, exp_enter))
    {
        return false;
    }

    if (dir >= -exp_enter)
    {
        return false;
    }

    const float min_fall = clampf(g_epr_exp_min_peak_fall_slm, 1.0f, 25.0f);
    const float fall_from_peak = s_cycle_insp_peak - dir;
    if (fall_from_peak < min_fall)
    {
        return false;
    }

    const uint32_t post_peak = (g_epr_exp_post_peak_ms > 0U)
                                 ? g_epr_exp_post_peak_ms : 1U;
    if ((now_ms - s_insp_peak_ms) < post_peak)
    {
        return false;
    }

    return true;
}

static void trackInspFlowPeak(float q_slm)
{
    const float q_dir = patientDirectedFlow(q_slm);
    if (q_dir > s_cycle_insp_peak_q_slm)
    {
        s_cycle_insp_peak_q_slm = q_dir;
        s_insp_peak_ms = HAL_GetTick();
    }
}

static bool inspTargetPressureReached(float p_meas_cmh2o, float therapy_base_sp)
{
    if (therapy_base_sp <= 0.0f)
    {
        return false;
    }

    const float band = clampf(g_epr_insp_pressure_band_cmh2o, 0.1f, 2.0f);
    return p_meas_cmh2o >= (therapy_base_sp - band);
}

static bool flowFallingFromPeak(float q_slm, float min_peak_q)
{
    if (s_cycle_insp_peak_q_slm < min_peak_q)
    {
        return false;
    }

    const float q_dir = patientDirectedFlow(q_slm);
    const float min_fall = clampf(g_epr_exp_min_peak_fall_slm, 1.0f, 25.0f);
    return (s_cycle_insp_peak_q_slm - q_dir) >= min_fall;
}

static bool inspirationEndingDetected(float q_slm, float dir, float p_meas_cmh2o,
                                      float therapy_base_sp, float exp_enter,
                                      float min_peak, uint32_t now_ms)
{
    if (g_epr_bump_mit_enable != 0U)
    {
        return expiratoryFlowQualified(q_slm, dir, exp_enter, min_peak, now_ms);
    }

    if (g_epr_peak_fall_detect_enable != 0U)
    {
        const float min_peak_q = clampf(g_epr_min_insp_peak_slm, 2.0f, 30.0f);
        const uint32_t post_peak = (g_epr_exp_post_peak_ms > 0U)
                                     ? g_epr_exp_post_peak_ms : 1U;

        if (flowFallingFromPeak(q_slm, min_peak_q)
            && inspTargetPressureReached(p_meas_cmh2o, therapy_base_sp)
            && ((now_ms - s_insp_peak_ms) >= post_peak))
        {
            return true;
        }
    }

    /* Fallback: sustained expiratory flow (Q clearly negative). */
    return (dir < -exp_enter) && (s_cycle_insp_peak >= min_peak);
}

static void beginExpiratoryPhase(uint32_t now_ms, BreathFsm_t next_fsm)
{
    if (g_epr_bump_mit_enable != 0U)
    {
        markExpiratoryReliefCommitted(now_ms);
    }
    s_awaiting_exp_complete = true;
    s_breath_fsm = next_fsm;
    s_phase_timer_ms = 0U;
}

static void updatePressurizeHold(float p_meas_cmh2o, uint32_t dt_ms)
{
    if (!s_pressurize_hold)
    {
        return;
    }

    if (fabsf(p_meas_cmh2o - s_hold_target_sp_cmh2o) <= EPR_PRESSURIZE_BAND_CMH2O)
    {
        s_hold_stable_ms += dt_ms;
        if (s_hold_stable_ms >= EPR_PRESSURIZE_SETTLE_MS)
        {
            s_pressurize_hold = false;
            s_hold_stable_ms = 0U;
        }
    }
    else
    {
        s_hold_stable_ms = 0U;
    }
}

void Epr_Update(float q_slm, float p_meas_cmh2o, float therapy_base_sp_cmh2o,
                float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    if (s_startup_grace_ms > 0U)
    {
        if (dt_ms >= s_startup_grace_ms)
        {
            s_startup_grace_ms = 0U;
            /* Keep flow baseline from grace; only reset phase tracking. */
            s_breath_fsm       = BREATH_IDLE;
            s_phase_timer_ms   = 0U;
            s_cycle_insp_peak  = 0.0f;
            s_insp_polarity    = 0;
            s_insp_enter_ms    = 0U;
            g_epr_insp_armed   = 0U;
            mapPhaseForGui(BREATH_IDLE);
            syncFsmDebug();
        }
        else
        {
            s_startup_grace_ms -= dt_ms;
        }
    }

    updatePressurizeHold(p_meas_cmh2o, dt_ms);

    /* Slow baseline tracks mid-breath flow. Only update when flow is near
     * the current baseline so inspiration/exhalation excursions do not drag
     * the reference with them (major noise-immunity improvement). */
    const float tau = clampf(g_epr_flow_baseline_tau_s, 0.5f, 5.0f);
    const float alpha = dt_s / (tau + dt_s);
    const float deadband = clampf(g_epr_baseline_track_deadband_slm, 2.0f, 20.0f);

    if (!s_baseline_seeded)
    {
        g_epr_flow_baseline_slm = q_slm;
        s_baseline_seeded = true;
    }
    else
    {
        const float delta_for_base = q_slm - g_epr_flow_baseline_slm;
        if (fabsf(delta_for_base) < deadband)
        {
            g_epr_flow_baseline_slm = alpha * q_slm
                                    + (1.0f - alpha) * g_epr_flow_baseline_slm;
        }
    }

    const float delta = q_slm - g_epr_flow_baseline_slm;
    g_epr_flow_delta_slm = delta;

    const float insp_th   = effectiveInspThresh();
    const float exp_enter = effectiveExpEnterThresh();
    const float min_peak  = clampf(g_epr_min_insp_peak_slm, 2.0f, 30.0f);
    const uint32_t insp_confirm = (g_epr_insp_confirm_ms > 0U)
                                    ? g_epr_insp_confirm_ms : 1U;
    const uint32_t exp_confirm  = (g_epr_exp_confirm_ms > 0U)
                                    ? g_epr_exp_confirm_ms : 1U;

    const float dir = directedDelta(delta);

    /* Ignore spin-up flow transients during startup grace. */
    if (s_startup_grace_ms > 0U)
    {
        s_breath_fsm = BREATH_IDLE;
        mapPhaseForGui(BREATH_IDLE);
        syncFsmDebug();
        g_epr_step_count++;
        return;
    }

    /* --- Breath FSM: confirm inhale, then accept moderate exhale dip --- */
    switch (s_breath_fsm)
    {
    case BREATH_IDLE:
    {
        g_epr_insp_armed = 0U;
        s_exp_relief_committed = false;
        s_force_exp_relief = false;
        s_cycle_insp_peak = 0.0f;
        s_cycle_insp_peak_q_slm = 0.0f;
        s_dropout_timer_ms = 0U;

        if (s_awaiting_exp_complete)
        {
            break;
        }

        const uint32_t now = HAL_GetTick();
        if (g_epr_bump_mit_enable != 0U)
        {
            /* Time-only refractory after exp relief. */
            if (s_exp_commit_ms != 0U)
            {
                const uint32_t since_commit = now - s_exp_commit_ms;
                const uint32_t refractory = (g_epr_insp_refractory_ms > 0U)
                                              ? g_epr_insp_refractory_ms : 0U;
                if ((refractory > 0U) && (since_commit < refractory))
                {
                    break;
                }
            }
        }

        if ((delta > insp_th) || (delta < -insp_th))
        {
            lockPolarityFromDelta(delta, insp_th);
        }

        if (patientInspiratoryFlow(q_slm, insp_th))
        {
            s_breath_fsm = BREATH_INSP_PENDING;
            s_phase_timer_ms = 0U;
            s_cycle_insp_peak = fabsf(dir);
            s_insp_peak_ms = now;
        }
        break;
    }

    case BREATH_INSP_PENDING:
        if (patientInspiratoryFlow(q_slm, insp_th))
        {
            s_dropout_timer_ms = 0U;
            s_phase_timer_ms += dt_ms;
            if (dir > s_cycle_insp_peak)
            {
                s_cycle_insp_peak = dir;
                s_insp_peak_ms = HAL_GetTick();
            }
            trackInspFlowPeak(q_slm);
            if (s_phase_timer_ms >= insp_confirm)
            {
                s_breath_fsm = BREATH_INSPIRATION;
                s_force_exp_relief = false;
                s_exp_relief_committed = false;
                s_early_exp_relief_latched = false;
                g_epr_insp_armed = 1U;
                g_epr_insp_peak_delta_slm = s_cycle_insp_peak;
                s_phase_timer_ms = 0U;
                s_insp_enter_ms = HAL_GetTick();
                s_insp_peak_ms = s_insp_enter_ms;
                s_last_breath_ms = s_insp_enter_ms;
            }
        }
        else
        {
            const uint32_t dropout = (g_epr_insp_dropout_ms > 0U)
                                       ? g_epr_insp_dropout_ms : 1U;
            s_dropout_timer_ms += dt_ms;
            s_phase_timer_ms = 0U;
            if (s_dropout_timer_ms >= dropout)
            {
                s_breath_fsm = BREATH_IDLE;
                s_dropout_timer_ms = 0U;
                s_cycle_insp_peak = 0.0f;
                s_cycle_insp_peak_q_slm = 0.0f;
            }
        }
        break;

    case BREATH_INSPIRATION:
        if (dir > s_cycle_insp_peak)
        {
            s_cycle_insp_peak = dir;
            g_epr_insp_peak_delta_slm = s_cycle_insp_peak;
            s_insp_peak_ms = HAL_GetTick();
        }
        trackInspFlowPeak(q_slm);

        {
            const uint32_t now = HAL_GetTick();
            const uint32_t insp_elapsed = now - s_insp_enter_ms;
            const uint32_t max_hold = g_epr_insp_max_hold_ms;

            if (max_hold > 0U && insp_elapsed >= max_hold)
            {
                s_force_exp_relief = true;
                beginExpiratoryPhase(now, BREATH_EXPIRATION);
                break;
            }

            if (inspirationEndingDetected(q_slm, dir, p_meas_cmh2o,
                                            therapy_base_sp_cmh2o, exp_enter,
                                            min_peak, now))
            {
                const uint32_t min_hold = (g_epr_insp_min_hold_ms > 0U)
                                            ? g_epr_insp_min_hold_ms : 1U;
                if (insp_elapsed >= min_hold)
                {
                    if (g_epr_exp_early_relief_enable != 0U)
                    {
                        s_early_exp_relief_latched = true;
                    }

                    s_phase_timer_ms += dt_ms;
                    if (s_phase_timer_ms >= exp_confirm)
                    {
                        beginExpiratoryPhase(now, BREATH_EXP_PENDING);
                    }
                }
                else
                {
                    s_phase_timer_ms = 0U;
                }
            }
            else
            {
                s_phase_timer_ms = 0U;
            }
        }
        break;

    case BREATH_EXP_PENDING:
        s_phase_timer_ms += dt_ms;
        if (s_phase_timer_ms >= exp_confirm)
        {
            s_breath_fsm = BREATH_EXPIRATION;
            s_phase_timer_ms = 0U;
        }
        break;

    case BREATH_EXPIRATION:
        if (patientFlowNearZero(q_slm))
        {
            s_phase_timer_ms += dt_ms;
            const uint32_t end_confirm = (g_epr_exp_end_confirm_ms > 0U)
                                           ? g_epr_exp_end_confirm_ms : 1U;
            if (s_phase_timer_ms >= end_confirm)
            {
                s_breath_fsm = BREATH_IDLE;
                g_epr_insp_armed = 0U;
                s_exp_relief_committed = false;
                s_force_exp_relief = false;
                s_exp_commit_ms = 0U;
                s_awaiting_exp_complete = false;
                s_early_exp_relief_latched = false;
                s_phase_timer_ms = 0U;
                s_cycle_insp_peak = 0.0f;
                s_cycle_insp_peak_q_slm = 0.0f;
                s_dropout_timer_ms = 0U;
            }
        }
        else
        {
            s_phase_timer_ms = 0U;
        }
        break;

    default:
        s_breath_fsm = BREATH_IDLE;
        break;
    }

    mapPhaseForGui(s_breath_fsm);
    syncFsmDebug();
    g_epr_step_count++;
}

float Epr_ModifySetpoint(float base_setpoint_cmh2o, float dt_s,
                         uint8_t therapy_state)
{
    if (!eprServiceActive(therapy_state))
    {
        g_epr_active_relief_cmh2o = 0.0f;
        return base_setpoint_cmh2o;
    }

    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        if (!eprInspirationActive())
        {
            g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
            return expiratorySetpointCmh2o(base_setpoint_cmh2o);
        }
        g_epr_active_relief_cmh2o = 0.0f;
        return base_setpoint_cmh2o;
    }

    float target_relief = 0.0f;
    if (!eprInspirationActive())
    {
        /* AirSense baseline: CPAP - level except during inspiration. */
        target_relief = g_epr_relief_cmh2o;
    }

    if (target_relief <= 0.0f)
    {
        /* Slewed restore during inspiration (comfort). */
        const float slew = clampf(g_epr_relief_restore_slew_cmh2o_per_s,
                                  1.0f, 40.0f);
        const float max_step = slew * dt_s;
        slewToward((float *)&g_epr_active_relief_cmh2o, 0.0f, max_step);
    }
    else
    {
        /* Snap expiratory relief when exhale is detected. */
        g_epr_active_relief_cmh2o = target_relief;
    }

    float sp = base_setpoint_cmh2o - g_epr_active_relief_cmh2o;
    if (sp < g_epr_min_pressure_cmh2o)
    {
        sp = g_epr_min_pressure_cmh2o;
    }

    if (s_pressurize_hold)
    {
        s_hold_target_sp_cmh2o = sp;
    }

    return sp;
}

bool Epr_IsInspiratoryPressureActive(void)
{
    return eprInspirationActive();
}

bool Epr_HasRecentBreath(uint32_t window_ms)
{
    if (s_last_breath_ms == 0U)
    {
        return false;
    }
    if (window_ms == 0U)
    {
        return false;
    }
    const uint32_t now = HAL_GetTick();
    return (now - s_last_breath_ms) < window_ms;
}
