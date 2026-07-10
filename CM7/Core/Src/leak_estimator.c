/**
  ******************************************************************************
  * @file    leak_estimator.c
  * @brief   Implementation: total - vent, slow-LPF, threshold compare.
  ******************************************************************************
  */

#include "leak_estimator.h"
#include "leak_estimator_vivo.h"

#include <math.h>

/* === Algorithm selection === */
volatile uint8_t g_leak_vivo_enable = 0U;
static LeakAlgo_t s_algo = LEAK_ALGO_SIMPLE;
static float s_therapy_sp_cmh2o = 8.0f;

/* === Public configuration (debugger-writable volatiles) === */
volatile float    g_leak_vent_k         = 8.7f;       /* slm / sqrt(cmH2O) */
volatile float    g_leak_tau_seconds    = 10.0f;       /* slow LPF time const*/
volatile float    g_leak_high_threshold = 24.0f;       /* slm                */
volatile float    g_leak_vent_p_tau_s   = 0.5f;        /* P mask LPF for vent */
volatile float    g_leak_patient_lpf_hz = 2.0f;        /* breath-band LPF    */

/* === State (debugger-readable volatiles) === */
volatile float    g_leak_total_slm      = 0.0f;
volatile float    g_leak_vent_slm       = 0.0f;
volatile float    g_leak_uninten_slm    = 0.0f;
volatile float    g_leak_instant_excess_slm = 0.0f;   /* Q_total - Q_vent   */
volatile float    g_flow_patient_raw_slm = 0.0f;       /* before breath LPF  */
volatile float    g_flow_patient_slm    = 0.0f;        /* LPF for EPR/plot   */
volatile uint8_t  g_leak_is_high        = 0U;
volatile uint32_t g_leak_step_count     = 0U;

/* === Internal === */
static bool s_seeded = false;
static bool s_patient_filt_seeded = false;
static float s_mask_p_filt_cmh2o = 0.0f;

#define LEAK_TWO_PI  6.28318530717958647692f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* === Public API === */

void Leak_Init(void)
{
    g_leak_vent_k         = 8.7f;
    g_leak_tau_seconds    = 10.0f;
    g_leak_high_threshold = 24.0f;
    g_leak_vent_p_tau_s   = 0.5f;
    g_leak_patient_lpf_hz = 2.0f;

    g_leak_total_slm   = 0.0f;
    g_leak_vent_slm    = 0.0f;
    g_leak_uninten_slm = 0.0f;
    g_leak_instant_excess_slm = 0.0f;
    g_flow_patient_raw_slm = 0.0f;
    g_flow_patient_slm = 0.0f;
    g_leak_is_high     = 0U;
    g_leak_step_count  = 0U;
    s_seeded           = false;
    s_patient_filt_seeded = false;
    s_mask_p_filt_cmh2o = 0.0f;

    s_algo = LEAK_ALGO_SIMPLE;
    g_leak_vivo_enable = 0U;
    s_therapy_sp_cmh2o = 8.0f;
    LeakVivo_Init();
}

void Leak_SetAlgo(LeakAlgo_t algo)
{
    s_algo = algo;
    g_leak_vivo_enable = (algo == LEAK_ALGO_VIVO) ? 1U : 0U;
}

LeakAlgo_t Leak_GetAlgo(void)
{
    return s_algo;
}

void Leak_SetVivoEnable(bool en)
{
    Leak_SetAlgo(en ? LEAK_ALGO_VIVO : LEAK_ALGO_SIMPLE);
}

bool Leak_IsVivoEnabled(void)
{
    return (s_algo == LEAK_ALGO_VIVO);
}

void Leak_SetTherapyPressureCmh2o(float cmh2o)
{
    if (cmh2o < 0.0f)
    {
        cmh2o = 0.0f;
    }
    s_therapy_sp_cmh2o = cmh2o;
}

void Leak_SetStandardRatio(int32_t ratio)
{
    LeakVivo_SetStandardRatio(ratio);
}

void Leak_SetVentCoefficient(float k)
{
    g_leak_vent_k = clampf(k, 5.0f, 20.0f);
}

void Leak_SetTimeConstant(float seconds)
{
    g_leak_tau_seconds = clampf(seconds, 1.0f, 60.0f);
}

void Leak_SetHighThreshold(float slm)
{
    g_leak_high_threshold = clampf(slm, 5.0f, 60.0f);
    LeakVivo_SetHighThreshold(g_leak_high_threshold);
}

float Leak_GetVentCoefficient(void) { return g_leak_vent_k; }
float Leak_GetTimeConstant(void)    { return g_leak_tau_seconds; }
float Leak_GetHighThreshold(void)   { return g_leak_high_threshold; }
float Leak_GetTotalSlm(void)        { return g_leak_total_slm; }
float Leak_GetVentSlm(void)         { return g_leak_vent_slm; }
float Leak_GetUnintentionalSlm(void){ return g_leak_uninten_slm; }
float Leak_GetInstantExcessSlm(void){ return g_leak_instant_excess_slm; }
float Leak_GetPatientSlm(void)      { return g_flow_patient_slm; }
bool  Leak_IsHigh(void)             { return (g_leak_is_high != 0U); }

int32_t Leak_GetMeanRatio(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetMeanRatio() : 0;
}

int32_t Leak_GetMeanRatio10Sec(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetMeanRatio10Sec() : 0;
}

int32_t Leak_GetInstantRatio(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetInstantRatio() : 0;
}

void Leak_SetDisconnectFreeze(bool freeze)
{
    if (Leak_IsVivoEnabled())
    {
        LeakVivo_SetAdaptationFreeze(freeze);
    }
}

float Leak_GetFlowLeakSlm(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_flow_leak_slm
                                : (g_leak_vent_slm + g_leak_uninten_slm);
}

float Leak_GetReportedLeakSlm(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetReportedLeakSlm()
                                : g_leak_uninten_slm;
}

float Leak_GetRestVolumeMl(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_rest_volume_ml : 0.0f;
}

uint8_t Leak_GetBreathInspiration(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_breath_insp : 0U;
}

void Leak_Reseed(void)
{
    /* Reseed active backend. */
    s_seeded = false;
    s_patient_filt_seeded = false;
    LeakVivo_Reset();
}

static void leakUpdateSimple(float total_q_slm, float mask_pressure_cmh2o, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    g_leak_total_slm = total_q_slm;

    /* Modelled vent flow. Smooth mask pressure before sqrt() so PID
     * ripple does not modulate Q_vent and appear as noise on Q_patient. */
    const float p_raw = (mask_pressure_cmh2o > 0.0f) ? mask_pressure_cmh2o : 0.0f;
    const float p_tau = clampf(g_leak_vent_p_tau_s, 0.05f, 2.0f);
    const float p_alpha = dt_s / (p_tau + dt_s);
    if (!s_seeded)
    {
        s_mask_p_filt_cmh2o = p_raw;
    }
    else
    {
        s_mask_p_filt_cmh2o = p_alpha * p_raw
                            + (1.0f - p_alpha) * s_mask_p_filt_cmh2o;
    }
    g_leak_vent_slm = g_leak_vent_k * sqrtf(s_mask_p_filt_cmh2o);

    /* Instantaneous "everything that isn't vent" flow. Over a complete
     * breath cycle this averages to Q_leak; cycle-by-cycle it swings
     * with tidal volume. The slow LPF below extracts the DC component. */
    const float instant_excess = total_q_slm - g_leak_vent_slm;
    g_leak_instant_excess_slm = instant_excess;

    /* Slow first-order IIR. tau = 10 s with 5 ms tick gives:
     *      alpha = dt / (tau + dt) = 0.005 / 10.005 = 4.998e-4
     * which is small (long memory) - exactly what we want for averaging
     * out the breath cycle. We use the bilinear form to stay accurate
     * if dt or tau is poked from the watch window. */
    const float tau = clampf(g_leak_tau_seconds, 1.0f, 60.0f);
    const float alpha = dt_s / (tau + dt_s);

    if (!s_seeded)
    {
        /* Reseed: snap to the instantaneous excess so we don't have to
         * wait ~30 s for the LPF to climb out of zero on therapy start. */
        g_leak_uninten_slm = instant_excess;
        s_seeded = true;
    }
    else
    {
        g_leak_uninten_slm = alpha * instant_excess
                           + (1.0f - alpha) * g_leak_uninten_slm;
    }

    g_leak_is_high = (fabsf(g_leak_uninten_slm) > g_leak_high_threshold)
                       ? 1U : 0U;

    /* Patient flow: remove vent orifice flow and the slow unintentional-
     * leak estimate from total flow. Over a full breath cycle this signal
     * is approximately zero-mean and tracks inhale/exhale shape.
     * The AC residual is high-passed sensor noise — apply a breath-band
     * LPF (default 2 Hz) before EPR / telemetry. */
    const float patient_raw = instant_excess - g_leak_uninten_slm;
    g_flow_patient_raw_slm = patient_raw;

    const float fc = clampf(g_leak_patient_lpf_hz, 0.5f, 8.0f);
    const float omega = LEAK_TWO_PI * fc;
    const float filt_alpha = (omega * dt_s) / (1.0f + omega * dt_s);

    if (!s_patient_filt_seeded)
    {
        g_flow_patient_slm = patient_raw;
        s_patient_filt_seeded = true;
    }
    else
    {
        g_flow_patient_slm = filt_alpha * patient_raw
                           + (1.0f - filt_alpha) * g_flow_patient_slm;
    }

    g_leak_step_count++;
}

void Leak_Update(float total_q_slm, float mask_pressure_cmh2o, float dt_s)
{
    if (s_algo == LEAK_ALGO_VIVO)
    {
        LeakVivo_Update(total_q_slm, mask_pressure_cmh2o, s_therapy_sp_cmh2o, dt_s);

        g_leak_total_slm          = LeakVivo_GetTotalSlm();
        g_leak_vent_slm           = LeakVivo_GetVentSlm();
        g_leak_uninten_slm        = LeakVivo_GetUnintentionalSlm();
        g_leak_instant_excess_slm = LeakVivo_GetInstantExcessSlm();
        g_flow_patient_raw_slm    = LeakVivo_GetPatientRawSlm();
        g_flow_patient_slm        = LeakVivo_GetPatientSlm();
        g_leak_is_high            = LeakVivo_IsHigh() ? 1U : 0U;
        g_leak_step_count++;
        return;
    }

    leakUpdateSimple(total_q_slm, mask_pressure_cmh2o, dt_s);
}
