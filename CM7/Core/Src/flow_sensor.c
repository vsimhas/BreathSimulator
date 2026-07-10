/**
  ******************************************************************************
  * @file    flow_sensor.c
  * @brief   Flow-element ΔP conditioning - implementation.
  *
  *  The 1st-order IIR LPF used here is the discrete-time bilinear (Tustin)
  *  approximation of an analog RC low-pass:
  *
  *      H(s) = 1 / (1 + s/(2*pi*fc))
  *
  *      In discrete form, with omega = 2*pi*fc and dt = sample period:
  *          y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  *      with
  *          alpha = (omega * dt) / (1 + omega * dt)
  *
  *  At fc = 10 Hz, dt = 5 ms:
  *          omega*dt = 2*pi*10*0.005 = 0.314
  *          alpha    = 0.314 / 1.314  = 0.239
  *
  *  Step response settles to 95% in ~3 time constants, i.e. ~50 ms - well
  *  inside the 0.5-2 s breath cycle so respiratory features are preserved.
  *
  *  Why bilinear instead of impulse-invariance (alpha = 1 - exp(-omega*dt)):
  *  both behave identically for omega*dt << 1; the bilinear form has no
  *  expf() call so it stays cheap even if you ever crank dt down to 1 ms.
  ******************************************************************************
  */

#include "flow_sensor.h"
#include "leak_estimator.h"

#include <math.h>
#include <stddef.h>

#define FLOW_TWO_PI   6.28318530717958647692f

/* === Public configuration (debugger-visible volatiles) === */
volatile float    g_flow_lpf_cutoff_hz   = 15.0f;      /* tunable cutoff    */
volatile float    g_flow_zero_offset_mbar = -0.02f;    /* new flow element  */

/* === State (debugger-visible volatiles) === */
volatile float    g_flow_dp_raw_mbar   = 0.0f;
volatile float    g_flow_dp_filt_mbar  = 0.0f;
volatile float    g_flow_dp_corr_mbar  = 0.0f;        /* filtered minus zero */
volatile float    g_flow_q_slm         = 0.0f;        /* from filtered ΔP   */
volatile float    g_flow_q_slm_raw     = 0.0f;        /* from raw ΔP        */
volatile float    g_flow_lpf_alpha     = 0.0f;        /* read-only output  */
volatile uint32_t g_flow_step_count    = 0U;

/* === Internal === */
static bool s_filt_seeded = false;       /* first sample bypasses filter */

/* === Calibration table =====================================================
 *
 *  New flow element bench data (user-supplied, 2026). Raw ΔP is the LPF
 *  output g_flow_dp_filt_mbar; zero flow reads -0.02 mbar. After subtracting
 *  the configured zero offset, the table below is ΔP_corrected -> slm.
 *
 *      raw ΔP (mbar)  ΔP_corr (mbar)   Q (slm)
 *      ---------------------------------------------
 *      -4.740         -4.720          -121.0
 *      -4.010         -3.990          -110.0
 *      -3.350         -3.330           -99.0
 *      -2.760         -2.740           -88.0
 *      -2.220         -2.200           -78.0
 *      -1.740         -1.720           -67.0
 *      -1.280         -1.260           -54.8
 *      -0.910         -0.890           -43.8
 *      -0.600         -0.580           -33.0
 *      -0.360         -0.340           -23.0
 *      -0.175         -0.155           -13.1
 *      -0.060         -0.040            -4.9
 *      -0.020          0.000             0.0
 *       0.035          0.055             5.0
 *       0.148          0.168            13.2
 *       0.346          0.366            23.2
 *       0.603          0.623            33.4
 *       0.935          0.955            44.2
 *       1.360          1.380            55.2
 *       1.860          1.880            67.3
 *       2.430          2.450            77.0
 *       3.055          3.075            88.0
 *       3.750          3.770            98.5
 *       4.600          4.620           109.0
 *       5.320          5.340           120.0
 *
 *  LUT reproduces measured points exactly; linear interpolation between,
 *  end-segment slope beyond table limits.
 */
typedef struct
{
    float dp_mbar;     /* ΔP after zero-offset correction */
    float q_slm;
} FlowCalPoint;

static const FlowCalPoint kFlowCal[] =
{
    { -4.720f, -121.0f },
    { -3.990f, -110.0f },
    { -3.330f,  -99.0f },
    { -2.740f,  -88.0f },
    { -2.200f,  -78.0f },
    { -1.720f,  -67.0f },
    { -1.260f,  -54.8f },
    { -0.890f,  -43.8f },
    { -0.580f,  -33.0f },
    { -0.340f,  -23.0f },
    { -0.155f,  -13.1f },
    { -0.040f,   -4.9f },
    {  0.000f,    0.0f },
    {  0.055f,    5.0f },
    {  0.168f,   13.2f },
    {  0.366f,   23.2f },
    {  0.623f,   33.4f },
    {  0.955f,   44.2f },
    {  1.380f,   55.2f },
    {  1.880f,   67.3f },
    {  2.450f,   77.0f },
    {  3.075f,   88.0f },
    {  3.770f,   98.5f },
    {  4.620f,  109.0f },
    {  5.340f,  120.0f },
};

#define FLOW_CAL_N   (sizeof(kFlowCal) / sizeof(kFlowCal[0]))

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float Flow_DpToSlm(float dp_corrected_mbar)
{
    const FlowCalPoint *first = &kFlowCal[0];
    const FlowCalPoint *next  = &kFlowCal[1];
    const FlowCalPoint *last  = &kFlowCal[FLOW_CAL_N - 1U];
    const FlowCalPoint *prev  = &kFlowCal[FLOW_CAL_N - 2U];

    if (dp_corrected_mbar <= first->dp_mbar)
    {
        const float slope = (next->q_slm - first->q_slm)
                          / (next->dp_mbar - first->dp_mbar);
        return first->q_slm + slope * (dp_corrected_mbar - first->dp_mbar);
    }

    if (dp_corrected_mbar >= last->dp_mbar)
    {
        const float slope = (last->q_slm - prev->q_slm)
                          / (last->dp_mbar - prev->dp_mbar);
        return last->q_slm + slope * (dp_corrected_mbar - last->dp_mbar);
    }

    /* Inside the table - linear scan is fine for 25 entries. */
    for (size_t i = 0U; i < (FLOW_CAL_N - 1U); ++i)
    {
        const FlowCalPoint *a = &kFlowCal[i];
        const FlowCalPoint *b = &kFlowCal[i + 1U];
        if (dp_corrected_mbar <= b->dp_mbar)
        {
            const float frac = (dp_corrected_mbar - a->dp_mbar)
                             / (b->dp_mbar - a->dp_mbar);
            return a->q_slm + frac * (b->q_slm - a->q_slm);
        }
    }

    /* Should be unreachable given the bounds above, but be defensive. */
    return last->q_slm;
}

/* === Public API === */

void Flow_Init(void)
{
    g_flow_lpf_cutoff_hz    = 10.0f;
    g_flow_zero_offset_mbar = -0.02f;
    g_flow_dp_raw_mbar      = 0.0f;
    g_flow_dp_filt_mbar     = 0.0f;
    g_flow_dp_corr_mbar     = 0.0f;
    g_flow_q_slm            = 0.0f;
    g_flow_q_slm_raw        = 0.0f;
    g_flow_lpf_alpha        = 0.0f;
    g_flow_step_count       = 0U;
    s_filt_seeded           = false;
}

void Flow_SetLpfCutoff(float hz)
{
    g_flow_lpf_cutoff_hz = clampf(hz,
                                  FLOW_LPF_CUTOFF_MIN_HZ,
                                  FLOW_LPF_CUTOFF_MAX_HZ);
}

float Flow_GetLpfCutoff(void)       { return g_flow_lpf_cutoff_hz; }
float Flow_GetDpRaw(void)           { return g_flow_dp_raw_mbar; }
float Flow_GetDpFiltered(void)      { return g_flow_dp_filt_mbar; }
float Flow_GetDpFilteredCorr(void)  { return g_flow_dp_corr_mbar; }
float Flow_GetSlm(void)             { return g_flow_q_slm; }
float Flow_GetSlmRaw(void)          { return g_flow_q_slm_raw; }
float Flow_GetPatientSlm(void)      { return Leak_GetPatientSlm(); }
float Flow_GetPatientRawSlm(void)   { return g_flow_patient_raw_slm; }
float Flow_GetZeroOffset(void)      { return g_flow_zero_offset_mbar; }

void Flow_SetZeroOffset(float mbar)
{
    /* Cal sheet zero is ~-0.02 mbar; clamp to a generous ±0.5 mbar
     * window so a typo in the watch window cannot break the flow chain. */
    g_flow_zero_offset_mbar = clampf(mbar, -0.5f, 0.5f);
}

void Flow_AutoZero(void)
{
    /* Snap the offset to whatever the LPF currently reads. Caller is
     * responsible for ensuring the blower is off / flow is zero when
     * this is invoked - we have no way to verify that here. */
    Flow_SetZeroOffset(g_flow_dp_filt_mbar);
}

void Flow_Update(float dp_mbar_raw, float dt_s)
{
    /* Same dt sanity guard pattern as the rest of the control code. */
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    g_flow_dp_raw_mbar = dp_mbar_raw;

    /* Compute alpha from the *currently configured* cutoff every tick.
     * That's only ~5 float ops; doing it every call lets the user retune
     * fc from the watch window without an explicit "apply" step. */
    const float fc = clampf(g_flow_lpf_cutoff_hz,
                            FLOW_LPF_CUTOFF_MIN_HZ,
                            FLOW_LPF_CUTOFF_MAX_HZ);
    const float omega_dt = FLOW_TWO_PI * fc * dt_s;
    const float alpha    = omega_dt / (1.0f + omega_dt);
    g_flow_lpf_alpha = alpha;

    /* First-sample seed: rather than ramping from 0 (which can take ~50 ms
     * to settle and momentarily looks like a transient), snap the
     * filter state to the first valid input. */
    if (!s_filt_seeded)
    {
        g_flow_dp_filt_mbar = dp_mbar_raw;
        s_filt_seeded       = true;
    }
    else
    {
        g_flow_dp_filt_mbar = alpha * dp_mbar_raw
                            + (1.0f - alpha) * g_flow_dp_filt_mbar;
    }

    /* Apply zero-offset correction and calibration LUT. We compute Q
     * for both the filtered and the raw signals: filtered is what the
     * therapy/diagnostics layers should consume; raw is exposed for
     * future band-pass / FOT processing. */
    const float zero  = g_flow_zero_offset_mbar;
    g_flow_dp_corr_mbar = g_flow_dp_filt_mbar - zero;
    g_flow_q_slm        = Flow_DpToSlm(g_flow_dp_corr_mbar);
    g_flow_q_slm_raw    = Flow_DpToSlm(dp_mbar_raw - zero);

    g_flow_step_count++;
}
