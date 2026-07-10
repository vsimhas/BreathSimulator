/**
  ******************************************************************************
  * @file    blower_osc.c
  * @brief   Sinusoidal pressure-setpoint perturbation generator (FOT).
  *
  *  See blower_osc.h for the design rationale. This file is implementation
  *  detail: a phase-accumulator DDS in [0, 1) driving sinf().
  ******************************************************************************
  */

#include "blower_osc.h"

#include <math.h>

#define BLOWER_OSC_TWO_PI   6.28318530717958647692f

/* === Public configuration (debugger-visible volatiles) === */
volatile uint8_t  g_blower_osc_enabled        = 0U;       /* default OFF */
volatile float    g_blower_osc_amplitude_cmh  = 1.0f;     /* peak cmH2O */
volatile float    g_blower_osc_freq_hz        = 4.0f;     /* default 4 Hz */

/* === State (debugger-visible volatiles) === */
volatile float    g_blower_osc_phase          = 0.0f;     /* [0, 1) */
volatile float    g_blower_osc_sample         = 0.0f;     /* last output */
volatile uint32_t g_blower_osc_step_count     = 0U;

/* === Internal === */
static uint8_t s_prev_enabled = 0U;          /* edge detection */

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* === Public API === */

void BlowerOsc_Init(void)
{
    g_blower_osc_enabled       = 0U;
    g_blower_osc_amplitude_cmh = 1.0f;
    g_blower_osc_freq_hz       = 4.0f;
    g_blower_osc_phase         = 0.0f;
    g_blower_osc_sample        = 0.0f;
    g_blower_osc_step_count    = 0U;
    s_prev_enabled             = 0U;
}

void BlowerOsc_SetEnable(bool en)
{
    g_blower_osc_enabled = en ? 1U : 0U;
    /* Edge handling happens inside StepAndGetSample so the phase reset
     * is synchronous with the loop tick that first sees the new state. */
}

bool BlowerOsc_IsEnabled(void)
{
    return (g_blower_osc_enabled != 0U);
}

void BlowerOsc_SetAmplitude(float cmh2o_peak)
{
    g_blower_osc_amplitude_cmh = clampf(cmh2o_peak,
                                        BLOWER_OSC_AMPL_MIN_CMH2O,
                                        BLOWER_OSC_AMPL_MAX_CMH2O);
}

void BlowerOsc_SetFrequency(float hz)
{
    g_blower_osc_freq_hz = clampf(hz,
                                  BLOWER_OSC_FREQ_MIN_HZ,
                                  BLOWER_OSC_FREQ_MAX_HZ);
}

float BlowerOsc_GetAmplitude(void)  { return g_blower_osc_amplitude_cmh; }
float BlowerOsc_GetFrequency(void)  { return g_blower_osc_freq_hz; }
float BlowerOsc_GetLastSample(void) { return g_blower_osc_sample; }

float BlowerOsc_StepAndGetSample(float dt_s)
{
    /* Refuse silly dt - same guard as in BlowerCtrl_Step. */
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return g_blower_osc_sample;
    }

    /* Detect rising edge: snap phase to a zero-crossing so the first
     * sample emitted is exactly 0 - no setpoint discontinuity. */
    const uint8_t en_now = g_blower_osc_enabled;
    if (en_now != 0U && s_prev_enabled == 0U)
    {
        g_blower_osc_phase = 0.0f;
    }
    s_prev_enabled = en_now;

    if (en_now == 0U)
    {
        /* Drop output to 0 instantly when disabled. The PID's output
         * low-pass filter absorbs the resulting sub-millisecond step. */
        g_blower_osc_sample = 0.0f;
        return 0.0f;
    }

    /* Pull current freq/amplitude with a one-shot clamp so a debugger
     * write that violates the safe range cannot escape. */
    const float freq = clampf(g_blower_osc_freq_hz,
                              BLOWER_OSC_FREQ_MIN_HZ,
                              BLOWER_OSC_FREQ_MAX_HZ);
    const float ampl = clampf(g_blower_osc_amplitude_cmh,
                              BLOWER_OSC_AMPL_MIN_CMH2O,
                              BLOWER_OSC_AMPL_MAX_CMH2O);

    /* Advance phase. dt_s is the elapsed seconds since the previous
     * call; phase increment = freq * dt cycles. Wrap into [0, 1). */
    float phase = g_blower_osc_phase + freq * dt_s;
    while (phase >= 1.0f) phase -= 1.0f;
    while (phase <  0.0f) phase += 1.0f;
    g_blower_osc_phase = phase;

    /* Compute sample. sinf() on the M7 with FPU is ~50 cycles - well
     * within the 25k-cycle budget we have at 200 Hz on a 400 MHz core. */
    const float sample = ampl * sinf(BLOWER_OSC_TWO_PI * phase);
    g_blower_osc_sample = sample;
    g_blower_osc_step_count++;

    return sample;
}
