/**
  * @file    breath_sim.c
  * @brief   Open-loop RPM waveform generator for breath simulation.
  */

#include "breath_sim.h"
#include "blower_ipc.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BREATH_RATE_MIN_BPM       4.0f
#define BREATH_RATE_MAX_BPM      60.0f
#define BREATH_INSP_MIN_S         0.3f
#define BREATH_INSP_MAX_S         4.0f
#define BREATH_IE_RATIO_MIN       1.0f
#define BREATH_IE_RATIO_MAX       4.0f
#define BREATH_PAUSE_MAX_S        1.0f
#define BREATH_EXP_PAUSE_MAX_S    2.0f
#define BREATH_RPM_BASE_MIN         2000
#define BREATH_RPM_BASE_DEFAULT     5000
#define BREATH_RPM_AMP_MAX        25000
#define BREATH_RPM_BASE_MAX       20000

static BreathSimParams_t g_params;
static bool     g_running;
static float    g_phase;
static int32_t  g_target_rpm;
static float    g_envelope;
static float    g_cycle_period_s;
static uint8_t  g_motor_started;

/* Over-voltage mitigation: when running a hard square wave with a large
 * amplitude step, the decel edge can pump Vbus and trip MC_OVER_VOLT.
 * We apply a down-slew limit only in that high-risk case. */
#define BREATH_SQUARE_FALL_SLEW_THRESHOLD_RPM   12000
#define BREATH_SQUARE_FALL_SLEW_RPM_PER_S       25000.0f

static float clampf(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

static int32_t BreathSim_ApplyFallSlew(int32_t desired_rpm, float dt_s)
{
  /* Only slow the falling edge for square wave with large amplitude. */
  const bool high_risk_square =
      (g_params.waveform == (uint8_t)BREATH_WAVE_SQUARE) &&
      (g_params.rpm_amplitude > BREATH_SQUARE_FALL_SLEW_THRESHOLD_RPM);

  if (!high_risk_square)
  {
    return desired_rpm;
  }

  if (dt_s <= 0.0f)
  {
    return desired_rpm;
  }

  if (desired_rpm >= g_target_rpm)
  {
    /* Rising edge: do not limit (keep response crisp). */
    return desired_rpm;
  }

  const float max_drop_f = BREATH_SQUARE_FALL_SLEW_RPM_PER_S * dt_s;
  const int32_t max_drop = (max_drop_f > 0.0f) ? (int32_t)lroundf(max_drop_f) : 0;
  const int32_t min_allowed = g_target_rpm - max_drop;
  return (desired_rpm < min_allowed) ? min_allowed : desired_rpm;
}

static void BreathSim_NormalizeParams(BreathSimParams_t *p)
{
  p->rate_bpm = clampf(p->rate_bpm, BREATH_RATE_MIN_BPM, BREATH_RATE_MAX_BPM);
  p->insp_time_s = clampf(p->insp_time_s, BREATH_INSP_MIN_S, BREATH_INSP_MAX_S);
  p->ie_ratio_exp = clampf(p->ie_ratio_exp, BREATH_IE_RATIO_MIN, BREATH_IE_RATIO_MAX);
  p->insp_pause_s = clampf(p->insp_pause_s, 0.0f, BREATH_PAUSE_MAX_S);
  p->exp_pause_s = clampf(p->exp_pause_s, 0.0f, BREATH_EXP_PAUSE_MAX_S);
  p->rpm_base = clamp_i32(p->rpm_base, BREATH_RPM_BASE_MIN, BREATH_RPM_BASE_MAX);
  p->rpm_amplitude = clamp_i32(p->rpm_amplitude, 0, BREATH_RPM_AMP_MAX);
  if (p->waveform >= BREATH_WAVE_COUNT)
  {
    p->waveform = (uint8_t)BREATH_WAVE_SINE;
  }

  g_cycle_period_s = 60.0f / p->rate_bpm;
  float max_exp = g_cycle_period_s - p->insp_time_s - p->insp_pause_s - 0.1f;
  if (max_exp < 0.0f)
  {
    max_exp = 0.0f;
  }
  if (p->exp_pause_s > max_exp)
  {
    p->exp_pause_s = max_exp;
  }

  const float max_insp = g_cycle_period_s - p->insp_pause_s - p->exp_pause_s - 0.1f;
  if (p->insp_time_s > max_insp)
  {
    p->insp_time_s = (max_insp > BREATH_INSP_MIN_S) ? max_insp : BREATH_INSP_MIN_S;
  }
}

static float BreathSim_ComputeEnvelope(float phase, const BreathSimParams_t *p)
{
  const float insp_frac = p->insp_time_s / g_cycle_period_s;
  const float insp_pause_frac = p->insp_pause_s / g_cycle_period_s;
  const float exp_time_s = p->insp_time_s * p->ie_ratio_exp;
  const float exp_frac = exp_time_s / g_cycle_period_s;
  const float exp_pause_frac = p->exp_pause_s / g_cycle_period_s;
  const float insp_end = insp_frac;
  const float insp_pause_end = insp_end + insp_pause_frac;
  const float exp_end = 1.0f - exp_pause_frac;

  if (phase < insp_end)
  {
    const float t = (insp_frac > 0.0f) ? (phase / insp_frac) : 0.0f;
    switch ((BreathWaveform_t)p->waveform)
    {
      case BREATH_WAVE_RAMP:
        return t;
      case BREATH_WAVE_SQUARE:
        return 1.0f;
      case BREATH_WAVE_SINE:
      default:
        return sinf(t * (float)M_PI * 0.5f);
    }
  }

  if (phase < insp_pause_end)
  {
    return 1.0f;
  }

  if (phase < exp_end)
  {
    const float exp_span = exp_end - insp_pause_end;
    const float t = (exp_span > 0.0f) ? ((phase - insp_pause_end) / exp_span) : 1.0f;
    switch ((BreathWaveform_t)p->waveform)
    {
      case BREATH_WAVE_RAMP:
        return 1.0f - t;
      case BREATH_WAVE_SQUARE:
        return 0.0f;
      case BREATH_WAVE_SINE:
      default:
        return cosf(t * (float)M_PI * 0.5f);
    }
  }

  return 0.0f;
}

void BreathSim_Init(void)
{
  memset(&g_params, 0, sizeof(g_params));
  g_params.rate_bpm = 12.0f;
  g_params.insp_time_s = 1.5f;
  g_params.ie_ratio_exp = 2.0f;
  g_params.rpm_base = BREATH_RPM_BASE_DEFAULT;
  g_params.rpm_amplitude = 4000;
  g_params.waveform = (uint8_t)BREATH_WAVE_SINE;
  g_params.insp_pause_s = 0.0f;
  g_params.exp_pause_s = 0.0f;
  BreathSim_NormalizeParams(&g_params);

  g_running = false;
  g_phase = 0.0f;
  g_target_rpm = 0;
  g_envelope = 0.0f;
  g_motor_started = 0U;
}

void BreathSim_GetParams(BreathSimParams_t *out)
{
  if (out == NULL)
  {
    return;
  }
  *out = g_params;
}

void BreathSim_SetParams(const BreathSimParams_t *params)
{
  if (params == NULL)
  {
    return;
  }
  g_params = *params;
  BreathSim_NormalizeParams(&g_params);
}

#define BLOWER_MC_STATE_FAULT_OVER  11U

static void BreathSim_AckFaultIfLatched(void)
{
  BlowerIpcStatus_t st;

  if (!BlowerIpc_CM7_GetStatus(&st))
  {
    return;
  }

  if ((st.mc_state == BLOWER_MC_STATE_FAULT_OVER) && (st.current_faults == 0U))
  {
    (void)BlowerIpc_CM7_AcknowledgeFault();
  }
}

static void BreathSim_EnsureMotorSpinning(void)
{
  if (g_motor_started == 0U)
  {
    (void)BlowerIpc_CM7_SetSpeedRpmSilent(g_target_rpm);
    BreathSim_AckFaultIfLatched();
    (void)BlowerIpc_CM7_Start();
    g_motor_started = 1U;
  }
}

void BreathSim_Start(void)
{
  g_running = true;
  g_phase = 0.0f;
  g_envelope = 0.0f;
  g_target_rpm = g_params.rpm_base;
  BreathSim_EnsureMotorSpinning();
}

void BreathSim_Stop(void)
{
  g_running = false;
  g_target_rpm = g_params.rpm_base;
  g_envelope = 0.0f;
  if (g_motor_started != 0U)
  {
    (void)BlowerIpc_CM7_SetSpeedRpmSilent(g_target_rpm);
    (void)BlowerIpc_CM7_Stop();
    g_motor_started = 0U;
  }
}

bool BreathSim_IsRunning(void)
{
  return g_running;
}

void BreathSim_Update(float dt_s)
{
  if (!g_running)
  {
    return;
  }

  if (g_motor_started == 0U)
  {
    BreathSim_EnsureMotorSpinning();
  }

  if (g_cycle_period_s <= 0.0f)
  {
    BreathSim_NormalizeParams(&g_params);
  }

  g_phase += dt_s / g_cycle_period_s;
  if (g_phase >= 1.0f)
  {
    g_phase -= floorf(g_phase);
  }

  g_envelope = BreathSim_ComputeEnvelope(g_phase, &g_params);
  const int32_t desired_rpm = g_params.rpm_base +
                              (int32_t)lroundf((float)g_params.rpm_amplitude * g_envelope);
  g_target_rpm = BreathSim_ApplyFallSlew(desired_rpm, dt_s);

  (void)BlowerIpc_CM7_SetSpeedRpmSilent(g_target_rpm);
}

int32_t BreathSim_GetTargetRpm(void)
{
  return g_target_rpm;
}

float BreathSim_GetPhase(void)
{
  return g_phase;
}

float BreathSim_GetCyclePeriodS(void)
{
  return g_cycle_period_s;
}

float BreathSim_GetEnvelope(void)
{
  return g_envelope;
}

const char *BreathSim_WaveformName(uint8_t waveform)
{
  switch ((BreathWaveform_t)waveform)
  {
    case BREATH_WAVE_RAMP:   return "Ramp";
    case BREATH_WAVE_SQUARE: return "Square";
    case BREATH_WAVE_SINE:
    default:                 return "Sine";
  }
}
