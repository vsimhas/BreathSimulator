/**
  * @file    breath_sim.c
  * @brief   Open-loop RPM waveform generator for breath simulation.
  *
  * See breath_sim.h for the pneumatic model this implements. In short: the
  * blower is the inspiratory muscle, expiration is passive lung recoil, and
  * the envelope computed here is an inspiratory *flow* demand.
  */

#include "breath_sim.h"
#include "blower_ipc.h"
#include "dbg_log.h"
#include "stm32h7xx_hal.h"   /* HAL_GetTick for sensor-sample ageing */

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ST MC SDK MCI_State_t values we care about (CM4/Core/Inc/mc_interface.h). */
#define MC_STATE_RUN                 6U
#define MC_STATE_FAULT_NOW          10U
#define MC_STATE_FAULT_OVER         11U

/* ---------------------------------------------------------------------------
 * Slew limiting / DC-bus protection
 *
 * The blower is an inertial load with no dissipative path other than the CM4
 * regen brake, so a fast decel pumps charge back into Vbus. OV trips at
 * 24.9 V and the CM4 brake engages at 24.0 V — a 0.9 V margin.
 *
 * The previous implementation limited the falling edge only for SQUARE with
 * amplitude > 12000, which left the genuinely worst cases unprotected: a
 * half-sine over a 0.3 s inspiration reaches ~5.2 1/s envelope slope, i.e.
 * ~131000 RPM/s at 25000 amplitude. The limit is now universal and is
 * additionally derated from the measured bus voltage, so it cooperates with
 * the CM4 brake instead of guessing independently of it.
 * ------------------------------------------------------------------------ */
#define BREATH_SLEW_RISE_RPM_PER_S      150000.0f
#define BREATH_SLEW_FALL_RPM_PER_S       25000.0f
#define BREATH_SLEW_FALL_MIN_RPM_PER_S    5000.0f
#define BREATH_VBUS_DERATE_START_V          23U
#define BREATH_VBUS_DERATE_FULL_V           24U

/* ---------------------------------------------------------------------------
 * Motor supervision
 * ------------------------------------------------------------------------ */
/** CM4 open-loop rev-up is PHASE1+PHASE2 = ~4.65 s; allow generous margin. */
#define BREATH_PRIME_TIMEOUT_S             12.0f
/** Consecutive time the speed must sit near baseline before breaths start. */
#define BREATH_PRIME_SETTLE_S               0.20f
/**
  * Fault checks are suppressed for this long after Start. A fault
  * acknowledge posted by Start() takes a few IPC round trips to clear on
  * CM4, so the mailbox can still read FAULT_OVER on the next tick; without
  * the grace period the run would abort on the fault it just cleared.
  */
#define BREATH_FAULT_GRACE_S                1.0f
/** Tracking error tolerance: the larger of these two. */
#define BREATH_TRACK_TOL_RPM                1500
#define BREATH_TRACK_TOL_FRAC               0.15f
/** How long a tracking error must persist before it is flagged. */
#define BREATH_TRACK_HOLD_S                 0.50f

/** Tolerance for float round-off when segment times are summed. */
#define BREATH_TIME_EPS_S                   1.0e-4f

/** Cheyne-Stokes modulation cycle and the amplitude below which it apneas. */
#define BREATH_CSR_CYCLE_S                 60.0f
#define BREATH_CSR_APNEA_LEVEL              0.15f

/* ===========================================================================
 * Module state
 * ======================================================================== */

/* Owned by the breath task; only ever written at a breath boundary. */
static BreathSimParams_t g_params;
static BreathSimTiming_t g_timing;

/* Staged by GUI / USB writers, latched by the breath task at a breath
 * boundary. This is what makes parameter updates both race-free (writers run
 * at lower priority than the breath task) and glitch-free (a change never
 * warps the breath already in progress). */
static BreathSimParams_t g_params_pending;
static BreathSimTiming_t g_timing_pending;
static volatile uint8_t  g_params_dirty;

/* Per-breath resolved timing, after jitter. */
static BreathSimTiming_t g_active;

static BreathSimState_t g_state;
static float    g_t_s;              /* time within the current breath */
static float    g_prime_t_s;
static float    g_prime_settle_s;
static uint32_t g_breath_index;
static float    g_envelope;         /* delivered demand, after amplitude scaling */
static float    g_env_raw;          /* shape only, before event/jitter scaling */
static float    g_env_exp_start;    /* raw envelope at the moment expiration began */
static int32_t  g_target_rpm;
static int32_t  g_rpm_act;
static uint16_t g_bus_voltage_v;
static uint16_t g_mc_state;
static uint16_t g_flags;
static BreathSegment_t g_segment;
static int32_t  g_last_posted_rpm;
static uint8_t  g_have_posted;
static float    g_track_err_s;

/* Per-breath modifiers, latched at each breath boundary. */
static float    g_amp_scale;
static float    g_flat_eff;

/* Events */
static uint8_t  g_event;
static float    g_event_severity;
static float    g_event_remaining_s;
static float    g_csr_t_s;

/* Script */
static BreathScriptStep_t g_script[BREATH_SCRIPT_MAX_STEPS];
static uint8_t  g_script_len;
static uint8_t  g_script_idx;
static uint8_t  g_script_running;
static uint8_t  g_script_loop;
static float    g_script_remaining_s;

/* Arbitrary waveform table */
static float    g_table[BREATH_TABLE_POINTS];
static uint8_t  g_table_len;

/* PRNG for breath-to-breath variability. */
static uint32_t g_rng;

/* ---------------------------------------------------------------------------
 * Closed-loop flow control state
 * ------------------------------------------------------------------------ */
/* Latest sensor inputs, pushed by SensorTask at 100 Hz. The breath loop runs
 * at 200 Hz and holds the most recent sample; that is a zero-order hold at
 * half the loop rate, which is untroubling here because the fastest feature
 * in a breath is ~0.3 s and the blower's own response is far slower than
 * either rate. */
static float    g_q_meas_lpm;
static uint8_t  g_q_valid;
static uint32_t g_q_stamp_ms;
static float    g_p_cmh2o;
static uint8_t  g_p_valid;

static float    g_q_target_lpm;
static float    g_q_peak_lpm;      /* peak this breath needs for tidal_ml */
static float    g_shape_area;      /* mean/peak of the inspiratory shape */
static float    g_ctrl_integ_rpm;  /* PI integrator, RPM */
static float    g_volume_ml;       /* inspired volume so far this breath */
static float    g_last_tidal_ml;
/* RPM as inspiration ended; the passive decay starts from here so
 * exp_tau_s means the same thing in both control modes. */
static int32_t  g_rpm_insp_end;

/* Airway-pressure statistics accumulated over the breath in progress. */
static float    g_p_min_acc;
static float    g_p_max_acc;
static float    g_p_sum;
static uint32_t g_p_count;
static float    g_p_min_last;
static float    g_p_max_last;
static float    g_p_mean_last;

/* Coherent snapshot published for other tasks. */
static BreathSimStatus_t g_status;

/* ===========================================================================
 * Helpers
 * ======================================================================== */

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

static uint32_t rng_next(void)
{
  /* xorshift32 — deterministic, so a fixed seed reproduces a run exactly. */
  uint32_t x = g_rng;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  g_rng = x;
  return x;
}

/** Uniform in [-1, +1]. */
static float rng_bipolar(void)
{
  return ((float)(rng_next() >> 8) / 8388607.5f) - 1.0f;
}

/* ===========================================================================
 * Sensor inputs
 * ======================================================================== */

void BreathSim_SetSensorInputs(float q_lpm, uint8_t q_valid,
                               float p_cmh2o, uint8_t p_valid)
{
  taskENTER_CRITICAL();
  if (q_valid != 0U)
  {
    g_q_meas_lpm = q_lpm;
    g_q_stamp_ms = HAL_GetTick();
  }
  g_q_valid = q_valid;

  if (p_valid != 0U)
  {
    g_p_cmh2o = p_cmh2o;
  }
  g_p_valid = p_valid;
  taskEXIT_CRITICAL();
}

/** 1 while the last flow sample is recent enough to close a loop around. */
static uint8_t BreathSim_FlowUsable(void)
{
  if (g_q_valid == 0U) { return 0U; }
  return ((HAL_GetTick() - g_q_stamp_ms) <= BREATH_FLOW_STALE_MS) ? 1U : 0U;
}

bool BreathSim_FlowSensorReady(void)
{
  return (BreathSim_FlowUsable() != 0U);
}

/* ===========================================================================
 * Parameter resolution
 *
 * Rate, inspiratory time and I:E are three knobs on two degrees of freedom.
 * The timing mode says which two are authoritative; the third is derived and
 * written back so the caller can read what will actually run. Nothing is
 * silently coerced — any adjustment makes this return false.
 * ======================================================================== */

static bool BreathSim_Resolve(BreathSimParams_t *p, BreathSimTiming_t *t)
{
  bool ok = true;

  /* --- scalar clamps ------------------------------------------------- */
  const float rate_in  = p->rate_bpm;
  const float insp_in  = p->insp_time_s;
  const float ie_in    = p->ie_ratio_exp;
  const float ip_in    = p->insp_pause_s;
  const float ep_in    = p->exp_pause_s;

  p->rate_bpm     = clampf(p->rate_bpm, BREATH_RATE_MIN_BPM, BREATH_RATE_MAX_BPM);
  p->insp_time_s  = clampf(p->insp_time_s, BREATH_INSP_MIN_S, BREATH_INSP_MAX_S);
  p->ie_ratio_exp = clampf(p->ie_ratio_exp, BREATH_IE_RATIO_MIN, BREATH_IE_RATIO_MAX);
  p->insp_pause_s = clampf(p->insp_pause_s, 0.0f, BREATH_INSP_PAUSE_MAX_S);
  p->exp_pause_s  = clampf(p->exp_pause_s, 0.0f, BREATH_EXP_PAUSE_MAX_S);
  p->exp_tau_s    = clampf(p->exp_tau_s, BREATH_EXP_TAU_MIN_S, BREATH_EXP_TAU_MAX_S);
  p->flattening   = clampf(p->flattening, 0.0f, BREATH_FLATTENING_MAX);
  p->jitter_pct   = clampf(p->jitter_pct, 0.0f, BREATH_JITTER_MAX_PCT);

  p->tidal_ml     = clampf(p->tidal_ml, BREATH_TIDAL_MIN_ML, BREATH_TIDAL_MAX_ML);
  p->flow_kp      = clampf(p->flow_kp, 0.0f, BREATH_FLOW_KP_MAX);
  p->flow_ki      = clampf(p->flow_ki, 0.0f, BREATH_FLOW_KI_MAX);
  p->flow_ff_rpm_per_lpm = clampf(p->flow_ff_rpm_per_lpm, 0.0f, BREATH_FLOW_FF_MAX);
  p->pressure_limit_cmh2o =
      clampf(p->pressure_limit_cmh2o, 0.0f, BREATH_PRESSURE_LIMIT_MAX_CMH2O);
  if (p->control_mode >= (uint8_t)BREATH_CTRL_COUNT)
  {
    p->control_mode = (uint8_t)BREATH_CTRL_OPEN_LOOP_RPM;
    ok = false;
  }

  p->rpm_base      = clamp_i32(p->rpm_base, BREATH_RPM_BASE_MIN, BREATH_RPM_BASE_MAX);
  p->rpm_amplitude = clamp_i32(p->rpm_amplitude, 0, BREATH_RPM_AMP_MAX);
  if ((p->rpm_base + p->rpm_amplitude) > BREATH_RPM_ABS_MAX)
  {
    p->rpm_amplitude = BREATH_RPM_ABS_MAX - p->rpm_base;
    ok = false;
  }

  if (p->waveform >= (uint8_t)BREATH_WAVE_COUNT)     { p->waveform = (uint8_t)BREATH_WAVE_SINE; ok = false; }
  if (p->timing_mode >= (uint8_t)BREATH_TIMING_COUNT) { p->timing_mode = (uint8_t)BREATH_TIMING_RATE_IE; ok = false; }
  if ((p->waveform == (uint8_t)BREATH_WAVE_TABLE) && (g_table_len < 2U))
  {
    /* No table loaded — fall back rather than emit a flat-zero breath. */
    p->waveform = (uint8_t)BREATH_WAVE_SINE;
    ok = false;
  }

  if ((rate_in != p->rate_bpm) || (insp_in != p->insp_time_s) ||
      (ie_in != p->ie_ratio_exp) || (ip_in != p->insp_pause_s) ||
      (ep_in != p->exp_pause_s))
  {
    ok = false;
  }

  /* --- timing resolution --------------------------------------------- */
  float period, insp, expt;
  float ipause = p->insp_pause_s;
  float epause = p->exp_pause_s;
  const float ie = p->ie_ratio_exp;

  if (p->timing_mode == (uint8_t)BREATH_TIMING_RATE_IE)
  {
    period = 60.0f / p->rate_bpm;

    /* Pauses may not consume so much of the cycle that no breath fits. */
    float active = period - ipause - epause;
    const float min_active = BREATH_INSP_MIN_S * (1.0f + ie);
    if (active < min_active)
    {
      const float excess = min_active - active;
      /* Give back from the expiratory pause first, then the inspiratory. */
      const float from_ep = (epause < excess) ? epause : excess;
      epause -= from_ep;
      float still = excess - from_ep;
      const float from_ip = (ipause < still) ? ipause : still;
      ipause -= from_ip;
      active = period - ipause - epause;
      ok = false;
    }

    insp = active / (1.0f + ie);

    /* Physiologic sanity: inspiration may not swallow the cycle. */
    const float max_insp = period * BREATH_MAX_INSP_DUTY;
    if (insp > max_insp) { insp = max_insp; ok = false; }
    if (insp > BREATH_INSP_MAX_S) { insp = BREATH_INSP_MAX_S; ok = false; }
    if (insp < BREATH_INSP_MIN_S) { insp = BREATH_INSP_MIN_S; ok = false; }

    expt = insp * ie;

    /* Any time the caps left over becomes end-expiratory pause, so the
     * segments always sum exactly to the period and the I:E stays exact.
     * The epsilon matters: insp + insp*ie against 60/rate lands a few ULP
     * negative for exactly-fitting inputs (12 bpm, I:E 1:2 among them), and
     * without it every such call would report a coercion that never
     * happened - which would train the operator to ignore the warning. */
    float leftover = period - insp - ipause - expt - epause;
    if (leftover < -BREATH_TIME_EPS_S)
    {
      /* Can only happen if insp was floored at BREATH_INSP_MIN_S. */
      expt += leftover;
      if (expt < 0.0f) { expt = 0.0f; }
      leftover = period - insp - ipause - expt - epause;
      if (leftover < 0.0f) { epause += leftover; leftover = 0.0f; }
      if (epause < 0.0f) { epause = 0.0f; }
      ok = false;
    }
    if (leftover < 0.0f) { leftover = 0.0f; }
    epause += leftover;

    p->insp_time_s = insp;   /* derived readback */
  }
  else /* BREATH_TIMING_EXPLICIT */
  {
    insp   = p->insp_time_s;
    expt   = insp * ie;
    period = insp + ipause + expt + epause;

    const float min_period = 60.0f / BREATH_RATE_MAX_BPM;
    const float max_period = 60.0f / BREATH_RATE_MIN_BPM;
    if (period < (min_period - BREATH_TIME_EPS_S))
    {
      epause += (min_period - period);
      period = min_period;
      ok = false;
    }
    else if (period > (max_period + BREATH_TIME_EPS_S))
    {
      /* Scale the whole breath down to fit the slowest permitted rate. */
      const float k = max_period / period;
      insp   *= k;
      expt   *= k;
      ipause *= k;
      epause *= k;
      period  = max_period;
      p->insp_time_s = insp;
      ok = false;
    }

    p->rate_bpm = 60.0f / period;   /* derived readback */
  }

  p->insp_pause_s = ipause;
  p->exp_pause_s  = epause;

  t->period_s     = period;
  t->insp_s       = insp;
  t->insp_pause_s = ipause;
  t->exp_s        = expt;
  t->exp_pause_s  = epause;
  t->rate_bpm     = 60.0f / period;

  return ok;
}

/* ===========================================================================
 * Waveform shaping
 * ======================================================================== */

static float BreathSim_TableInterp(float u)
{
  if (g_table_len < 2U) { return 0.0f; }

  const float x = u * (float)(g_table_len - 1U);
  int32_t i = (int32_t)x;
  if (i < 0) { return g_table[0]; }
  if (i >= (int32_t)(g_table_len - 1U)) { return g_table[g_table_len - 1U]; }

  const float f = x - (float)i;
  return g_table[i] + ((g_table[i + 1] - g_table[i]) * f);
}

/**
  * @brief Morph a normal inspiratory flow shape toward a flow-limited one.
  *
  * Flow limitation presents as a flattened / plateaued inspiratory limb —
  * exactly what an APAP's flattening index looks for. Scaling the shape up
  * and clipping at unity widens the top and steepens the flanks while
  * keeping peak flow normalised, which is the characteristic appearance.
  */
static float BreathSim_ApplyFlattening(float e, float flat)
{
  if (flat <= 0.0f) { return e; }
  const float k = 1.0f - (0.85f * flat);   /* 1.00 .. 0.15 */
  const float v = e / k;
  return (v > 1.0f) ? 1.0f : v;
}

/** Unflattened normalised inspiratory flow demand for u in [0,1]. */
static float BreathSim_BaseShape(float u, const BreathSimParams_t *p)
{
  float e;

  switch ((BreathWaveform_t)p->waveform)
  {
    case BREATH_WAVE_RAMP:
      /* Symmetric triangular flow. */
      e = (u < 0.5f) ? (2.0f * u) : (2.0f * (1.0f - u));
      break;

    case BREATH_WAVE_SQUARE:
      /* Constant inspiratory flow. */
      e = 1.0f;
      break;

    case BREATH_WAVE_TABLE:
      e = BreathSim_TableInterp(u);
      break;

    case BREATH_WAVE_SINE:
    default:
      /* Half-sine: flow starts at zero, peaks mid-inspiration and returns
       * to zero at end-inspiration. This is a flow shape, not the quarter-
       * sine *volume* shape the previous implementation used. */
      e = sinf((float)M_PI * u);
      break;
  }

  return e;
}

/**
 * Mean of the normalised inspiratory envelope for a given flattening.
 *
 * Tidal volume is proportional to this mean, so the ratio between two
 * flattening values is the amplitude correction needed to hold Vt constant.
 * Integrated numerically rather than in closed form so it stays correct for
 * every waveform, including an arbitrary uploaded table.
 */
#define BREATH_ENV_MEAN_STEPS 64U

static float BreathSim_EnvelopeMean(const BreathSimParams_t *p, float flat)
{
  float sum = 0.0f;

  for (uint32_t i = 0U; i < BREATH_ENV_MEAN_STEPS; i++)
  {
    const float u = ((float)i + 0.5f) / (float)BREATH_ENV_MEAN_STEPS;
    sum += BreathSim_ApplyFlattening(BreathSim_BaseShape(u, p), flat);
  }

  return sum / (float)BREATH_ENV_MEAN_STEPS;
}

/** Normalised inspiratory flow demand for u in [0,1]. */
static float BreathSim_InspShape(float u, const BreathSimParams_t *p)
{
  return BreathSim_ApplyFlattening(BreathSim_BaseShape(u, p), g_flat_eff);
}

/**
  * @brief Envelope for the current point in the breath.
  *
  * Inspiration is commanded. Expiration is not: the command decays
  * exponentially to baseline so the blower stops opposing the test lung's
  * elastic recoil. The delivered expiratory waveform is set by the physical
  * R*C of lung + blower backflow resistance + airway chamber.
  */
static float BreathSim_Envelope(float t, const BreathSimParams_t *p,
                                const BreathSimTiming_t *tm,
                                BreathSegment_t *seg_out)
{
  const float insp_end       = tm->insp_s;
  const float insp_pause_end = insp_end + tm->insp_pause_s;
  const float exp_end        = insp_pause_end + tm->exp_s;

  if (t < insp_end)
  {
    *seg_out = BREATH_SEG_INSP;
    const float u = (tm->insp_s > 0.0f) ? (t / tm->insp_s) : 1.0f;
    return BreathSim_InspShape(u, p);
  }

  if (t < insp_pause_end)
  {
    *seg_out = BREATH_SEG_INSP_PAUSE;
    /* Hold the end-inspiratory demand through the plateau. */
    return BreathSim_InspShape(1.0f, p);
  }

  if (t < exp_end)
  {
    *seg_out = BREATH_SEG_EXP;
    const float te = t - insp_pause_end;
    return g_env_exp_start * expf(-te / p->exp_tau_s);
  }

  *seg_out = BREATH_SEG_EXP_PAUSE;
  return 0.0f;
}

/* ===========================================================================
 * Closed-loop flow control
 *
 * Feed-forward + PI on inspiratory flow, output in RPM. Only inspiration is
 * regulated: during expiration the lung, not the blower, moves the gas, and
 * a controller asked to hold zero flow against the recoil would push forward
 * to cancel it - actively fighting the very thing it is meant to allow.
 * ======================================================================== */

static int32_t BreathSim_FlowControlRpm(float dt_s, BreathSegment_t seg)
{
  const float rpm_base_f = (float)g_params.rpm_base;
  const float rpm_max_f  = (float)BREATH_RPM_ABS_MAX;

  /* The envelope already carries event and jitter amplitude scaling. */
  g_q_target_lpm = g_q_peak_lpm * g_envelope;

  const uint8_t inspiring = ((seg == BREATH_SEG_INSP) ||
                             (seg == BREATH_SEG_INSP_PAUSE)) ? 1U : 0U;

  if (inspiring == 0U)
  {
    /* Passive expiration: decay from wherever inspiration left the blower,
     * with the same time constant the open-loop mode uses. Freeze - do not
     * zero - the integrator, so the operating point learned during this
     * breath is still there for the next one. */
    const float t_exp = g_t_s - (g_active.insp_s + g_active.insp_pause_s);
    const float k = (t_exp > 0.0f) ? expf(-t_exp / g_params.exp_tau_s) : 1.0f;
    const float out = rpm_base_f + (((float)g_rpm_insp_end - rpm_base_f) * k);
    return (int32_t)lroundf(clampf(out, rpm_base_f, rpm_max_f));
  }

  if (BreathSim_FlowUsable() == 0U)
  {
    /* No usable feedback. Fall back to the open-loop RPM shape rather than
     * integrating against a stale or bad sample - a flow controller driven
     * by a dead sensor is the one failure mode that could drive the blower
     * to a limit and hold it there. */
    g_flags |= BREATH_FLAG_FLOW_INVALID;
    const float out = rpm_base_f +
                      ((float)g_params.rpm_amplitude * g_envelope);
    return (int32_t)lroundf(clampf(out, rpm_base_f, rpm_max_f));
  }

  const float err = g_q_target_lpm - g_q_meas_lpm;
  const float ff  = g_params.flow_ff_rpm_per_lpm * g_q_target_lpm;

  const float unsat = rpm_base_f + ff + (g_params.flow_kp * err) +
                      g_ctrl_integ_rpm;
  const float sat   = clampf(unsat, rpm_base_f, rpm_max_f);

  /* Conditional integration: while saturated, integrate only in the
   * direction that walks the output back into the linear region. */
  uint8_t may_integrate = 1U;
  if (sat != unsat)
  {
    g_flags |= BREATH_FLAG_FLOW_SAT;
    may_integrate = (((sat < unsat) && (err < 0.0f)) ||
                     ((sat > unsat) && (err > 0.0f))) ? 1U : 0U;
  }
  if (may_integrate != 0U)
  {
    g_ctrl_integ_rpm += g_params.flow_ki * err * dt_s;
    g_ctrl_integ_rpm = clampf(g_ctrl_integ_rpm, 0.0f, rpm_max_f - rpm_base_f);
  }

  return (int32_t)lroundf(sat);
}

/* ===========================================================================
 * Slew limiting
 * ======================================================================== */

static float BreathSim_FallSlewLimit(void)
{
  if (g_bus_voltage_v <= BREATH_VBUS_DERATE_START_V)
  {
    return BREATH_SLEW_FALL_RPM_PER_S;
  }
  if (g_bus_voltage_v >= BREATH_VBUS_DERATE_FULL_V)
  {
    g_flags |= BREATH_FLAG_VBUS_DERATE;
    return BREATH_SLEW_FALL_MIN_RPM_PER_S;
  }

  g_flags |= BREATH_FLAG_VBUS_DERATE;
  const float span = (float)(BREATH_VBUS_DERATE_FULL_V - BREATH_VBUS_DERATE_START_V);
  const float k = (float)(g_bus_voltage_v - BREATH_VBUS_DERATE_START_V) / span;
  return BREATH_SLEW_FALL_RPM_PER_S +
         ((BREATH_SLEW_FALL_MIN_RPM_PER_S - BREATH_SLEW_FALL_RPM_PER_S) * k);
}

static int32_t BreathSim_ApplySlew(int32_t desired_rpm, float dt_s)
{
  if ((dt_s <= 0.0f) || (g_have_posted == 0U))
  {
    return desired_rpm;
  }

  const int32_t prev = g_target_rpm;

  if (desired_rpm > prev)
  {
    const int32_t max_rise = (int32_t)lroundf(BREATH_SLEW_RISE_RPM_PER_S * dt_s);
    if ((desired_rpm - prev) > max_rise)
    {
      g_flags |= BREATH_FLAG_SLEW_LIMITED;
      return prev + max_rise;
    }
    return desired_rpm;
  }

  if (desired_rpm < prev)
  {
    const int32_t max_drop = (int32_t)lroundf(BreathSim_FallSlewLimit() * dt_s);
    if ((prev - desired_rpm) > max_drop)
    {
      g_flags |= BREATH_FLAG_SLEW_LIMITED;
      return prev - max_drop;
    }
    return desired_rpm;
  }

  return desired_rpm;
}

/* ===========================================================================
 * Events and scripting
 * ======================================================================== */

static void BreathSim_SetActiveEvent(uint8_t type, float duration_s, float severity)
{
  g_event             = type;
  g_event_severity    = clampf(severity, 0.0f, 1.0f);
  g_event_remaining_s = (duration_s > 0.0f) ? duration_s : 0.0f;
  if (type == (uint8_t)BREATH_EVENT_CSR)
  {
    g_csr_t_s = 0.0f;
  }
}

static void BreathSim_ScriptAdvance(void)
{
  if (g_script_running == 0U) { return; }

  g_script_idx++;
  if (g_script_idx >= g_script_len)
  {
    if (g_script_loop != 0U)
    {
      g_script_idx = 0U;
    }
    else
    {
      g_script_running = 0U;
      g_script_idx = 0xFFU;
      BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);
      DBG_I("SIM", "script complete");
      return;
    }
  }

  const BreathScriptStep_t *st = &g_script[g_script_idx];
  g_script_remaining_s = st->duration_s;
  BreathSim_SetActiveEvent(st->event, st->duration_s, st->severity);
  DBG_I("SIM", "script step %u: %s sev=%d%% for %d s",
        (unsigned)g_script_idx, BreathSim_EventName(st->event),
        (int)(st->severity * 100.0f), (int)st->duration_s);
}

static void BreathSim_TickEvents(float dt_s)
{
  if (g_script_running != 0U)
  {
    g_script_remaining_s -= dt_s;
    if (g_script_remaining_s <= 0.0f)
    {
      BreathSim_ScriptAdvance();
    }
    return;
  }

  if ((g_event != (uint8_t)BREATH_EVENT_NONE) && (g_event_remaining_s > 0.0f))
  {
    g_event_remaining_s -= dt_s;
    if (g_event_remaining_s <= 0.0f)
    {
      DBG_I("SIM", "event %s ended", BreathSim_EventName(g_event));
      BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);
    }
  }
}

/**
  * @brief Mean/peak of the inspiratory shape, integrated numerically.
  *
  * Needed to turn a tidal volume into a peak flow:
  *     V_T = Q_peak * T_insp * area
  * Doing it numerically rather than with a per-waveform constant means the
  * flattening morph and an uploaded table are handled for free - both change
  * the area, and a hard-coded 2/pi for the half-sine would quietly deliver
  * the wrong volume as soon as flattening was dialled in.
  */
static float BreathSim_ShapeArea(const BreathSimParams_t *p)
{
  const uint32_t N = 32U;
  float sum = 0.0f;

  for (uint32_t i = 0U; i < N; i++)
  {
    const float u = ((float)i + 0.5f) / (float)N;
    sum += BreathSim_InspShape(u, p);
  }
  const float area = sum / (float)N;
  return (area > 0.05f) ? area : 0.05f;   /* guard a degenerate table */
}

/** Amplitude / shape modifiers for the breath about to start. */
static void BreathSim_LatchBreathModifiers(void)
{
  g_amp_scale = 1.0f;
  g_flat_eff  = g_params.flattening;

  switch ((BreathEventType_t)g_event)
  {
    case BREATH_EVENT_APNEA:
      g_amp_scale = 0.0f;
      break;

    case BREATH_EVENT_HYPOPNEA:
      g_amp_scale = 1.0f - g_event_severity;
      break;

    case BREATH_EVENT_FLOW_LIMIT:
      if (g_event_severity > g_flat_eff) { g_flat_eff = g_event_severity; }
      /* Hold tidal volume across the flattening.
       *
       * BreathSim_ApplyFlattening() divides the envelope by k and clips at
       * unity: the peak is preserved but the top widens, so the AREA grows
       * (a half-sine at severity 0.8 gains ~40% Vt). A breath that delivers
       * MORE air than the unflattened one is not flow-limited by any
       * definition, and an APAP will correctly score nothing for it -
       * confirmed against an AirSense 11 on 2026-09-04, where a flattened
       * run raised Vt 0.366 -> 0.459 L and left FlowLim.2s pinned at 0.000.
       *
       * Scaling the amplitude by the inverse area ratio caps flow instead of
       * widening the breath, which is what a limited airway actually does.
       * This is a first-order correction in RPM-envelope space; RPM->flow is
       * not perfectly linear, so trim Amplitude against measured Vt when the
       * exact volume matters. */
      {
        const float mean_flat = BreathSim_EnvelopeMean(&g_params, g_flat_eff);
        const float mean_base = BreathSim_EnvelopeMean(&g_params, 0.0f);
        if (mean_flat > 1.0e-3f)
        {
          g_amp_scale *= (mean_base / mean_flat);
        }
      }
      break;

    case BREATH_EVENT_CSR:
    {
      /* Crescendo-decrescendo with a central apnea through the nadir. */
      const float ph = g_csr_t_s / BREATH_CSR_CYCLE_S;
      float lvl = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * ph));
      /* severity deepens the trough. */
      lvl = lvl * (1.0f - (0.2f * g_event_severity));
      g_amp_scale = (lvl < BREATH_CSR_APNEA_LEVEL) ? 0.0f : lvl;
      break;
    }

    case BREATH_EVENT_NONE:
    default:
      break;
  }

  /* Breath-to-breath variability. Real breathing is never metronomic, and a
   * perfectly periodic stimulus lets a detection algorithm lock on in ways
   * it never could clinically. Seeded, so runs stay reproducible. */
  if (g_params.jitter_pct > 0.0f)
  {
    const float j = g_params.jitter_pct * 0.01f;
    const float k_period = 1.0f + (j * rng_bipolar());
    const float k_insp   = 1.0f + (j * rng_bipolar());
    const float k_amp    = 1.0f + (j * rng_bipolar());

    g_active.period_s     = g_timing.period_s * k_period;
    g_active.insp_s       = g_timing.insp_s * k_insp;
    g_active.insp_pause_s = g_timing.insp_pause_s;
    g_active.exp_s        = g_timing.exp_s;

    /* Absorb the period change in the end-expiratory pause so the segments
     * always still sum to the (jittered) period. */
    float rest = g_active.period_s - g_active.insp_s -
                 g_active.insp_pause_s - g_active.exp_s;
    if (rest < 0.0f)
    {
      g_active.exp_s += rest;
      if (g_active.exp_s < 0.0f) { g_active.exp_s = 0.0f; }
      rest = 0.0f;
      g_active.period_s = g_active.insp_s + g_active.insp_pause_s +
                          g_active.exp_s;
    }
    g_active.exp_pause_s = rest;
    g_active.rate_bpm    = 60.0f / g_active.period_s;

    g_amp_scale *= k_amp;
    if (g_amp_scale < 0.0f) { g_amp_scale = 0.0f; }
  }
  else
  {
    g_active = g_timing;
  }

  /* Peak inspiratory flow this breath needs to deliver the target volume.
   * g_flat_eff is already latched above, so the area reflects any flow
   * limitation applied to this particular breath. */
  g_shape_area = BreathSim_ShapeArea(&g_params);
  if (g_active.insp_s > 0.01f)
  {
    const float litres = g_params.tidal_ml * 0.001f;
    g_q_peak_lpm = (litres / (g_active.insp_s * g_shape_area)) * 60.0f;
  }
  else
  {
    g_q_peak_lpm = 0.0f;
  }
  /* Deliberately NOT scaled by g_amp_scale here: the target flow is formed
   * as q_peak * envelope, and the envelope already carries the event and
   * jitter scaling. Applying it in both places would square it, so a
   * 50% hypopnea would come out at 25%. This value is therefore the
   * nominal peak for a full-amplitude breath. */
  g_q_peak_lpm = clampf(g_q_peak_lpm, 0.0f, BREATH_Q_PEAK_MAX_LPM);
}

/* ===========================================================================
 * Motor supervision
 * ======================================================================== */

static void BreathSim_PostRpm(int32_t rpm)
{
  /* Skip redundant posts: each one bumps cm7_cmd_seq, releases an HSEM and
   * reprograms the CM4 speed ramp. During a plateau or expiratory pause the
   * target does not move, so there is nothing to say. */
  if ((g_have_posted != 0U) && (rpm == g_last_posted_rpm))
  {
    return;
  }
  (void)BlowerIpc_CM7_SetSpeedRpmSilent(rpm);
  g_last_posted_rpm = rpm;
  g_have_posted = 1U;
}

static void BreathSim_AckFaultIfLatched(void)
{
  BlowerIpcStatus_t st;

  if (!BlowerIpc_CM7_GetStatus(&st)) { return; }

  if ((st.mc_state == MC_STATE_FAULT_OVER) && (st.current_faults == 0U))
  {
    (void)BlowerIpc_CM7_AcknowledgeFault();
  }
}

static void BreathSim_EnterFault(const char *why)
{
  g_flags |= BREATH_FLAG_MOTOR_FAULT;
  g_state = BREATH_STATE_FAULT;
  DBG_E("SIM", "run aborted: %s (mc_state=%u rpm_cmd=%ld rpm_act=%ld vbus=%u)",
        why, (unsigned)g_mc_state, (long)g_target_rpm,
        (long)g_rpm_act, (unsigned)g_bus_voltage_v);
  (void)BlowerIpc_CM7_Stop();
  g_have_posted = 0U;
}

/** Read CM4 status and decide whether the run is still valid. */
static void BreathSim_Supervise(float dt_s)
{
  BlowerIpcStatus_t st;

  if (!BlowerIpc_CM7_GetStatus(&st)) { return; }

  g_mc_state       = st.mc_state;
  g_rpm_act        = st.mech_speed_rpm;
  g_bus_voltage_v  = st.bus_voltage_v;

  if ((g_state == BREATH_STATE_PRIMING) && (g_prime_t_s < BREATH_FAULT_GRACE_S))
  {
    return;   /* still settling after the fault acknowledge in Start() */
  }

  if ((st.mc_state == MC_STATE_FAULT_NOW) || (st.mc_state == MC_STATE_FAULT_OVER) ||
      (st.current_faults != 0U))
  {
    BreathSim_EnterFault("motor fault");
    return;
  }

  if (g_state != BREATH_STATE_RUNNING) { return; }

  /* The observer cannot be trusted below OBS_MINIMUM_SPEED_RPM, so a command
   * under it makes the measured speed — and therefore the whole run —
   * questionable. Flag rather than clamp: how low the baseline can go is a
   * motor-tuning property, and the operator needs to see the trade-off. */
  if (g_target_rpm < BREATH_RPM_OBS_MIN_RPM)
  {
    g_flags |= BREATH_FLAG_BELOW_OBS_MIN;
  }

  /* Tracking: an open-loop RPM rig that silently fails to deliver the
   * requested waveform produces plausible data and wrong conclusions. */
  const int32_t err = (g_target_rpm > g_rpm_act) ? (g_target_rpm - g_rpm_act)
                                                 : (g_rpm_act - g_target_rpm);
  int32_t tol = (int32_t)((float)g_target_rpm * BREATH_TRACK_TOL_FRAC);
  if (tol < BREATH_TRACK_TOL_RPM) { tol = BREATH_TRACK_TOL_RPM; }

  if (err > tol)
  {
    g_track_err_s += dt_s;
    if (g_track_err_s >= BREATH_TRACK_HOLD_S)
    {
      if ((g_flags & BREATH_FLAG_TRACK_ERROR) == 0U)
      {
        DBG_W("SIM", "blower not tracking: cmd=%ld act=%ld (tol=%ld)",
              (long)g_target_rpm, (long)g_rpm_act, (long)tol);
      }
      g_flags |= BREATH_FLAG_TRACK_ERROR;
    }
  }
  else
  {
    g_track_err_s = 0.0f;
  }
}

static void BreathSim_PublishStatus(void)
{
  BreathSimStatus_t s;

  s.state         = (uint8_t)g_state;
  s.segment       = (uint8_t)g_segment;
  s.event         = g_event;
  s.script_step   = (g_script_running != 0U) ? g_script_idx : 0xFFU;
  s.flags         = g_flags;
  s.breath_index  = g_breath_index;
  s.phase         = (g_active.period_s > 0.0f) ? (g_t_s / g_active.period_s) : 0.0f;
  s.envelope      = g_envelope;
  s.rpm_cmd       = g_target_rpm;
  s.rpm_act       = g_rpm_act;
  s.bus_voltage_v = g_bus_voltage_v;
  s.mc_state      = g_mc_state;

  s.control_mode   = g_params.control_mode;
  s.flow_valid     = BreathSim_FlowUsable();
  s.q_target_lpm   = g_q_target_lpm;
  s.q_meas_lpm     = g_q_meas_lpm;
  s.q_peak_lpm     = g_q_peak_lpm;
  s.volume_ml      = g_volume_ml;
  s.ctrl_integ_rpm = g_ctrl_integ_rpm;
  s.last_tidal_ml  = g_last_tidal_ml;
  s.p_cmh2o        = g_p_cmh2o;
  s.p_min_cmh2o    = g_p_min_last;
  s.p_max_cmh2o    = g_p_max_last;
  s.p_mean_cmh2o   = g_p_mean_last;

  taskENTER_CRITICAL();
  g_status = s;
  taskEXIT_CRITICAL();
}

/* ===========================================================================
 * Public API
 * ======================================================================== */

void BreathSim_Init(void)
{
  memset(&g_params, 0, sizeof(g_params));
  g_params.timing_mode   = (uint8_t)BREATH_TIMING_RATE_IE;
  g_params.rate_bpm      = 12.0f;
  g_params.insp_time_s   = 1.5f;
  g_params.ie_ratio_exp  = 2.0f;
  g_params.insp_pause_s  = 0.0f;
  g_params.exp_pause_s   = 0.0f;
  g_params.rpm_base      = BREATH_RPM_BASE_DEFAULT;
  g_params.rpm_amplitude = 4000;
  g_params.waveform      = (uint8_t)BREATH_WAVE_SINE;
  g_params.exp_tau_s     = 0.40f;
  g_params.flattening    = 0.0f;
  g_params.jitter_pct    = 0.0f;
  g_params.jitter_seed   = 0x1234ABCDU;

  /* Closed-loop flow control. Off by default so a board with no flow sensor
   * behaves exactly as before. */
  g_params.control_mode  = (uint8_t)BREATH_CTRL_OPEN_LOOP_RPM;
  g_params.tidal_ml      = BREATH_TIDAL_DEFAULT_ML;
  /* Feed-forward starts at zero on purpose: the RPM-per-(L/min) slope is a
   * property of this rig's pneumatic impedance, and a guessed value would
   * overshoot on the first breath of every session. With ff = 0 the
   * integrator finds the operating point within a breath or two and then
   * holds it. Measure the slope once and set it to get the first breath
   * right too. */
  g_params.flow_kp       = 80.0f;
  g_params.flow_ki       = 400.0f;
  g_params.flow_ff_rpm_per_lpm = 0.0f;
  g_params.pressure_limit_cmh2o = 0.0f;

  g_table_len = 0U;
  (void)BreathSim_Resolve(&g_params, &g_timing);

  g_params_pending = g_params;
  g_timing_pending = g_timing;
  g_params_dirty   = 0U;
  g_active         = g_timing;

  g_state           = BREATH_STATE_IDLE;
  g_t_s             = 0.0f;
  g_prime_t_s       = 0.0f;
  g_prime_settle_s  = 0.0f;
  g_breath_index    = 0U;
  g_envelope        = 0.0f;
  g_env_raw         = 0.0f;
  g_env_exp_start   = 0.0f;
  g_target_rpm      = 0;
  g_rpm_act         = 0;
  g_bus_voltage_v   = 0U;
  g_mc_state        = 0U;
  g_flags           = 0U;
  g_segment         = BREATH_SEG_EXP_PAUSE;
  g_last_posted_rpm = 0;
  g_have_posted     = 0U;
  g_track_err_s     = 0.0f;
  g_amp_scale       = 1.0f;
  g_flat_eff        = 0.0f;

  g_event             = (uint8_t)BREATH_EVENT_NONE;
  g_event_severity    = 0.0f;
  g_event_remaining_s = 0.0f;
  g_csr_t_s           = 0.0f;

  g_script_len         = 0U;
  g_script_idx         = 0xFFU;
  g_script_running     = 0U;
  g_script_loop        = 0U;
  g_script_remaining_s = 0.0f;

  g_rng = g_params.jitter_seed;

  g_q_meas_lpm     = 0.0f;
  g_q_valid        = 0U;
  g_q_stamp_ms     = 0U;
  g_p_cmh2o        = 0.0f;
  g_p_valid        = 0U;
  g_q_target_lpm   = 0.0f;
  g_q_peak_lpm     = 0.0f;
  g_shape_area     = 0.637f;
  g_ctrl_integ_rpm = 0.0f;
  g_volume_ml      = 0.0f;
  g_last_tidal_ml  = 0.0f;
  g_rpm_insp_end   = 0;
  g_p_min_acc      = 1.0e9f;
  g_p_max_acc      = -1.0e9f;
  g_p_sum          = 0.0f;
  g_p_count        = 0U;
  g_p_min_last     = 0.0f;
  g_p_max_last     = 0.0f;
  g_p_mean_last    = 0.0f;

  memset(&g_status, 0, sizeof(g_status));
  g_status.script_step = 0xFFU;
}

void BreathSim_GetParams(BreathSimParams_t *out)
{
  if (out == NULL) { return; }

  taskENTER_CRITICAL();
  *out = g_params_pending;
  taskEXIT_CRITICAL();
}

bool BreathSim_SetParams(const BreathSimParams_t *params)
{
  if (params == NULL) { return false; }

  BreathSimParams_t p = *params;
  BreathSimTiming_t t;
  const bool ok = BreathSim_Resolve(&p, &t);

  /* Stage under a critical section: the breath task runs at a higher
   * priority than both writers (GUI and USB), so an unguarded struct copy
   * could be observed half-updated. */
  taskENTER_CRITICAL();
  g_params_pending = p;
  g_timing_pending = t;
  g_params_dirty   = 1U;
  taskEXIT_CRITICAL();

  /* When idle there is no breath boundary coming, so apply immediately. */
  if (g_state == BREATH_STATE_IDLE)
  {
    taskENTER_CRITICAL();
    g_params       = g_params_pending;
    g_timing       = g_timing_pending;
    g_active       = g_timing;
    g_params_dirty = 0U;
    taskEXIT_CRITICAL();
  }

  return ok;
}

void BreathSim_GetTiming(BreathSimTiming_t *out)
{
  if (out == NULL) { return; }

  taskENTER_CRITICAL();
  *out = g_timing_pending;
  taskEXIT_CRITICAL();
}

void BreathSim_GetStatus(BreathSimStatus_t *out)
{
  if (out == NULL) { return; }

  taskENTER_CRITICAL();
  *out = g_status;
  taskEXIT_CRITICAL();
}

void BreathSim_Start(void)
{
  /* Latch whatever is staged, then rev up. */
  taskENTER_CRITICAL();
  g_params       = g_params_pending;
  g_timing       = g_timing_pending;
  g_params_dirty = 0U;
  taskEXIT_CRITICAL();

  g_active          = g_timing;
  g_state           = BREATH_STATE_PRIMING;
  g_t_s             = 0.0f;
  g_prime_t_s       = 0.0f;
  g_prime_settle_s  = 0.0f;
  g_breath_index    = 0U;
  g_envelope        = 0.0f;
  g_env_raw         = 0.0f;
  g_env_exp_start   = 0.0f;
  g_segment         = BREATH_SEG_EXP_PAUSE;
  g_flags           = 0U;
  g_track_err_s     = 0.0f;
  g_amp_scale       = 1.0f;
  g_flat_eff        = g_params.flattening;
  g_csr_t_s         = 0.0f;
  g_rng             = (g_params.jitter_seed != 0U) ? g_params.jitter_seed : 0x1234ABCDU;
  g_target_rpm      = g_params.rpm_base;
  g_have_posted     = 0U;
  g_ctrl_integ_rpm  = 0.0f;
  g_volume_ml       = 0.0f;
  g_last_tidal_ml   = 0.0f;
  g_q_target_lpm    = 0.0f;
  g_rpm_insp_end    = g_params.rpm_base;
  g_p_min_acc       = 1.0e9f;
  g_p_max_acc       = -1.0e9f;
  g_p_sum           = 0.0f;
  g_p_count         = 0U;

  (void)BlowerIpc_CM7_SetSpeedRpmSilent(g_target_rpm);
  g_last_posted_rpm = g_target_rpm;
  g_have_posted     = 1U;
  BreathSim_AckFaultIfLatched();
  (void)BlowerIpc_CM7_Start();

  if (g_params.control_mode == (uint8_t)BREATH_CTRL_FLOW)
  {
    DBG_I("SIM", "flow control: Vt=%d mL over %d ms -> peak ~%d L/min",
          (int)g_params.tidal_ml, (int)(g_timing.insp_s * 1000.0f),
          (int)((g_params.tidal_ml * 0.001f /
                 (g_timing.insp_s * BreathSim_ShapeArea(&g_params))) * 60.0f));
  }
  DBG_I("SIM", "start: %d bpm I:E 1:%d insp=%dms base=%ld amp=%ld %s",
        (int)g_timing.rate_bpm, (int)g_params.ie_ratio_exp,
        (int)(g_timing.insp_s * 1000.0f),
        (long)g_params.rpm_base, (long)g_params.rpm_amplitude,
        BreathSim_WaveformName(g_params.waveform));

  BreathSim_PublishStatus();
}

void BreathSim_Stop(void)
{
  const bool was_active = (g_state != BREATH_STATE_IDLE);

  g_state      = BREATH_STATE_IDLE;
  g_envelope   = 0.0f;
  g_env_raw    = 0.0f;
  g_segment    = BREATH_SEG_EXP_PAUSE;
  g_target_rpm = g_params.rpm_base;

  BreathSim_ScriptStop();
  BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);

  if (was_active)
  {
    (void)BlowerIpc_CM7_SetSpeedRpmSilent(g_target_rpm);
    (void)BlowerIpc_CM7_Stop();
    DBG_I("SIM", "stop: %lu breaths, flags=0x%04X",
          (unsigned long)g_breath_index, (unsigned)g_flags);
  }
  g_have_posted = 0U;

  BreathSim_PublishStatus();
}

bool BreathSim_IsRunning(void)
{
  return (g_state == BREATH_STATE_PRIMING) || (g_state == BREATH_STATE_RUNNING);
}

void BreathSim_Update(float dt_s)
{
  if (g_state == BREATH_STATE_IDLE)
  {
    return;
  }

  if (dt_s <= 0.0f) { dt_s = 0.005f; }

  BreathSim_Supervise(dt_s);

  if (g_state == BREATH_STATE_FAULT)
  {
    BreathSim_PublishStatus();
    return;
  }

  /* ---- priming: the CM4 rev-up takes ~4.6 s, during which no breath the
   * generator produces would be delivered. Wait for the motor to be in RUN
   * and sitting at baseline before the first breath, so the run does not
   * begin with several breaths of garbage. ---------------------------- */
  if (g_state == BREATH_STATE_PRIMING)
  {
    g_prime_t_s += dt_s;
    g_target_rpm = g_params.rpm_base;
    BreathSim_PostRpm(g_target_rpm);

    const int32_t err = (g_rpm_act > g_target_rpm) ? (g_rpm_act - g_target_rpm)
                                                   : (g_target_rpm - g_rpm_act);
    int32_t tol = (int32_t)((float)g_target_rpm * BREATH_TRACK_TOL_FRAC);
    if (tol < BREATH_TRACK_TOL_RPM) { tol = BREATH_TRACK_TOL_RPM; }

    if ((g_mc_state == MC_STATE_RUN) && (err <= tol))
    {
      g_prime_settle_s += dt_s;
      if (g_prime_settle_s >= BREATH_PRIME_SETTLE_S)
      {
        g_state = BREATH_STATE_RUNNING;
        g_t_s = 0.0f;
        g_env_raw = 0.0f;
        g_env_exp_start = 0.0f;
        BreathSim_LatchBreathModifiers();
        DBG_I("SIM", "primed in %d ms, breaths starting",
              (int)(g_prime_t_s * 1000.0f));
      }
    }
    else
    {
      g_prime_settle_s = 0.0f;
    }

    if ((g_state == BREATH_STATE_PRIMING) && (g_prime_t_s >= BREATH_PRIME_TIMEOUT_S))
    {
      if (g_mc_state != MC_STATE_RUN)
      {
        BreathSim_EnterFault("motor never reached RUN");
        BreathSim_PublishStatus();
        return;
      }
      /* In RUN but off-target: proceed, flagged, rather than hang. */
      g_flags |= BREATH_FLAG_TRACK_ERROR;
      g_state = BREATH_STATE_RUNNING;
      g_t_s = 0.0f;
      BreathSim_LatchBreathModifiers();
      DBG_W("SIM", "prime timeout at %ld rpm (target %ld); starting anyway",
            (long)g_rpm_act, (long)g_target_rpm);
    }

    BreathSim_PublishStatus();
    return;
  }

  /* ---- running ------------------------------------------------------ */
  BreathSim_TickEvents(dt_s);
  g_csr_t_s += dt_s;
  if (g_csr_t_s >= BREATH_CSR_CYCLE_S) { g_csr_t_s -= BREATH_CSR_CYCLE_S; }

  g_t_s += dt_s;
  if (g_t_s >= g_active.period_s)
  {
    g_t_s -= g_active.period_s;
    if (g_t_s < 0.0f) { g_t_s = 0.0f; }
    g_breath_index++;

    /* Latch the breath that just finished. This pair - delivered volume and
     * the pressure the device responded with - is what a test report is
     * actually made of. */
    g_last_tidal_ml = g_volume_ml;
    if (g_p_count > 0U)
    {
      g_p_min_last  = g_p_min_acc;
      g_p_max_last  = g_p_max_acc;
      g_p_mean_last = g_p_sum / (float)g_p_count;
    }
    if ((g_params.control_mode == (uint8_t)BREATH_CTRL_FLOW) &&
        (g_amp_scale > 0.5f))
    {
      /* Only meaningful on a normal breath: an apnea or deep hypopnea is
       * supposed to fall short. */
      const float shortfall = g_params.tidal_ml - g_last_tidal_ml;
      if (shortfall > (0.2f * g_params.tidal_ml))
      {
        g_flags |= BREATH_FLAG_VOLUME_SHORT;
      }
    }
    g_volume_ml = 0.0f;
    g_p_min_acc = 1.0e9f;
    g_p_max_acc = -1.0e9f;
    g_p_sum     = 0.0f;
    g_p_count   = 0U;

    /* Breath boundary: the one safe point to swap parameters. */
    if (g_params_dirty != 0U)
    {
      taskENTER_CRITICAL();
      g_params       = g_params_pending;
      g_timing       = g_timing_pending;
      g_params_dirty = 0U;
      taskEXIT_CRITICAL();
    }
    BreathSim_LatchBreathModifiers();
  }

  /* Airway pressure is a measurement, never a controlled variable - but it
   * is the device under test's response, so the per-breath statistics are
   * the actual test result. Accumulate them every tick. */
  if (g_p_valid != 0U)
  {
    if (g_p_cmh2o < g_p_min_acc) { g_p_min_acc = g_p_cmh2o; }
    if (g_p_cmh2o > g_p_max_acc) { g_p_max_acc = g_p_cmh2o; }
    g_p_sum += g_p_cmh2o;
    g_p_count++;

    if ((g_params.pressure_limit_cmh2o > 0.0f) &&
        (g_p_cmh2o > g_params.pressure_limit_cmh2o))
    {
      g_flags |= BREATH_FLAG_PRESSURE_LIMIT;
      BreathSim_EnterFault("airway pressure limit exceeded");
      BreathSim_PublishStatus();
      return;
    }
  }

  const BreathSegment_t prev_seg = g_segment;
  BreathSegment_t seg;
  float env = BreathSim_Envelope(g_t_s, &g_params, &g_active, &seg);

  /* Capture the envelope at the instant expiration begins so the passive
   * decay starts from wherever inspiration actually left off. This must be
   * the RAW shape value: g_envelope has already had the event/jitter
   * amplitude scale applied, and the scale is applied again below, so
   * seeding the decay from it would square the scaling. */
  if ((seg == BREATH_SEG_EXP) && (prev_seg != BREATH_SEG_EXP))
  {
    g_env_exp_start = g_env_raw;
    env = g_env_exp_start;
  }
  g_segment = seg;
  g_env_raw = env;

  env *= g_amp_scale;
  if (env < 0.0f) { env = 0.0f; }
  if (env > 1.0f) { env = 1.0f; }
  g_envelope = env;

  /* Inspired volume. Only positive flow during inspiration counts toward
   * tidal volume; expiratory (negative) flow is the lung emptying. */
  const uint8_t inspiring = ((seg == BREATH_SEG_INSP) ||
                             (seg == BREATH_SEG_INSP_PAUSE)) ? 1U : 0U;
  if ((inspiring != 0U) && (BreathSim_FlowUsable() != 0U) &&
      (g_q_meas_lpm > 0.0f))
  {
    /* L/min -> mL over dt: q * dt * 1000/60 */
    g_volume_ml += g_q_meas_lpm * dt_s * (1000.0f / 60.0f);
  }

  int32_t desired_rpm;
  if (g_params.control_mode == (uint8_t)BREATH_CTRL_FLOW)
  {
    desired_rpm = BreathSim_FlowControlRpm(dt_s, seg);
  }
  else
  {
    g_q_target_lpm = 0.0f;
    desired_rpm = g_params.rpm_base +
                  (int32_t)lroundf((float)g_params.rpm_amplitude * g_envelope);
  }

  /* Remember where inspiration left the blower so the passive decay starts
   * from the right place next tick. */
  if (inspiring != 0U)
  {
    g_rpm_insp_end = desired_rpm;
  }

  g_target_rpm = BreathSim_ApplySlew(
      clamp_i32(desired_rpm, 0, BREATH_RPM_ABS_MAX), dt_s);

  BreathSim_PostRpm(g_target_rpm);
  BreathSim_PublishStatus();
}

/* ===========================================================================
 * Table / events / script
 * ======================================================================== */

bool BreathSim_SetTable(const float *points, uint8_t n)
{
  if ((points == NULL) || (n < 2U) || (n > (uint8_t)BREATH_TABLE_POINTS))
  {
    return false;
  }

  taskENTER_CRITICAL();
  for (uint8_t i = 0U; i < n; ++i)
  {
    g_table[i] = clampf(points[i], 0.0f, 1.0f);
  }
  g_table_len = n;
  taskEXIT_CRITICAL();
  return true;
}

uint8_t BreathSim_GetTableLength(void)
{
  return g_table_len;
}

bool BreathSim_TriggerEvent(const BreathEvent_t *ev)
{
  if (ev == NULL) { return false; }
  if (ev->type >= (uint8_t)BREATH_EVENT_COUNT) { return false; }
  if (g_script_running != 0U) { return false; }

  taskENTER_CRITICAL();
  BreathSim_SetActiveEvent(ev->type, ev->duration_s, ev->severity);
  taskEXIT_CRITICAL();

  DBG_I("SIM", "event %s sev=%d%% for %d s",
        BreathSim_EventName(ev->type),
        (int)(ev->severity * 100.0f), (int)ev->duration_s);
  return true;
}

void BreathSim_CancelEvent(void)
{
  taskENTER_CRITICAL();
  BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);
  taskEXIT_CRITICAL();
}

void BreathSim_ScriptClear(void)
{
  taskENTER_CRITICAL();
  g_script_len     = 0U;
  g_script_idx     = 0xFFU;
  g_script_running = 0U;
  taskEXIT_CRITICAL();
}

bool BreathSim_ScriptAppend(const BreathScriptStep_t *step)
{
  if (step == NULL) { return false; }
  if (g_script_len >= (uint8_t)BREATH_SCRIPT_MAX_STEPS) { return false; }
  if (step->event >= (uint8_t)BREATH_EVENT_COUNT) { return false; }
  if (step->duration_s <= 0.0f) { return false; }

  taskENTER_CRITICAL();
  g_script[g_script_len].duration_s = step->duration_s;
  g_script[g_script_len].event      = step->event;
  g_script[g_script_len].severity   = clampf(step->severity, 0.0f, 1.0f);
  g_script_len++;
  taskEXIT_CRITICAL();
  return true;
}

uint8_t BreathSim_ScriptLength(void)
{
  return g_script_len;
}

bool BreathSim_ScriptStart(bool loop)
{
  if (g_script_len == 0U) { return false; }

  taskENTER_CRITICAL();
  g_script_running     = 1U;
  g_script_loop        = loop ? 1U : 0U;
  g_script_idx         = 0U;
  g_script_remaining_s = g_script[0].duration_s;
  BreathSim_SetActiveEvent(g_script[0].event, g_script[0].duration_s,
                           g_script[0].severity);
  taskEXIT_CRITICAL();

  DBG_I("SIM", "script start: %u steps%s",
        (unsigned)g_script_len, loop ? " (loop)" : "");
  return true;
}

void BreathSim_ScriptStop(void)
{
  taskENTER_CRITICAL();
  g_script_running = 0U;
  g_script_idx     = 0xFFU;
  taskEXIT_CRITICAL();
}

bool BreathSim_ScriptIsRunning(void)
{
  return (g_script_running != 0U);
}

/* ===========================================================================
 * Names
 * ======================================================================== */

const char *BreathSim_WaveformName(uint8_t waveform)
{
  switch ((BreathWaveform_t)waveform)
  {
    case BREATH_WAVE_RAMP:   return "Ramp";
    case BREATH_WAVE_SQUARE: return "Square";
    case BREATH_WAVE_TABLE:  return "Table";
    case BREATH_WAVE_SINE:
    default:                 return "Sine";
  }
}

const char *BreathSim_ControlModeName(uint8_t mode)
{
  return (mode == (uint8_t)BREATH_CTRL_FLOW) ? "Flow" : "OpenRPM";
}

const char *BreathSim_EventName(uint8_t event)
{
  switch ((BreathEventType_t)event)
  {
    case BREATH_EVENT_APNEA:      return "Apnea";
    case BREATH_EVENT_HYPOPNEA:   return "Hypopnea";
    case BREATH_EVENT_FLOW_LIMIT: return "FlowLimit";
    case BREATH_EVENT_CSR:        return "CSR";
    case BREATH_EVENT_NONE:
    default:                      return "None";
  }
}

const char *BreathSim_StateName(uint8_t state)
{
  switch ((BreathSimState_t)state)
  {
    case BREATH_STATE_PRIMING: return "Priming";
    case BREATH_STATE_RUNNING: return "Running";
    case BREATH_STATE_FAULT:   return "Fault";
    case BREATH_STATE_IDLE:
    default:                   return "Idle";
  }
}

const char *BreathSim_SegmentName(uint8_t segment)
{
  switch ((BreathSegment_t)segment)
  {
    case BREATH_SEG_INSP:       return "Insp";
    case BREATH_SEG_INSP_PAUSE: return "InspPause";
    case BREATH_SEG_EXP:        return "Exp";
    case BREATH_SEG_EXP_PAUSE:
    default:                    return "ExpPause";
  }
}

/* ===========================================================================
 * Legacy scalar accessors
 * ======================================================================== */

int32_t BreathSim_GetTargetRpm(void) { return g_target_rpm; }

float BreathSim_GetPhase(void)
{
  return (g_active.period_s > 0.0f) ? (g_t_s / g_active.period_s) : 0.0f;
}

float BreathSim_GetCyclePeriodS(void) { return g_active.period_s; }

float BreathSim_GetEnvelope(void) { return g_envelope; }
