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
/* 2026-09-28: 25000 -> 60000, to make ~50 slm peak inspiratory flow
 * reachable. A half-sine of amplitude A over T_insp demands A*pi/T_insp on
 * BOTH flanks, and the falling flank is inside inspiration - so this limit
 * shapes the breath, it does not just protect the decel at the end of it.
 * At the full 24000 amplitude: 45k rpm/s for 900 mL at 12 bpm, 57k for
 * 700 mL at 15 bpm. At 25000 those breaths were clipped 1.8-2.3x and the
 * flow simply could not come back down in time, spilling into expiration.
 *
 * This is a real increase in regenerative energy per breath. It is allowed
 * to be, because the Vbus derate below now has somewhere to arbitrate: the
 * ladder was widened on the same day (brake 25.5 V, derate 26.0-27.2 V,
 * OV trip 28.0 V) and it pulls this number back automatically when the bus
 * actually climbs, raising BREATH_FLAG_VBUS_DERATE so the run says so.
 * Faster breaths than the above will still clip and flag rather than
 * silently distort. */
/* 60000, raised deliberately on 2026-09-28 - the second time, and this time
 * with the protection actually working underneath it.
 *
 * History worth keeping: it was raised to 60000 once before on the reasoning
 * that "the Vbus derate now has somewhere to arbitrate". That was false. CM4
 * was publishing 0 mV because BlowerBrake_Update() had been commented out, so
 * the derate never engaged and neither did the brake, and the first fast
 * deceleration tripped OVER_VOLT. It was put back to 25000 until that was
 * fixed.
 *
 * It now is: CM4 publishes bus voltage unconditionally, and runs since show
 * no VBUS_DERATE flag, which is the positive confirmation - the guard raises
 * that flag whenever the reading is 0.
 *
 * Why 60000: a half-sine of amplitude A over Ti demands A*pi/Ti on both
 * flanks, and the falling flank is INSIDE inspiration, so this limit shapes
 * the breath rather than merely protecting the decel after it. At the full
 * 24000 amplitude over a 1.667 s inspiration that is ~45200 rpm/s. 25000 was
 * clipping it by 1.8x, which showed up as SLEW_LIMITED on every breath of
 * every characterisation run. 60000 clears it with margin.
 *
 * The ladder still arbitrates if the bus actually climbs: brake at 25.5 V,
 * this derate 26.0-27.2 V, OV trip 28.0 V. */
#define BREATH_SLEW_FALL_RPM_PER_S       25000.0f
/* ^ 25000, and the question is now settled by measurement. Do not raise it
 * expecting more flow: it will not give any, and it costs bus margin.
 *
 * Paired runs at CPAP 4, healthy sensor, one constant changed:
 *
 *              25000              60000
 *   volume     516.2 mL           521.8 mL
 *   peak flow   43.7 slm           43.7 slm      <- identical
 *   flags      SLEW_LIMITED       VBUS_DERATE
 *
 * The performance is the same; only the mechanism doing the limiting moves.
 * At 25000 this constant binds. At 60000 it stops binding, the bus climbs
 * into the derate band, and the derate pulls the effective rate back to
 * roughly the same place. So the real limit is REGEN CAPACITY - how fast the
 * bus can absorb the energy of a decelerating impeller - not this number.
 *
 * 25000 is preferred over 60000 at equal performance because the bus then
 * stays below the 26 V derate band rather than sitting inside it, just under
 * the 28 V trip. Same output, more margin.
 *
 * To actually decelerate faster the energy needs somewhere to go: a real
 * brake resistor on the regen path, more over-voltage headroom, or a
 * lower-inertia impeller (energy goes as omega squared, so this is the big
 * lever). SLEW_LIMITED will stay set on every breath - that is the rig
 * telling you the truth about the falling flank, not a fault.
 *
 * History, because two earlier attempts reached wrong conclusions:
 *
 * A half-sine of amplitude A over Ti demands A*pi/Ti on BOTH flanks, and the
 * falling flank is inside inspiration - so this limit shapes the breath, it
 * does not merely protect the decel after it. At the full 24000 amplitude
 * over a 1.667 s inspiration that is ~45200 rpm/s, and at 25000 SLEW_LIMITED
 * was set on every breath of every run.
 *
 * Attempt 1 (60000): tripped OVER_VOLT. Cause was not the slew rate - CM4 was
 * publishing 0 mV because BlowerBrake_Update() had been commented out, so
 * neither the brake nor this derate ever engaged. Reverted to 25000.
 *
 * Attempt 2 (60000, bus voltage fixed): no trip - the ladder held - but the
 * bus reached the derate band and delivered volume read zero. The zero was
 * later traced to a DEAD FLOW SENSOR, not the slew rate, so that run proved
 * nothing about volume. A comment here previously claimed 25000 was the
 * bus's measured regen limit; that claim rested on this invalid run and has
 * been removed.
 *
 * Attempt 3 (60000, sensor verified healthy): the result recorded above.
 * VBUS_DERATE set, performance unchanged, so 60000 buys nothing. Reverted. */
/* Floor for the derate, NOT a safe-mode value. 5000 was too low to be
 * useful: when the derate engaged, the blower could no longer return to
 * baseline inside a breath (24000 rpm at 5000 rpm/s is 4.8 s against a 5 s
 * period), so the waveform degenerated into constant RPM, the lung
 * equilibrated and delivered volume collapsed to ZERO. Measured 2026-09-28.
 *
 * A derate should degrade the breath, not abolish it. 15000 still sheds
 * 24000 rpm inside 1.6 s, so breaths continue - smaller and distorted, and
 * flagged as such - while the bus is given relief. */
#define BREATH_SLEW_FALL_MIN_RPM_PER_S   15000.0f

/*
 * Derate band, in MILLIVOLTS. These were 23 V / 24 V in whole volts, which
 * was wrong twice over on this board:
 *
 *  - The supply measures 23.86 V (V_MOT across R138 = 1.491 V on the
 *    as-drawn 180K/12K divider, measured 2026-09-28). A band starting at
 *    23 V therefore starts BELOW the idle bus.
 *  - bus_voltage_v is whole volts, so the band spanned one integer count:
 *    the fall limit was not a ramp but a step from 25000 to 5000 RPM/s the
 *    instant the reading ticked to 24. Shedding 20000 RPM at 5000 RPM/s
 *    takes 4 s, which distorts any breath faster than about 7 BPM - a
 *    strong candidate for why fast, deep breaths have been hard to get.
 *
 * The band sits above the CM4 brake (25.5 V on, 24.5 V off) so the brake
 * dissipates first and backing off the commanded waveform is the second
 * line of defence, and below the over-voltage trip, which was raised from
 * 24.9 V to 28.0 V on 2026-09-28 precisely to make room for this ladder.
 * Full ladder: supply 23.86 -> brake 25.5 -> derate 26.0..27.2 -> trip 28.0
 * -> lowest-rated caps on the regen path 35.
 */
#define BREATH_VBUS_DERATE_START_MV      26000U
#define BREATH_VBUS_DERATE_FULL_MV       27200U

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
/* CM4 holds a start off while the rotor coasts down after a stop. */
static uint16_t g_restart_holdoff_ms;
static uint32_t g_bus_voltage_mv;
static uint16_t g_occurred_faults;
static uint16_t g_current_faults;
static uint16_t g_flags;
static BreathSegment_t g_segment;
static int32_t  g_last_posted_rpm;
static uint8_t  g_have_posted;
static float    g_track_err_s;
static uint8_t  g_holdoff_logged;

/* Per-breath modifiers, latched at each breath boundary. */
static float    g_amp_scale;
static float    g_flat_eff;
static uint8_t  g_fl_shape_eff;
/* The event those modifiers were latched from, and the one that shaped the
 * breath before it. Held apart from g_event so telemetry can mark an event
 * where it actually took effect rather than where its seconds timer expired
 * - see the note on event_applied in breath_sim.h. */
static uint8_t  g_event_applied;
static float    g_event_sev_applied;
static uint8_t  g_last_event;
static float    g_last_event_sev;
static uint8_t  g_recovery_left;   /* recovery breaths still owed */

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
static float    g_script_remaining_s;
static uint8_t  g_script_rep_left;    /* repeats left of the current step  */
static uint16_t g_script_passes_left; /* 0 = run for ever                  */
static uint8_t  g_script_overridden;  /* last step changed the parameters  */
/* Parameters as they were when the script started. Step overrides apply to
 * THIS, not to whatever the previous step left behind, so a step means the
 * same thing wherever it appears in the script. */
static BreathSimParams_t g_script_base;

/* Arbitrary waveform table */
static float    g_table[BREATH_TABLE_POINTS];
static uint8_t  g_table_len;

/* Snore modulation phase, 0..1. */
static float    g_snore_phase;

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
/* Time the pressure has been invalid while the run depends on it - see the
 * watchdog in BreathSim_Update. */
#define BREATH_P_INVALID_ABORT_S  0.3f
static float    g_p_invalid_s;

/* Stop ramp-down. A stop (`sim stop`, the GUI) and an abort the motor itself
 * did not cause (pressure limit, pressure invalid) used to post STOP straight
 * away, which switches PWM off at whatever speed the blower was at. Instead
 * the speed is walked down at the normal, bus-voltage-derated fall rate to
 * STOP_RAMP_END_RPM and only then stopped. Active decel also drops the airway
 * pressure faster than a free coast does. STOP_RAMP_SLACK_S bounds the wait
 * for the rotor to follow, so a drive that will not decelerate is still
 * stopped. The ramp runs from BreathSim_Update in IDLE (stop) or FAULT
 * (abort). */
#define BREATH_STOP_RAMP_END_RPM    BREATH_RPM_OBS_MIN_RPM
#define BREATH_STOP_RAMP_TOL_RPM    1000
#define BREATH_STOP_RAMP_SLACK_S    2.0f
static volatile uint8_t g_stop_ramping;
static float    g_stop_ramp_left_s;

static float    g_q_target_lpm;
static float    g_q_peak_lpm;      /* peak this breath needs for tidal_ml */
static float    g_shape_area;      /* mean/peak of the inspiratory shape */
static float    g_ctrl_integ_rpm;  /* PI integrator, RPM */
/* Pressure-compensated feed-forward. */
static float    g_ff_rpm;          /* feed-forward part of the command */
static float    g_ff_p_inlet;      /* estimated blower inlet pressure  */
static uint8_t  g_ff_p_inlet_valid;
static float    g_ff_pacc_sum;     /* end-expiratory pressure, summed   */
static uint32_t g_ff_pacc_n;
static float    g_ff_pacc_qmax;    /* largest |flow| in that window     */
static float    g_ff_pacc_qsum;    /* signed flow in that window, summed */
static uint32_t g_ff_no_est_breaths; /* pressure-mode breaths with no estimate */
static uint8_t  g_ff_fallback;     /* breathing on the static path for want of one */
static uint32_t g_sat_ticks;       /* ticks spent at the rail this breath */
static uint32_t g_insp_ticks;      /* inspiratory ticks this breath       */
static float    g_q_target_peak;   /* highest flow commanded this breath   */
static float    g_q_meas_peak;     /* highest flow actually seen this breath */
static float    g_q_meas_sum;      /* sum of measured inspiratory flow       */
static uint32_t g_q_meas_n;        /* ticks in that sum                      */
static uint8_t  g_no_response_strikes;
static uint8_t  g_ctrl_disabled;   /* closed loop abandoned for this run   */
static float    g_volume_ml;       /* inspired volume so far this breath */
static float    g_last_tidal_ml;
static float    g_last_insp_s;     /* Ti of the completed breath, jittered  */
static float    g_last_q_peak;     /* measured peak flow, completed breath  */
static float    g_last_flat_idx;   /* measured insp mean/peak, completed    */
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
  if (p->recovery_breaths > (uint8_t)BREATH_RECOVERY_BREATHS_MAX)
  {
    p->recovery_breaths = (uint8_t)BREATH_RECOVERY_BREATHS_MAX;
    ok = false;
  }
  p->recovery_gain = clampf(p->recovery_gain, BREATH_RECOVERY_GAIN_MIN,
                            BREATH_RECOVERY_GAIN_MAX);
  if (p->snore_hz > 0.0f)
  {
    /* 0 is off; anything else has to be a frequency the 200 Hz task can
     * actually form, so silently accepting 2 Hz would be a lie. */
    if (p->snore_hz < BREATH_SNORE_HZ_MIN)
    {
      p->snore_hz = BREATH_SNORE_HZ_MIN;
      ok = false;
    }
    else if (p->snore_hz > BREATH_SNORE_HZ_MAX)
    {
      p->snore_hz = BREATH_SNORE_HZ_MAX;
      ok = false;
    }
    else
    {
      /* in range */
    }
  }
  if (p->snore_rpm < 0)                       { p->snore_rpm = 0; ok = false; }
  if (p->snore_rpm > BREATH_SNORE_RPM_MAX)
  {
    p->snore_rpm = BREATH_SNORE_RPM_MAX;
    ok = false;
  }

  p->tidal_ml     = clampf(p->tidal_ml, BREATH_TIDAL_MIN_ML, BREATH_TIDAL_MAX_ML);
  p->flow_kp      = clampf(p->flow_kp, 0.0f, BREATH_FLOW_KP_MAX);
  p->flow_ki      = clampf(p->flow_ki, 0.0f, BREATH_FLOW_KI_MAX);
  p->flow_ff_rpm_per_lpm = clampf(p->flow_ff_rpm_per_lpm, 0.0f, BREATH_FLOW_FF_MAX);
  if (p->ff_mode >= (uint8_t)BREATH_FF_COUNT) { p->ff_mode = (uint8_t)BREATH_FF_STATIC; ok = false; }
  p->ff_a = clampf(p->ff_a, BREATH_FF_A_MIN, BREATH_FF_A_MAX);
  p->ff_b = clampf(p->ff_b, BREATH_FF_B_MIN, BREATH_FF_B_MAX);
  p->ff_c = clampf(p->ff_c, -BREATH_FF_C_MAX, BREATH_FF_C_MAX);
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
  if (p->fl_shape >= (uint8_t)BREATH_FL_COUNT)       { p->fl_shape = (uint8_t)BREATH_FL_PLATEAU; ok = false; }
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
static float BreathSim_ApplyFlattening(float u, float e, float flat,
                                       uint8_t shape)
{
  if (flat <= 0.0f) { return e; }
  const float k = 1.0f - (0.85f * flat);   /* 1.00 .. 0.15 */
  const float v = e / k;
  float pl = (v > 1.0f) ? 1.0f : v;

  /* The variants tilt or notch that plateau. Every modulation is scaled by
   * `flat`, so at zero flattening they all collapse to the unflattened
   * waveform and the selector cannot alter a breath that is not flow
   * limited - which also keeps every existing run reproducible.
   *
   * None of them renormalise the peak back to 1, and none need to: peak
   * flow for the target volume is q = V / (Ti * area) with `area` measured
   * from this same shape, so the delivered volume comes out at V whatever
   * the shape's own maximum happens to be. */
  switch ((BreathFlShape_t)shape)
  {
    case BREATH_FL_CHAIR:
      /* Early peak falling away to a lower plateau: the airway recruits,
       * then partially gives way for the rest of inspiration. */
      pl *= 1.0f - (BREATH_FL_TILT * flat * u);
      break;

    case BREATH_FL_LATE:
      /* The mirror image - the airway opens late in inspiration. */
      pl *= 1.0f - (BREATH_FL_TILT * flat * (1.0f - u));
      break;

    case BREATH_FL_M:
      /* Mid-inspiratory dip. A Starling resistor hunting around its
       * critical pressure produces this double hump, and it is the case
       * worth testing hardest: holding tidal volume keeps peak flow up,
       * while the notch pulls the mean down, so a mean/peak flattening
       * detector can miss it entirely. */
      pl *= 1.0f - (BREATH_FL_DIP * flat * sinf((float)M_PI * u));
      break;

    case BREATH_FL_PLATEAU:
    default:
      break;
  }

  return pl;
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

static float BreathSim_EnvelopeMean(const BreathSimParams_t *p, float flat,
                                    uint8_t shape)
{
  float sum = 0.0f;

  for (uint32_t i = 0U; i < BREATH_ENV_MEAN_STEPS; i++)
  {
    const float u = ((float)i + 0.5f) / (float)BREATH_ENV_MEAN_STEPS;
    sum += BreathSim_ApplyFlattening(u, BreathSim_BaseShape(u, p), flat,
                                     shape);
  }

  return sum / (float)BREATH_ENV_MEAN_STEPS;
}

/** Normalised inspiratory flow demand for u in [0,1]. */
static float BreathSim_InspShape(float u, const BreathSimParams_t *p)
{
  return BreathSim_ApplyFlattening(u, BreathSim_BaseShape(u, p), g_flat_eff,
                                   g_fl_shape_eff);
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

/**
  * @brief Feed-forward rpm from the bench-fitted plant model.
  *
  * Inverts  q = a*N^2 - b*(P - P_inlet) + c  for N at the target flow and
  * the lung pressure measured this tick. Because P is measured rather than
  * predicted, the command rises on its own as the lung fills - which is what
  * holding a flow, and above all holding a FLAT top, requires.
  *
  * Pressure freshness rides on the flow check: both arrive from SensorTask
  * in the same BreathSim_SetSensorInputs call, so a fresh flow sample means
  * an equally fresh pressure sample, and g_p_valid already folds in "sensor
  * present and not a held reading".
  *
  * @return rpm, or a negative value when the model cannot be used this tick.
  */
static float BreathSim_FfPressureRpm(float q_target)
{
  if ((g_ff_p_inlet_valid == 0U) || (g_p_valid == 0U)) { return -1.0f; }

  const float p_rel = g_p_cmh2o - g_ff_p_inlet;
  float n2 = (q_target - g_params.ff_c + (g_params.ff_b * p_rel)) /
             g_params.ff_a;                          /* krpm^2 */
  if (n2 < 0.0f) { n2 = 0.0f; }
  return sqrtf(n2) * 1000.0f;
}

static int32_t BreathSim_FlowControlRpm(float dt_s, BreathSegment_t seg)
{
  const float rpm_base_f = (float)g_params.rpm_base;
  const float rpm_max_f  = (float)BREATH_RPM_ABS_MAX;

  /* The envelope already carries event, recovery and jitter amplitude
   * scaling, and may therefore exceed 1 - so the product needs the same
   * ceiling the nominal peak gets. */
  g_q_target_lpm = clampf(g_q_peak_lpm * g_envelope,
                          0.0f, BREATH_Q_PEAK_MAX_LPM);

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

  /* A commanded apnea is zero inspiratory effort, so there is nothing to
   * regulate - the target is zero for the whole breath. Running the PI
   * against it anyway is not merely pointless, it is actively harmful.
   *
   * With flow_ff at 0 the integrator carries the entire operating point:
   * measured 22900 rpm on the bench, 2026-10-09. The first inspiratory
   * tick of an apnea breath has q_meas still near zero, so err is near
   * zero, and the output is rpm_base + 22900 - the blower was commanded
   * to 31300 rpm in the middle of a commanded APNEA, every breath, while
   * the integrator unwound at ki*err. That is why peak flow took 20 s to
   * fall below the 10 slm the device under test treats as a breath, and
   * why dropping rpm_base from 8000 to 6000 changed nothing whatsoever:
   * the baseline was never what kept the blower pushing.
   *
   * Command the baseline and FREEZE the integrator, exactly as the
   * expiration branch above does and for the same reason - the operating
   * point it learned is still the right one for the breaths that follow,
   * and the recovery breaths after an apnea need it immediately rather
   * than having to re-learn it from zero.
   *
   * amp_scale is 0 only for a full apnea and for the nadir of a CSR
   * cycle; a hypopnea scales it down but leaves it positive, and that is
   * a real reduced breath which the loop should still regulate. */
  if (g_amp_scale <= 0.0f)
  {
    return (int32_t)lroundf(clampf(rpm_base_f, 0.0f, rpm_max_f));
  }

  /* The measured peak is accumulated in BreathSim_Update, not here: it is
   * needed in open-loop mode too, and only when the sample is fresh. */
  if (g_q_target_lpm > g_q_target_peak) { g_q_target_peak = g_q_target_lpm; }

  if ((BreathSim_FlowUsable() == 0U) || (g_ctrl_disabled != 0U))
  {
    /* No usable feedback, or closed loop has been abandoned because the flow
     * never responded. Fall back to the open-loop RPM shape rather than
     * integrating against a sample that is stale, bad, or simply not
     * describing this breath - a flow controller chasing a flow it cannot
     * see is precisely the failure mode that drives the blower to its limit
     * and holds it there. */
    if (g_ctrl_disabled == 0U) { g_flags |= BREATH_FLAG_FLOW_INVALID; }
    const float out = rpm_base_f +
                      ((float)g_params.rpm_amplitude * g_envelope);
    return (int32_t)lroundf(clampf(out, rpm_base_f, rpm_max_f));
  }

  const uint8_t want_pressure =
      (g_params.ff_mode == (uint8_t)BREATH_FF_PRESSURE) ? 1U : 0U;

  /* Pressure mode with no inlet estimate yet - the first breath of a run.
   * Hold rpm_base and freeze the integrator rather than falling back to the
   * static path: that path would let the integrator wind up over a whole
   * inspiration, and the next breath would then stack it on top of a ~27000
   * rpm model command. The held breath also gives the end-expiratory window
   * a still lung to estimate from, so the estimate is ready by breath two.
   * The cost is one quiet breath at the start of every pressure-mode run.
   *
   * But only for BREATH_FF_PREF_HOLD_BREATHS breaths. Holding until an
   * estimate arrives deadlocked against a CPAP oscillating (FOT) from the
   * start of a run: no still lung, so no estimate, so no breath, so the CPAP
   * never stopped oscillating. After that, breathe on the static path below
   * (FfPressureRpm returns -1 and FF_NO_PRESSURE is flagged). */
  if ((want_pressure != 0U) && (g_ff_p_inlet_valid == 0U) &&
      (g_ff_no_est_breaths < BREATH_FF_PREF_HOLD_BREATHS))
  {
    g_ff_rpm = rpm_base_f;
    return (int32_t)lroundf(rpm_base_f);
  }

  const float err = g_q_target_lpm - g_q_meas_lpm;

  float   ff_cmd = rpm_base_f + (g_params.flow_ff_rpm_per_lpm * g_q_target_lpm);
  uint8_t pressure_ff = 0U;
  if (want_pressure != 0U)
  {
    const float n = BreathSim_FfPressureRpm(g_q_target_lpm);
    if (n >= 0.0f)
    {
      ff_cmd = n;
      pressure_ff = 1U;
      if (g_ff_fallback != 0U)
      {
        /* The estimate arrived after a fallback. On the static path the
         * integrator carried the whole operating point; stacked on the
         * model's command it would overshoot, so start the trim afresh. */
        g_ctrl_integ_rpm = 0.0f;
        g_ff_fallback = 0U;
      }
    }
    else
    {
      /* Sample unusable mid-run, or no estimate yet after the hold. The
       * static path under-commands with an integrator near zero, which is
       * the safe direction to fail. */
      g_flags |= BREATH_FLAG_FF_NO_PRESSURE;
      if (g_ff_p_inlet_valid == 0U) { g_ff_fallback = 1U; }
    }
  }
  g_ff_rpm = ff_cmd;

  const float unsat = ff_cmd + (g_params.flow_kp * err) + g_ctrl_integ_rpm;
  const float sat   = clampf(unsat, rpm_base_f, rpm_max_f);

  /* Conditional integration: while saturated, integrate only in the
   * direction that walks the output back into the linear region. This part
   * is about either rail. */
  g_insp_ticks++;
  uint8_t may_integrate = 1U;
  if (sat != unsat)
  {
    may_integrate = (((sat < unsat) && (err < 0.0f)) ||
                     ((sat > unsat) && (err > 0.0f))) ? 1U : 0U;

    /* But only the UPPER rail counts as evidence that the target is
     * unreachable, and only the upper rail is worth flagging.
     *
     * The clamp is two-sided. Its lower bound is rpm_base, and a half-sine
     * target passes through zero at both ends of inspiration - so near the
     * start and end of EVERY normal breath the target flow is ~0, the
     * measured flow is not (rpm_base is still pushing), the error is
     * negative, and the output is clamped up to rpm_base. That is the
     * controller working correctly, not running out of actuator.
     *
     * Counting it as saturation did real damage. It is a large fraction of
     * every breath, so g_sat_ticks routinely passed the 50% test at the
     * breath boundary and bled the integrator by 15% - every breath, for
     * ever. The controller could then never accumulate enough to reach the
     * target: measured 2026-10-09, Vt crept up over eight breaths and
     * stalled short (459 of 500 mL at Ti 1.667 s, 489 of 500 at Ti 2.5 s),
     * with FLOW_SAT and VOLUME_SHORT raised on every run including ones
     * that were nowhere near the speed ceiling. The flag was also useless
     * as a diagnostic, since it was set on all of them.
     */
    if (sat < unsat)                      /* clamped DOWN = at rpm_max */
    {
      g_flags |= BREATH_FLAG_FLOW_SAT;
      g_sat_ticks++;
    }
  }
  if (may_integrate != 0U)
  {
    /* With the static feed-forward the integrator IS the operating point,
     * which is always above rpm_base, so it is clamped at zero from below.
     * With the pressure model it only trims the model's error, and that
     * can go either way - an over-commanding model needs a negative trim,
     * and a floor at zero would leave the proportional term alone to fight
     * a steady excess. */
    const float span = rpm_max_f - rpm_base_f;
    g_ctrl_integ_rpm += g_params.flow_ki * err * dt_s;
    g_ctrl_integ_rpm = clampf(g_ctrl_integ_rpm,
                              (pressure_ff != 0U) ? -span : 0.0f, span);
  }

  return (int32_t)lroundf(sat);
}

/**
  * @brief RPM to add on top of the slew-limited command for snore.
  *
  * Added AFTER the slew limiter on purpose. At 150000 rpm/s the limiter
  * permits 750 rpm per 200 Hz tick, which would shred a 30 Hz modulation
  * of any useful depth; and routing it through the limiter would also feed
  * the oscillation into the limiter's own previous-value state.
  *
  * Inspiration only, and scaled by the envelope: snoring is an inspiratory
  * noise that is loudest at peak flow, not a square burst switched on at
  * the start of the limb. The phase is reset between breaths so every
  * breath's modulation starts the same way and runs stay reproducible.
  */
static int32_t BreathSim_SnoreOffset(float dt_s, BreathSegment_t seg)
{
  if ((g_params.snore_hz <= 0.0f) || (g_params.snore_rpm <= 0) ||
      (seg != BREATH_SEG_INSP))
  {
    g_snore_phase = 0.0f;
    return 0;
  }

  g_snore_phase += g_params.snore_hz * dt_s;
  if (g_snore_phase >= 1.0f) { g_snore_phase -= floorf(g_snore_phase); }

  const float env = (g_envelope > 1.0f) ? 1.0f : g_envelope;
  return (int32_t)lroundf((float)g_params.snore_rpm * env *
                          sinf(2.0f * (float)M_PI * g_snore_phase));
}

/* ===========================================================================
 * Slew limiting
 * ======================================================================== */

static float BreathSim_FallSlewLimit(void)
{
  /* Fall back to the whole-volt reading only if CM4 has not published mV
   * yet (older CM4 image against a newer CM7). Scaling the volts up keeps
   * the comparison meaningful rather than reading as 0 mV and disabling
   * the derate entirely. */
  uint32_t mv = (g_bus_voltage_mv != 0U)
                    ? g_bus_voltage_mv
                    : ((uint32_t)g_bus_voltage_v * 1000U);

  if (mv == 0U)
  {
    /* CM4 is not publishing a bus voltage at all, so this derate is blind.
     * Do not pretend the bus is healthy: fall back to the most conservative
     * limit and flag it, rather than allowing the fastest decel with no
     * protection behind it. That combination tripped OVER_VOLT once. */
    g_flags |= BREATH_FLAG_VBUS_DERATE;
    return BREATH_SLEW_FALL_MIN_RPM_PER_S;
  }

  if (mv <= BREATH_VBUS_DERATE_START_MV)
  {
    return BREATH_SLEW_FALL_RPM_PER_S;
  }
  if (mv >= BREATH_VBUS_DERATE_FULL_MV)
  {
    g_flags |= BREATH_FLAG_VBUS_DERATE;
    return BREATH_SLEW_FALL_MIN_RPM_PER_S;
  }

  g_flags |= BREATH_FLAG_VBUS_DERATE;
  const float span =
      (float)(BREATH_VBUS_DERATE_FULL_MV - BREATH_VBUS_DERATE_START_MV);
  const float k = (float)(mv - BREATH_VBUS_DERATE_START_MV) / span;
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

/**
  * @brief Stage the parameter overrides a script step carries.
  *
  * Restaging happens only when this step overrides something, or when the
  * previous one did and the base has to be put back. Doing it on every
  * step would silently revert a manual parameter change made while the
  * script runs; never doing it would let one step's override leak into
  * every step after it. Like any parameter change, these land at the next
  * breath boundary - so a step shorter than a breath may not show its
  * override at all.
  */
static void BreathSim_ScriptApplyStep(const BreathScriptStep_t *st)
{
  const uint8_t has = ((st->rate_bpm > 0.0f) || (st->tidal_ml > 0.0f) ||
                       (st->rpm_amplitude >= 0)) ? 1U : 0U;

  if ((has == 0U) && (g_script_overridden == 0U)) { return; }

  BreathSimParams_t p = g_script_base;
  if (st->rate_bpm > 0.0f)    { p.rate_bpm      = st->rate_bpm; }
  if (st->tidal_ml > 0.0f)    { p.tidal_ml      = st->tidal_ml; }
  if (st->rpm_amplitude >= 0) { p.rpm_amplitude = st->rpm_amplitude; }
  (void)BreathSim_SetParams(&p);

  g_script_overridden = has;
}

/** Begin running step @p idx. */
static void BreathSim_ScriptEnter(uint8_t idx)
{
  const BreathScriptStep_t *st = &g_script[idx];

  g_script_idx         = idx;
  g_script_rep_left    = (st->repeat > 0U) ? st->repeat : 1U;
  g_script_remaining_s = st->duration_s;
  BreathSim_SetActiveEvent(st->event, st->duration_s, st->severity);
  BreathSim_ScriptApplyStep(st);

  DBG_I("SIM", "script step %u: %s sev=%d%% for %d s x%u",
        (unsigned)idx, BreathSim_EventName(st->event),
        (int)(st->severity * 100.0f), (int)st->duration_s,
        (unsigned)g_script_rep_left);
}

static void BreathSim_ScriptAdvance(void)
{
  if (g_script_running == 0U) { return; }

  /* Another pass of the same step before moving on. */
  if (g_script_rep_left > 1U)
  {
    g_script_rep_left--;
    g_script_remaining_s = g_script[g_script_idx].duration_s;
    BreathSim_SetActiveEvent(g_script[g_script_idx].event,
                             g_script[g_script_idx].duration_s,
                             g_script[g_script_idx].severity);
    return;
  }

  if (((uint16_t)g_script_idx + 1U) >= (uint16_t)g_script_len)
  {
    /* End of a pass. g_script_passes_left == 0 means run for ever. */
    if (g_script_passes_left > 0U)
    {
      g_script_passes_left--;
      if (g_script_passes_left == 0U)
      {
        g_script_running = 0U;
        g_script_idx = 0xFFU;
        BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);
        /* Put back whatever the last step overrode, or the run ends on a
         * step's pattern rather than the one the user configured. */
        if (g_script_overridden != 0U)
        {
          (void)BreathSim_SetParams(&g_script_base);
          g_script_overridden = 0U;
        }
        DBG_I("SIM", "script complete");
        return;
      }
    }
    BreathSim_ScriptEnter(0U);
    return;
  }

  BreathSim_ScriptEnter((uint8_t)(g_script_idx + 1U));
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
  const uint8_t prev_event = g_event_applied;

  g_amp_scale    = 1.0f;
  g_flat_eff     = g_params.flattening;
  g_fl_shape_eff = g_params.fl_shape;

  /* This instant, not the timer expiry, is when an event begins shaping
   * breaths - and so the only honest place to timestamp its start. */
  g_event_applied     = g_event;
  g_event_sev_applied = g_event_severity;

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
       * exact volume matters.
       *
       * OPEN LOOP ONLY. In flow mode the target peak is already recomputed
       * from the flattened shape's own area (g_shape_area, latched below),
       * which holds Vt by itself - applying this as well scaled the breath
       * by mean_base/mean_flat a SECOND time. At severity 1.0 that is
       * 0.637/0.952 = 0.67: ~335 mL instead of 500, and a plateau at ~45%
       * of the normal breath's peak, which an APAP's hypopnea rule (peak
       * < 50% of baseline) scores as a hypopnea rather than flow
       * limitation. Found 2026-10-12 while setting up the first AutoSet
       * trial; the earlier flattening tests set `flattening` directly and
       * never went through this event path. */
      if (g_params.control_mode != (uint8_t)BREATH_CTRL_FLOW)
      {
        const float mean_flat = BreathSim_EnvelopeMean(&g_params, g_flat_eff,
                                                      g_fl_shape_eff);
        const float mean_base = BreathSim_EnvelopeMean(&g_params, 0.0f,
                                                      (uint8_t)BREATH_FL_PLATEAU);
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

  /* Post-event recovery (arousal) breaths.
   *
   * A real apnea or hypopnea does not end into a normal breath; it ends in
   * an arousal, with two or three large ones. That overshoot is part of
   * what an APAP's baseline tracker responds to, and leaving it out makes
   * every scripted event end in a way no patient produces.
   *
   * Counted in breaths rather than seconds because that is what it is -
   * the existing event timer cannot express "three breaths", and rounding
   * it to a duration would drift against the (jittered) breath period.
   *
   * These are marked BREATH_EVENT_RECOVERY rather than left unlabelled, so
   * a scoring script can tell an arousal breath from a normal one instead
   * of inferring it from position. */
  if ((g_params.recovery_breaths > 0U) &&
      (g_event == (uint8_t)BREATH_EVENT_NONE) &&
      ((prev_event == (uint8_t)BREATH_EVENT_APNEA) ||
       (prev_event == (uint8_t)BREATH_EVENT_HYPOPNEA) ||
       (prev_event == (uint8_t)BREATH_EVENT_CSR)))
  {
    g_recovery_left = g_params.recovery_breaths;
  }

  if (g_event != (uint8_t)BREATH_EVENT_NONE)
  {
    /* A new event interrupts recovery - the airway is obstructed again,
     * and finishing the old arousal on top of it would be nonsense. */
    g_recovery_left = 0U;
  }
  else if (g_recovery_left > 0U)
  {
    g_amp_scale        *= g_params.recovery_gain;
    g_event_applied     = (uint8_t)BREATH_EVENT_RECOVERY;
    g_event_sev_applied = 0.0f;
    g_recovery_left--;
  }
  else
  {
    /* no recovery owed */
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

/** Begin a ramp-down from g_target_rpm before STOP. Only while the drive is
 *  in RUN and healthy; returns false if the caller must stop at once. */
static bool BreathSim_BeginStopRamp(const char *what)
{
  if ((g_mc_state != MC_STATE_RUN) || (g_current_faults != 0U) ||
      (g_have_posted == 0U) || (g_target_rpm <= BREATH_STOP_RAMP_END_RPM))
  {
    return false;
  }
  const float span = (float)(g_target_rpm - BREATH_STOP_RAMP_END_RPM);
  g_stop_ramp_left_s = (span / BREATH_SLEW_FALL_MIN_RPM_PER_S) +
                       BREATH_STOP_RAMP_SLACK_S;
  g_stop_ramping = 1U;   /* last: the breath task may run the ramp from here */
  DBG_W("SIM", "%s: ramping %ld -> %d rpm before stop",
        what, (long)g_target_rpm, (int)BREATH_STOP_RAMP_END_RPM);
  return true;
}

/** Abort the run. @p soft_stop asks for a ramp-down before STOP; it is only
 *  honoured while the drive is in RUN and healthy - a motor fault, or a drive
 *  that never got to RUN, is stopped at once. */
static void BreathSim_EnterFault(const char *why, bool soft_stop)
{
  const bool was_ramping = (g_stop_ramping != 0U);

  g_stop_ramping = 0U;
  if (was_ramping)
  {
    /* A fault arriving during the ramp: the ramp is void, stop now. */
    DBG_E("SIM", "stop ramp cut short: %s (mc=%u cur=0x%04X rpm_act=%ld)",
          why, (unsigned)g_mc_state, (unsigned)g_current_faults, (long)g_rpm_act);
    (void)BlowerIpc_CM7_Stop();
    g_have_posted = 0U;
    return;
  }

  g_flags |= BREATH_FLAG_MOTOR_FAULT;
  g_state = BREATH_STATE_FAULT;
  /* Log occurred_faults, and log it BEFORE the stop below. Aborting sends a
   * STOP, the CM4 handler acknowledges in FAULT_OVER, and the drive is back
   * in IDLE by the time anyone looks - so mc_state on its own reads 0 and
   * says nothing about what happened. */
  DBG_E("SIM", "run aborted: %s (mc=%u faults=%s cur=0x%04X occ=0x%04X)",
        why, (unsigned)g_mc_state,
        BreathSim_McFaultName(g_occurred_faults),
        (unsigned)g_current_faults, (unsigned)g_occurred_faults);
  DBG_E("SIM", "  at rpm_cmd=%ld rpm_act=%ld vbus=%lu mV flags=0x%04X",
        (long)g_target_rpm, (long)g_rpm_act,
        (unsigned long)g_bus_voltage_mv, (unsigned)g_flags);

  if (soft_stop && BreathSim_BeginStopRamp("abort"))
  {
    return;
  }

  (void)BlowerIpc_CM7_Stop();
  g_have_posted = 0U;
}

/** One tick of the stop ramp-down; posts STOP once the rotor is down. */
static void BreathSim_StopRampStep(float dt_s)
{
  const int32_t max_drop = (int32_t)lroundf(BreathSim_FallSlewLimit() * dt_s);
  int32_t next = g_target_rpm - max_drop;

  if (next < BREATH_STOP_RAMP_END_RPM) { next = BREATH_STOP_RAMP_END_RPM; }
  g_target_rpm = next;
  BreathSim_PostRpm(next);

  g_stop_ramp_left_s -= dt_s;
  const bool down = (next == BREATH_STOP_RAMP_END_RPM) &&
                    (g_rpm_act <= (BREATH_STOP_RAMP_END_RPM + BREATH_STOP_RAMP_TOL_RPM));
  if (down || (g_stop_ramp_left_s <= 0.0f))
  {
    if (!down)
    {
      DBG_W("SIM", "stop ramp timed out: cmd=%ld act=%ld, stopping",
            (long)g_target_rpm, (long)g_rpm_act);
    }
    else
    {
      DBG_I("SIM", "stop ramp done at rpm_act=%ld, stopping", (long)g_rpm_act);
    }
    g_stop_ramping = 0U;
    (void)BlowerIpc_CM7_Stop();
    g_have_posted = 0U;
    if (g_state == BREATH_STATE_IDLE) { g_target_rpm = g_params.rpm_base; }
  }
}

/** Read CM4 status and decide whether the run is still valid. */
static void BreathSim_Supervise(float dt_s)
{
  BlowerIpcStatus_t st;

  if (!BlowerIpc_CM7_GetStatus(&st)) { return; }

  g_mc_state           = st.mc_state;
  g_rpm_act            = st.mech_speed_rpm;
  g_bus_voltage_v      = st.bus_voltage_v;
  g_restart_holdoff_ms = st.restart_holdoff_ms;
  g_bus_voltage_mv     = st.bus_voltage_mv;
  g_occurred_faults    = st.occurred_faults;
  g_current_faults     = st.current_faults;

  if ((g_state == BREATH_STATE_PRIMING) && (g_prime_t_s < BREATH_FAULT_GRACE_S))
  {
    return;   /* still settling after the fault acknowledge in Start() */
  }

  if ((st.mc_state == MC_STATE_FAULT_NOW) || (st.mc_state == MC_STATE_FAULT_OVER) ||
      (st.current_faults != 0U))
  {
    BreathSim_EnterFault("motor fault", false);
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
  s.mc_occurred_faults = g_occurred_faults;

  s.control_mode   = g_params.control_mode;
  s.flow_valid     = BreathSim_FlowUsable();
  s.q_target_lpm   = g_q_target_lpm;
  s.q_meas_lpm     = g_q_meas_lpm;
  s.q_peak_lpm     = g_q_peak_lpm;
  s.volume_ml      = g_volume_ml;
  s.ctrl_integ_rpm = g_ctrl_integ_rpm;
  s.ff_mode          = g_params.ff_mode;
  s.ff_p_inlet_valid = g_ff_p_inlet_valid;
  s.ff_rpm           = g_ff_rpm;
  s.ff_p_inlet       = g_ff_p_inlet;
  s.last_tidal_ml  = g_last_tidal_ml;
  s.p_cmh2o        = g_p_cmh2o;
  s.p_min_cmh2o    = g_p_min_last;
  s.p_max_cmh2o    = g_p_max_last;
  s.p_mean_cmh2o   = g_p_mean_last;
  /* --- ground truth for the breath just completed --- */
  s.last_insp_s         = g_last_insp_s;
  s.last_q_peak_lpm     = g_last_q_peak;
  s.last_flat_index     = g_last_flat_idx;
  s.event_applied       = g_event_applied;
  s.last_event          = g_last_event;
  s.event_severity      = g_event_sev_applied;
  s.last_event_severity = g_last_event_sev;

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
  g_params.fl_shape      = (uint8_t)BREATH_FL_PLATEAU;
  g_params.snore_hz      = 0.0f;       /* off */
  g_params.snore_rpm     = 0;
  g_params.recovery_breaths = 0U;      /* off: preserves existing behaviour */
  g_params.recovery_gain    = 1.5f;
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
  g_params.ff_mode       = (uint8_t)BREATH_FF_STATIC;   /* until proven */
  g_params.ff_a          = BREATH_FF_A_DEFAULT;
  g_params.ff_b          = BREATH_FF_B_DEFAULT;
  g_params.ff_c          = BREATH_FF_C_DEFAULT;
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
  g_holdoff_logged  = 0U;
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
  g_fl_shape_eff    = (uint8_t)BREATH_FL_PLATEAU;

  g_event             = (uint8_t)BREATH_EVENT_NONE;
  g_event_severity    = 0.0f;
  g_event_remaining_s = 0.0f;
  g_event_applied     = (uint8_t)BREATH_EVENT_NONE;
  g_event_sev_applied = 0.0f;
  g_last_event        = (uint8_t)BREATH_EVENT_NONE;
  g_last_event_sev    = 0.0f;
  g_recovery_left     = 0U;
  g_csr_t_s           = 0.0f;

  g_script_len         = 0U;
  g_script_idx         = 0xFFU;
  g_script_running     = 0U;
  g_script_remaining_s = 0.0f;
  g_script_rep_left    = 0U;
  g_script_passes_left = 0U;
  g_script_overridden  = 0U;

  g_snore_phase = 0.0f;
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
  g_ff_rpm         = 0.0f;
  g_ff_p_inlet     = 0.0f;
  g_ff_p_inlet_valid = 0U;
  g_ff_pacc_sum    = 0.0f;
  g_ff_pacc_n      = 0U;
  g_ff_pacc_qmax   = 0.0f;
  g_ff_pacc_qsum   = 0.0f;
  g_ff_no_est_breaths = 0U;
  g_ff_fallback    = 0U;
  g_sat_ticks      = 0U;
  g_insp_ticks     = 0U;
  g_q_target_peak  = 0.0f;
  g_q_meas_peak    = 0.0f;
  g_q_meas_sum     = 0.0f;
  g_q_meas_n       = 0U;
  g_no_response_strikes = 0U;
  g_ctrl_disabled  = 0U;
  g_volume_ml      = 0.0f;
  g_last_tidal_ml  = 0.0f;
  g_last_insp_s    = 0.0f;
  g_last_q_peak    = 0.0f;
  g_last_flat_idx  = 0.0f;
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
  g_p_invalid_s     = 0.0f;
  g_stop_ramping    = 0U;
  g_amp_scale       = 1.0f;
  g_flat_eff        = g_params.flattening;
  g_fl_shape_eff    = g_params.fl_shape;
  g_event_applied     = (uint8_t)BREATH_EVENT_NONE;
  g_event_sev_applied = 0.0f;
  g_last_event        = (uint8_t)BREATH_EVENT_NONE;
  g_last_event_sev    = 0.0f;
  g_recovery_left     = 0U;
  g_csr_t_s         = 0.0f;
  g_snore_phase     = 0.0f;
  g_rng             = (g_params.jitter_seed != 0U) ? g_params.jitter_seed : 0x1234ABCDU;
  g_target_rpm      = g_params.rpm_base;
  g_have_posted     = 0U;
  g_ctrl_integ_rpm  = 0.0f;
  /* A new run may have a new CPAP pressure behind it: estimate afresh. */
  g_ff_rpm          = 0.0f;
  g_ff_p_inlet      = 0.0f;
  g_ff_p_inlet_valid = 0U;
  g_ff_pacc_sum     = 0.0f;
  g_ff_pacc_n       = 0U;
  g_ff_pacc_qmax    = 0.0f;
  g_ff_pacc_qsum    = 0.0f;
  g_ff_no_est_breaths = 0U;
  g_ff_fallback     = 0U;
  g_sat_ticks       = 0U;
  g_insp_ticks      = 0U;
  g_q_target_peak   = 0.0f;
  g_q_meas_peak     = 0.0f;
  g_q_meas_sum      = 0.0f;
  g_q_meas_n        = 0U;
  g_no_response_strikes = 0U;
  g_ctrl_disabled   = 0U;
  g_volume_ml       = 0.0f;
  g_last_tidal_ml   = 0.0f;
  g_last_insp_s     = 0.0f;
  g_last_q_peak     = 0.0f;
  g_last_flat_idx   = 0.0f;
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

  BreathSim_ScriptStop();
  BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);

  /* Ramp down from the speed the run was at (an abort ramp already under way
   * just carries on). Only a drive that is not in RUN is stopped at once,
   * from baseline as before. */
  if (was_active && (g_stop_ramping == 0U) && !BreathSim_BeginStopRamp("stop"))
  {
    g_target_rpm = g_params.rpm_base;
    (void)BlowerIpc_CM7_SetSpeedRpmSilent(g_target_rpm);
    (void)BlowerIpc_CM7_Stop();
  }
  if (was_active)
  {
    DBG_I("SIM", "stop: %lu breaths, flags=0x%04X",
          (unsigned long)g_breath_index, (unsigned)g_flags);
  }
  if (g_stop_ramping == 0U)
  {
    g_target_rpm  = g_params.rpm_base;
    g_have_posted = 0U;
  }

  BreathSim_PublishStatus();
}

bool BreathSim_IsRunning(void)
{
  return (g_state == BREATH_STATE_PRIMING) || (g_state == BREATH_STATE_RUNNING);
}

void BreathSim_Update(float dt_s)
{
  if (dt_s <= 0.0f) { dt_s = 0.005f; }

  if (g_state == BREATH_STATE_IDLE)
  {
    if (g_stop_ramping != 0U)
    {
      BreathSim_Supervise(dt_s);   /* a fault here cuts the ramp short */
      if (g_stop_ramping != 0U) { BreathSim_StopRampStep(dt_s); }
      BreathSim_PublishStatus();
    }
    return;
  }

  BreathSim_Supervise(dt_s);

  if (g_state == BREATH_STATE_FAULT)
  {
    if (g_stop_ramping != 0U) { BreathSim_StopRampStep(dt_s); }
    BreathSim_PublishStatus();
    return;
  }

  /* ---- priming: the CM4 rev-up takes ~4.6 s, during which no breath the
   * generator produces would be delivered. Wait for the motor to be in RUN
   * and sitting at baseline before the first breath, so the run does not
   * begin with several breaths of garbage. ---------------------------- */
  if (g_state == BREATH_STATE_PRIMING)
  {
    /* CM4 refuses to start while the rotor is still coasting down from the
     * previous run, and publishes the remaining wait. That is deliberate -
     * restarting into a spinning rotor gives a "false RUN" with no pressure
     * and, after an over-voltage trip, repeated DP_FAULTs.
     *
     * It is up to ~9 s from full speed, which is longer than
     * BREATH_PRIME_TIMEOUT_S on its own, so charging it against the prime
     * timeout would abort every restart as "motor never reached RUN". Wait
     * it out without counting it, and say so once. */
    if (g_restart_holdoff_ms > 0U)
    {
      if (g_holdoff_logged == 0U)
      {
        g_holdoff_logged = 1U;
        DBG_I("SIM", "waiting %u ms for rotor coast-down before start",
              (unsigned)g_restart_holdoff_ms);
      }
      g_target_rpm = g_params.rpm_base;
      BreathSim_PostRpm(g_target_rpm);
      BreathSim_PublishStatus();
      return;
    }

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
        BreathSim_EnterFault("motor never reached RUN", false);
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
    /* Ti and the applied event must be read BEFORE the parameter swap and
     * the re-latch at the end of this block, both of which overwrite them
     * for the breath about to start. */
    g_last_insp_s     = g_active.insp_s;
    g_last_event      = g_event_applied;
    g_last_event_sev  = g_event_sev_applied;
    g_last_q_peak     = g_q_meas_peak;
    g_last_flat_idx   = ((g_q_meas_n > 0U) && (g_q_meas_peak > 1.0e-3f))
                        ? ((g_q_meas_sum / (float)g_q_meas_n) / g_q_meas_peak)
                        : 0.0f;
    if (g_p_count > 0U)
    {
      g_p_min_last  = g_p_min_acc;
      g_p_max_last  = g_p_max_acc;
      g_p_mean_last = g_p_sum / (float)g_p_count;
    }
    if ((g_params.control_mode == (uint8_t)BREATH_CTRL_FLOW) &&
        (g_amp_scale > 0.5f))
    {
      /* Against what the breath was actually ASKED for, not the nominal
       * tidal volume: g_amp_scale still holds the scale for the breath
       * that just finished, and a jittered-down or recovery breath has a
       * target of its own. Comparing to the nominal flagged a correctly
       * delivered 0.8x jitter breath as short, and would have let a
       * recovery breath that never got above nominal pass silently. */
      const float expect    = g_params.tidal_ml * g_amp_scale;
      const float shortfall = expect - g_last_tidal_ml;
      if (shortfall > (0.2f * expect))
      {
        g_flags |= BREATH_FLAG_VOLUME_SHORT;
      }
    }
    /* Anti-latch. Conditional integration stops the integrator GROWING while
     * saturated, but nothing brought it back down, and it is deliberately
     * frozen through expiration so it keeps the operating point it learned.
     * Against a target the blower cannot reach that combination is a trap:
     * the integrator parks at its clamp, and every subsequent inspiration
     * starts already pinned at the ceiling. The half-sine then degenerates
     * into a square pulse at maximum RPM - which is both the wrong waveform
     * and sustained operation at the very edge of where the drive is stable.
     *
     * So if a breath spent most of its inspiration on the rail, bleed the
     * integrator. A reachable target is unaffected (it never saturates for
     * long); an unreachable one settles at the best the drive can actually
     * do instead of welding itself to the limit. FLOW_SAT still reports it. */
    if ((g_insp_ticks > 0U) &&
        ((g_sat_ticks * 2U) > g_insp_ticks))   /* saturated >50% of inspiration */
    {
      g_ctrl_integ_rpm *= 0.85f;
    }

    /* Did the flow actually respond to the breath we commanded?
     *
     * If a real flow was asked for and almost none came back, closing a loop
     * around it is worse than useless: the error never falls, the integrator
     * walks to its clamp, and the blower is commanded to maximum RPM for
     * every inspiration from then on. That is the same command whether the
     * cause is a sensor mounted backwards, a sensor plumbed somewhere the
     * breath does not pass, or a blower that simply cannot make this flow -
     * and in all three cases it is the worst thing to do to the drive.
     *
     * Three consecutive unresponsive breaths and closed-loop is abandoned
     * for the rest of the run. The waveform continues open-loop so the rig
     * keeps working; the flag and the log say why. */
    if ((g_params.control_mode == (uint8_t)BREATH_CTRL_FLOW) &&
        (g_ctrl_disabled == 0U) && (g_q_target_peak > 5.0f))
    {
      if (g_q_meas_peak < (0.25f * g_q_target_peak))
      {
        g_no_response_strikes++;
        if (g_no_response_strikes >= 3U)
        {
          g_flags |= BREATH_FLAG_FLOW_NO_RESPONSE;
          g_ctrl_disabled = 1U;
          g_ctrl_integ_rpm = 0.0f;
          DBG_E("SIM", "flow never responded: asked %d L/min, saw %d L/min",
                (int)g_q_target_peak, (int)g_q_meas_peak);
          DBG_E("SIM", "  closed loop OFF. Check the sensor direction and"
                       " that the breath passes through it, then 'flow status'.");
        }
      }
      else
      {
        g_no_response_strikes = 0U;
      }
    }

    /* Inlet-pressure estimate for the feed-forward - see BREATH_FF_PREF_*.
     * Uses this breath's parameters (the swap below has not happened yet),
     * and runs in every mode so a mid-run switch to the pressure model has
     * an estimate waiting. Halved toward each new reading rather than
     * replaced, so one odd breath cannot jerk the whole next inspiration. */
    if ((g_ff_pacc_n > 0U) &&
        (fabsf(g_ff_pacc_qsum / (float)g_ff_pacc_n) <= BREATH_FF_PREF_MAX_FLOW_LPM))
    {
      const float nb   = (float)g_params.rpm_base * 0.001f;
      const float head = ((g_params.ff_a * nb * nb) + g_params.ff_c) /
                         g_params.ff_b;
      const float est  = (g_ff_pacc_sum / (float)g_ff_pacc_n) - head;
      g_ff_p_inlet = (g_ff_p_inlet_valid != 0U)
                     ? (0.5f * (g_ff_p_inlet + est)) : est;
      g_ff_p_inlet_valid = 1U;
      g_ff_no_est_breaths = 0U;
    }
    else if ((g_params.ff_mode == (uint8_t)BREATH_FF_PRESSURE) &&
             (g_ff_p_inlet_valid == 0U) &&
             (g_ff_no_est_breaths < 255U))
    {
      g_ff_no_est_breaths++;      /* see BREATH_FF_PREF_HOLD_BREATHS */
    }
    g_ff_pacc_sum  = 0.0f;
    g_ff_pacc_n    = 0U;
    g_ff_pacc_qmax = 0.0f;
    g_ff_pacc_qsum = 0.0f;

    g_q_target_peak = 0.0f;
    g_q_meas_peak   = 0.0f;
    g_q_meas_sum    = 0.0f;
    g_q_meas_n      = 0U;
    g_sat_ticks     = 0U;
    g_insp_ticks    = 0U;

    g_volume_ml = 0.0f;
    g_p_min_acc = 1.0e9f;
    g_p_max_acc = -1.0e9f;
    g_p_sum     = 0.0f;
    g_p_count   = 0U;

    /* Breath boundary: the one safe point to swap parameters. */
    if (g_params_dirty != 0U)
    {
      taskENTER_CRITICAL();
      /* A different feed-forward changes what the integrator is trimming.
       * Carried across a switch from static to pressure it would stack the
       * old 22900 rpm operating point on top of a ~27000 rpm model command;
       * across a coefficient change it holds a correction for a model that
       * no longer runs. Start the trim again from zero. */
      if ((g_params_pending.ff_mode != g_params.ff_mode) ||
          (g_params_pending.ff_a    != g_params.ff_a)    ||
          (g_params_pending.ff_b    != g_params.ff_b)    ||
          (g_params_pending.ff_c    != g_params.ff_c))
      {
        g_ctrl_integ_rpm = 0.0f;
      }
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
      BreathSim_EnterFault("airway pressure limit exceeded", true);
      BreathSim_PublishStatus();
      return;
    }
  }

  /* The limit above only runs on valid samples, so a frozen sensor switches
   * it off. 2026-10-30: the gauge held 32.4-33.2 cmH2O for several seconds
   * at a time, the pressure feed-forward fell back to the static path, and
   * when the sensor returned the lung was at 40.1 cmH2O. Whenever the run
   * depends on pressure - a limit is set, or the pressure feed-forward is
   * selected - stop instead of continuing blind. Brief holds (busy, torn
   * reads: a tick or two) are normal and stay well under the threshold;
   * SensorTask also attempts an SPI recovery after 200 ms (main.c). */
  if ((g_params.pressure_limit_cmh2o > 0.0f) ||
      (g_params.ff_mode == (uint8_t)BREATH_FF_PRESSURE))
  {
    g_p_invalid_s = (g_p_valid != 0U) ? 0.0f : (g_p_invalid_s + dt_s);
    if (g_p_invalid_s >= BREATH_P_INVALID_ABORT_S)
    {
      g_flags |= BREATH_FLAG_PRESSURE_INVALID;
      BreathSim_EnterFault("airway pressure invalid (sensor held / not responding)", true);
      BreathSim_PublishStatus();
      return;
    }
  }
  else
  {
    g_p_invalid_s = 0.0f;
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
  if (env > BREATH_ENV_MAX) { env = BREATH_ENV_MAX; }
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

  /* Measured inspiratory flow statistics, for the per-breath ground-truth
   * line. Kept here rather than in the flow controller so they exist in
   * open-loop mode too, and taken over BREATH_SEG_INSP only: an end-
   * inspiratory pause carries almost no flow and would drag the mean - and
   * with it the flattening index - down for a breath that was never
   * flattened at all. Expiratory samples are floored at zero rather than
   * skipped, so this stays a mean of the inspiratory waveform. */
  /* End-expiratory pressure, for the feed-forward's inlet estimate. */
  if (((seg == BREATH_SEG_EXP) || (seg == BREATH_SEG_EXP_PAUSE)) &&
      ((g_active.period_s - g_t_s) <= BREATH_FF_PREF_TAIL_S) &&
      (g_p_valid != 0U) && (BreathSim_FlowUsable() != 0U))
  {
    const float aq = fabsf(g_q_meas_lpm);
    g_ff_pacc_sum += g_p_cmh2o;
    g_ff_pacc_qsum += g_q_meas_lpm;
    g_ff_pacc_n++;
    if (aq > g_ff_pacc_qmax) { g_ff_pacc_qmax = aq; }
  }

  if ((seg == BREATH_SEG_INSP) && (BreathSim_FlowUsable() != 0U))
  {
    const float q = (g_q_meas_lpm > 0.0f) ? g_q_meas_lpm : 0.0f;
    if (q > g_q_meas_peak) { g_q_meas_peak = q; }
    g_q_meas_sum += q;
    g_q_meas_n++;
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

  /* Snore rides on top of the command without becoming part of it - see
   * BreathSim_SnoreOffset and the note on rpm_cmd in breath_sim.h. Note
   * that with it enabled the command differs every tick, so the redundant-
   * post suppression in BreathSim_PostRpm stops helping and the mailbox is
   * written at the full 200 Hz for the whole inspiratory limb. */
  const int32_t snore = BreathSim_SnoreOffset(dt_s, seg);
  BreathSim_PostRpm(clamp_i32(g_target_rpm + snore, 0, BREATH_RPM_ABS_MAX));
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

void BreathSim_ScriptStepInit(BreathScriptStep_t *step)
{
  if (step == NULL) { return; }
  memset(step, 0, sizeof(*step));
  step->event         = (uint8_t)BREATH_EVENT_NONE;
  step->repeat        = 1U;
  step->rpm_amplitude = -1;        /* 0 is a legal amplitude, so -1 inherits */
}

bool BreathSim_ScriptAppend(const BreathScriptStep_t *step)
{
  if (step == NULL) { return false; }
  if (g_script_len >= (uint8_t)BREATH_SCRIPT_MAX_STEPS) { return false; }
  if (step->event >= (uint8_t)BREATH_EVENT_COUNT) { return false; }
  /* RECOVERY is raised by the generator itself after an event ends.
   * Letting a script assert it directly would make the marker mean two
   * different things in the same log. */
  if (step->event == (uint8_t)BREATH_EVENT_RECOVERY) { return false; }
  if (step->duration_s <= 0.0f) { return false; }

  taskENTER_CRITICAL();
  g_script[g_script_len] = *step;
  g_script[g_script_len].severity = clampf(step->severity, 0.0f, 1.0f);
  if (g_script[g_script_len].repeat == 0U)
  {
    g_script[g_script_len].repeat = 1U;
  }
  g_script_len++;
  taskEXIT_CRITICAL();
  return true;
}

bool BreathSim_ScriptGet(uint8_t idx, BreathScriptStep_t *out)
{
  if ((out == NULL) || (idx >= g_script_len)) { return false; }
  taskENTER_CRITICAL();
  *out = g_script[idx];
  taskEXIT_CRITICAL();
  return true;
}

uint8_t BreathSim_ScriptLength(void)
{
  return g_script_len;
}

bool BreathSim_ScriptStart(uint16_t passes)
{
  if (g_script_len == 0U) { return false; }

  taskENTER_CRITICAL();
  g_script_running     = 1U;
  g_script_passes_left = passes;
  g_script_overridden  = 0U;
  g_script_base        = g_params;
  taskEXIT_CRITICAL();

  /* Outside that critical section: entering a step may stage a parameter
   * set, which takes a critical section of its own. */
  BreathSim_ScriptEnter(0U);

  DBG_I("SIM", "script start: %u steps, %u pass(es)",
        (unsigned)g_script_len, (unsigned)passes);
  return true;
}

void BreathSim_ScriptStop(void)
{
  taskENTER_CRITICAL();
  g_script_running = 0U;
  g_script_idx     = 0xFFU;
  /* Clear the step's event too. It was armed with the step duration, so
   * without this a stopped script leaves an apnea running for whatever is
   * left of it - and the ground-truth markers go on asserting an event
   * nobody asked for. */
  BreathSim_SetActiveEvent((uint8_t)BREATH_EVENT_NONE, 0.0f, 0.0f);
  taskEXIT_CRITICAL();

  /* Put back anything a step overrode, as the completion path does. */
  if (g_script_overridden != 0U)
  {
    (void)BreathSim_SetParams(&g_script_base);
    g_script_overridden = 0U;
  }
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

const char *BreathSim_McFaultName(uint16_t faults)
{
  /* Static buffer: this is only ever called from logging and the USB status
   * reply, both of which consume it immediately. */
  static char buf[96];
  size_t n = 0U;

  buf[0] = '\0';
  if (faults == 0U)
  {
    return "none";
  }

  static const struct { uint16_t bit; const char *name; } k_bits[] = {
    { 0x0001U, "DURATION"   },  /* FOC rate too high          */
    { 0x0002U, "OVER_VOLT"  },
    { 0x0004U, "UNDER_VOLT" },
    { 0x0008U, "OVER_TEMP"  },
    { 0x0010U, "START_UP"   },
    { 0x0020U, "SPEED_FDBK" },  /* observer lost the rotor    */
    { 0x0040U, "OVER_CURR"  },
    { 0x0080U, "SW_ERROR"   },
    { 0x0400U, "DP_FAULT"   },  /* STSPIN driver protection   */
  };

  for (size_t i = 0U; i < (sizeof(k_bits) / sizeof(k_bits[0])); i++)
  {
    if ((faults & k_bits[i].bit) == 0U) { continue; }
    const size_t len = strlen(k_bits[i].name);
    if ((n + len + 2U) >= sizeof(buf)) { break; }
    if (n > 0U) { buf[n++] = '|'; }
    (void)memcpy(&buf[n], k_bits[i].name, len);
    n += len;
    buf[n] = '\0';
  }
  return (n > 0U) ? buf : "unknown";
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
    case BREATH_EVENT_RECOVERY:   return "Recovery";
    case BREATH_EVENT_NONE:
    default:                      return "None";
  }
}

const char *BreathSim_FfModeName(uint8_t mode)
{
  return (mode == (uint8_t)BREATH_FF_PRESSURE) ? "pressure" : "static";
}

const char *BreathSim_FlShapeName(uint8_t shape)
{
  switch ((BreathFlShape_t)shape)
  {
    case BREATH_FL_CHAIR: return "chair";
    case BREATH_FL_M:     return "m";
    case BREATH_FL_LATE:  return "late";
    case BREATH_FL_PLATEAU:
    default:              return "plateau";
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
