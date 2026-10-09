/**
  * @file    breath_sim.h
  * @brief   Open-loop blower RPM waveform generator for breath simulation.
  *
  * ---------------------------------------------------------------------------
  * Pneumatic model
  * ---------------------------------------------------------------------------
  * The rig is plumbed in series:
  *
  *   CPAP/APAP --[22 mm, 1.8 m hose]--> collapsible tube chamber (upper airway)
  *             --> BREATH SIMULATOR BOX (this blower) --> test lung
  *
  * The blower therefore acts as the *respiratory muscle*, not as a gas source:
  *
  *   - INSPIRATION is active. Blower RPM rises, drawing gas from the CPAP
  *     through the airway chamber and pushing it into the test lung. The
  *     commanded envelope is an inspiratory *flow* shape, so it starts at
  *     zero, peaks mid-inspiration and returns to zero at end-inspiration.
  *
  *   - EXPIRATION is passive. The test lung's elastic recoil drives gas back
  *     out through the (now slow) blower toward the CPAP. Firmware does not
  *     command an expiratory flow; it commands the blower to *get out of the
  *     way*, decaying RPM exponentially to baseline with @ref exp_tau_s.
  *     The delivered expiratory waveform is set by the physical R*C of the
  *     lung, the blower's backflow resistance and the airway chamber.
  *
  * Consequence: @ref rpm_base should be as LOW as the motor tolerates. Any
  * baseline RPM actively pushes into the lung during expiration and impedes
  * recoil. See BREATH_RPM_OBS_MIN_RPM for the motor-side limit on how low
  * the baseline can usefully go.
  *
  * ---------------------------------------------------------------------------
  * Timing model
  * ---------------------------------------------------------------------------
  * Breath rate, inspiratory time and I:E ratio are mutually over-determined.
  * Rather than silently coercing the user's input, the caller picks which
  * pair is authoritative via @ref BreathTimingMode_t, and the resolved
  * timing is always readable through BreathSim_GetTiming().
  *
  * ---------------------------------------------------------------------------
  * Control modes
  * ---------------------------------------------------------------------------
  * BREATH_CTRL_OPEN_LOOP_RPM is the original behaviour: the envelope scales
  * RPM directly. Delivered flow then depends on the load, so tidal volume is
  * unknown and varies with the pressure the device under test happens to be
  * holding. Useful for blower characterisation, not for quantitative tests.
  *
  * BREATH_CTRL_FLOW closes the loop on the SFM3300: the envelope becomes a
  * target inspiratory flow, and a feed-forward + PI controller finds the RPM
  * that delivers it. Breaths are then specified the way they are clinically -
  * tidal volume in mL, rate, I:E - and the delivered volume is measured
  * rather than assumed.
  *
  * Flow control, not pressure control, is the right choice here. The CPAP
  * under test is itself a pressure regulator; a pressure-controlled simulator
  * would put two pressure loops on the same node fighting each other, and the
  * result would say more about which controller won than about the device. A
  * flow source is also what a patient actually is. Pressure is therefore a
  * measurement, not a controlled variable: it is the device's response, and
  * the per-breath pressure statistics are the test result.
  *
  * Only inspiration is regulated. Expiration stays passive (see above), so
  * the controller is disabled and the command decays to baseline - there is
  * nothing to regulate when the lung, not the blower, is moving the gas.
  */

#ifndef BREATH_SIM_H
#define BREATH_SIM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * Limits — single source of truth. The USB command parser and the TouchGFX
 * settings editor must both validate against these, never against their own
 * copies (they used to, and the three copies drifted).
 * ======================================================================== */

#define BREATH_RATE_MIN_BPM             4.0f
#define BREATH_RATE_MAX_BPM            60.0f
#define BREATH_INSP_MIN_S               0.3f
#define BREATH_INSP_MAX_S               4.0f
#define BREATH_IE_RATIO_MIN             1.0f
#define BREATH_IE_RATIO_MAX             4.0f
#define BREATH_INSP_PAUSE_MAX_S         1.0f
#define BREATH_EXP_PAUSE_MAX_S          2.0f
#define BREATH_EXP_TAU_MIN_S            0.10f
#define BREATH_EXP_TAU_MAX_S            2.00f
#define BREATH_FLATTENING_MAX           1.0f
#define BREATH_JITTER_MAX_PCT          25.0f

#define BREATH_RPM_BASE_MIN              2000
/*
 * 20000 made sense while rpm_base only ever meant "the floor between
 * breaths". It also caps constant-speed running (amplitude 0), which is how
 * the blower is characterised against a calibrated orifice and how a CPAP
 * actually runs one - both want the full speed range. The binding limit is
 * still rpm_base + rpm_amplitude <= BREATH_RPM_ABS_MAX, enforced in
 * BreathSim_Resolve, so this does not let anything exceed the drive ceiling.
 *
 * Note the consequence for breathing use: a high base leaves little room for
 * amplitude, and pushes hard into the lung throughout expiration.
 */
#define BREATH_RPM_BASE_MAX             BREATH_RPM_ABS_MAX
#define BREATH_RPM_BASE_DEFAULT         14500
/*
 * Amplitude is NOT independently limited any more. The binding constraint is
 * rpm_base + rpm_amplitude <= BREATH_RPM_ABS_MAX, enforced in
 * BreathSim_Resolve, and a second lower cap here only ever hid that.
 *
 * At the previous 25000 it was not even reachable: with rpm_base at its
 * 14500 default the sum clamp already capped amplitude at 21500, so raising
 * this alone did nothing. It is now set to the largest amplitude the sum
 * clamp can ever allow (ABS_MAX - BASE_MIN), which keeps it honest: ask for
 * more than the drive can give and Resolve reduces it and says so, rather
 * than an arbitrary number deciding silently.
 */
#define BREATH_RPM_AMP_MAX              (BREATH_RPM_ABS_MAX - BREATH_RPM_BASE_MIN)
/**
  * Absolute ceiling. Must track MAX_APPLICATION_SPEED_RPM on CM4 (38500) and
  * BLOWER_IPC_MAX_SPEED_RPM; commanding above it just saturates on CM4 and
  * makes the mailbox disagree with the motor. Raised from 36000 on
  * 2026-09-28 - see the note in CM4/Core/Inc/drive_parameters.h for why
  * 38500 and what it costs in regen.
  */
#define BREATH_RPM_ABS_MAX              38500

/**
  * Speed below which a commanded RPM raises BREATH_FLAG_BELOW_OBS_MIN.
  * Deliberately NOT a clamp - it is a warning, so the real floor could be
  * measured by simply commanding below it.
  *
  * It was CM4's OBS_MINIMUM_SPEED_RPM (12965), on the assumption that the ST
  * profiler's figure was the hardware limit. It is not. Stepping rpm_base
  * down at constant speed on 2026-09-28, the drive held cleanly to 6500 rpm
  * - the lowest tested - with no MOTOR_FAULT and no MC_SPEED_FDBK at any
  * step. The profiler's value is roughly 2x conservative, and no observer
  * retune (GAIN1/GAIN2/PLL) is needed to run well below it.
  *
  * 7000 here so the flag means something again. CM4's OBS_MINIMUM_SPEED_RPM
  * is deliberately left alone: it gates the rev-up switch-over, so lowering
  * it changes STARTING behaviour, which is a separate question from how low
  * the drive can hold once it is running.
  *
  * Worth knowing before lowering rpm_base to exploit this: peak speed is
  * base + swing, and the swing is limited by motor torque against impeller
  * inertia to roughly 24000 rpm over a 1.667 s inspiration. Lowering the
  * base therefore costs peak speed unless the inspiratory time grows with
  * it - at Ti 3.0 s (ISO 10 BPM) a 30500 rpm swing tracks cleanly, at
  * 1.667 s it does not and TRACK_ERROR is set.
  */
#define BREATH_RPM_OBS_MIN_RPM           7000

/** Fraction of the cycle inspiration may occupy (physiologic sanity cap). */
#define BREATH_MAX_INSP_DUTY            0.60f

/* Closed-loop flow control limits. */
#define BREATH_TIDAL_MIN_ML             50.0f
#define BREATH_TIDAL_MAX_ML           2000.0f
#define BREATH_TIDAL_DEFAULT_ML        500.0f
#define BREATH_FLOW_KP_MAX             500.0f   /**< RPM per (L/min) */
#define BREATH_FLOW_KI_MAX            5000.0f   /**< RPM per (L/min) per s */
#define BREATH_FLOW_FF_MAX            2000.0f   /**< RPM per (L/min) */
/** Peak inspiratory flow the rig will ask for, whatever the tidal volume. */
#define BREATH_Q_PEAK_MAX_LPM          200.0f
/** Abort the run above this airway pressure. 0 disables the check. */
#define BREATH_PRESSURE_LIMIT_MAX_CMH2O 80.0f
/** A flow sample older than this is not usable as feedback. */
#define BREATH_FLOW_STALE_MS              50U

/* ---------------------------------------------------------------------------
 * Pressure-compensated flow feed-forward (BREATH_FF_PRESSURE)
 * ---------------------------------------------------------------------------
 * Plant model fitted on the bench 2026-10-09 from 3086 inspiratory samples
 * (pure feed-forward runs, 2 L lung, CPAP off, 12 bpm I:E 1:1):
 *
 *     q = a*N^2 - b*(P - P_inlet) + c       N in krpm, P in cmH2O, q in slm
 *
 * R^2 0.80, against 0.31 for a straight flow-vs-rpm line. Lung pressure is
 * the input a static flow_ff cannot see, and it is why the integrator had to
 * carry a 22900 rpm constant: holding a target flow needs MORE speed as the
 * lung fills, so a flat-topped breath needs rpm rising through inspiration -
 * the opposite of the half-sine a static slope commands. Inverted, the model
 * gives the rpm for the target flow at the pressure the lung is at right now.
 *
 * Fitted with the CPAP OFF, so P_inlet was atmosphere. With the CPAP on the
 * blower inlet sits at CPAP pressure, and absolute P in this model would
 * over-command badly, so P_inlet is estimated every breath (BREATH_FF_PREF_*).
 * That estimate and these coefficients are UNVERIFIED with the CPAP running:
 * refit before trusting this in an APAP run. All three are settable at
 * runtime (sim set ff_a / ff_b / ff_c) so a refit needs no reflash.
 */
#define BREATH_FF_A_DEFAULT            0.0550f  /**< slm per krpm^2 */
#define BREATH_FF_B_DEFAULT            2.812f   /**< slm per cmH2O */
#define BREATH_FF_C_DEFAULT            0.26f    /**< slm */
#define BREATH_FF_A_MIN                0.005f
#define BREATH_FF_A_MAX                0.5f
#define BREATH_FF_B_MIN                0.1f     /**< also guards a divide */
#define BREATH_FF_B_MAX               10.0f
#define BREATH_FF_C_MAX               20.0f
/**
  * Inlet-pressure estimate. At the end of expiration the lung has stopped
  * moving (q ~ 0) and the blower is back at rpm_base, so the model gives
  *     P_inlet = P_end - (a*Nb^2 + c) / b
  * P_end is averaged over the last BREATH_FF_PREF_TAIL_S of expiration, and
  * the estimate is only taken if no flow above BREATH_FF_PREF_MAX_FLOW_LPM
  * moved in that window: a lung still emptying breaks the q ~ 0 premise, and
  * the previous estimate is kept instead.
  */
#define BREATH_FF_PREF_TAIL_S          0.30f
#define BREATH_FF_PREF_MAX_FLOW_LPM    3.0f

/**
  * Depth of the flow-limitation shape variants, as a fraction of the
  * plateau at full severity. TILT is how far the chair/late-peak shapes
  * slope across inspiration; DIP is how deep the mid-inspiratory notch of
  * the M shape cuts. Both scale with severity, so at zero flattening every
  * variant collapses back to the unflattened waveform.
  */
#define BREATH_FL_TILT                  0.35f
#define BREATH_FL_DIP                   0.45f

/**
  * Snore: a high-frequency modulation of the blower command through
  * inspiration, so the device under test sees ripple on the inspiratory
  * flow in the band its snore detector listens to (above 20 Hz).
  *
  * Read this before trusting a snore result. The modulation is COMMANDED,
  * and there is good reason to doubt the rig reproduces it:
  *
  *  - The breath task runs at 200 Hz, so 60 Hz is 3.3 samples per cycle.
  *    What reaches the motor is a coarse staircase, not a sine.
  *  - The impeller carries 14 g/cm2 of inertia (blower datasheet). Its
  *    mechanical response rolls off far below 30 Hz, so most of the
  *    commanded ripple will never appear as flow.
  *  - NOTHING on the rig can measure 60 Hz. The simulator's SFM3300 and
  *    the external sensor at the patient port both sample at ~100 Hz, so
  *    both stop at a ~50 Hz Nyquist. Test in the 30-45 Hz band, where the
  *    external sensor can actually see the ripple, and read the device's
  *    own snore metric (residual above 20 Hz against a 0.35 L/min
  *    threshold) - that detector is the thing that has to fire, and it is
  *    the only check that does not depend on the rig resolving the
  *    modulation itself.
  *
  * The range below still reaches 60 Hz because that is a real snore
  * frequency and the limit belongs to the instrumentation, not to the
  * stimulus - but a run above ~45 Hz cannot be verified here.
  *
  * The useful outcome of a snore run is therefore a measurement of how far
  * short the blower falls, not a detection. Treat a negative result as a
  * fact about the actuator, not about the detector.
  */
#define BREATH_SNORE_HZ_MIN            10.0f
#define BREATH_SNORE_HZ_MAX            60.0f
#define BREATH_SNORE_RPM_MAX             3000

/* Post-event recovery ("arousal") breaths. */
#define BREATH_RECOVERY_BREATHS_MAX        5
#define BREATH_RECOVERY_GAIN_MIN        1.0f
#define BREATH_RECOVERY_GAIN_MAX        3.0f

/**
  * Ceiling on the amplitude-scaled inspiratory envelope.
  *
  * Deliberately NOT 1.0. A recovery breath and a positive jitter draw both
  * scale demand above the nominal peak, and clipping there does not merely
  * cap the breath - it flat-tops it, which is a flow-limited SHAPE, the
  * very thing the device under test is being asked to score. (Positive
  * jitter has been silently clipped this way all along, which also made
  * jitter asymmetric: it could shrink a breath but never grow one.)
  *
  * Nothing downstream relies on the envelope being <= 1: the open-loop
  * command is clamped to BREATH_RPM_ABS_MAX and the flow target to
  * BREATH_Q_PEAK_MAX_LPM, so those limits still bound what actually runs.
  */
#define BREATH_ENV_MAX                                                          (BREATH_RECOVERY_GAIN_MAX * (1.0f + (BREATH_JITTER_MAX_PCT * 0.01f)))

/** Arbitrary-waveform table size (normalised inspiratory flow, 0..1). */
#define BREATH_TABLE_POINTS             64U

/**
  * Maximum steps in a scripted test protocol.
  *
  * 64 rather than 16 so a realistic night - repeated obstructive cycles,
  * positional changes, a CSR bout - fits in one script instead of being
  * driven from the host, which would put PC scheduling jitter inside the
  * stimulus. At 24 bytes a step this costs about 1.5 kB of RAM.
  */
#define BREATH_SCRIPT_MAX_STEPS         64U
/** Times one step may repeat before the script moves on. */
#define BREATH_SCRIPT_REPEAT_MAX       255U

/* ===========================================================================
 * Enumerations
 * ======================================================================== */

typedef enum
{
  /** Half-sine inspiratory flow: 0 -> peak at mid-inspiration -> 0. */
  BREATH_WAVE_SINE = 0,
  /** Symmetric triangular inspiratory flow. */
  BREATH_WAVE_RAMP,
  /** Constant ("square wave") inspiratory flow. */
  BREATH_WAVE_SQUARE,
  /** Arbitrary normalised profile loaded via BreathSim_SetTable(). */
  BREATH_WAVE_TABLE,
  BREATH_WAVE_COUNT
} BreathWaveform_t;

/**
  * Inspiratory morphology of a flow-limited breath.
  *
  * All of these hold tidal volume: the peak flow needed for the target
  * volume is recomputed from the shape's own area every breath, so
  * changing the shape changes the MORPHOLOGY and not the dose. That is the
  * point - a breath that delivers more air than an unobstructed one is not
  * flow limited by any definition, and an APAP will rightly score nothing
  * for it.
  *
  * They differ in flattening index (inspiratory mean/peak), which is what
  * the device under test actually thresholds on, so the shape label is not
  * interchangeable with the index. The per-breath telemetry reports the
  * measured index for exactly this reason.
  */
typedef enum
{
  /** Symmetric plateau: the classic flattened limb. */
  BREATH_FL_PLATEAU = 0,
  /** Early peak falling to a lower plateau - partial airway give-way. */
  BREATH_FL_CHAIR,
  /** Mid-inspiratory dip; the double-humped "M" of a fluttering airway. */
  BREATH_FL_M,
  /** Peak at end-inspiration - the airway opens late. */
  BREATH_FL_LATE,
  BREATH_FL_COUNT
} BreathFlShape_t;

typedef enum
{
  /** rate_bpm + ie_ratio_exp authoritative; insp_time_s is derived. */
  BREATH_TIMING_RATE_IE = 0,
  /** insp_time_s + ie_ratio_exp authoritative; rate_bpm is derived. */
  BREATH_TIMING_EXPLICIT,
  BREATH_TIMING_COUNT
} BreathTimingMode_t;

typedef enum
{
  /** Envelope scales RPM directly. Delivered volume is unknown. */
  BREATH_CTRL_OPEN_LOOP_RPM = 0,
  /** Envelope becomes a target inspiratory flow, regulated against the
    * SFM3300. Breaths are specified by tidal volume. */
  BREATH_CTRL_FLOW,
  BREATH_CTRL_COUNT
} BreathControlMode_t;

/** Feed-forward used by BREATH_CTRL_FLOW. */
typedef enum
{
  /** rpm = rpm_base + flow_ff * q_target. flow_ff defaults to 0, so the
    * integrator carries the whole operating point. The original behaviour,
    * and the default until the pressure model is proven on the bench. */
  BREATH_FF_STATIC = 0,
  /** rpm from the bench-fitted plant model against the measured lung
    * pressure; the PI is left to trim the model's error. */
  BREATH_FF_PRESSURE,
  BREATH_FF_COUNT
} BreathFfMode_t;

typedef enum
{
  BREATH_SEG_INSP = 0,
  BREATH_SEG_INSP_PAUSE,
  BREATH_SEG_EXP,
  BREATH_SEG_EXP_PAUSE,
  BREATH_SEG_COUNT
} BreathSegment_t;

typedef enum
{
  BREATH_STATE_IDLE = 0,
  /** Motor revving up (CM4 open-loop rev-up is ~4.6 s); no valid breaths. */
  BREATH_STATE_PRIMING,
  BREATH_STATE_RUNNING,
  /** Motor faulted mid-run. Waveform generation halted, run marked invalid. */
  BREATH_STATE_FAULT,
  BREATH_STATE_COUNT
} BreathSimState_t;

typedef enum
{
  BREATH_EVENT_NONE = 0,
  /**
    * Zero inspiratory effort. This reproduces a CENTRAL apnea directly.
    * An OBSTRUCTIVE apnea additionally requires the collapsible airway
    * chamber to be closed, so that the device under test sees a high
    * airway impedance; that actuator is not driven by this module.
    */
  BREATH_EVENT_APNEA,
  /** Inspiratory amplitude scaled to (1 - severity). */
  BREATH_EVENT_HYPOPNEA,
  /** Inspiratory flow flattened to `severity` regardless of the base param. */
  BREATH_EVENT_FLOW_LIMIT,
  /** Cheyne-Stokes: crescendo-decrescendo with central apnea at the nadir. */
  BREATH_EVENT_CSR,
  /**
    * Post-event recovery (arousal) breath. Not user-triggerable: the
    * waveform generator raises it by itself for @ref recovery_breaths
    * breaths after an amplitude-reducing event ends, so that the markers
    * in the telemetry distinguish an arousal breath from a normal one.
    * An APAP's baseline tracker sees these, and a scoring script must be
    * able to exclude them.
    */
  BREATH_EVENT_RECOVERY,
  BREATH_EVENT_COUNT
} BreathEventType_t;

/* Status flags — every one of these marks data a test report must not trust
 * at face value. They are sticky for the duration of a run and reported in
 * the USB telemetry so post-processing can discard affected breaths. */
#define BREATH_FLAG_SLEW_LIMITED    0x0001U /**< Command was rate-limited; delivered shape != requested. */
#define BREATH_FLAG_TRACK_ERROR     0x0002U /**< Motor did not follow the commanded RPM. */
#define BREATH_FLAG_BELOW_OBS_MIN   0x0004U /**< Commanded below sensorless observer minimum. */
#define BREATH_FLAG_MOTOR_FAULT     0x0008U /**< CM4 reported a fault during the run. */
#define BREATH_FLAG_TICK_OVERRUN    0x0010U /**< Control task missed its deadline. */
#define BREATH_FLAG_VBUS_DERATE     0x0020U /**< Decel slowed to protect the DC bus. */
#define BREATH_FLAG_FLOW_INVALID    0x0040U /**< Flow feedback unusable; fell back to open loop. */
#define BREATH_FLAG_FLOW_SAT        0x0080U /**< Flow controller output hit a limit. */
#define BREATH_FLAG_PRESSURE_LIMIT  0x0100U /**< Airway pressure limit tripped; run aborted. */
#define BREATH_FLAG_VOLUME_SHORT    0x0200U /**< Delivered tidal volume missed target by >20%. */
/**
  * Flow barely moved while a real breath was commanded, for several breaths
  * running. Either the sensor is not seeing the breath (mounted backwards,
  * plumbed in a branch, not tared) or the blower cannot produce the demanded
  * flow at all. Closed-loop control is abandoned for the rest of the run
  * when this is raised - see the note in BreathSim_FlowControlRpm.
  */
#define BREATH_FLAG_FLOW_NO_RESPONSE 0x0400U
/** Pressure feed-forward selected, but the pressure sample was unusable, so
  * the controller fell back to the static path for those ticks. */
#define BREATH_FLAG_FF_NO_PRESSURE   0x0800U

/* ===========================================================================
 * Data
 * ======================================================================== */

typedef struct
{
  /* --- timing --- */
  uint8_t  timing_mode;     /**< BreathTimingMode_t */
  float    rate_bpm;        /**< Breaths per minute (4..60). Derived in EXPLICIT mode. */
  float    insp_time_s;     /**< Inspiration time (0.3..4.0 s). Derived in RATE_IE mode. */
  float    ie_ratio_exp;    /**< Expiratory time = inspiratory time * this (1..4). */
  float    insp_pause_s;    /**< Plateau held at end-inspiration (0..1.0 s). */
  float    exp_pause_s;     /**< Quiet time at baseline before the next breath (0..2.0 s). */

  /* --- amplitude --- */
  int32_t  rpm_base;        /**< Baseline RPM between breaths. Keep as low as the motor allows. */
  int32_t  rpm_amplitude;   /**< Additional RPM at peak inspiratory flow. */

  /* --- shape --- */
  uint8_t  waveform;        /**< BreathWaveform_t — inspiratory flow shape. */
  float    exp_tau_s;       /**< Passive expiratory decay time constant (0.1..2.0 s). */
  float    flattening;      /**< 0 = normal, 1 = fully flow-limited inspiratory plateau. */
  uint8_t  fl_shape;        /**< BreathFlShape_t - morphology of the limitation. */

  /* --- realism --- */
  float    jitter_pct;      /**< Breath-to-breath variability, 0..25 %. */
  uint32_t jitter_seed;     /**< PRNG seed; a fixed seed makes runs reproducible. */
  /**
    * Large breaths after an apnea or hypopnea ends, as an arousal produces
    * clinically. 0 disables them, which is the old behaviour: events then
    * end into an abruptly normal breath, and an APAP that tracks a
    * baseline never sees the overshoot it would see on a patient.
    */
  uint8_t  recovery_breaths;  /**< 0..BREATH_RECOVERY_BREATHS_MAX. */
  float    recovery_gain;     /**< Amplitude multiplier, 1.0..3.0. */
  float    snore_hz;          /**< 0 = off, else 10..60 Hz. */
  int32_t  snore_rpm;         /**< Peak RPM of the modulation, 0..3000. */

  /* --- closed-loop flow control (BREATH_CTRL_FLOW) --- */
  uint8_t  control_mode;    /**< BreathControlMode_t */
  float    tidal_ml;        /**< Target inspired volume per breath. */
  float    flow_kp;         /**< RPM per (L/min) of flow error. */
  float    flow_ki;         /**< RPM per (L/min) per second. */
  float    flow_ff_rpm_per_lpm; /**< Static feed-forward slope (BREATH_FF_STATIC). */
  uint8_t  ff_mode;         /**< BreathFfMode_t. */
  float    ff_a;            /**< Plant model - see BREATH_FF_A_DEFAULT. */
  float    ff_b;
  float    ff_c;
  float    pressure_limit_cmh2o; /**< Abort above this. 0 = disabled. */
} BreathSimParams_t;

/** Resolved (derived) timing — always consistent, always sums to period_s. */
typedef struct
{
  float period_s;
  float insp_s;
  float insp_pause_s;
  float exp_s;
  float exp_pause_s;
  float rate_bpm;
} BreathSimTiming_t;

/** Coherent runtime snapshot — captured atomically inside the breath task. */
typedef struct
{
  uint8_t  state;           /**< BreathSimState_t */
  uint8_t  segment;         /**< BreathSegment_t */
  uint8_t  event;           /**< Active BreathEventType_t */
  uint8_t  script_step;     /**< Active script step index, 0xFF when idle. */
  uint16_t flags;           /**< Sticky BREATH_FLAG_* for this run. */
  uint32_t breath_index;    /**< Breaths completed since the run started. */
  float    phase;           /**< 0..1 within the current breath. */
  float    envelope;        /**< 0..1 normalised inspiratory flow demand. */
  /**
    * Slew-limited RPM command. This is what CM4 is posted, EXCEPT while
    * snore modulation is active: that is added after the slew limiter, on
    * the way to the mailbox, and deliberately does not appear here. The
    * slew limiter and the tracking supervisor both reason about this
    * value, and feeding an oscillation back into either would make the
    * limiter chase itself and the supervisor raise TRACK_ERROR for a
    * ripple the motor was never going to follow.
    */
  int32_t  rpm_cmd;
  int32_t  rpm_act;         /**< Measured mechanical RPM from CM4. */
  uint16_t bus_voltage_v;
  uint16_t mc_state;        /**< Raw MCI_State_t from CM4. */
  /*
   * Raw MC_* fault bits latched by CM4, captured at the moment the run was
   * aborted. mc_state is NOT enough on its own: aborting sends a STOP, and
   * the CM4 handler acknowledges in FAULT_OVER, so the drive is back in
   * IDLE (0) by the time anyone reads it. This is the only field that says
   * WHICH fault fired.
   */
  uint16_t mc_occurred_faults;

  /* --- closed-loop flow control --- */
  uint8_t  control_mode;
  uint8_t  flow_valid;      /**< 1 = feedback usable this tick. */
  float    q_target_lpm;    /**< Commanded inspiratory flow. */
  float    q_meas_lpm;      /**< Measured flow. */
  float    q_peak_lpm;      /**< Peak flow this breath implies for tidal_ml. */
  float    volume_ml;       /**< Inspired volume so far in this breath. */
  float    ctrl_integ_rpm;  /**< Integrator state, for tuning visibility. */
  uint8_t  ff_mode;         /**< BreathFfMode_t in use. */
  uint8_t  ff_p_inlet_valid; /**< 0 until the first end-expiratory estimate. */
  float    ff_rpm;          /**< Feed-forward part of the command, rpm. */
  float    ff_p_inlet;      /**< Estimated blower inlet pressure, cmH2O. */

  /* --- last completed breath: this is the test result --- */
  float    last_tidal_ml;   /**< Delivered inspired volume. */
  float    p_cmh2o;         /**< Current airway pressure. */
  float    p_min_cmh2o;     /**< Minimum over the last breath (~EPAP). */
  float    p_max_cmh2o;     /**< Maximum over the last breath (~IPAP). */
  float    p_mean_cmh2o;    /**< Mean over the last breath. */
  float    last_insp_s;     /**< Ti actually used, after jitter. */
  float    last_q_peak_lpm; /**< MEASURED peak inspiratory flow. */
  /**
    * Measured inspiratory mean/peak over the completed breath - the same
    * quantity the APAP under test thresholds on (> 0.78 scores as flow
    * limited). Reported because the event LABEL is not ground truth: the
    * flow-limitation shapes differ in mean/peak, and what the device
    * responds to is this number, not the name of the shape behind it.
    * 0 when the breath had no usable flow feedback.
    */
  float    last_flat_index;
  /**
    * The event that actually SHAPED a breath, as distinct from @ref event,
    * which is whatever the event timer holds right now. Modifiers are
    * latched at the breath boundary (see BreathSim_LatchBreathModifiers),
    * so events only ever apply to whole breaths - and one that starts and
    * ends inside a single breath shapes nothing and is never reported.
    * Marking event boundaries from @ref event instead would place them up
    * to a breath early, which is enough to mis-score every detection the
    * device under test makes.
    */
  uint8_t  event_applied;       /**< ...for the breath in progress. */
  uint8_t  last_event;          /**< ...for the breath just completed. */
  float    event_severity;      /**< Severity of @ref event_applied. */
  float    last_event_severity; /**< Severity of @ref last_event. */
} BreathSimStatus_t;

typedef struct
{
  uint8_t type;             /**< BreathEventType_t */
  float   duration_s;
  float   severity;         /**< 0..1 */
} BreathEvent_t;

/**
  * One step of a scripted protocol.
  *
  * The override fields let a step change the breathing pattern as well as
  * the event - a positional change, a different depth of sleep - without
  * needing a host in the loop. Each is applied on top of the parameters as
  * they were when the script STARTED, never on top of the previous step,
  * so a step means the same thing wherever it appears in the script.
  *
  * Overrides land at the next breath boundary like any other parameter
  * change, so a step shorter than one breath may not show its override at
  * all. Use BreathSim_ScriptStepInit() to get the "inherit everything"
  * defaults rather than zeroing the struct by hand: the sentinels are not
  * all zero.
  */
typedef struct
{
  float   duration_s;
  uint8_t event;            /**< BreathEventType_t */
  float   severity;
  uint8_t repeat;           /**< Run this step this many times; 0 means 1. */
  float   rate_bpm;         /**< Override; <= 0 inherits. */
  float   tidal_ml;         /**< Override; <= 0 inherits. */
  int32_t rpm_amplitude;    /**< Override; < 0 inherits (0 is a valid value). */
} BreathScriptStep_t;

/* ===========================================================================
 * API
 * ======================================================================== */

void BreathSim_Init(void);

void BreathSim_GetParams(BreathSimParams_t *out);
/**
  * @brief Validate and stage a new parameter set.
  *
  * The parameters are normalised immediately (so the caller can read back
  * what will actually run) but are only applied to the running waveform at
  * the next breath boundary. This keeps a parameter change from warping the
  * breath in progress, and makes the update atomic with respect to the
  * higher-priority breath task.
  *
  * @return false if the request was inconsistent and had to be adjusted.
  *         The adjusted values are what BreathSim_GetParams() will return.
  */
bool BreathSim_SetParams(const BreathSimParams_t *params);

/** Resolved timing for the parameters currently staged. */
void BreathSim_GetTiming(BreathSimTiming_t *out);
/** Coherent runtime snapshot. Safe to call from any task. */
void BreathSim_GetStatus(BreathSimStatus_t *out);

void BreathSim_Start(void);
void BreathSim_Stop(void);
bool BreathSim_IsRunning(void);

/** Call at a fixed rate (200 Hz) from the breath task. */
void BreathSim_Update(float dt_s);

/**
  * @brief Publish the latest sensor readings for the control loop.
  *
  * Called from SensorTask at 100 Hz. The breath loop runs at 200 Hz and
  * simply holds the most recent sample; a sample older than
  * BREATH_FLOW_STALE_MS is treated as no feedback at all, which drops the
  * controller back to open loop rather than letting it act on stale data.
  *
  * @param q_lpm    Flow in L/min, positive = into the test lung (inspiration).
  * @param q_valid  0 if the sample failed CRC / range checks.
  * @param p_cmh2o  Airway pressure. Measurement only, never a controlled variable.
  * @param p_valid  0 if the pressure sample is not trustworthy.
  */
void BreathSim_SetSensorInputs(float q_lpm, uint8_t q_valid,
                               float p_cmh2o, uint8_t p_valid);

/**
  * @brief 1 when the flow feedback is fresh enough to close a loop around.
  *
  * Lets the GUI refuse to offer BREATH_CTRL_FLOW when the sensor is not
  * measuring, without the GUI having to know about the sensor driver.
  */
bool BreathSim_FlowSensorReady(void);

/** Load a normalised (0..1) inspiratory flow profile. @p n <= BREATH_TABLE_POINTS. */
bool BreathSim_SetTable(const float *points, uint8_t n);
uint8_t BreathSim_GetTableLength(void);

/** Inject a one-shot respiratory event, applied from the next breath. */
bool BreathSim_TriggerEvent(const BreathEvent_t *ev);
void BreathSim_CancelEvent(void);

/* Scripted test protocols: a sequence of timed segments run back to back. */
void BreathSim_ScriptClear(void);
/** Fill @p step with the "inherit everything, run once" defaults. */
void BreathSim_ScriptStepInit(BreathScriptStep_t *step);
bool BreathSim_ScriptAppend(const BreathScriptStep_t *step);
uint8_t BreathSim_ScriptLength(void);
/** Read back step @p idx; false if it is past the end. */
bool BreathSim_ScriptGet(uint8_t idx, BreathScriptStep_t *out);
/**
  * @brief Run the loaded script.
  * @param passes  Number of times to run it through; 0 runs for ever.
  */
bool BreathSim_ScriptStart(uint16_t passes);
void BreathSim_ScriptStop(void);
bool BreathSim_ScriptIsRunning(void);

const char *BreathSim_WaveformName(uint8_t waveform);
const char *BreathSim_FlShapeName(uint8_t shape);
const char *BreathSim_ControlModeName(uint8_t mode);
const char *BreathSim_FfModeName(uint8_t mode);
/** Human-readable MC_* fault bits, e.g. "SPEED_FDBK|DP_FAULT". */
const char *BreathSim_McFaultName(uint16_t faults);
const char *BreathSim_EventName(uint8_t event);
const char *BreathSim_StateName(uint8_t state);
const char *BreathSim_SegmentName(uint8_t segment);

/* Legacy scalar accessors retained for existing callers. Prefer
 * BreathSim_GetStatus(), which returns a self-consistent set. */
int32_t BreathSim_GetTargetRpm(void);
float   BreathSim_GetPhase(void);
float   BreathSim_GetCyclePeriodS(void);
float   BreathSim_GetEnvelope(void);

#ifdef __cplusplus
}
#endif

#endif /* BREATH_SIM_H */
