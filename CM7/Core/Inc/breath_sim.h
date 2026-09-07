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
#define BREATH_RPM_BASE_MAX             20000
#define BREATH_RPM_BASE_DEFAULT         14500
#define BREATH_RPM_AMP_MAX              25000
/** Absolute ceiling; MAX_APPLICATION_SPEED_RPM on CM4 is 36012. */
#define BREATH_RPM_ABS_MAX              36000

/**
  * Sensorless observer minimum from CM4 drive_parameters.h
  * (OBS_MINIMUM_SPEED_RPM). Below this the STO/PLL estimate degrades and
  * MC_SPEED_FDBK faults become likely, so a commanded RPM under this value
  * raises BREATH_FLAG_BELOW_OBS_MIN rather than being silently trusted.
  * It is deliberately NOT a clamp: the limit is a motor-tuning property, and
  * lowering it (retuning GAIN1/GAIN2/PLL gains) is the correct long-term fix
  * for letting the blower stand further out of the way during expiration.
  */
#define BREATH_RPM_OBS_MIN_RPM          12965

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

/** Arbitrary-waveform table size (normalised inspiratory flow, 0..1). */
#define BREATH_TABLE_POINTS             64U

/** Maximum steps in a scripted test protocol. */
#define BREATH_SCRIPT_MAX_STEPS         16U

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

  /* --- realism --- */
  float    jitter_pct;      /**< Breath-to-breath variability, 0..25 %. */
  uint32_t jitter_seed;     /**< PRNG seed; a fixed seed makes runs reproducible. */

  /* --- closed-loop flow control (BREATH_CTRL_FLOW) --- */
  uint8_t  control_mode;    /**< BreathControlMode_t */
  float    tidal_ml;        /**< Target inspired volume per breath. */
  float    flow_kp;         /**< RPM per (L/min) of flow error. */
  float    flow_ki;         /**< RPM per (L/min) per second. */
  float    flow_ff_rpm_per_lpm; /**< Feed-forward slope; 0 until characterised. */
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
  int32_t  rpm_cmd;         /**< RPM actually posted to CM4 (post slew limit). */
  int32_t  rpm_act;         /**< Measured mechanical RPM from CM4. */
  uint16_t bus_voltage_v;
  uint16_t mc_state;        /**< Raw MCI_State_t from CM4. */

  /* --- closed-loop flow control --- */
  uint8_t  control_mode;
  uint8_t  flow_valid;      /**< 1 = feedback usable this tick. */
  float    q_target_lpm;    /**< Commanded inspiratory flow. */
  float    q_meas_lpm;      /**< Measured flow. */
  float    q_peak_lpm;      /**< Peak flow this breath implies for tidal_ml. */
  float    volume_ml;       /**< Inspired volume so far in this breath. */
  float    ctrl_integ_rpm;  /**< Integrator state, for tuning visibility. */

  /* --- last completed breath: this is the test result --- */
  float    last_tidal_ml;   /**< Delivered inspired volume. */
  float    p_cmh2o;         /**< Current airway pressure. */
  float    p_min_cmh2o;     /**< Minimum over the last breath (~EPAP). */
  float    p_max_cmh2o;     /**< Maximum over the last breath (~IPAP). */
  float    p_mean_cmh2o;    /**< Mean over the last breath. */
} BreathSimStatus_t;

typedef struct
{
  uint8_t type;             /**< BreathEventType_t */
  float   duration_s;
  float   severity;         /**< 0..1 */
} BreathEvent_t;

typedef struct
{
  float   duration_s;
  uint8_t event;            /**< BreathEventType_t */
  float   severity;
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
bool BreathSim_ScriptAppend(const BreathScriptStep_t *step);
uint8_t BreathSim_ScriptLength(void);
bool BreathSim_ScriptStart(bool loop);
void BreathSim_ScriptStop(void);
bool BreathSim_ScriptIsRunning(void);

const char *BreathSim_WaveformName(uint8_t waveform);
const char *BreathSim_ControlModeName(uint8_t mode);
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
