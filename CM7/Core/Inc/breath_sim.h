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
  * No pressure or flow sensors are required. This is an open-loop RPM
  * generator: it commands the blower, it does not regulate delivered flow.
  * BreathSim_GetStatus() reports whether the motor actually tracked the
  * command, so an invalid run announces itself instead of producing
  * plausible-looking but wrong data.
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
