/**
  ******************************************************************************
  * @file    therapy.h
  * @brief   CPAP therapy state machine.
  *
  *  Architecture
  *  ------------
  *      [GUI: START THERAPY click] -> Therapy_Start()
  *
  *      SensorTask (200 Hz, high): sensors + leak + PID + Therapy_Update
  *      SafetyTask (200 Hz, realtime): safety_supervisor (preempts SensorTask)
  *
  *      States: IDLE -> RAMP -> RUNNING -> STOPPING -> IDLE
  *              FAULT latched by SafetyTask or motor fault
  *
  *  Outputs of the state machine:
  *      - blower start/stop commands via blower_ipc_cm7,
  *      - PID enable + setpoint via blower_ctrl,
  *      - leak monitoring threshold (leak_estimator already runs free
  *        but only matters during therapy).
  *
 *  Therapy modes (AirSense-11 style)
 *  ---------------------------------
 *      - CPAP: fixed therapy pressure (4-25 cmH2O).
 *      - Autoset / Autoset for her: APAP titration between min and max
 *        pressure (see apap_ctrl.c). Starts at min; raises on OSA.
 *      - Optional pressure ramp from RAMP_START to target over the
 *        ramp_minutes setting (Off, Auto = 5 min, or 5..45 min).
 *      - Run timer, leak status, current/target pressure available
 *        for the GUI to display.
 *
 *  Out of scope for this iteration:
 *      - Snore / flow-limitation response, CSA detection, long-term
 *        pressure reduction after stable periods (full AirSense feature set).
  *      - SmartStart/SmartStop - needs flow-based start/stop detection.
  *      - Therapy session logging to flash/SD.
  ******************************************************************************
  */

#ifndef THERAPY_H
#define THERAPY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define THERAPY_PRESS_MIN_CMH2O  4.0f
#define THERAPY_PRESS_MAX_CMH2O  25.0f

typedef enum
{
    THERAPY_STATE_IDLE = 0,    /* blower off, PID off, waiting for Start    */
    THERAPY_STATE_RAMP,        /* blower on, PID on, setpoint ramping       */
    THERAPY_STATE_RUNNING,     /* at target pressure, monitoring leak       */
    THERAPY_STATE_STOPPING,    /* user stopped, blower spinning down        */
    THERAPY_STATE_FAULT        /* error: motor fault, persistent high leak  */
} TherapyState_t;

typedef enum
{
    THERAPY_MODE_CPAP = 0,           /* fixed pressure                      */
    THERAPY_MODE_AUTOSET,            /* APAP titration (apap_ctrl.c)        */
    THERAPY_MODE_AUTOSET_FOR_HER     /* gentler APAP response             */
} TherapyMode_t;

/* === Safety supervisor =====================================================
 *
 *  Fault classes that cause the therapy supervisor to shut the blower down
 *  and transition to THERAPY_STATE_FAULT. The screen reports the reason so
 *  the user knows what went wrong.
 */
typedef enum
{
    THERAPY_FAULT_NONE = 0,

    /* Pressure stayed below `g_therapy_low_press_cmh2o` (default 1.5 cmH2O)
     * for `g_therapy_low_press_timeout_s` (default 4 s) while the PID was
     * commanding more than `g_therapy_high_rpm_threshold` (default 12 krpm).
     * Means: pressure-sensor disconnected, hose disconnected at the blower
     * outlet, or a leak so large that the controller is winding up. */
    THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM,

    /* Measured pressure exceeded `target + g_therapy_overpress_band_cmh2o`
     * (default +5 cmH2O over target) for `g_therapy_overpress_timeout_s`
     * (default 3 s). Means: sensor stuck high, controller runaway, or
     * mechanical occlusion downstream. */
    THERAPY_FAULT_OVER_PRESSURE,

    /* Measured pressure fell outside the AMS5935 plausible range
     * (-10 cmH2O .. +50 cmH2O) for 2 s. Means: sensor electrical fault,
     * SPI corruption, bad cable. */
    THERAPY_FAULT_SENSOR_OUT_OF_RANGE,

    /* RPM has been pinned at `g_therapy_safe_rpm_max` for
     * `g_therapy_saturated_timeout_s` (default 6 s) without the pressure
     * converging within `g_therapy_saturate_band_cmh2o` (default 1.5
     * cmH2O) of target. Means: the controller can't deliver setpoint -
     * usually a too-large leak or undersized blower for this circuit. */
    THERAPY_FAULT_RPM_SATURATED,

    /* CM4 `cm4_status_seq` in the blower IPC mailbox stopped advancing
     * while therapy expected the motor domain to be alive. */
    THERAPY_FAULT_CM4_HEARTBEAT
} TherapyFault_t;

/* Bring-up at boot. Idempotent; safe to call multiple times. */
void   Therapy_Init(void);

/* Tick. Call every dt_s seconds from the same task that runs the PID
 * (SensorTask, 5 ms). State transitions and ramp updates happen here. */
void   Therapy_Update(float dt_s);

/* User-driven session control. Safe to call from any thread/context. */
void   Therapy_Start(void);
void   Therapy_Stop(void);

/* Configuration (the GUI calls these from cpap_settings on edit). */
void   Therapy_SetTargetPressure(float cmh2o);   /* clamped 4..25         */
float  Therapy_GetTargetPressure(void);
void   Therapy_SetRampMinutes(uint8_t minutes);  /* 0 = no ramp           */
uint8_t Therapy_GetRampMinutes(void);
void   Therapy_SetMode(TherapyMode_t m);
TherapyMode_t Therapy_GetMode(void);
const char*    Therapy_GetModeName(void);
void   Therapy_SetApapMinPressure(float cmh2o);  /* clamped 4..25       */
void   Therapy_SetApapMaxPressure(float cmh2o);  /* clamped 4..25       */
float  Therapy_GetApapMinPressure(void);
float  Therapy_GetApapMaxPressure(void);
const char* Therapy_GetApapStateName(void);

/* EPR (Expiratory Pressure Relief). AirSense-style type + level. */
void   Therapy_SetEprEnabled(bool en);
bool   Therapy_IsEprEnabled(void);
void   Therapy_SetEprType(uint8_t type);       /* 0=Full time, 1=Ramp only */
uint8_t Therapy_GetEprType(void);
void   Therapy_SetEprLevel(uint8_t level);     /* 1, 2, or 3 cmH2O       */
uint8_t Therapy_GetEprLevel(void);
void   Therapy_SetEprReliefCmh2o(float cmh2o); /* alias for level          */
float  Therapy_GetEprReliefCmh2o(void);

/* Mask-off during therapy: on high leak, hold blower at low RPM instead of
 * chasing setpoint; resume pressure control when the circuit seals again.
 *
 * Enter paths (OR):
 *   - Fast: instant excess flow or slow high-leak LPF (simple or Vivo).
 *   - Slow: Vivo MeanRatio10Sec > limit for ratio_enter_ms (default 15 s).
 *
 * Reconnect at hold RPM (default 6000): blower stays running for sensing;
 * resume therapy pressure when flow drops and backpressure returns.
 * Debugger: g_therapy_mask_off_reconn_flow_ok / _press_ok during mask-off. */
bool   Therapy_IsMaskOffActive(void);
void   Therapy_SetMaskOffEnabled(bool en);
bool   Therapy_IsMaskOffEnabled(void);

/* Live state for GUI / logging. */
TherapyState_t Therapy_GetState(void);
const char*    Therapy_GetStateName(void);
float          Therapy_GetSetpointCmh2o(void);    /* what we *ask* for     */
float          Therapy_GetMeasuredCmh2o(void);    /* mask pressure         */
float          Therapy_GetLeakSlm(void);          /* unintentional leak    */
bool           Therapy_IsLeakHigh(void);
uint32_t       Therapy_GetRunSeconds(void);       /* time in RAMP+RUNNING  */
uint32_t       Therapy_GetRampRemainingSeconds(void); /* 0 if not ramping  */

/* Fault inspection / recovery. */
TherapyFault_t Therapy_GetFault(void);
const char*    Therapy_GetFaultName(void);
void           Therapy_ClearFault(void);          /* operator acknowledge   */

/* Called by safety_supervisor.c on trip. */
void           Therapy_LatchFault(TherapyFault_t f, const char *why);
bool           Therapy_SafetyGraceActive(void);

#ifdef __cplusplus
}
#endif

#endif /* THERAPY_H */
