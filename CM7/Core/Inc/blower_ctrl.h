/**
  ******************************************************************************
  * @file    blower_ctrl.h
  * @brief   Outer pressure-control PID for the CPAP blower. Runs on CM7,
  *          posts speed targets to the CM4 motor controller via the existing
  *          blower IPC mailbox.
  *
  *  Topology
  *  --------
  *      Setpoint (cmH2O)
  *           |
  *           v   (slewed)
  *      [PID @ 200 Hz on CM7]  <--- pressure_meas (cmH2O, from measPS)
  *           |
  *           v   (rpm reference)
  *      [BlowerIpc_CM7_SetSpeedRpmSilent]  ->  CM4 FOC inner loop
  *
  *  Why gain scheduling
  *  -------------------
  *  CPAP blowers are non-linear: at low therapy pressures (~4 cmH2O) the
  *  RPM-to-pressure slope is steep (small RPM step -> large pressure swing,
  *  so the loop needs LOW gains), while at high pressures (~20 cmH2O) the
  *  blower has already done most of the air-moving work and small pressure
  *  changes need bigger RPM corrections (so the loop needs HIGHER gains).
  *
  *  This module exposes three (Kp, Ki, Kd) sets selected by the *current
  *  setpoint*. We schedule on the setpoint - not the measurement - because
  *  setpoint is monotonic and slew-limited, while a noisy measurement at
  *  the boundary would chatter between zones.
  *
 *  All gains and feature flags are debugger-visible volatile globals so
 *  you can tune live with the Keil watch window or via STM Studio.
 *
 *  WM6850 reference mode
 *  ---------------------
 *  Set g_blower_ctrl_wm6850_mode = 1 for pressure_pid.c-compatible behaviour
 *  (see blower_ctrl.c design note 8). PID runs at 100 Hz (every 2nd 5 ms
 *  SensorTask tick). Overrides FF/slew/zones/tube-comp on the PID path.
 *  Tune via g_blower_ctrl_wm6850_kp/ki/kd.
  ******************************************************************************
  */

#ifndef BLOWER_CTRL_H
#define BLOWER_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* CPAP therapy range expected by the controller. The setpoint is clamped
 * to [BLOWER_CTRL_SETPOINT_MIN, BLOWER_CTRL_SETPOINT_MAX] regardless of
 * what the caller passes in. */
#define BLOWER_CTRL_SETPOINT_MIN_CMH2O   4.0f
#define BLOWER_CTRL_SETPOINT_MAX_CMH2O   25.0f

/* Zone boundaries used for gain scheduling. Anything below LOW_HI uses the
 * "low" set; anything between LOW_HI and MID_HI uses "mid"; anything above
 * MID_HI uses "high". A small hysteresis prevents zone flapping at the
 * boundaries. */
#define BLOWER_CTRL_ZONE_LOW_HI_CMH2O    8.0f
#define BLOWER_CTRL_ZONE_MID_HI_CMH2O    15.0f
#define BLOWER_CTRL_ZONE_HYST_CMH2O      0.5f

/* Identifies which gain set was used in the most recent step. Useful to
 * watch in the debugger; also drives the Diagnostics page if you want to
 * surface it later. */
typedef enum
{
    BLOWER_CTRL_ZONE_LOW = 0,
    BLOWER_CTRL_ZONE_MID,
    BLOWER_CTRL_ZONE_HIGH
} BlowerCtrlZone_t;

/* Public API ---------------------------------------------------------------- */

/* Reset internal state and load default gains. Call once before the loop
 * task starts using the controller. */
void BlowerCtrl_Init(void);

/* Master enable/disable. While disabled the controller does not post any
 * IPC commands - manual blower control (g_bench_blower_enable, debugger writes)
 * keeps working. While enabled the controller owns target_speed_rpm. */
void BlowerCtrl_SetEnable(bool en);
bool BlowerCtrl_IsEnabled(void);

/* Update the user setpoint in cmH2O. The controller slews internally
 * toward this target at g_blower_ctrl_slew_cmh2o_per_s so a step input
 * does not provoke huge transient overshoot or anti-windup glitches. */
void BlowerCtrl_SetSetpoint(float cmh2o);
/* Set user + slewed setpoint immediately (therapy start / EPR snap). */
void BlowerCtrl_SnapSetpoint(float cmh2o);
float BlowerCtrl_GetSetpoint(void);          /* user setpoint              */
float BlowerCtrl_GetSetpointSlewed(void);    /* internal slewed target     */
float BlowerCtrl_GetMeasured(void);          /* PID measurement (mask est.) */
float BlowerCtrl_GetSensorCmh2o(void);       /* device-side AMS5935 cmH2O  */
float BlowerCtrl_GetOutputRpm(void);         /* last commanded rpm         */

/* Open-loop manual blower (USB / bench). Stops pressure PID and posts RPM
 * targets directly to CM4 with optional CM4-side ramp. */
void BlowerCtrl_ManualStart(int32_t rpm, uint16_t ramp_ms);
void BlowerCtrl_ManualSetSpeed(int32_t rpm, uint16_t ramp_ms);
void BlowerCtrl_ManualStop(void);
bool BlowerCtrl_IsManualActive(void);

/* Run one PID iteration. Should be called periodically from the sensor
 * task at 5 ms (200 Hz). WM6850 mode decimates internally to 100 Hz.
 * dt_s is the elapsed seconds since the previous call. */
void BlowerCtrl_Step(float dt_s);

/* Enforce mutual exclusion between physics FF and legacy sqrt/flow/dip FF. */
void BlowerCtrl_EnforceFfMutualExclusion(void);

#ifdef __cplusplus
}
#endif

#endif /* BLOWER_CTRL_H */
