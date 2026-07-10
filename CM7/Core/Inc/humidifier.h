/**
  ******************************************************************************
  * @file    humidifier.h
  * @brief   Closed-loop humidity control for the bowl-coil humidifier.
  ******************************************************************************
  *
  *  Plant:      ~25 W coil heating ~100 mL water in a steel bowl.
  *  Sensor:     SHT4x on hSht4x measures air %RH and air temperature in
  *              the chamber above the water.
  *  Actuator:   TIM4 CH1 PWM (period 65535, prescaler 0).
  *
  *  Open-loop reference observation (provided by the user):
  *    full duty (CCR1 = 65000) ~ 50 deg C in 10-15 min.
  *  This implies a thermal time constant on the order of minutes and
  *  significant dead time, so the loop is intentionally slow:
  *    - 1 Hz update rate (decimated from the 5 ms main loop).
  *    - PI with conditional-integration anti-windup.
  *    - Slew-rate limit on PWM output.
  *    - Hard min/max PWM clamps.
  *    - Over-temperature trip from the SHT4x air temperature.
  *    - "Heater stuck on" watchdog: if PWM is saturated near max for
  *      a long time but RH does not rise, we trip a fault.
  *
  *  All knobs are exposed as volatile globals so they can be tuned from
  *  the debugger watch window without rebuilding.
  *
  *  Safety notes:
  *    - This module ONLY drives TIM4 CH1. CH2 is left untouched.
  *    - Boot default is DISABLED (PWM = 0). Set
  *      g_humidifier_enable = 1 to start; or call Humidifier_SetEnabled(true).
  *    - Faults latch the heater OFF until acknowledged.
  ******************************************************************************
  */

#ifndef HUMIDIFIER_H
#define HUMIDIFIER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * Types
 * --------------------------------------------------------------------------*/
typedef enum
{
  HUMIDIFIER_STATE_OFF        = 0,  /* disabled, PWM forced to 0 */
  HUMIDIFIER_STATE_IDLE       = 1,  /* enabled but waiting for valid sensor */
  HUMIDIFIER_STATE_RUNNING    = 2,  /* PI loop active */
  HUMIDIFIER_STATE_FAULT      = 3,  /* latched fault, heater off */
} HumidifierState_t;

/* Fault flags (bitfield in g_humidifier_fault_bits). */
#define HUMIDIFIER_FAULT_NONE          0x0000U
#define HUMIDIFIER_FAULT_SENSOR        0x0001U  /* SHT4x not responding / CRC bad */
#define HUMIDIFIER_FAULT_OVERTEMP      0x0002U  /* air temp > limit */
#define HUMIDIFIER_FAULT_DRY_RUN       0x0004U  /* heater on but RH not rising */
#define HUMIDIFIER_FAULT_BAD_SETPOINT  0x0008U  /* setpoint outside [0..100] */

/* ----------------------------------------------------------------------------
 * Tuning knobs (debugger-writable). Defaults are conservative; expect to
 * adjust them live during commissioning.
 * --------------------------------------------------------------------------*/
extern volatile uint8_t  g_humidifier_enable;          /* 1 = run, 0 = stop */
extern volatile uint8_t  g_humidifier_ack_fault;       /* set 1 to clear latched fault */

extern volatile float    g_humidifier_setpoint_rh;     /* %, target RH */
extern volatile float    g_humidifier_kp;              /* PWM / %RH */
extern volatile float    g_humidifier_ki;              /* PWM / %RH / second */
extern volatile float    g_humidifier_deadband_rh;     /* %, error |e| below this -> hold output */

extern volatile uint16_t g_humidifier_pwm_max;         /* 0..65535, hard cap on output */
extern volatile uint16_t g_humidifier_pwm_min;         /* 0..65535, applied while running */
extern volatile uint16_t g_humidifier_pwm_slew_per_s;  /* max change in PWM units per second */

extern volatile float    g_humidifier_temp_limit_c;    /* trip OVERTEMP above this */
extern volatile float    g_humidifier_temp_resume_c;   /* hysteresis for clearing OVERTEMP */
extern volatile uint16_t g_humidifier_dry_run_timeout_s; /* fault if PWM>=max-cap for N s and dRH/dt small */
extern volatile float    g_humidifier_dry_run_dRH;     /* RH rise required during the timeout window */

/* ----------------------------------------------------------------------------
 * Live state (debugger-readable; do not write).
 * --------------------------------------------------------------------------*/
extern volatile uint8_t  g_humidifier_state;            /* HumidifierState_t */
extern volatile uint16_t g_humidifier_fault_bits;
extern volatile uint32_t g_humidifier_update_count;     /* Number of PI updates */
extern volatile uint32_t g_humidifier_seconds_running;  /* Heater-on time */

extern volatile float    g_humidifier_rh;               /* last measured RH % */
extern volatile float    g_humidifier_temp_c;           /* last measured temp C */
extern volatile float    g_humidifier_error;            /* setpoint - rh */
extern volatile float    g_humidifier_p_term;
extern volatile float    g_humidifier_i_term;
extern volatile float    g_humidifier_output_raw;       /* PI output before clamps */
extern volatile uint16_t g_humidifier_pwm;              /* what is written to CCR1 */

/* ----------------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------------*/

/**
 * @brief Initialise the controller. Forces the heater off.
 *        Must be called after MX_TIM4_Init() and the SensirionTH_Init() of
 *        the SHT4x handle.
 */
void Humidifier_Init(void);

/**
 * @brief Periodic tick. Call from the main loop on every iteration; the
 *        controller decimates internally to 1 Hz.
 */
void Humidifier_Update(void);

/**
 * @brief Convenience setters - equivalent to writing the corresponding
 *        global. Provided for use from a future UI / console.
 */
void Humidifier_SetEnabled(bool enable);
void Humidifier_SetSetpoint(float rh_pct);
void Humidifier_SetGains(float kp, float ki);
void Humidifier_AcknowledgeFault(void);

#ifdef __cplusplus
}
#endif

#endif /* HUMIDIFIER_H */
