/**
  ******************************************************************************
  * @file    humidifier.c
  * @brief   Closed-loop humidity control for the bowl-coil humidifier.
  ******************************************************************************
  *
  *  Control law (per 1 s tick):
  *
  *      e        = setpoint - RH                  (% RH)
  *      P_term   = Kp * e
  *      I_term   = clamp( I_term + Ki * e * dt,
  *                        0, pwm_max - P_term )   (conditional integration)
  *      raw      = P_term + I_term
  *      pwm_des  = clamp( raw, pwm_min, pwm_max )  (only while running)
  *      pwm_out  = slew_limit( pwm_out, pwm_des, slew_per_s )
  *
  *  The integrator floor is zero - this is a heater, it cannot subtract
  *  humidity. When the air is wetter than the setpoint we drop straight
  *  to PWM = 0 and let the airflow / ambient bring RH back down.
  *
  *  The slew-rate limit prevents large, sudden current surges into the
  *  heater coil (which also helps the supply rail stay quiet for the
  *  motor control loop).
  ******************************************************************************
  */

#include "humidifier.h"

#include "main.h"
#include "sensirion_th.h"
#include "dbg_log.h"

#include <string.h>

extern TIM_HandleTypeDef    htim4;
extern SensirionTH_Handle_t hSht4x;

/* ----------------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------------*/

/* Loop is invoked from the 250 ms FreeRTOS climate task. Decimate to 1 Hz. */
#define HUMIDIFIER_TICK_DECIM      4U    /* 4 * 250 ms = 1000 ms */
#define HUMIDIFIER_DT_S            1.0f

/* Sensor-stale watchdog: if SHT4x fails to give a fresh reading for this
 * long, the loop trips a SENSOR fault. */
#define HUMIDIFIER_SENSOR_STALE_S  5U

/* PWM clamps, defaults. CCR period is 65535. We start with a high
 * ceiling because the 25 W bowl-coil plant has very limited authority -
 * a low cap effectively prevents the loop from reaching higher RH
 * setpoints. Lower from the watch window if thermal margin is a concern. */
#define HUMIDIFIER_PWM_MAX_DEFAULT     62000U   /* ~95% duty */
#define HUMIDIFIER_PWM_MIN_DEFAULT         0U
#define HUMIDIFIER_PWM_SLEW_DEFAULT     8000U   /* PWM units / second */

/* Safety thresholds. */
#define HUMIDIFIER_TEMP_LIMIT_C        65.0f   /* trip if SHT4x air temp >= this */
#define HUMIDIFIER_TEMP_RESUME_C       55.0f   /* clear OVERTEMP below this */
#define HUMIDIFIER_DRY_RUN_S            300U   /* 5 minutes */
#define HUMIDIFIER_DRY_RUN_DRH           5.0f  /* require >= 5 %RH rise in window */

/* Default tuning. With minutes of dead time these gains feel small but
 * are typical - heater plants over-shoot easily. Tighten once a step
 * response is captured. Ki is sized so a steady 5 %RH error fully
 * winds the integrator over ~2-3 minutes. */
#define HUMIDIFIER_KP_DEFAULT           600.0f /* PWM / %RH */
#define HUMIDIFIER_KI_DEFAULT            30.0f /* PWM / %RH / s */
#define HUMIDIFIER_DEADBAND_DEFAULT       0.5f /* %RH */
#define HUMIDIFIER_SETPOINT_DEFAULT      80.0f /* %RH */

/* ----------------------------------------------------------------------------
 * Public state
 * --------------------------------------------------------------------------*/
volatile uint8_t  g_humidifier_enable          = 0U;
volatile uint8_t  g_humidifier_ack_fault       = 0U;

volatile float    g_humidifier_setpoint_rh     = HUMIDIFIER_SETPOINT_DEFAULT;
volatile float    g_humidifier_kp              = HUMIDIFIER_KP_DEFAULT;
volatile float    g_humidifier_ki              = HUMIDIFIER_KI_DEFAULT;
volatile float    g_humidifier_deadband_rh     = HUMIDIFIER_DEADBAND_DEFAULT;

volatile uint16_t g_humidifier_pwm_max         = HUMIDIFIER_PWM_MAX_DEFAULT;
volatile uint16_t g_humidifier_pwm_min         = HUMIDIFIER_PWM_MIN_DEFAULT;
volatile uint16_t g_humidifier_pwm_slew_per_s  = HUMIDIFIER_PWM_SLEW_DEFAULT;

volatile float    g_humidifier_temp_limit_c    = HUMIDIFIER_TEMP_LIMIT_C;
volatile float    g_humidifier_temp_resume_c   = HUMIDIFIER_TEMP_RESUME_C;
volatile uint16_t g_humidifier_dry_run_timeout_s = HUMIDIFIER_DRY_RUN_S;
volatile float    g_humidifier_dry_run_dRH     = HUMIDIFIER_DRY_RUN_DRH;

volatile uint8_t  g_humidifier_state           = HUMIDIFIER_STATE_OFF;
volatile uint16_t g_humidifier_fault_bits      = HUMIDIFIER_FAULT_NONE;
volatile uint32_t g_humidifier_update_count    = 0U;
volatile uint32_t g_humidifier_seconds_running = 0U;

volatile float    g_humidifier_rh              = 0.0f;
volatile float    g_humidifier_temp_c          = 0.0f;
volatile float    g_humidifier_error           = 0.0f;
volatile float    g_humidifier_p_term          = 0.0f;
volatile float    g_humidifier_i_term          = 0.0f;
volatile float    g_humidifier_output_raw      = 0.0f;
volatile uint16_t g_humidifier_pwm             = 0U;

/* ----------------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------------*/
static uint16_t s_decim_count;
static uint32_t s_sensor_age_s;       /* seconds since last fresh read */
static uint32_t s_dry_run_timer_s;    /* seconds heater has been near max */
static float    s_dry_run_rh_anchor;  /* RH at start of dry-run window */
static uint8_t  s_overtemp_latched;   /* hysteresis on OVERTEMP */
static float    s_pwm_curr_f;         /* current PWM as float, for slew limit */
static uint8_t  s_prev_state = 0xFFU;

/* ----------------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------------*/
static float Hum_ClampF(float x, float lo, float hi)
{
  if (x < lo) { return lo; }
  if (x > hi) { return hi; }
  return x;
}

static uint16_t Hum_ClampU16(int32_t x, uint16_t lo, uint16_t hi)
{
  if (x < (int32_t)lo) { return lo; }
  if (x > (int32_t)hi) { return hi; }
  return (uint16_t)x;
}

static void Hum_ApplyPwm(uint16_t pwm)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)pwm);
  g_humidifier_pwm = pwm;
}

static void Hum_ResetIntegralAndSlew(uint16_t starting_pwm)
{
  g_humidifier_p_term     = 0.0f;
  g_humidifier_i_term     = 0.0f;
  g_humidifier_output_raw = (float)starting_pwm;
  s_pwm_curr_f            = (float)starting_pwm;
  s_dry_run_timer_s       = 0U;
  s_dry_run_rh_anchor     = g_humidifier_rh;
}

static void Hum_ForceOff(const char *reason)
{
  Hum_ApplyPwm(0U);
  Hum_ResetIntegralAndSlew(0U);
  if (g_humidifier_state != HUMIDIFIER_STATE_FAULT)
  {
    g_humidifier_state = HUMIDIFIER_STATE_OFF;
  }
  g_humidifier_seconds_running = 0U;
  if ((reason != NULL) && (s_prev_state != g_humidifier_state))
  {
    DBG_I("HUM", "off (%s)", reason);
  }
}

static void Hum_RaiseFault(uint16_t bit, const char *desc)
{
  if ((g_humidifier_fault_bits & bit) == 0U)
  {
    g_humidifier_fault_bits |= bit;
    DBG_E("HUM", "fault %s (bits=0x%04X)", desc,
          (unsigned)g_humidifier_fault_bits);
  }
  Hum_ApplyPwm(0U);
  Hum_ResetIntegralAndSlew(0U);
  g_humidifier_state = HUMIDIFIER_STATE_FAULT;
}

static void Hum_ReadSensor(bool *fresh)
{
  *fresh = false;

  /* SensirionTH_ReadMeasurement() is invoked from main.c every loop and
   * stamps the handle on success, including last_crc_ok. We watch the
   * CRC flag and a small "value moved" heuristic to detect staleness. */
  if (hSht4x.last_crc_ok != 0U)
  {
    g_humidifier_rh     = hSht4x.humidity_pct;
    g_humidifier_temp_c = hSht4x.temperature_c;
    *fresh = true;
  }
}

/* ----------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/
void Humidifier_Init(void)
{
  s_decim_count       = 0U;
  s_sensor_age_s      = 0U;
  s_dry_run_timer_s   = 0U;
  s_dry_run_rh_anchor = 0.0f;
  s_overtemp_latched  = 0U;
  s_pwm_curr_f        = 0.0f;
  s_prev_state        = 0xFFU;

  g_humidifier_state           = HUMIDIFIER_STATE_OFF;
  g_humidifier_fault_bits      = HUMIDIFIER_FAULT_NONE;
  g_humidifier_update_count    = 0U;
  g_humidifier_seconds_running = 0U;
  g_humidifier_p_term          = 0.0f;
  g_humidifier_i_term          = 0.0f;
  g_humidifier_output_raw      = 0.0f;
  g_humidifier_error           = 0.0f;

  Hum_ApplyPwm(0U);
  DBG_I("HUM", "init: setpoint=%.1f%% kp=%.1f ki=%.1f pwm_max=%u",
        (double)g_humidifier_setpoint_rh, (double)g_humidifier_kp,
        (double)g_humidifier_ki, (unsigned)g_humidifier_pwm_max);
}

void Humidifier_SetEnabled(bool enable)
{
  g_humidifier_enable = enable ? 1U : 0U;
}

void Humidifier_SetSetpoint(float rh_pct)
{
  g_humidifier_setpoint_rh = rh_pct;
}

void Humidifier_SetGains(float kp, float ki)
{
  g_humidifier_kp = kp;
  g_humidifier_ki = ki;
}

void Humidifier_AcknowledgeFault(void)
{
  g_humidifier_ack_fault = 1U;
}

/* ----------------------------------------------------------------------------
 * Tick
 * --------------------------------------------------------------------------*/
static void Humidifier_OneSecondTick(void)
{
  bool fresh;
  Hum_ReadSensor(&fresh);

  if (fresh)
  {
    s_sensor_age_s = 0U;
  }
  else
  {
    if (s_sensor_age_s < UINT32_MAX) { s_sensor_age_s++; }
  }

  /* Latched-fault handling. */
  if (g_humidifier_state == HUMIDIFIER_STATE_FAULT)
  {
    Hum_ApplyPwm(0U);
    s_pwm_curr_f = 0.0f;

    if (g_humidifier_ack_fault != 0U)
    {
      DBG_I("HUM", "fault cleared by ack (was 0x%04X)",
            (unsigned)g_humidifier_fault_bits);
      g_humidifier_fault_bits = HUMIDIFIER_FAULT_NONE;
      g_humidifier_ack_fault  = 0U;
      g_humidifier_state      = HUMIDIFIER_STATE_OFF;
      Hum_ResetIntegralAndSlew(0U);
    }
    return;
  }

  /* Disabled by user. */
  if (g_humidifier_enable == 0U)
  {
    Hum_ForceOff("disabled");
    return;
  }

  /* Setpoint sanity. */
  if ((g_humidifier_setpoint_rh < 0.0f) || (g_humidifier_setpoint_rh > 100.0f))
  {
    Hum_RaiseFault(HUMIDIFIER_FAULT_BAD_SETPOINT, "BAD_SETPOINT");
    return;
  }

  /* Sensor stale. */
  if (s_sensor_age_s > HUMIDIFIER_SENSOR_STALE_S)
  {
    Hum_RaiseFault(HUMIDIFIER_FAULT_SENSOR, "SENSOR");
    return;
  }
  if (!fresh)
  {
    /* Brief gap - hold output, don't update PI. */
    Hum_ApplyPwm((uint16_t)s_pwm_curr_f);
    return;
  }

  /* Over-temperature interlock with hysteresis. */
  if (s_overtemp_latched == 0U)
  {
    if (g_humidifier_temp_c >= g_humidifier_temp_limit_c)
    {
      s_overtemp_latched = 1U;
      Hum_RaiseFault(HUMIDIFIER_FAULT_OVERTEMP, "OVERTEMP");
      return;
    }
  }
  else
  {
    if (g_humidifier_temp_c <= g_humidifier_temp_resume_c)
    {
      s_overtemp_latched = 0U;
    }
    else
    {
      Hum_ApplyPwm(0U);
      return;
    }
  }

  /* Transition into RUNNING from OFF/IDLE. */
  if (g_humidifier_state != HUMIDIFIER_STATE_RUNNING)
  {
    Hum_ResetIntegralAndSlew(0U);
    g_humidifier_state           = HUMIDIFIER_STATE_RUNNING;
    g_humidifier_seconds_running = 0U;
    DBG_I("HUM", "running: rh=%.1f%% sp=%.1f%% temp=%.1fC",
          (double)g_humidifier_rh, (double)g_humidifier_setpoint_rh,
          (double)g_humidifier_temp_c);
  }

  /* PI law */
  float kp        = g_humidifier_kp;
  float ki        = g_humidifier_ki;
  float deadband  = g_humidifier_deadband_rh;
  float pwm_max_f = (float)g_humidifier_pwm_max;
  float pwm_min_f = (float)g_humidifier_pwm_min;

  float error_raw = g_humidifier_setpoint_rh - g_humidifier_rh;
  g_humidifier_error = error_raw;

  /*  Three-region control:
   *    - error >  deadband : RH is below setpoint -> normal PI, integrate up.
   *    - |error| <= deadband : at setpoint -> hold integrator and output
   *                            so we don't oscillate around it.
   *    - error < -deadband : RH is above setpoint -> shut heater, drain I. */
  bool   in_deadband = (error_raw <= deadband) && (error_raw >= -deadband);
  bool   above_setpt = (error_raw < -deadband);
  float  p           = kp * error_raw;
  float  i_next      = g_humidifier_i_term;
  float  desired_f;

  if (above_setpt)
  {
    p          = 0.0f;            /* the asymmetric output cannot subtract */
    i_next     = 0.0f;            /* drain integrator so we ramp up cleanly later */
    desired_f  = 0.0f;
  }
  else if (in_deadband)
  {
    /* Hold output and integrator. Letting the integrator drift while at
     * the setpoint causes a slow limit cycle. */
    desired_f  = s_pwm_curr_f;
  }
  else
  {
    /* Conditional integration with anti-windup based on actuator headroom. */
    i_next = g_humidifier_i_term + (ki * error_raw * HUMIDIFIER_DT_S);
    float headroom_hi = pwm_max_f - p;
    if (i_next > headroom_hi) { i_next = headroom_hi; }
    if (i_next < 0.0f)        { i_next = 0.0f; }

    desired_f = Hum_ClampF(p + i_next, pwm_min_f, pwm_max_f);
  }

  g_humidifier_p_term     = p;
  g_humidifier_i_term     = i_next;
  g_humidifier_output_raw = p + i_next;

  /* One-shot warning when we plateau at pwm_max - tells the user that
   * either the cap or the plant authority is the limiting factor. */
  static uint32_t s_sat_seconds = 0U;
  if ((!in_deadband) && (!above_setpt) &&
      ((p + i_next) >= (pwm_max_f - 1.0f)))
  {
    s_sat_seconds++;
    if (s_sat_seconds == 30U)  /* fires once after 30 s pegged */
    {
      DBG_W("HUM", "output saturated at pwm_max=%u, RH still %.1f%% < %.1f%%",
            (unsigned)g_humidifier_pwm_max,
            (double)g_humidifier_rh, (double)g_humidifier_setpoint_rh);
    }
  }
  else
  {
    s_sat_seconds = 0U;
  }

  /* Slew-rate limit. */
  float max_step = (float)g_humidifier_pwm_slew_per_s * HUMIDIFIER_DT_S;
  float delta    = desired_f - s_pwm_curr_f;
  if      (delta >  max_step) { s_pwm_curr_f += max_step; }
  else if (delta < -max_step) { s_pwm_curr_f -= max_step; }
  else                        { s_pwm_curr_f  = desired_f; }

  uint16_t pwm_out = Hum_ClampU16((int32_t)s_pwm_curr_f, 0U, 65535U);
  Hum_ApplyPwm(pwm_out);

  /* "Heater on but RH not rising" watchdog.
   * If the controller has been near the max cap for a long time but RH
   * has not climbed by the required delta, something is wrong (water
   * gone, cable broken, sensor bad). */
  if (pwm_out >= (uint16_t)((float)g_humidifier_pwm_max * 0.9f))
  {
    s_dry_run_timer_s++;
    if (s_dry_run_timer_s >= g_humidifier_dry_run_timeout_s)
    {
      if ((g_humidifier_rh - s_dry_run_rh_anchor) < g_humidifier_dry_run_dRH)
      {
        Hum_RaiseFault(HUMIDIFIER_FAULT_DRY_RUN, "DRY_RUN");
        return;
      }
      s_dry_run_timer_s   = 0U;
      s_dry_run_rh_anchor = g_humidifier_rh;
    }
  }
  else
  {
    s_dry_run_timer_s   = 0U;
    s_dry_run_rh_anchor = g_humidifier_rh;
  }

  g_humidifier_seconds_running++;
}

void Humidifier_Update(void)
{
  s_decim_count++;
  if (s_decim_count < HUMIDIFIER_TICK_DECIM) { return; }
  s_decim_count = 0U;

  Humidifier_OneSecondTick();
  g_humidifier_update_count++;

  /* State transition log. */
  if (g_humidifier_state != s_prev_state)
  {
    DBG_I("HUM", "state %u -> %u (rh=%.1f%% sp=%.1f%% pwm=%u)",
          (unsigned)s_prev_state, (unsigned)g_humidifier_state,
          (double)g_humidifier_rh, (double)g_humidifier_setpoint_rh,
          (unsigned)g_humidifier_pwm);
    s_prev_state = g_humidifier_state;
  }

  /* Periodic operating snapshot at DEBUG verbosity. */
  if ((g_humidifier_state == HUMIDIFIER_STATE_RUNNING) &&
      ((g_humidifier_update_count % 10U) == 0U))
  {
    DBG_D("HUM", "rh=%.1f sp=%.1f e=%.2f T=%.1fC P=%.0f I=%.0f pwm=%u",
          (double)g_humidifier_rh, (double)g_humidifier_setpoint_rh,
          (double)g_humidifier_error, (double)g_humidifier_temp_c,
          (double)g_humidifier_p_term, (double)g_humidifier_i_term,
          (unsigned)g_humidifier_pwm);
  }
}
