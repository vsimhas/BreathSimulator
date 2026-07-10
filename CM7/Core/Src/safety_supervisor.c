/**
  ******************************************************************************
  * @file    safety_supervisor.c
  * @brief   Independent safety supervisor task logic.
  ******************************************************************************
  */

#include "safety_supervisor.h"
#include "therapy.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "dbg_log.h"
#include "stm32h7xx_hal.h"

#include <math.h>

#define SAFETY_SENSOR_MIN_CMH2O     (-10.0f)
#define SAFETY_SENSOR_MAX_CMH2O     ( 50.0f)
#define SAFETY_SENSOR_TIMEOUT_S     ( 2.0f)

volatile int32_t  g_safety_safe_rpm_max           = 30000;
volatile float    g_safety_low_press_cmh2o        = 1.5f;
volatile int32_t  g_safety_high_rpm_threshold     = 12000;
volatile float    g_safety_low_press_timeout_s    = 4.0f;
volatile float    g_safety_overpress_band_cmh2o   = 5.0f;
volatile float    g_safety_overpress_timeout_s    = 3.0f;
volatile float    g_safety_saturate_band_cmh2o    = 1.5f;
volatile float    g_safety_saturated_timeout_s     = 6.0f;
volatile uint8_t  g_safety_enabled                = 1U;
volatile float    g_safety_grace_s                = 10.0f;

volatile uint32_t g_safety_cm4_heartbeat_timeout_ms = 100U;
volatile uint32_t g_safety_cm4_last_seq           = 0U;
volatile uint32_t g_safety_cm4_stale_ms           = 0U;

static uint32_t s_low_press_ms   = 0U;
static uint32_t s_overpress_ms   = 0U;
static uint32_t s_outrange_ms    = 0U;
static uint32_t s_saturated_ms   = 0U;
static uint32_t s_cm4_seen_seq   = 0U;
static uint32_t s_cm4_last_change_ms = 0U;
static uint8_t  s_cm4_seq_valid  = 0U;

static void resetFaultTimers(void)
{
    s_low_press_ms  = 0U;
    s_overpress_ms  = 0U;
    s_outrange_ms   = 0U;
    s_saturated_ms  = 0U;
}

static bool therapyNeedsCm4Heartbeat(void)
{
    const TherapyState_t st = Therapy_GetState();
    return (st == THERAPY_STATE_RAMP)
        || (st == THERAPY_STATE_RUNNING)
        || (st == THERAPY_STATE_STOPPING);
}

static void updateCm4Heartbeat(uint32_t dt_ms)
{
    uint32_t seq;

    __DMB();
    seq = g_blower_ipc.cm4_status_seq;
    __DMB();

    g_safety_cm4_last_seq = seq;

    if (!s_cm4_seq_valid || (seq != s_cm4_seen_seq))
    {
        s_cm4_seen_seq = seq;
        s_cm4_last_change_ms = HAL_GetTick();
        s_cm4_seq_valid = 1U;
        g_safety_cm4_stale_ms = 0U;
        return;
    }

    if (!therapyNeedsCm4Heartbeat())
    {
        g_safety_cm4_stale_ms = 0U;
        return;
    }

    const uint32_t now = HAL_GetTick();
    g_safety_cm4_stale_ms = now - s_cm4_last_change_ms;

    const uint32_t limit = (g_safety_cm4_heartbeat_timeout_ms > 0U)
                               ? g_safety_cm4_heartbeat_timeout_ms
                               : 1U;
    if (g_safety_cm4_stale_ms >= limit)
    {
        Therapy_LatchFault(THERAPY_FAULT_CM4_HEARTBEAT,
                           "CM4 status heartbeat stale");
    }

    (void)dt_ms;
}

static bool runPressureChecks(uint32_t dt_ms)
{
    const float meas   = Therapy_GetMeasuredCmh2o();
    const float target = Therapy_GetTargetPressure();
    const int32_t rpm  = (int32_t)BlowerCtrl_GetOutputRpm();
    const bool grace   = Therapy_SafetyGraceActive();

    if ((meas < SAFETY_SENSOR_MIN_CMH2O) || (meas > SAFETY_SENSOR_MAX_CMH2O))
    {
        s_outrange_ms += dt_ms;
        if (s_outrange_ms >= (uint32_t)(SAFETY_SENSOR_TIMEOUT_S * 1000.0f))
        {
            Therapy_LatchFault(THERAPY_FAULT_SENSOR_OUT_OF_RANGE,
                               "pressure reading outside plausible range");
            return true;
        }
    }
    else
    {
        s_outrange_ms = 0U;
    }

    if (!grace)
    {
        const float p_sensor = BlowerCtrl_GetSensorCmh2o();
        if ((rpm > g_safety_high_rpm_threshold)
            && (p_sensor < g_safety_low_press_cmh2o))
        {
            s_low_press_ms += dt_ms;
            if (s_low_press_ms
                  >= (uint32_t)(g_safety_low_press_timeout_s * 1000.0f))
            {
                Therapy_LatchFault(THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM,
                                   "low pressure while RPM high (sensor/hose lost?)");
                return true;
            }
        }
        else
        {
            s_low_press_ms = 0U;
        }
    }
    else
    {
        s_low_press_ms = 0U;
    }

    if (!grace && (meas > (target + g_safety_overpress_band_cmh2o)))
    {
        s_overpress_ms += dt_ms;
        if (s_overpress_ms >= (uint32_t)(g_safety_overpress_timeout_s * 1000.0f))
        {
            Therapy_LatchFault(THERAPY_FAULT_OVER_PRESSURE,
                               "measured pressure exceeds target+band");
            return true;
        }
    }
    else
    {
        s_overpress_ms = 0U;
    }

    if (!grace)
    {
        const bool at_safe_max = (rpm >= g_safety_safe_rpm_max);
        const bool not_close   = (fabsf(target - meas) > g_safety_saturate_band_cmh2o);
        if (at_safe_max && not_close)
        {
            s_saturated_ms += dt_ms;
            if (s_saturated_ms >= (uint32_t)(g_safety_saturated_timeout_s * 1000.0f))
            {
                Therapy_LatchFault(THERAPY_FAULT_RPM_SATURATED,
                                   "RPM saturated without reaching target");
                return true;
            }
        }
        else
        {
            s_saturated_ms = 0U;
        }
    }
    else
    {
        s_saturated_ms = 0U;
    }

    return false;
}

void Safety_Supervisor_Init(void)
{
    resetFaultTimers();
    s_cm4_seq_valid = 0U;
    s_cm4_seen_seq = 0U;
    s_cm4_last_change_ms = HAL_GetTick();
    g_safety_cm4_stale_ms = 0U;

    extern volatile int32_t g_blower_ctrl_rpm_max;
    g_blower_ctrl_rpm_max = g_safety_safe_rpm_max;
}

void Safety_Supervisor_ResetFaultTimers(void)
{
    resetFaultTimers();
}

void Safety_Supervisor_Update(float dt_s)
{
    if (!(dt_s > 0.0f) || (dt_s > 0.5f))
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    updateCm4Heartbeat(dt_ms);

    if (g_safety_enabled == 0U)
    {
        return;
    }

    if (Therapy_GetFault() != THERAPY_FAULT_NONE)
    {
        return;
    }

    const TherapyState_t st = Therapy_GetState();
    if ((st != THERAPY_STATE_RAMP) && (st != THERAPY_STATE_RUNNING))
    {
        resetFaultTimers();
        return;
    }

    (void)runPressureChecks(dt_ms);
}
