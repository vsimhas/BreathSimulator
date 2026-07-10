/**
  ******************************************************************************
  * @file    safety_supervisor.h
  * @brief   Independent CM7 safety supervisor (pressure faults + CM4 heartbeat).
  *
  *  Runs in a dedicated FreeRTOS task at osPriorityRealtime so it can preempt
  *  the sensor/therapy/PID loop. Calls Therapy_LatchFault() on trip.
  ******************************************************************************
  */

#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Pressure / pneumatic fault tunables (debugger-writable). */
extern volatile int32_t  g_safety_safe_rpm_max;
extern volatile float    g_safety_low_press_cmh2o;
extern volatile int32_t  g_safety_high_rpm_threshold;
extern volatile float    g_safety_low_press_timeout_s;
extern volatile float    g_safety_overpress_band_cmh2o;
extern volatile float    g_safety_overpress_timeout_s;
extern volatile float    g_safety_saturate_band_cmh2o;
extern volatile float    g_safety_saturated_timeout_s;
extern volatile uint8_t  g_safety_enabled;
extern volatile float    g_safety_grace_s;

/* CM4 inter-core heartbeat. */
extern volatile uint32_t g_safety_cm4_heartbeat_timeout_ms;
extern volatile uint32_t g_safety_cm4_last_seq;
extern volatile uint32_t g_safety_cm4_stale_ms;

void Safety_Supervisor_Init(void);
void Safety_Supervisor_ResetFaultTimers(void);
void Safety_Supervisor_Update(float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_SUPERVISOR_H */
