/**
  ******************************************************************************
  * @file    leak_estimator_vivo.h
  * @brief   Vivo50-style leak estimator (table x adaptive ratio).
  *
  *  Port of MainProcessor LeakageCalculations + FlowCalculations leak path:
  *    leak(t) = table[P] x MeanRatio / NORMFACTOR
  *    patient = measured - leak
  *    MeanRatio updated once per breath from RestVolume imbalance.
  ******************************************************************************
  */

#ifndef LEAK_ESTIMATOR_VIVO_H
#define LEAK_ESTIMATOR_VIVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define LEAK_VIVO_NORM_FACTOR        1000
#define LEAK_VIVO_STANDARD_RATIO     1000   /* table is absolute bench curve */
#define LEAK_VIVO_MAX_RATIO_CHANGE   5000
#define LEAK_VIVO_MAX_RATIO_VALUE    32000
#define LEAK_VIVO_MAX_REST_VOL_ML    8000
#define LEAK_VIVO_BACKUP_MS          15000U

void  LeakVivo_Init(void);
void  LeakVivo_Reset(void);

void  LeakVivo_Update(float total_q_slm, float mask_pressure_cmh2o,
                      float therapy_setpoint_cmh2o, float dt_s);

float LeakVivo_GetTotalSlm(void);
float LeakVivo_GetVentSlm(void);
float LeakVivo_GetUnintentionalSlm(void);
float LeakVivo_GetInstantExcessSlm(void);
float LeakVivo_GetPatientSlm(void);
float LeakVivo_GetPatientRawSlm(void);
bool  LeakVivo_IsHigh(void);

int32_t LeakVivo_GetMeanRatio(void);
int32_t LeakVivo_GetMeanRatio10Sec(void);
float   LeakVivo_GetReportedLeakSlm(void);

void    LeakVivo_SetHighThreshold(float slm);
float   LeakVivo_GetHighThreshold(void);
void    LeakVivo_SetStandardRatio(int32_t ratio);

/* Vivo MaskOn_Off_Check: Q*1000/table[P] (updated every 40 ms). */
int32_t LeakVivo_GetInstantRatio(void);

/* Pause per-breath MeanRatio adaptation during mask-off (Vivo disconnect). */
void    LeakVivo_SetAdaptationFreeze(bool freeze);
bool    LeakVivo_IsAdaptationFrozen(void);

/* Debugger-visible state */
extern volatile uint8_t  g_leak_vivo_breath_insp;
extern volatile int32_t  g_leak_vivo_mean_ratio;
extern volatile int32_t  g_leak_vivo_mean_ratio_10s;
extern volatile float    g_leak_vivo_flow_leak_slm;
extern volatile float    g_leak_vivo_rest_volume_ml;
extern volatile float    g_leak_vivo_patient_lpf_hz;

#ifdef __cplusplus
}
#endif

#endif /* LEAK_ESTIMATOR_VIVO_H */
