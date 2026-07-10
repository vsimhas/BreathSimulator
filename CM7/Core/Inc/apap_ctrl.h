/**
  ******************************************************************************
  * @file    apap_ctrl.h
  * @brief   AirSense-style Autoset (APAP) pressure titration.
  *
  *  Algorithm (AirSense 11–aligned behaviour)
  *  ---------------------------------------
  *    1. Therapy starts at minimum pressure; hold while breaths are detected.
  *    2. ~10 s without breath -> start FOT: ~2 cmH2O p-p @ 4 Hz on setpoint
 *       (mask p-p is lower due to circuit compliance; tune amplitude).
  *    3. FOT continues until breathing resumes (mask flow returns).
 *    4. Every eval window, compare AC mask pressure vs AC patient flow:
 *       Z_eff = dP_pp / dQ_pp (high when airway blocked). OSA when
 *       dP_pp is adequate and dQ_pp < threshold; central when dQ_pp is larger.
  *    5. AutoSet Response Standard / Soft sets raise gentleness.
  *    6. AutoSet for Her applies an additional gentleness scale.
  *
  *  Call Apap_Update() from Therapy_Update() after Epr_Update().
  ******************************************************************************
  */

#ifndef APAP_CTRL_H
#define APAP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    APAP_STATE_IDLE = 0,       /* therapy not running                    */
    APAP_STATE_MONITOR,        /* breathing: hold current pressure       */
    APAP_STATE_NO_BREATH,      /* counting time since last breath        */
    APAP_STATE_OSCILLATE,      /* continuous FOT until breath resumes    */
    APAP_STATE_RESPOND         /* legacy alias: same as OSCILLATE+raise  */
} ApapState_t;

void       Apap_Init(void);

void       Apap_OnTherapyStart(float min_cmh2o, float max_cmh2o,
                               uint8_t therapy_mode);
void       Apap_OnTherapyStop(void);
void       Apap_OnMaskReconnect(void);

/* AirSense AutoSet Response: false=Standard, true=Soft (gentler rises). */
void       Apap_SetResponseSoft(bool soft);
bool       Apap_IsResponseSoft(void);

float      Apap_Update(float dt_s, uint8_t therapy_state, bool mask_off);

float      Apap_GetTargetPressure(void);
ApapState_t Apap_GetState(void);
const char* Apap_GetStateName(void);
bool       Apap_IsOscillating(void);

/* FOT window metrics (updated during APAP_STATE_OSCILLATE). */
float      Apap_GetFotPressurePpCmh2o(void);
float      Apap_GetFotFlowPpSlm(void);
float      Apap_GetFotImpedanceCmh2oPerSlm(void);  /* dP_pp/dQ_pp; high => blocked */
float      Apap_GetOsaFlowPpThreshSlm(void);
bool       Apap_IsOsaLikely(void);
bool       Apap_IsOsaTreating(void);

#ifdef __cplusplus
}
#endif

#endif /* APAP_CTRL_H */
