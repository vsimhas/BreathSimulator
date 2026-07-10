/**
  ******************************************************************************
  * @file    leak_btps.h
  * @brief   STPD/BTPS flow correction (Vivo-style hook points).
  *
  *  MainProcessor applies temperature, barometric, and humidity correction when
  *  converting reference table flow and measured flow. Until environmental
  *  sensors are wired, all correction factors are unity (1.0).
  ******************************************************************************
  */

#ifndef LEAK_BTPS_H
#define LEAK_BTPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void LeakBtps_Init(void);

/* Reference table value [ml/s STPD] -> patient BTPS [ml/s]. */
int32_t LeakBtps_TableToBtpsMlps(int32_t flow_mlps_stpd);

/* Measured STPD flow [ml/s] -> BTPS [ml/s] (identity for now). */
int32_t LeakBtps_MeasuredStpdToBtpsMlps(int32_t flow_mlps_stpd);

/* Runtime overrides (default 1.0). Future: drive from temp/RH/baro sensors. */
void    LeakBtps_SetTemperatureFactor(float factor);
void    LeakBtps_SetHumidityFactor(float factor);
void    LeakBtps_SetAltitudeFactor(float factor);

float   LeakBtps_GetTemperatureFactor(void);
float   LeakBtps_GetHumidityFactor(void);
float   LeakBtps_GetAltitudeFactor(void);

#ifdef __cplusplus
}
#endif

#endif /* LEAK_BTPS_H */
