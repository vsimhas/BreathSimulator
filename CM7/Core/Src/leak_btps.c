/**
  ******************************************************************************
  * @file    leak_btps.c
  * @brief   BTPS correction stubs (unity gain until sensors are available).
  ******************************************************************************
  */

#include "leak_btps.h"

static float s_temp_factor = 1.0f;
static float s_humid_factor = 1.0f;
static float s_alt_factor = 1.0f;

static float clampUnity(float f)
{
    if (f < 0.25f) return 0.25f;
    if (f > 4.0f)  return 4.0f;
    return f;
}

static int32_t applyFactors(int32_t flow_mlps)
{
    const float f = s_temp_factor * s_humid_factor * s_alt_factor;
    const float out = (float)flow_mlps * f;
    if (out <= 0.0f) return 0;
    return (int32_t)(out + 0.5f);
}

void LeakBtps_Init(void)
{
    s_temp_factor  = 1.0f;
    s_humid_factor = 1.0f;
    s_alt_factor   = 1.0f;
}

int32_t LeakBtps_TableToBtpsMlps(int32_t flow_mlps_stpd)
{
    return applyFactors(flow_mlps_stpd);
}

int32_t LeakBtps_MeasuredStpdToBtpsMlps(int32_t flow_mlps_stpd)
{
    return applyFactors(flow_mlps_stpd);
}

void LeakBtps_SetTemperatureFactor(float factor) { s_temp_factor  = clampUnity(factor); }
void LeakBtps_SetHumidityFactor(float factor)    { s_humid_factor = clampUnity(factor); }
void LeakBtps_SetAltitudeFactor(float factor)    { s_alt_factor   = clampUnity(factor); }

float LeakBtps_GetTemperatureFactor(void) { return s_temp_factor; }
float LeakBtps_GetHumidityFactor(void)    { return s_humid_factor; }
float LeakBtps_GetAltitudeFactor(void)      { return s_alt_factor; }
