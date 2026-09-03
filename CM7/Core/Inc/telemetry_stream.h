/**
  ******************************************************************************
  * @file    telemetry_stream.h
  * @brief   Breath simulator telemetry + USB command interface.
  ******************************************************************************
  */

#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

#include <stdint.h>

#include "breath_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TELEM_ENABLED
#define TELEM_ENABLED  1
#endif

#if TELEM_ENABLED

extern volatile uint8_t  g_telem_enabled;
extern volatile uint32_t g_telem_sample_count;
extern volatile uint32_t g_telem_dropped_count;

void Telem_Init(void);
void Telem_Pump(void);
void Telem_ProcessCommands(void);
void Telem_PushBreathSample(const BreathSimStatus_t *status);
void Telem_OnRx(const uint8_t *data, uint32_t len);

#else

#define Telem_Init() ((void)0)
#define Telem_Pump() ((void)0)
#define Telem_ProcessCommands() ((void)0)
#define Telem_PushBreathSample(s) ((void)0)

#endif /* TELEM_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_STREAM_H */
