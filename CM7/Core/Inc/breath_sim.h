/**
  * @file    breath_sim.h
  * @brief   Open-loop blower RPM waveform for breath simulation.
  *
  * Drives the CM4 motor via BlowerIpc with a periodic inspiration /
  * expiration profile. No pressure sensors are required.
  */

#ifndef BREATH_SIM_H
#define BREATH_SIM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  BREATH_WAVE_SINE = 0,
  BREATH_WAVE_RAMP,
  BREATH_WAVE_SQUARE,
  BREATH_WAVE_COUNT
} BreathWaveform_t;

typedef struct
{
  float    rate_bpm;        /**< Breaths per minute (4..60). */
  float    insp_time_s;     /**< Inspiration time in seconds (0.3..4.0). */
  float    ie_ratio_exp;    /**< Expiratory time = insp * ratio (1..4, I:1..I:4). */
  int32_t  rpm_base;        /**< RPM at end-expiration / baseline flow. */
  int32_t  rpm_amplitude;   /**< Additional RPM at peak inspiration. */
  uint8_t  waveform;        /**< BreathWaveform_t */
  float    insp_pause_s;    /**< Plateau at peak inspiration (0..1.0 s). */
  float    exp_pause_s;     /**< Pause at baseline before next breath (0..2.0 s). */
} BreathSimParams_t;

void BreathSim_Init(void);
void BreathSim_GetParams(BreathSimParams_t *out);
void BreathSim_SetParams(const BreathSimParams_t *params);

void BreathSim_Start(void);
void BreathSim_Stop(void);
bool BreathSim_IsRunning(void);

/** Call at fixed rate (e.g. 200 Hz) from the breath task. */
void BreathSim_Update(float dt_s);

int32_t BreathSim_GetTargetRpm(void);
float   BreathSim_GetPhase(void);
float   BreathSim_GetCyclePeriodS(void);
float   BreathSim_GetEnvelope(void);

const char *BreathSim_WaveformName(uint8_t waveform);

#ifdef __cplusplus
}
#endif

#endif /* BREATH_SIM_H */
