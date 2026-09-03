#ifndef BREATH_SIM_SETTINGS_HPP
#define BREATH_SIM_SETTINGS_HPP

#include <stdint.h>

extern "C" {
#include "breath_sim.h"
}

namespace breath_sim
{

/**
  * The editor works directly on BreathSimParams_t. There used to be a
  * parallel `Settings` struct plus two conversion functions, which meant the
  * screen, the USB parser and the waveform generator each carried their own
  * copy of the parameter list and the limits — and they drifted. There is
  * now one definition, and the limits come from breath_sim.h.
  */
using Settings = BreathSimParams_t;

/** Editor working copy, seeded from the backend on first use. */
Settings& getSettings();
/** Re-read the backend (e.g. after it normalised a derived field). */
void refreshSettings();

float   stepRateBpm(float v, int delta);
float   stepInspTime(float v, int delta);
float   stepIeRatio(float v, int delta);
int32_t stepRpm(int32_t v, int delta, int32_t min_v, int32_t max_v);
int32_t stepRpmBase(int32_t v, int delta);
int32_t stepRpmAmplitude(int32_t v, int delta);
uint8_t stepWaveform(uint8_t v, int delta);
uint8_t stepTimingMode(uint8_t v, int delta);
float   stepInspPause(float v, int delta);
float   stepExpPause(float v, int delta);
float   stepExpTau(float v, int delta);
float   stepFlattening(float v, int delta);
float   stepJitter(float v, int delta);

const char* timingModeName(uint8_t mode);

} // namespace breath_sim

#endif
