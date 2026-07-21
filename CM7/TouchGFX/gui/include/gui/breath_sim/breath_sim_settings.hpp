#ifndef BREATH_SIM_SETTINGS_HPP
#define BREATH_SIM_SETTINGS_HPP

#include <stdint.h>

extern "C" {
#include "breath_sim.h"
}

namespace breath_sim
{

enum class Waveform : uint8_t
{
    SINE = 0,
    RAMP,
    SQUARE,
    NUM_VALUES
};

struct Settings
{
    uint8_t  rate_bpm;
    float    insp_time_s;
    uint8_t  ie_ratio_exp;
    int32_t  rpm_base;
    int32_t  rpm_amplitude;
    Waveform waveform;
    float    insp_pause_s;
    float    exp_pause_s;
};

Settings& getSettings();

uint8_t  stepRateBpm(uint8_t v, int delta);
float    stepInspTime(float v, int delta);
uint8_t  stepIeRatio(uint8_t v, int delta);
int32_t  stepRpm(int32_t v, int delta, int32_t min_v, int32_t max_v);
int32_t  stepRpmBase(int32_t v, int delta);
Waveform stepWaveform(Waveform v, int delta);
float    stepPause(float v, int delta);
float    stepExpPause(float v, int delta);

const char* toString(Waveform v);

void settingsToParams(const Settings& s, BreathSimParams_t* out);
void paramsToSettings(const BreathSimParams_t& in, Settings* out);

} // namespace breath_sim

#endif
