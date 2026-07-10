#include <gui/breath_sim/breath_sim_settings.hpp>

#include <stdio.h>

namespace breath_sim
{

static Settings g_settings;

Settings& getSettings()
{
    static bool init = false;
    if (!init)
    {
        BreathSimParams_t p;
        BreathSim_GetParams(&p);
        paramsToSettings(p, &g_settings);
        init = true;
    }
    return g_settings;
}

uint8_t stepRateBpm(uint8_t v, int delta)
{
    int nv = static_cast<int>(v) + delta;
    if (nv < 4) { nv = 4; }
    if (nv > 60) { nv = 60; }
    return static_cast<uint8_t>(nv);
}

float stepInspTime(float v, int delta)
{
    float nv = v + static_cast<float>(delta) * 0.1f;
    if (nv < 0.3f) { nv = 0.3f; }
    if (nv > 4.0f) { nv = 4.0f; }
    return nv;
}

uint8_t stepIeRatio(uint8_t v, int delta)
{
    int nv = static_cast<int>(v) + delta;
    if (nv < 1) { nv = 1; }
    if (nv > 4) { nv = 4; }
    return static_cast<uint8_t>(nv);
}

int32_t stepRpm(int32_t v, int delta, int32_t min_v, int32_t max_v)
{
    int32_t nv = v + static_cast<int32_t>(delta) * 250;
    if (nv < min_v) { nv = min_v; }
    if (nv > max_v) { nv = max_v; }
    return nv;
}

int32_t stepRpmBase(int32_t v, int delta)
{
    return stepRpm(v, delta, 2000, 20000);
}

Waveform stepWaveform(Waveform v, int delta)
{
    int nv = static_cast<int>(v) + delta;
    if (nv < 0) { nv = 0; }
    if (nv >= static_cast<int>(Waveform::NUM_VALUES)) { nv = static_cast<int>(Waveform::NUM_VALUES) - 1; }
    return static_cast<Waveform>(nv);
}

float stepPause(float v, int delta)
{
    float nv = v + static_cast<float>(delta) * 0.1f;
    if (nv < 0.0f) { nv = 0.0f; }
    if (nv > 1.0f) { nv = 1.0f; }
    return nv;
}

const char* toString(Waveform v)
{
    switch (v)
    {
        case Waveform::RAMP:   return "Ramp";
        case Waveform::SQUARE: return "Square";
        case Waveform::SINE:
        default:               return "Sine";
    }
}

void settingsToParams(const Settings& s, BreathSimParams_t* out)
{
    if (out == nullptr) { return; }
    out->rate_bpm = static_cast<float>(s.rate_bpm);
    out->insp_time_s = s.insp_time_s;
    out->ie_ratio_exp = static_cast<float>(s.ie_ratio_exp);
    out->rpm_base = s.rpm_base;
    out->rpm_amplitude = s.rpm_amplitude;
    out->waveform = static_cast<uint8_t>(s.waveform);
    out->insp_pause_s = s.insp_pause_s;
    out->exp_pause_s = s.exp_pause_s;
}

void paramsToSettings(const BreathSimParams_t& in, Settings* out)
{
    if (out == nullptr) { return; }
    out->rate_bpm = static_cast<uint8_t>(in.rate_bpm);
    out->insp_time_s = in.insp_time_s;
    out->ie_ratio_exp = static_cast<uint8_t>(in.ie_ratio_exp);
    out->rpm_base = in.rpm_base;
    out->rpm_amplitude = in.rpm_amplitude;
    out->waveform = static_cast<Waveform>(in.waveform);
    out->insp_pause_s = in.insp_pause_s;
    out->exp_pause_s = in.exp_pause_s;
}

} // namespace breath_sim
