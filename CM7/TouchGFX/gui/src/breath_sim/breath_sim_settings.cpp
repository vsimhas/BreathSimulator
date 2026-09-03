#include <gui/breath_sim/breath_sim_settings.hpp>

#include <stdio.h>

namespace breath_sim
{

namespace
{

Settings g_settings;
bool     g_init = false;

float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

uint8_t stepEnum(uint8_t v, int delta, uint8_t count)
{
    int nv = static_cast<int>(v) + delta;
    if (nv < 0) { nv = 0; }
    if (nv >= static_cast<int>(count)) { nv = static_cast<int>(count) - 1; }
    return static_cast<uint8_t>(nv);
}

} // namespace

Settings& getSettings()
{
    if (!g_init)
    {
        BreathSim_GetParams(&g_settings);
        g_init = true;
    }
    return g_settings;
}

void refreshSettings()
{
    BreathSim_GetParams(&g_settings);
    g_init = true;
}

/* Every limit below comes from breath_sim.h so the screen can never offer a
 * value the waveform generator would then quietly clamp. */

float stepRateBpm(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.5f,
                  BREATH_RATE_MIN_BPM, BREATH_RATE_MAX_BPM);
}

float stepInspTime(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.1f,
                  BREATH_INSP_MIN_S, BREATH_INSP_MAX_S);
}

float stepIeRatio(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.5f,
                  BREATH_IE_RATIO_MIN, BREATH_IE_RATIO_MAX);
}

int32_t stepRpm(int32_t v, int delta, int32_t min_v, int32_t max_v)
{
    return clampi(v + static_cast<int32_t>(delta) * 250, min_v, max_v);
}

int32_t stepRpmBase(int32_t v, int delta)
{
    return stepRpm(v, delta, BREATH_RPM_BASE_MIN, BREATH_RPM_BASE_MAX);
}

int32_t stepRpmAmplitude(int32_t v, int delta)
{
    return stepRpm(v, delta, 0, BREATH_RPM_AMP_MAX);
}

uint8_t stepWaveform(uint8_t v, int delta)
{
    uint8_t nv = stepEnum(v, delta, static_cast<uint8_t>(BREATH_WAVE_COUNT));
    /* Skip the table entry when nothing has been uploaded over USB. */
    if ((nv == static_cast<uint8_t>(BREATH_WAVE_TABLE)) &&
        (BreathSim_GetTableLength() < 2U))
    {
        nv = (delta >= 0) ? static_cast<uint8_t>(BREATH_WAVE_SINE)
                          : static_cast<uint8_t>(BREATH_WAVE_SQUARE);
    }
    return nv;
}

uint8_t stepTimingMode(uint8_t v, int delta)
{
    return stepEnum(v, delta, static_cast<uint8_t>(BREATH_TIMING_COUNT));
}

float stepInspPause(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.1f,
                  0.0f, BREATH_INSP_PAUSE_MAX_S);
}

float stepExpPause(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.1f,
                  0.0f, BREATH_EXP_PAUSE_MAX_S);
}

float stepExpTau(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.05f,
                  BREATH_EXP_TAU_MIN_S, BREATH_EXP_TAU_MAX_S);
}

float stepFlattening(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 0.05f,
                  0.0f, BREATH_FLATTENING_MAX);
}

float stepJitter(float v, int delta)
{
    return clampf(v + static_cast<float>(delta) * 1.0f,
                  0.0f, BREATH_JITTER_MAX_PCT);
}

const char* timingModeName(uint8_t mode)
{
    return (mode == static_cast<uint8_t>(BREATH_TIMING_EXPLICIT))
               ? "Insp+I:E" : "Rate+I:E";
}

} // namespace breath_sim
