#include <gui/breath_sim/breath_sim_menu.hpp>
#include <gui/breath_sim/breath_sim_settings.hpp>

extern "C" {
#include "breath_sim.h"
}

#include <stdio.h>

namespace breath_sim
{

namespace
{

struct ListItem
{
    const char* label;
    Page        target_page;
    Mode        target_mode;
};

const ListItem k_home_items[] = {
    { "Settings",         Page::SETTINGS,    Mode::LIST },
    { "Start Simulation", Page::RUNNING,     Mode::INFO },
    { "Events",           Page::EVENTS,      Mode::LIST },
    { "Diagnostics",      Page::DIAGNOSTICS, Mode::INFO },
};

const ListItem k_settings_items[] = {
    { "<- Back",      Page::HOME,             Mode::LIST },
    { "Timing Mode",  Page::VALUE_TIMING,     Mode::VALUE_EDIT },
    { "Breath Rate",  Page::VALUE_RATE,       Mode::VALUE_EDIT },
    { "Insp Time",    Page::VALUE_INSP_TIME,  Mode::VALUE_EDIT },
    { "I:E Ratio",    Page::VALUE_IE_RATIO,   Mode::VALUE_EDIT },
    { "RPM Base",     Page::VALUE_RPM_BASE,   Mode::VALUE_EDIT },
    { "Amplitude",    Page::VALUE_RPM_AMP,    Mode::VALUE_EDIT },
    { "Waveform",     Page::VALUE_WAVEFORM,   Mode::VALUE_EDIT },
    { "Exp Tau",      Page::VALUE_EXP_TAU,    Mode::VALUE_EDIT },
    { "Flattening",   Page::VALUE_FLATTENING, Mode::VALUE_EDIT },
    { "Jitter",       Page::VALUE_JITTER,     Mode::VALUE_EDIT },
    { "Insp Pause",   Page::VALUE_INSP_PAUSE, Mode::VALUE_EDIT },
    { "Exp Pause",    Page::VALUE_EXP_PAUSE,  Mode::VALUE_EDIT },
};

/* Respiratory events, triggerable from the rig without a host PC. An APAP
 * only raises pressure in response to these, so a bench session that cannot
 * produce them is not exercising the "AP" half of the device under test. */
struct EventItem
{
    uint8_t type;
    float   duration_s;
    float   severity;
};

/* Parallel to k_events_items[1..4]; the labels live there only. */
const EventItem k_event_items[] = {
    { static_cast<uint8_t>(BREATH_EVENT_APNEA),      20.0f, 1.00f },
    { static_cast<uint8_t>(BREATH_EVENT_HYPOPNEA),   30.0f, 0.50f },
    { static_cast<uint8_t>(BREATH_EVENT_FLOW_LIMIT), 30.0f, 0.80f },
    { static_cast<uint8_t>(BREATH_EVENT_CSR),       180.0f, 0.60f },
};

const ListItem k_events_items[] = {
    { "<- Back",           Page::HOME,   Mode::LIST },
    { "Apnea 20 s",        Page::EVENTS, Mode::LIST },
    { "Hypopnea 30 s 50%", Page::EVENTS, Mode::LIST },
    { "Flow Limit 30 s",   Page::EVENTS, Mode::LIST },
    { "Cheyne-Stokes 3 m", Page::EVENTS, Mode::LIST },
    { "Cancel Event",      Page::EVENTS, Mode::LIST },
};

template <size_t N>
constexpr uint8_t arrLen(const ListItem (&)[N])
{
    return static_cast<uint8_t>(N);
}

const ListItem* itemsForPage(Page p, uint8_t& out_count)
{
    switch (p)
    {
        case Page::HOME:
            out_count = arrLen(k_home_items);
            return k_home_items;
        case Page::SETTINGS:
            out_count = arrLen(k_settings_items);
            return k_settings_items;
        case Page::EVENTS:
            out_count = arrLen(k_events_items);
            return k_events_items;
        default:
            out_count = 0;
            return nullptr;
    }
}

Page parentList(Page p)
{
    switch (p)
    {
        case Page::VALUE_RATE:
        case Page::VALUE_INSP_TIME:
        case Page::VALUE_IE_RATIO:
        case Page::VALUE_RPM_BASE:
        case Page::VALUE_RPM_AMP:
        case Page::VALUE_WAVEFORM:
        case Page::VALUE_INSP_PAUSE:
        case Page::VALUE_EXP_PAUSE:
        case Page::VALUE_TIMING:
        case Page::VALUE_EXP_TAU:
        case Page::VALUE_FLATTENING:
        case Page::VALUE_JITTER:
            return Page::SETTINGS;
        case Page::SETTINGS:
        case Page::EVENTS:
        case Page::RUNNING:
        case Page::DIAGNOSTICS:
            return Page::HOME;
        default:
            return Page::HOME;
    }
}

uint8_t selectedIndexForChild(Page parent, Page child)
{
    uint8_t count;
    const ListItem* items = itemsForPage(parent, count);
    if (items == nullptr) { return 0; }
    for (uint8_t i = 0; i < count; ++i)
    {
        if (items[i].target_page == child) { return i; }
    }
    return 0;
}

MenuController g_menu;
char g_editor_value_buf[32] = {0};

} // namespace

MenuController::MenuController()
    : current_page_(Page::WELCOME),
      current_mode_(Mode::INFO),
      selected_index_(0U),
      needs_redraw_(true),
      sim_running_(false)
{
}

MenuController& getMenu()
{
    return g_menu;
}

uint8_t MenuController::itemCount() const
{
    uint8_t count = 0U;
    (void)itemsForPage(current_page_, count);
    return count;
}

const char* MenuController::itemLabel(uint8_t idx) const
{
    uint8_t count;
    const ListItem* items = itemsForPage(current_page_, count);
    if (items == nullptr || idx >= count) { return nullptr; }
    return items[idx].label;
}

const char* MenuController::itemValue(uint8_t idx) const
{
    if (current_page_ != Page::SETTINGS || idx == 0U) { return nullptr; }

    const Settings& s = getSettings();
    BreathSimTiming_t t;
    BreathSim_GetTiming(&t);

    /* Whichever of rate / inspiratory time is derived is shown with a "="
     * prefix, so the operator can see at a glance which knob is in charge
     * and which one the timing model is computing for them. */
    const bool rate_derived =
        (s.timing_mode == static_cast<uint8_t>(BREATH_TIMING_EXPLICIT));

    static char buf[24];
    switch (idx)
    {
        case 1: snprintf(buf, sizeof(buf), "%s", timingModeName(s.timing_mode)); break;
        case 2: snprintf(buf, sizeof(buf), "%s%.1f BPM",
                         rate_derived ? "=" : "", (double)t.rate_bpm); break;
        case 3: snprintf(buf, sizeof(buf), "%s%.2f s",
                         rate_derived ? "" : "=", (double)t.insp_s); break;
        case 4: snprintf(buf, sizeof(buf), "1:%.1f", (double)s.ie_ratio_exp); break;
        case 5: snprintf(buf, sizeof(buf), "%ld%s", (long)s.rpm_base,
                         (s.rpm_base < BREATH_RPM_OBS_MIN_RPM) ? " !" : ""); break;
        case 6: snprintf(buf, sizeof(buf), "%ld", (long)s.rpm_amplitude); break;
        case 7: snprintf(buf, sizeof(buf), "%s", BreathSim_WaveformName(s.waveform)); break;
        case 8: snprintf(buf, sizeof(buf), "%.2f s", (double)s.exp_tau_s); break;
        case 9: snprintf(buf, sizeof(buf), "%d %%", (int)(s.flattening * 100.0f)); break;
        case 10: snprintf(buf, sizeof(buf), "%d %%", (int)s.jitter_pct); break;
        case 11: snprintf(buf, sizeof(buf), "%.1f s", (double)s.insp_pause_s); break;
        case 12: snprintf(buf, sizeof(buf), "%.1f s", (double)s.exp_pause_s); break;
        default: return nullptr;
    }
    return buf;
}

const char* MenuController::editorValueText() const
{
    const Settings& s = getSettings();
    BreathSimTiming_t t;
    BreathSim_GetTiming(&t);

    char* b = g_editor_value_buf;
    const size_t n = sizeof(g_editor_value_buf);

    switch (current_page_)
    {
        case Page::VALUE_TIMING:     snprintf(b, n, "%s", timingModeName(s.timing_mode)); break;
        case Page::VALUE_RATE:       snprintf(b, n, "%.1f BPM", (double)s.rate_bpm); break;
        case Page::VALUE_INSP_TIME:  snprintf(b, n, "%.2f s", (double)s.insp_time_s); break;
        case Page::VALUE_IE_RATIO:   snprintf(b, n, "1:%.1f", (double)s.ie_ratio_exp); break;
        case Page::VALUE_RPM_BASE:   snprintf(b, n, "%ld RPM", (long)s.rpm_base); break;
        case Page::VALUE_RPM_AMP:    snprintf(b, n, "%ld RPM", (long)s.rpm_amplitude); break;
        case Page::VALUE_WAVEFORM:   snprintf(b, n, "%s", BreathSim_WaveformName(s.waveform)); break;
        case Page::VALUE_EXP_TAU:    snprintf(b, n, "%.2f s", (double)s.exp_tau_s); break;
        case Page::VALUE_FLATTENING: snprintf(b, n, "%d %%", (int)(s.flattening * 100.0f)); break;
        case Page::VALUE_JITTER:     snprintf(b, n, "%d %%", (int)s.jitter_pct); break;
        case Page::VALUE_INSP_PAUSE: snprintf(b, n, "%.1f s", (double)s.insp_pause_s); break;
        case Page::VALUE_EXP_PAUSE:  snprintf(b, n, "%.1f s", (double)s.exp_pause_s); break;
        default: b[0] = '\0'; break;
    }
    return b;
}

const char* MenuController::editorTitle() const
{
    switch (current_page_)
    {
        case Page::WELCOME:          return "Breath Simulator";
        case Page::HOME:             return "Home";
        case Page::SETTINGS:         return "Settings";
        case Page::VALUE_RATE:       return "Breath Rate";
        case Page::VALUE_INSP_TIME:  return "Insp Time";
        case Page::VALUE_IE_RATIO:   return "I:E Ratio";
        case Page::VALUE_RPM_BASE:   return "RPM Base";
        case Page::VALUE_RPM_AMP:    return "Amplitude";
        case Page::VALUE_WAVEFORM:   return "Waveform";
        case Page::VALUE_INSP_PAUSE: return "Insp Pause";
        case Page::VALUE_EXP_PAUSE:  return "Exp Pause";
        case Page::VALUE_TIMING:     return "Timing Mode";
        case Page::VALUE_EXP_TAU:    return "Exp Tau";
        case Page::VALUE_FLATTENING: return "Flattening";
        case Page::VALUE_JITTER:     return "Jitter";
        case Page::EVENTS:           return "Events";
        case Page::RUNNING:          return "Running";
        case Page::DIAGNOSTICS:      return "Diagnostics";
        default:                     return "Breath Simulator";
    }
}

const char* MenuController::editorHint() const
{
    if (current_mode_ == Mode::VALUE_EDIT)
    {
        return "Rotate to change, press to save";
    }
    if (current_page_ == Page::WELCOME)
    {
        return "Press to continue";
    }
    if (current_page_ == Page::RUNNING)
    {
        return "Press to stop simulation";
    }
    if (current_page_ == Page::EVENTS)
    {
        return "Press to inject event";
    }
    return "Rotate to select, press to enter";
}

void MenuController::applySettingsToBackend()
{
    /* SetParams normalises and writes back the derived fields, so pull the
     * result straight into the editor copy. That is what makes the "="
     * readbacks on the settings list show what will actually run. */
    BreathSim_SetParams(&getSettings());
    refreshSettings();
}

void MenuController::goTo(Page p, Mode m, uint8_t initial_index)
{
    current_page_ = p;
    current_mode_ = m;
    selected_index_ = initial_index;
    needs_redraw_ = true;
}

void MenuController::goBack()
{
    if (current_mode_ == Mode::VALUE_EDIT)
    {
        applySettingsToBackend();
        Page parent = parentList(current_page_);
        goTo(parent, Mode::LIST, selectedIndexForChild(parent, current_page_));
        return;
    }

    if (current_page_ == Page::RUNNING)
    {
        BreathSim_Stop();
        sim_running_ = false;
        goTo(Page::HOME, Mode::LIST, 0U);
        return;
    }

    Page parent = parentList(current_page_);
    goTo(parent, Mode::LIST, (parent == Page::SETTINGS) ? 1U : 0U);
}

void MenuController::enterSelectedItem()
{
    uint8_t count;
    const ListItem* items = itemsForPage(current_page_, count);
    if (items == nullptr || selected_index_ >= count) { return; }

    if (current_page_ == Page::EVENTS)
    {
        if (selected_index_ == 0U) { goBack(); return; }

        const uint8_t k_cancel_index =
            static_cast<uint8_t>(arrLen(k_events_items) - 1U);
        if (selected_index_ == k_cancel_index)
        {
            BreathSim_CancelEvent();
        }
        else
        {
            const EventItem& e = k_event_items[selected_index_ - 1U];
            BreathEvent_t ev;
            ev.type       = e.type;
            ev.duration_s = e.duration_s;
            ev.severity   = e.severity;
            (void)BreathSim_TriggerEvent(&ev);
        }
        needs_redraw_ = true;
        return;
    }

    const ListItem& item = items[selected_index_];
    if (item.target_page == Page::RUNNING)
    {
        applySettingsToBackend();
        BreathSim_Start();
        sim_running_ = true;
        goTo(Page::RUNNING, Mode::INFO, 0U);
        return;
    }

    goTo(item.target_page, item.target_mode, 0U);
}

void MenuController::onRotaryDelta(int delta)
{
    Settings& s = getSettings();

    if (current_mode_ == Mode::VALUE_EDIT)
    {
        switch (current_page_)
        {
            case Page::VALUE_TIMING:     s.timing_mode = stepTimingMode(s.timing_mode, delta); break;
            case Page::VALUE_RATE:       s.rate_bpm = stepRateBpm(s.rate_bpm, delta); break;
            case Page::VALUE_INSP_TIME:  s.insp_time_s = stepInspTime(s.insp_time_s, delta); break;
            case Page::VALUE_IE_RATIO:   s.ie_ratio_exp = stepIeRatio(s.ie_ratio_exp, delta); break;
            case Page::VALUE_RPM_BASE:   s.rpm_base = stepRpmBase(s.rpm_base, delta); break;
            case Page::VALUE_RPM_AMP:    s.rpm_amplitude = stepRpmAmplitude(s.rpm_amplitude, delta); break;
            case Page::VALUE_WAVEFORM:   s.waveform = stepWaveform(s.waveform, delta); break;
            case Page::VALUE_EXP_TAU:    s.exp_tau_s = stepExpTau(s.exp_tau_s, delta); break;
            case Page::VALUE_FLATTENING: s.flattening = stepFlattening(s.flattening, delta); break;
            case Page::VALUE_JITTER:     s.jitter_pct = stepJitter(s.jitter_pct, delta); break;
            case Page::VALUE_INSP_PAUSE: s.insp_pause_s = stepInspPause(s.insp_pause_s, delta); break;
            case Page::VALUE_EXP_PAUSE:  s.exp_pause_s = stepExpPause(s.exp_pause_s, delta); break;
            default: break;
        }
        applySettingsToBackend();
        needs_redraw_ = true;
        return;
    }

    if (current_mode_ == Mode::LIST)
    {
        int idx = static_cast<int>(selected_index_) + delta;
        const int max_idx = static_cast<int>(itemCount()) - 1;
        if (idx < 0) { idx = 0; }
        if (idx > max_idx) { idx = max_idx; }
        selected_index_ = static_cast<uint8_t>(idx);
        needs_redraw_ = true;
    }
}

void MenuController::onButtonPress()
{
    if (current_page_ == Page::WELCOME)
    {
        goTo(Page::HOME, Mode::LIST, 0U);
        return;
    }

    if (current_mode_ == Mode::VALUE_EDIT)
    {
        goBack();
        return;
    }

    if (current_page_ == Page::RUNNING)
    {
        goBack();
        return;
    }

    if (current_mode_ == Mode::LIST)
    {
        uint8_t count;
        const ListItem* items = itemsForPage(current_page_, count);
        if (items != nullptr && selected_index_ < count &&
            items[selected_index_].label[0] == '<')
        {
            goBack();
            return;
        }
        enterSelectedItem();
    }
    else if (current_mode_ == Mode::INFO)
    {
        goBack();
    }
}

} // namespace breath_sim
