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
    { "Diagnostics",      Page::DIAGNOSTICS, Mode::INFO },
};

const ListItem k_settings_items[] = {
    { "<- Back",      Page::HOME,             Mode::LIST },
    { "Breath Rate",  Page::VALUE_RATE,       Mode::VALUE_EDIT },
    { "Insp Time",    Page::VALUE_INSP_TIME,  Mode::VALUE_EDIT },
    { "I:E Ratio",    Page::VALUE_IE_RATIO,   Mode::VALUE_EDIT },
    { "RPM Base",     Page::VALUE_RPM_BASE,   Mode::VALUE_EDIT },
    { "Amplitude",    Page::VALUE_RPM_AMP,    Mode::VALUE_EDIT },
    { "Waveform",     Page::VALUE_WAVEFORM,   Mode::VALUE_EDIT },
    { "Insp Pause",   Page::VALUE_INSP_PAUSE, Mode::VALUE_EDIT },
    { "Exp Pause",    Page::VALUE_EXP_PAUSE,  Mode::VALUE_EDIT },
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
            return Page::SETTINGS;
        case Page::SETTINGS:
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
    static char buf[24];
    switch (idx)
    {
        case 1: snprintf(buf, sizeof(buf), "%u BPM", (unsigned)s.rate_bpm); break;
        case 2: snprintf(buf, sizeof(buf), "%.1f s", (double)s.insp_time_s); break;
        case 3: snprintf(buf, sizeof(buf), "1:%u", (unsigned)s.ie_ratio_exp); break;
        case 4: snprintf(buf, sizeof(buf), "%ld", (long)s.rpm_base); break;
        case 5: snprintf(buf, sizeof(buf), "%ld", (long)s.rpm_amplitude); break;
        case 6: snprintf(buf, sizeof(buf), "%s", toString(s.waveform)); break;
        case 7: snprintf(buf, sizeof(buf), "%.1f s", (double)s.insp_pause_s); break;
        case 8: snprintf(buf, sizeof(buf), "%.1f s", (double)s.exp_pause_s); break;
        default: return nullptr;
    }
    return buf;
}

const char* MenuController::editorValueText() const
{
    const Settings& s = getSettings();
    switch (current_page_)
    {
        case Page::VALUE_RATE:       snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%u BPM", (unsigned)s.rate_bpm); break;
        case Page::VALUE_INSP_TIME:  snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%.1f s", (double)s.insp_time_s); break;
        case Page::VALUE_IE_RATIO:   snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "1:%u", (unsigned)s.ie_ratio_exp); break;
        case Page::VALUE_RPM_BASE:   snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%ld RPM", (long)s.rpm_base); break;
        case Page::VALUE_RPM_AMP:    snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%ld RPM", (long)s.rpm_amplitude); break;
        case Page::VALUE_WAVEFORM:   snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%s", toString(s.waveform)); break;
        case Page::VALUE_INSP_PAUSE: snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%.1f s", (double)s.insp_pause_s); break;
        case Page::VALUE_EXP_PAUSE:  snprintf(g_editor_value_buf, sizeof(g_editor_value_buf), "%.1f s", (double)s.exp_pause_s); break;
        default: g_editor_value_buf[0] = '\0'; break;
    }
    return g_editor_value_buf;
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
    return "Rotate to select, press to enter";
}

void MenuController::applySettingsToBackend()
{
    BreathSimParams_t p;
    settingsToParams(getSettings(), &p);
    BreathSim_SetParams(&p);
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
            case Page::VALUE_RATE:       s.rate_bpm = stepRateBpm(s.rate_bpm, delta); break;
            case Page::VALUE_INSP_TIME:  s.insp_time_s = stepInspTime(s.insp_time_s, delta); break;
            case Page::VALUE_IE_RATIO:   s.ie_ratio_exp = stepIeRatio(s.ie_ratio_exp, delta); break;
            case Page::VALUE_RPM_BASE:   s.rpm_base = stepRpmBase(s.rpm_base, delta); break;
            case Page::VALUE_RPM_AMP:    s.rpm_amplitude = stepRpm(s.rpm_amplitude, delta, 0, 25000); break;
            case Page::VALUE_WAVEFORM:   s.waveform = stepWaveform(s.waveform, delta); break;
            case Page::VALUE_INSP_PAUSE: s.insp_pause_s = stepPause(s.insp_pause_s, delta); break;
            case Page::VALUE_EXP_PAUSE:  s.exp_pause_s = stepPause(s.exp_pause_s, delta); break;
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
