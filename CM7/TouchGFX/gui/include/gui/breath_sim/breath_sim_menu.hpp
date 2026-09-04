#ifndef BREATH_SIM_MENU_HPP
#define BREATH_SIM_MENU_HPP

#include <stdint.h>

namespace breath_sim
{

enum class Page : uint8_t
{
    WELCOME = 0,
    HOME,
    SETTINGS,
    VALUE_RATE,
    VALUE_INSP_TIME,
    VALUE_IE_RATIO,
    VALUE_RPM_BASE,
    VALUE_RPM_AMP,
    VALUE_WAVEFORM,
    VALUE_INSP_PAUSE,
    VALUE_EXP_PAUSE,
    VALUE_TIMING,
    VALUE_EXP_TAU,
    VALUE_FLATTENING,
    VALUE_JITTER,
    EVENTS,
    PROTOCOLS,
    RUNNING,
    DIAGNOSTICS,
    NUM_PAGES
};

enum class Mode : uint8_t
{
    LIST,
    VALUE_EDIT,
    INFO
};

class MenuController
{
public:
    MenuController();

    void onRotaryDelta(int delta);
    void onButtonPress();

    Page    page()          const { return current_page_; }
    Mode    mode()          const { return current_mode_; }
    uint8_t selectedIndex() const { return selected_index_; }
    uint8_t itemCount()     const;
    bool    simRunning()    const { return sim_running_; }

    bool    needsRedraw()   const { return needs_redraw_; }
    void    clearRedraw()         { needs_redraw_ = false; }

    const char* itemLabel(uint8_t idx) const;
    const char* itemValue(uint8_t idx) const;
    const char* editorValueText() const;
    const char* editorTitle()     const;
    const char* editorHint()      const;

    void goTo(Page p, Mode m = Mode::LIST, uint8_t initial_index = 0);

private:
    Page    current_page_;
    Mode    current_mode_;
    uint8_t selected_index_;
    bool    needs_redraw_;
    bool    sim_running_;

    void enterSelectedItem();
    void goBack();
    void applySettingsToBackend();
};

MenuController& getMenu();

} // namespace breath_sim

#endif
