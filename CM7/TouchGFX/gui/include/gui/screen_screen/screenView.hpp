#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <gui/screen_screen/screenPresenter.hpp>
#include <gui/cpap/icon_widget.hpp>

#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>

/**
 * CPAP menu view.
 *
 * The entire UI is created programmatically in setupScreen(). The Designer
 * file is left as a one-screen placeholder so future Designer regenerations
 * don't conflict with our hand-built widgets.
 *
 * Layout (480 x 272):
 *   - Status bar         Y 0..30
 *   - Title bar          Y 30..70
 *   - Content area       Y 70..272
 *
 * Sources of content:
 *   - The menu controller (cpap::getMenu()) drives the page state.
 *   - The settings store (cpap::getSettings()) holds the values.
 *   - System telemetry is pulled from existing C globals (blower state,
 *     humidifier RH, etc.) to populate the status bar live.
 */
class screenView : public screenViewBase
{
public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

protected:
    /* --------- Status bar (top, always visible) ---------------------- */
    touchgfx::Box                  status_bar_bg_;
    touchgfx::TextAreaWithOneWildcard status_left_text_;   /* "HOME" etc.   */
    touchgfx::Unicode::UnicodeChar status_left_buf_[24];

    /* Status-bar icons. Right-anchored, in slot order from rightmost to
     * leftmost: Home, Airplane, Cellular, Bluetooth, Humidifier. Each is
     * shown only when the matching condition holds; cellular shows either
     * a bar tower OR the no-signal X, never both. */
    cpap::IconWidget               icon_home_;
    cpap::IconWidget               icon_airplane_;
    cpap::IconWidget               icon_cellular_;     /* picks bars1..4 or no-cell */
    cpap::IconWidget               icon_bluetooth_;
    cpap::IconWidget               icon_humidifier_;   /* fault / warming / cooling */

    /* --------- Title bar -------------------------------------------- */
    touchgfx::Box                  title_bar_bg_;
    touchgfx::TextAreaWithOneWildcard title_text_;
    touchgfx::Unicode::UnicodeChar title_buf_[40];

    /* --------- Content background ----------------------------------- */
    touchgfx::Box                  content_bg_;

    /* --------- Menu list rows --------------------------------------- */
    /* 7 rows fit in the 200-px content area at 28-px line height, which is
     * exactly what MY OPTIONS needs (its longest list).
     *
     * Using an enum (not static constexpr) so ODR-use of the constant in
     * for-loops doesn't require an out-of-class definition. */
    enum : int { kMaxRows = 7 };
    touchgfx::Box                  row_bg_     [kMaxRows];  /* highlight */
    touchgfx::TextAreaWithOneWildcard row_label_[kMaxRows];
    touchgfx::TextAreaWithOneWildcard row_value_[kMaxRows];
    touchgfx::Unicode::UnicodeChar row_label_buf_[kMaxRows][32];
    touchgfx::Unicode::UnicodeChar row_value_buf_[kMaxRows][24];

    /* Tracks which rows are visible (so we can hide leftover widgets when
     * moving from a 7-item list to a 3-item one). */
    uint8_t row_visible_count_;
    /* Offset of the first visible item, for scrolling lists > kMaxRows. */
    uint8_t list_scroll_offset_;

    /* --------- Value-editor widgets --------------------------------- */
    touchgfx::TextAreaWithOneWildcard editor_value_text_;   /* big */
    touchgfx::TextAreaWithOneWildcard editor_hint_text_;    /* small */
    touchgfx::Unicode::UnicodeChar editor_value_buf_[24];
    touchgfx::Unicode::UnicodeChar editor_hint_buf_[64];

    /* --------- THERAPY page: live pressure trace ---------------------- */
    enum : int { kWaveBars = 40 };
    enum : int { kTherapyGraphH = 100 };
    static constexpr float kTherapyGraphPmax = 25.0f;   /* cmH2O Y-axis top */
    static constexpr float kTherapyGraphPmin = 0.0f;

    touchgfx::Box                  wave_bars_[kWaveBars];
    touchgfx::Box                  therapy_setpoint_line_;
    touchgfx::TextAreaWithOneWildcard therapy_pressure_text_;
    touchgfx::TextAreaWithOneWildcard therapy_live_big_text_;
    touchgfx::TextAreaWithOneWildcard therapy_summary_text_;
    touchgfx::Unicode::UnicodeChar therapy_pressure_buf_[48];
    touchgfx::Unicode::UnicodeChar therapy_live_big_buf_[12];
    touchgfx::Unicode::UnicodeChar therapy_summary_buf_[64];

    float                          therapy_pressure_hist_[kWaveBars];
    uint8_t                        therapy_hist_count_;
    uint8_t                        therapy_hist_decim_;

    /* --------- WELCOME page ----------------------------------------- */
    touchgfx::TextAreaWithOneWildcard welcome_big_text_;
    touchgfx::TextAreaWithOneWildcard welcome_sub_text_;
    touchgfx::Unicode::UnicodeChar welcome_big_buf_[16];
    touchgfx::Unicode::UnicodeChar welcome_sub_buf_[40];

    /* --------- DIAGNOSTICS page ------------------------------------- */
    /* Two-column layout. Indices 0..kDiagLeftRows-1 sit in the left
     * column (self-test results); indices kDiagLeftRows..kDiagRows-1 sit
     * in the right column (live sensor readings). kDiagHeaderIdx and
     * kDiagRightHeaderIdx point at the column-title rows so we can
     * paint them in the bright colour without having to special-case
     * formatting elsewhere. */
    enum : int {
        kDiagLeftRows  = 5,    /* 1 header + 4 self-tests */
        kDiagRightRows = 8,    /* 1 header + 7 live values */
        kDiagRows      = kDiagLeftRows + kDiagRightRows
    };
    touchgfx::TextAreaWithOneWildcard diag_label_[kDiagRows];
    touchgfx::TextAreaWithOneWildcard diag_value_[kDiagRows];
    touchgfx::Unicode::UnicodeChar    diag_label_buf_[kDiagRows][16];
    touchgfx::Unicode::UnicodeChar    diag_value_buf_[kDiagRows][20];

    /* --------- Internal helpers ------------------------------------- */
    void buildWidgets();
    void renderCurrentPage();
    void hideAllPageWidgets();
    void renderListPage();
    void renderValueEditorPage();
    void renderInfoPage();
    void renderWelcomePage();
    void renderTherapyPage();
    void renderDiagnosticsPage();
    void updateStatusBar();
    void updateTherapyPressureGraph();
    void resetTherapyPressureHistory(float seed_cmh2o);

    /* Temporary - cycles SystemStatus values every ~2 s so all status-bar
     * icons can be eyeballed without back-end drivers wired up. Delete this
     * declaration AND its definition + caller in screenView.cpp once real
     * telemetry is in place (search for "ICON DEMO"). */
    void cycleStatusIconDemo();
};

#endif // SCREENVIEW_HPP
