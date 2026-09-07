#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <gui/screen_screen/screenPresenter.hpp>

#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>

class screenView : public screenViewBase
{
public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

protected:
    touchgfx::Box status_bar_bg_;
    touchgfx::TextAreaWithOneWildcard status_left_text_;
    touchgfx::Unicode::UnicodeChar status_left_buf_[24];

    touchgfx::Box title_bar_bg_;
    touchgfx::TextAreaWithOneWildcard title_text_;
    touchgfx::Unicode::UnicodeChar title_buf_[40];

    touchgfx::Box content_bg_;

    enum : int { kMaxRows = 8 };
    enum : int { kVisibleRows = 7 };
    touchgfx::Box row_bg_[kMaxRows];
    touchgfx::TextAreaWithOneWildcard row_label_[kMaxRows];
    touchgfx::TextAreaWithOneWildcard row_value_[kMaxRows];
    touchgfx::Unicode::UnicodeChar row_label_buf_[kMaxRows][32];
    touchgfx::Unicode::UnicodeChar row_value_buf_[kMaxRows][24];
    uint8_t row_visible_count_;
    uint8_t list_scroll_offset_;
    uint8_t last_page_;

    touchgfx::TextAreaWithOneWildcard editor_value_text_;
    touchgfx::TextAreaWithOneWildcard editor_hint_text_;
    touchgfx::Unicode::UnicodeChar editor_value_buf_[32];
    touchgfx::Unicode::UnicodeChar editor_hint_buf_[64];

    touchgfx::TextAreaWithOneWildcard welcome_big_text_;
    touchgfx::TextAreaWithOneWildcard welcome_sub_text_;
    touchgfx::Unicode::UnicodeChar welcome_big_buf_[24];
    touchgfx::Unicode::UnicodeChar welcome_sub_buf_[48];

    enum : int { kWaveBars = 48 };
    touchgfx::Box wave_bars_[kWaveBars];
    touchgfx::TextAreaWithOneWildcard run_live_text_;
    touchgfx::TextAreaWithOneWildcard run_summary_text_;
    touchgfx::Unicode::UnicodeChar run_live_buf_[32];
    touchgfx::Unicode::UnicodeChar run_summary_buf_[64];
    float run_hist_[kWaveBars];
    uint8_t run_hist_count_;

    enum : int { kDiagRows = 9 };
    touchgfx::TextAreaWithOneWildcard diag_label_[kDiagRows];
    touchgfx::TextAreaWithOneWildcard diag_value_[kDiagRows];
    touchgfx::Unicode::UnicodeChar diag_label_buf_[kDiagRows][16];
    touchgfx::Unicode::UnicodeChar diag_value_buf_[kDiagRows][24];

    void buildWidgets();
    void renderCurrentPage();
    void hideAllPageWidgets();
    void renderListPage();
    void renderValueEditorPage();
    void renderWelcomePage();
    void renderRunningPage();
    void renderDiagnosticsPage();
    void updateStatusBar();
    void updateRunningGraph();
};

#endif
