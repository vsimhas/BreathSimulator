#include <gui/screen_screen/screenView.hpp>
#include <gui/breath_sim/breath_sim_menu.hpp>
#include <gui/breath_sim/breath_sim_settings.hpp>

#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Color.hpp>
#include <touchgfx/Unicode.hpp>

#include <stdio.h>
#include <string.h>

extern "C" {
#include "breath_sim.h"
#include "diagnostics_data.h"
}

using namespace touchgfx;

namespace
{

const colortype kColBg        = Color::getColorFromRGB(12, 18, 32);
const colortype kColStatusBg  = Color::getColorFromRGB(28, 40, 64);
const colortype kColTitleBg   = Color::getColorFromRGB(18, 28, 48);
const colortype kColContentBg = Color::getColorFromRGB(18, 28, 48);
const colortype kColRowSel    = Color::getColorFromRGB(60, 140, 200);
const colortype kColText      = Color::getColorFromRGB(235, 240, 248);
const colortype kColTextDim   = Color::getColorFromRGB(160, 175, 200);
const colortype kColWave      = Color::getColorFromRGB(120, 200, 255);
const colortype kColTestPass  = Color::getColorFromRGB(90, 200, 110);

constexpr int16_t kScreenW  = 480;
constexpr int16_t kScreenH  = 272;
constexpr int16_t kStatusH  = 32;
constexpr int16_t kTitleY   = 32;
constexpr int16_t kTitleH   = 38;
constexpr int16_t kContentY = 70;
constexpr int16_t kContentH = kScreenH - kContentY;
constexpr int16_t kRowH     = 28;
constexpr int16_t kRowsTopY = kContentY + 4;
/* The diagnostics page packs more rows than the menu pages, so it uses its
 * own pitch. At the shared kRowH of 28 only 7 rows fit in the 202 px content
 * area - the 8th already started at y=270 and was clipped off-screen. */
constexpr int16_t kDiagRowH = 22;

void utf8ToBuf(const char* src, Unicode::UnicodeChar* dst, uint16_t buf_chars)
{
    if (src == nullptr) { src = ""; }
    Unicode::strncpy(dst, src, buf_chars - 1U);
    dst[buf_chars - 1U] = 0;
}

} // namespace

screenView::screenView()
    : row_visible_count_(0U),
      list_scroll_offset_(0U),
      last_page_(0xFFU),
      run_hist_count_(0U)
{
    memset(run_hist_, 0, sizeof(run_hist_));
}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
    buildWidgets();
    renderCurrentPage();
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
}

void screenView::buildWidgets()
{
    image1.setVisible(false);
    buttonWithLabel1.setVisible(false);
    __background.setPosition(0, 0, kScreenW, kScreenH);
    __background.setColor(kColBg);

    status_bar_bg_.setPosition(0, 0, kScreenW, kStatusH);
    status_bar_bg_.setColor(kColStatusBg);
    add(status_bar_bg_);

    status_left_text_.setPosition(8, 6, 300, kStatusH - 12);
    status_left_text_.setColor(kColText);
    status_left_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("Home", status_left_buf_, sizeof(status_left_buf_) / sizeof(status_left_buf_[0]));
    status_left_text_.setWildcard(status_left_buf_);
    add(status_left_text_);

    title_bar_bg_.setPosition(0, kTitleY, kScreenW, kTitleH);
    title_bar_bg_.setColor(kColTitleBg);
    add(title_bar_bg_);

    title_text_.setPosition(16, kTitleY + 4, kScreenW - 32, kTitleH - 8);
    title_text_.setColor(kColText);
    title_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("Breath Simulator", title_buf_, sizeof(title_buf_) / sizeof(title_buf_[0]));
    title_text_.setWildcard(title_buf_);
    add(title_text_);

    content_bg_.setPosition(0, kContentY, kScreenW, kContentH);
    content_bg_.setColor(kColContentBg);
    add(content_bg_);

    for (int i = 0; i < kMaxRows; ++i)
    {
        const int16_t y = kRowsTopY + static_cast<int16_t>(i * kRowH);
        row_bg_[i].setPosition(8, y, kScreenW - 16, kRowH - 2);
        row_bg_[i].setColor(kColContentBg);
        row_bg_[i].setVisible(false);
        add(row_bg_[i]);

        row_label_[i].setPosition(16, y + 4, 260, kRowH - 6);
        row_label_[i].setColor(kColText);
        row_label_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", row_label_buf_[i], sizeof(row_label_buf_[i]) / sizeof(row_label_buf_[i][0]));
        row_label_[i].setWildcard(row_label_buf_[i]);
        row_label_[i].setVisible(false);
        add(row_label_[i]);

        row_value_[i].setPosition(kScreenW - 170, y + 4, 150, kRowH - 6);
        row_value_[i].setColor(kColTextDim);
        row_value_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", row_value_buf_[i], sizeof(row_value_buf_[i]) / sizeof(row_value_buf_[i][0]));
        row_value_[i].setWildcard(row_value_buf_[i]);
        row_value_[i].setVisible(false);
        add(row_value_[i]);
    }

    editor_value_text_.setPosition(40, kContentY + 50, kScreenW - 80, 60);
    editor_value_text_.setColor(kColText);
    editor_value_text_.setTypedText(TypedText(T_DYN_M_CENTER));
    utf8ToBuf("", editor_value_buf_, sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    editor_value_text_.setWildcard(editor_value_buf_);
    editor_value_text_.setVisible(false);
    add(editor_value_text_);

    editor_hint_text_.setPosition(40, kContentY + 120, kScreenW - 80, 40);
    editor_hint_text_.setColor(kColTextDim);
    editor_hint_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("", editor_hint_buf_, sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_hint_text_.setWildcard(editor_hint_buf_);
    editor_hint_text_.setVisible(false);
    add(editor_hint_text_);

    welcome_big_text_.setPosition(40, kContentY + 40, kScreenW - 80, 50);
    welcome_big_text_.setColor(kColText);
    welcome_big_text_.setTypedText(TypedText(T_DYN_M_CENTER));
    utf8ToBuf("Breath Simulator", welcome_big_buf_, sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setWildcard(welcome_big_buf_);
    welcome_big_text_.setVisible(false);
    add(welcome_big_text_);

    welcome_sub_text_.setPosition(40, kContentY + 100, kScreenW - 80, 60);
    welcome_sub_text_.setColor(kColTextDim);
    welcome_sub_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("Press encoder to begin", welcome_sub_buf_, sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setWildcard(welcome_sub_buf_);
    welcome_sub_text_.setVisible(false);
    add(welcome_sub_text_);

    const int16_t bar_w = (kScreenW - 32) / kWaveBars;
    for (int i = 0; i < kWaveBars; ++i)
    {
        wave_bars_[i].setPosition(16 + i * bar_w, kContentY + 120, bar_w - 1, 4);
        wave_bars_[i].setColor(kColWave);
        wave_bars_[i].setVisible(false);
        add(wave_bars_[i]);
    }

    run_live_text_.setPosition(16, kContentY + 20, kScreenW - 32, 30);
    run_live_text_.setColor(kColText);
    run_live_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("", run_live_buf_, sizeof(run_live_buf_) / sizeof(run_live_buf_[0]));
    run_live_text_.setWildcard(run_live_buf_);
    run_live_text_.setVisible(false);
    add(run_live_text_);

    run_summary_text_.setPosition(16, kContentY + 55, kScreenW - 32, 50);
    run_summary_text_.setColor(kColTextDim);
    run_summary_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("", run_summary_buf_, sizeof(run_summary_buf_) / sizeof(run_summary_buf_[0]));
    run_summary_text_.setWildcard(run_summary_buf_);
    run_summary_text_.setVisible(false);
    add(run_summary_text_);

    for (int i = 0; i < kDiagRows; ++i)
    {
        const int16_t y = kRowsTopY + static_cast<int16_t>(i * kDiagRowH);
        diag_label_[i].setPosition(16, y + 2, 180, kDiagRowH - 4);
        diag_label_[i].setColor(kColTextDim);
        diag_label_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", diag_label_buf_[i], sizeof(diag_label_buf_[i]) / sizeof(diag_label_buf_[i][0]));
        diag_label_[i].setWildcard(diag_label_buf_[i]);
        diag_label_[i].setVisible(false);
        add(diag_label_[i]);

        diag_value_[i].setPosition(200, y + 2, 260, kDiagRowH - 4);
        diag_value_[i].setColor(kColText);
        diag_value_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", diag_value_buf_[i], sizeof(diag_value_buf_[i]) / sizeof(diag_value_buf_[i][0]));
        diag_value_[i].setWildcard(diag_value_buf_[i]);
        diag_value_[i].setVisible(false);
        add(diag_value_[i]);
    }
}

void screenView::hideAllPageWidgets()
{
    for (int i = 0; i < kMaxRows; ++i)
    {
        row_bg_[i].setVisible(false);
        row_label_[i].setVisible(false);
        row_value_[i].setVisible(false);
        utf8ToBuf("", row_label_buf_[i],
                  sizeof(row_label_buf_[i]) / sizeof(row_label_buf_[i][0]));
        utf8ToBuf("", row_value_buf_[i],
                  sizeof(row_value_buf_[i]) / sizeof(row_value_buf_[i][0]));
    }
    row_visible_count_ = 0U;
    editor_value_text_.setVisible(false);
    editor_hint_text_.setVisible(false);
    utf8ToBuf("", editor_value_buf_,
              sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    utf8ToBuf("", editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    welcome_big_text_.setVisible(false);
    welcome_sub_text_.setVisible(false);
    utf8ToBuf("", welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    utf8ToBuf("", welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    run_live_text_.setVisible(false);
    run_summary_text_.setVisible(false);
    utf8ToBuf("", run_live_buf_,
              sizeof(run_live_buf_) / sizeof(run_live_buf_[0]));
    utf8ToBuf("", run_summary_buf_,
              sizeof(run_summary_buf_) / sizeof(run_summary_buf_[0]));
    for (int i = 0; i < kWaveBars; ++i) { wave_bars_[i].setVisible(false); }
    for (int i = 0; i < kDiagRows; ++i)
    {
        diag_label_[i].setVisible(false);
        diag_value_[i].setVisible(false);
        utf8ToBuf("", diag_label_buf_[i],
                  sizeof(diag_label_buf_[i]) / sizeof(diag_label_buf_[i][0]));
        utf8ToBuf("", diag_value_buf_[i],
                  sizeof(diag_value_buf_[i]) / sizeof(diag_value_buf_[i][0]));
    }
    content_bg_.invalidate();
}

void screenView::updateStatusBar()
{
    using namespace breath_sim;
    utf8ToBuf(getMenu().editorTitle(), status_left_buf_,
              sizeof(status_left_buf_) / sizeof(status_left_buf_[0]));
    status_left_text_.invalidate();
}

void screenView::handleTickEvent()
{
    screenViewBase::handleTickEvent();

    if (breath_sim::getMenu().page() == breath_sim::Page::RUNNING)
    {
        updateRunningGraph();
    }

    static uint32_t s_status_tick = 0U;
    if (++s_status_tick >= 60U)
    {
        s_status_tick = 0U;
        updateStatusBar();
        if (breath_sim::getMenu().page() == breath_sim::Page::DIAGNOSTICS)
        {
            renderDiagnosticsPage();
        }
    }

    if (breath_sim::getMenu().needsRedraw())
    {
        renderCurrentPage();
        breath_sim::getMenu().clearRedraw();
    }
}

void screenView::renderCurrentPage()
{
    using namespace breath_sim;
    const uint8_t page_id = static_cast<uint8_t>(getMenu().page());
    if (page_id != last_page_)
    {
        list_scroll_offset_ = 0U;
        last_page_ = page_id;
    }

    hideAllPageWidgets();
    updateStatusBar();

    utf8ToBuf(getMenu().editorTitle(), title_buf_,
              sizeof(title_buf_) / sizeof(title_buf_[0]));
    title_text_.invalidate();

    switch (getMenu().page())
    {
        case Page::WELCOME: renderWelcomePage(); break;
        case Page::RUNNING: renderRunningPage(); break;
        case Page::DIAGNOSTICS: renderDiagnosticsPage(); break;
        default:
            if (getMenu().mode() == Mode::VALUE_EDIT) { renderValueEditorPage(); }
            else { renderListPage(); }
            break;
    }
}

void screenView::renderListPage()
{
    using namespace breath_sim;
    const uint8_t count = getMenu().itemCount();
    const uint8_t sel = getMenu().selectedIndex();

    if (sel < list_scroll_offset_)
    {
        list_scroll_offset_ = sel;
    }
    if ((sel >= list_scroll_offset_ + static_cast<uint8_t>(kVisibleRows)) &&
        (kVisibleRows > 0))
    {
        list_scroll_offset_ = static_cast<uint8_t>(sel - static_cast<uint8_t>(kVisibleRows) + 1U);
    }

    row_visible_count_ = 0U;
    for (int row = 0; row < kVisibleRows; ++row)
    {
        const uint8_t item_idx = static_cast<uint8_t>(list_scroll_offset_ + static_cast<uint8_t>(row));
        if (item_idx >= count)
        {
            row_bg_[row].setVisible(false);
            row_label_[row].setVisible(false);
            row_value_[row].setVisible(false);
            continue;
        }

        row_visible_count_++;
        const bool selected = (item_idx == sel);
        row_bg_[row].setColor(selected ? kColRowSel : kColContentBg);
        row_bg_[row].setVisible(true);
        utf8ToBuf(getMenu().itemLabel(item_idx), row_label_buf_[row],
                  sizeof(row_label_buf_[row]) / sizeof(row_label_buf_[row][0]));
        row_label_[row].setVisible(true);

        const char* val = getMenu().itemValue(item_idx);
        if (val != nullptr)
        {
            utf8ToBuf(val, row_value_buf_[row],
                      sizeof(row_value_buf_[row]) / sizeof(row_value_buf_[row][0]));
            row_value_[row].setVisible(true);
        }
        else
        {
            utf8ToBuf("", row_value_buf_[row],
                      sizeof(row_value_buf_[row]) / sizeof(row_value_buf_[row][0]));
            row_value_[row].setVisible(false);
        }

        row_label_[row].invalidate();
        row_value_[row].invalidate();
        row_bg_[row].invalidate();
    }

    for (int row = kVisibleRows; row < kMaxRows; ++row)
    {
        row_bg_[row].setVisible(false);
        row_label_[row].setVisible(false);
        row_value_[row].setVisible(false);
    }
}

void screenView::renderValueEditorPage()
{
    using namespace breath_sim;
    utf8ToBuf(getMenu().editorValueText(), editor_value_buf_,
              sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    utf8ToBuf(getMenu().editorHint(), editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_value_text_.setVisible(true);
    editor_hint_text_.setVisible(true);
    editor_value_text_.invalidate();
    editor_hint_text_.invalidate();
}

void screenView::renderWelcomePage()
{
    welcome_big_text_.setVisible(true);
    welcome_sub_text_.setVisible(true);
    welcome_big_text_.invalidate();
    welcome_sub_text_.invalidate();
}

void screenView::updateRunningGraph()
{
    BreathSimStatus_t st;
    BreathSim_GetStatus(&st);

    const float env = st.envelope;
    if (run_hist_count_ < static_cast<uint8_t>(kWaveBars))
    {
        run_hist_[run_hist_count_++] = env;
    }
    else
    {
        memmove(&run_hist_[0], &run_hist_[1], sizeof(float) * (kWaveBars - 1));
        run_hist_[kWaveBars - 1] = env;
    }

    const int16_t graph_h = 80;
    const int16_t base_y = kContentY + 120;
    const int16_t bar_w = (kScreenW - 32) / kWaveBars;
    for (int i = 0; i < kWaveBars; ++i)
    {
        const float v = (i < static_cast<int>(run_hist_count_)) ? run_hist_[i] : 0.0f;
        const int16_t h = static_cast<int16_t>(v * static_cast<float>(graph_h));
        wave_bars_[i].setPosition(16 + i * bar_w, base_y + graph_h - h, bar_w - 1, h + 1);
        wave_bars_[i].setVisible(true);
        wave_bars_[i].invalidate();
    }

    char line[48];
    const breath_sim::Settings& s = breath_sim::getSettings();
    BreathSimTiming_t t;
    BreathSim_GetTiming(&t);

    /* Line 1 tracks the motor. While priming there are no valid breaths yet,
     * and a non-zero flag word means the delivered waveform may not match
     * what was asked for - both need to be visible on the rig itself, not
     * only in the USB log. */
    if (st.state == static_cast<uint8_t>(BREATH_STATE_PRIMING))
    {
        snprintf(line, sizeof(line), "Priming... %ld / %ld RPM",
                 (long)st.rpm_act, (long)st.rpm_cmd);
    }
    else if (st.state == static_cast<uint8_t>(BREATH_STATE_FAULT))
    {
        snprintf(line, sizeof(line), "FAULT  mc=%u  flags 0x%04X",
                 (unsigned)st.mc_state, (unsigned)st.flags);
    }
    else if (st.flags != 0U)
    {
        snprintf(line, sizeof(line), "RPM %ld/%ld  ! 0x%04X",
                 (long)st.rpm_cmd, (long)st.rpm_act, (unsigned)st.flags);
    }
    else if (st.control_mode == static_cast<uint8_t>(BREATH_CTRL_FLOW))
    {
        snprintf(line, sizeof(line), "Q %.1f/%.1f L/min  %ld rpm",
                 (double)st.q_meas_lpm, (double)st.q_target_lpm,
                 (long)st.rpm_cmd);
    }
    else
    {
        snprintf(line, sizeof(line), "RPM %ld  (act %ld)",
                 (long)st.rpm_cmd, (long)st.rpm_act);
    }
    utf8ToBuf(line, run_live_buf_, sizeof(run_live_buf_) / sizeof(run_live_buf_[0]));
    run_live_text_.invalidate();

    if (st.event != static_cast<uint8_t>(BREATH_EVENT_NONE))
    {
        snprintf(line, sizeof(line), "#%lu  %s  %s",
                 (unsigned long)st.breath_index,
                 BreathSim_SegmentName(st.segment),
                 BreathSim_EventName(st.event));
    }
    else if (st.control_mode == static_cast<uint8_t>(BREATH_CTRL_FLOW))
    {
        /* In flow mode the numbers that matter are what was delivered and
         * how the device under test responded, not the commanded RPM. */
        snprintf(line, sizeof(line), "#%lu  Vt %.0f mL  P %.1f/%.1f",
                 (unsigned long)st.breath_index,
                 (double)st.last_tidal_ml,
                 (double)st.p_min_cmh2o, (double)st.p_max_cmh2o);
    }
    else
    {
        snprintf(line, sizeof(line), "#%lu  %.1f BPM  Ti %.2fs  peak %ld",
                 (unsigned long)st.breath_index, (double)t.rate_bpm,
                 (double)t.insp_s, (long)(s.rpm_base + s.rpm_amplitude));
    }
    utf8ToBuf(line, run_summary_buf_, sizeof(run_summary_buf_) / sizeof(run_summary_buf_[0]));
    run_summary_text_.invalidate();
}

void screenView::renderRunningPage()
{
    run_live_text_.setVisible(true);
    run_summary_text_.setVisible(true);
    utf8ToBuf("Press to stop", editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_hint_text_.setVisible(true);
    updateRunningGraph();
}

static const char* diagStateText(DiagnosticsTestState st)
{
    switch (st)
    {
        case DIAG_TEST_PASS:     return "PASS";
        case DIAG_TEST_FAIL:     return "FAIL";
        case DIAG_TEST_RUNNING:  return "RUN...";
        case DIAG_TEST_NO_CARD:  return "NO CARD";
        default:                 return "---";
    }
}

void screenView::renderDiagnosticsPage()
{
    DiagnosticsSnapshot_t snap;
    Diagnostics_GetSnapshot(&snap);

    char val[kDiagRows][24];
    const char* labels[kDiagRows];
    colortype colors[kDiagRows];

    snprintf(val[0], sizeof(val[0]), "%s", diagStateText(snap.sd_state));
    labels[0] = "SD Card";
    colors[0] = (snap.sd_state == DIAG_TEST_PASS) ? kColTestPass : kColText;

    snprintf(val[1], sizeof(val[1]), "%s ID=0x%02X",
             diagStateText(snap.nfc_state), (unsigned)snap.nfc_chip_id);
    labels[1] = "NFC";
    colors[1] = (snap.nfc_state == DIAG_TEST_PASS) ? kColTestPass : kColText;

    snprintf(val[2], sizeof(val[2]), "%s", diagStateText(snap.sdram_state));
    labels[2] = "SDRAM";
    colors[2] = (snap.sdram_state == DIAG_TEST_PASS) ? kColTestPass : kColText;

    snprintf(val[3], sizeof(val[3]), "%s", diagStateText(snap.qspi_state));
    labels[3] = "QSPI";
    colors[3] = (snap.qspi_state == DIAG_TEST_PASS) ? kColTestPass : kColText;

    snprintf(val[4], sizeof(val[4]), "%s", snap.sim_running ? "RUN" : "STOP");
    labels[4] = "Simulator";
    colors[4] = kColText;

    snprintf(val[5], sizeof(val[5]), "%ld", (long)snap.rpm_cmd);
    labels[5] = "RPM cmd";
    colors[5] = kColText;

    /* The Running page already shows breath index and segment, so this row
     * is better spent on the pressure sensor than on repeating the phase. */
    if (snap.press_present == 0U)
    {
        snprintf(val[6], sizeof(val[6]), "NO SENSOR");
    }
    else
    {
        snprintf(val[6], sizeof(val[6]), "%+.2f cmH2O", (double)snap.press_cmh2o);
    }
    labels[6] = "Pressure";
    colors[6] = (snap.press_present && !snap.press_held) ? kColTestPass : kColText;

    if (snap.climate_enabled == 0U)
    {
        snprintf(val[7], sizeof(val[7]), "OFF");
    }
    else
    {
        snprintf(val[7], sizeof(val[7]), "%.1f C", (double)snap.tube_temp_c);
    }
    labels[7] = "Tube temp";
    colors[7] = snap.tube_temp_valid ? kColText : kColTextDim;

    if (snap.flow_present == 0U)
    {
        snprintf(val[8], sizeof(val[8]), "NO SENSOR");
    }
    else
    {
        snprintf(val[8], sizeof(val[8]), "%+.1f slm (%u)",
                 (double)snap.flow_slm, (unsigned)snap.flow_raw);
    }
    labels[8] = "Flow";
    colors[8] = (snap.flow_present && snap.flow_valid) ? kColTestPass : kColText;

    for (int i = 0; i < kDiagRows; ++i)
    {
        utf8ToBuf(labels[i], diag_label_buf_[i],
                  sizeof(diag_label_buf_[i]) / sizeof(diag_label_buf_[i][0]));
        utf8ToBuf(val[i], diag_value_buf_[i],
                  sizeof(diag_value_buf_[i]) / sizeof(diag_value_buf_[i][0]));
        diag_value_[i].setColor(colors[i]);
        diag_label_[i].setVisible(true);
        diag_value_[i].setVisible(true);
        diag_label_[i].invalidate();
        diag_value_[i].invalidate();
    }
}
