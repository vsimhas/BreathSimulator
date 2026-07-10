/**
  ******************************************************************************
  * @file    leak_estimator_vivo.c
  * @brief   Vivo50-style leak estimation (table shape + adaptive MeanRatio).
  ******************************************************************************
  */

#include "leak_estimator_vivo.h"
#include "leak_norm_table.h"
#include "leak_btps.h"

#include <math.h>

#define LEAK_VIVO_TWO_PI           6.28318530717958647692f
#define LEAK_VIVO_RATIO_BUF        400
#define LEAK_VIVO_RATIO_10S_DIV    250
#define LEAK_VIVO_RATIO_TICKS      8U      /* 8 x 5 ms = 40 ms */
#define LEAK_VIVO_MAX_BUF_RATIO    320000
#define LEAK_VIVO_INSP_ENTER_SLM     2.0f
#define LEAK_VIVO_EXP_ENTER_SLM      1.5f
#define LEAK_VIVO_INSP_CONFIRM_S     0.080f
#define LEAK_VIVO_EXP_CONFIRM_S      0.050f

/* === Public tunables === */
volatile uint8_t  g_leak_vivo_breath_insp       = 0U;
volatile int32_t  g_leak_vivo_mean_ratio        = LEAK_VIVO_STANDARD_RATIO;
volatile int32_t  g_leak_vivo_mean_ratio_10s    = 0;
volatile float    g_leak_vivo_flow_leak_slm     = 0.0f;
volatile float    g_leak_vivo_rest_volume_ml    = 0.0f;
volatile float    g_leak_vivo_patient_lpf_hz    = 2.0f;

static volatile float g_high_threshold_slm = 24.0f;

/* === Outputs mirrored to Leak_Get* API === */
static float s_total_slm = 0.0f;
static float s_vent_ref_slm = 0.0f;
static float s_uninten_slm = 0.0f;
static float s_instant_excess_slm = 0.0f;
static float s_patient_raw_slm = 0.0f;
static float s_patient_slm = 0.0f;
static float s_reported_leak_slm = 0.0f;
static bool  s_is_high = false;

/* === Ratio / backup state === */
static int32_t s_ratio_ring[LEAK_VIVO_RATIO_BUF];
static int     s_ratio_idx = 0;
static int64_t s_ratio_sum_div10 = 0;
static int32_t s_breath_ratio_sum = 0;
static int     s_breath_ratio_count = 0;
static int32_t s_mean_breath_ratio = 0;

static uint32_t s_ratio_tick = 0U;
static uint32_t s_ms_since_boot = 0U;
static uint32_t s_last_insp_ms = 0U;

/* === Breath / volume state === */
static bool    s_breath_insp = false;
static bool    s_prev_breath_insp = false;
static float   s_insp_confirm_s = 0.0f;
static float   s_exp_confirm_s = 0.0f;

static float   s_live_volume_ml = 0.0f;
static float   s_rest_volume_ml = 0.0f;
static float   s_therapy_sp_cmh2o = 8.0f;

static uint32_t s_insp_start_ms = 0U;
static uint32_t s_exp_start_ms = 0U;
static float   s_insp_press_sum = 0.0f;
static uint32_t s_insp_press_n = 0U;
static float   s_exp_press_sum = 0.0f;
static uint32_t s_exp_press_n = 0U;
static uint16_t s_last_insp_press_idx = 0U;
static uint16_t s_last_exp_press_idx = 0U;
static uint32_t s_last_insp_dur_ms = 1000U;
static uint32_t s_last_exp_dur_ms = 1000U;

static bool    s_patient_filt_seeded = false;
static bool    s_adaptation_frozen   = false;
static int32_t s_last_instant_ratio  = 0;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float mlpsToSlm(int32_t mlps)
{
    return (float)mlps * (60.0f / 1000.0f);
}

static int32_t slmToMlps(float slm)
{
    return (int32_t)(slm * (1000.0f / 60.0f));
}

static uint16_t pressureToIndex(float p_cmh2o)
{
    if (p_cmh2o <= 0.0f)
    {
        return 0U;
    }
    int idx = (int)(p_cmh2o * 10.0f + 0.5f);
    if (idx >= (int)LEAK_NORM_TAB_COUNT)
    {
        idx = (int)LEAK_NORM_TAB_COUNT - 1;
    }
    return (uint16_t)idx;
}

static int32_t tableRefBtpsMlps(uint16_t idx)
{
    if (idx >= LEAK_NORM_TAB_COUNT)
    {
        return 0;
    }
    return LeakBtps_TableToBtpsMlps((int32_t)g_leak_norm_tab_mlps[idx]);
}

static int32_t leakFlowMlps(uint16_t idx, int32_t mean_ratio)
{
    const int32_t ref = tableRefBtpsMlps(idx);
    if (ref <= 0 || mean_ratio <= 0)
    {
        return 0;
    }
    return (int32_t)(((int64_t)ref * (int64_t)mean_ratio) / (int64_t)LEAK_VIVO_NORM_FACTOR);
}

static int32_t calcInstantRatio(uint16_t idx, int32_t q_mlps_btps)
{
    if (idx == 0U || q_mlps_btps <= 0)
    {
        return 0;
    }
    const int32_t ref = tableRefBtpsMlps(idx);
    if (ref <= 0)
    {
        return 0;
    }
    int64_t ratio = ((int64_t)q_mlps_btps * (int64_t)LEAK_VIVO_NORM_FACTOR) / (int64_t)ref;
    if (ratio > LEAK_VIVO_MAX_BUF_RATIO)
    {
        ratio = LEAK_VIVO_MAX_BUF_RATIO;
    }
    if (ratio < 0)
    {
        ratio = 0;
    }
    return (int32_t)ratio;
}

static void bufferRatio(int32_t ratio)
{
    const int start = (s_ratio_idx - LEAK_VIVO_RATIO_10S_DIV + LEAK_VIVO_RATIO_BUF) % LEAK_VIVO_RATIO_BUF;
    s_ratio_sum_div10 += (int64_t)ratio / 10;
    s_ratio_sum_div10 -= (int64_t)s_ratio_ring[start] / 10;

    s_ratio_ring[s_ratio_idx] = ratio;
    s_ratio_idx = (s_ratio_idx + 1) % LEAK_VIVO_RATIO_BUF;

    int64_t mean = s_ratio_sum_div10 / LEAK_VIVO_RATIO_10S_DIV;
    if (mean < 0)
    {
        mean = 0;
    }
    g_leak_vivo_mean_ratio_10s = (int32_t)mean;
}

static void calcLeakLevelFromRestVolume(void)
{
    if (s_adaptation_frozen)
    {
        return;
    }

    int rest_ml = (int)s_rest_volume_ml;
    if (rest_ml > LEAK_VIVO_MAX_REST_VOL_ML)
    {
        rest_ml = LEAK_VIVO_MAX_REST_VOL_ML;
    }
    if (rest_ml < -LEAK_VIVO_MAX_REST_VOL_ML)
    {
        rest_ml = -LEAK_VIVO_MAX_REST_VOL_ML;
    }

    const int32_t flow_i = tableRefBtpsMlps(s_last_insp_press_idx);
    const int32_t flow_e = tableRefBtpsMlps(s_last_exp_press_idx);

    const int64_t norm_vol_insp = (int64_t)flow_i * (int64_t)s_last_insp_dur_ms / 1000LL;
    const int64_t norm_vol_exp  = (int64_t)flow_e * (int64_t)s_last_exp_dur_ms / 1000LL;
    const int64_t norm_sum = norm_vol_insp + norm_vol_exp;
    if (norm_sum <= 0LL)
    {
        return;
    }

    const int rest_exp = (int)((int64_t)rest_ml * norm_vol_exp / norm_sum);
    int64_t add_on_exp = 0;
    if (s_last_exp_dur_ms > 0U)
    {
        add_on_exp = ((int64_t)rest_exp * 1000LL) / (int64_t)s_last_exp_dur_ms;
    }

    const uint16_t set_idx = pressureToIndex(s_therapy_sp_cmh2o);
    const int32_t norm_flow = tableRefBtpsMlps(set_idx);
    if (norm_flow <= 0)
    {
        return;
    }

    int32_t ratio_delta = (int32_t)((add_on_exp * (int64_t)LEAK_VIVO_NORM_FACTOR) / (int64_t)norm_flow);
    if (ratio_delta < -LEAK_VIVO_MAX_RATIO_CHANGE)
    {
        ratio_delta = -LEAK_VIVO_MAX_RATIO_CHANGE;
    }
    if (ratio_delta > LEAK_VIVO_MAX_RATIO_CHANGE)
    {
        ratio_delta = LEAK_VIVO_MAX_RATIO_CHANGE;
    }

    if (ratio_delta < 0)
    {
        if (((int64_t)g_leak_vivo_mean_ratio + (int64_t)ratio_delta) > 0LL)
        {
            g_leak_vivo_mean_ratio += ratio_delta;
        }
        else
        {
            g_leak_vivo_mean_ratio /= 2;
        }
        if (g_leak_vivo_mean_ratio < 0)
        {
            g_leak_vivo_mean_ratio = 10;
        }
    }
    else
    {
        if ((g_leak_vivo_mean_ratio + ratio_delta) < LEAK_VIVO_MAX_RATIO_VALUE)
        {
            g_leak_vivo_mean_ratio += ratio_delta;
        }
        else
        {
            g_leak_vivo_mean_ratio = LEAK_VIVO_MAX_RATIO_VALUE;
        }
    }
}

static void onInspirationStart(uint32_t now_ms)
{
    s_rest_volume_ml = s_live_volume_ml;
    g_leak_vivo_rest_volume_ml = s_rest_volume_ml;
    s_live_volume_ml = 0.0f;

    calcLeakLevelFromRestVolume();

    s_last_insp_ms = now_ms;
    s_insp_start_ms = now_ms;
    s_insp_press_sum = 0.0f;
    s_insp_press_n = 0U;
}

static void onExpirationStart(uint32_t now_ms)
{
    if (s_insp_start_ms > 0U)
    {
        s_last_insp_dur_ms = now_ms - s_insp_start_ms;
        if (s_last_insp_dur_ms < 100U)
        {
            s_last_insp_dur_ms = 100U;
        }
        if (s_insp_press_n > 0U)
        {
            const float mean_p = s_insp_press_sum / (float)s_insp_press_n;
            s_last_insp_press_idx = pressureToIndex(mean_p);
        }
    }
    s_exp_start_ms = now_ms;
    s_exp_press_sum = 0.0f;
    s_exp_press_n = 0U;
}

static void onInspirationFromExpiration(uint32_t now_ms)
{
    if (s_exp_start_ms > 0U)
    {
        s_last_exp_dur_ms = now_ms - s_exp_start_ms;
        if (s_last_exp_dur_ms < 100U)
        {
            s_last_exp_dur_ms = 100U;
        }
        if (s_exp_press_n > 0U)
        {
            const float mean_p = s_exp_press_sum / (float)s_exp_press_n;
            s_last_exp_press_idx = pressureToIndex(mean_p);
        }
    }
    onInspirationStart(now_ms);
}

static void updateBreathFsm(float patient_slm, float mask_p_cmh2o, float dt_s)
{
    const bool want_insp = (patient_slm >= LEAK_VIVO_INSP_ENTER_SLM);
    const bool want_exp  = (patient_slm <= -LEAK_VIVO_EXP_ENTER_SLM);

    if (!s_breath_insp)
    {
        if (want_insp)
        {
            s_insp_confirm_s += dt_s;
            s_exp_confirm_s = 0.0f;
            if (s_insp_confirm_s >= LEAK_VIVO_INSP_CONFIRM_S)
            {
                s_breath_insp = true;
            }
        }
        else
        {
            s_insp_confirm_s = 0.0f;
        }
    }
    else
    {
        if (want_exp)
        {
            s_exp_confirm_s += dt_s;
            s_insp_confirm_s = 0.0f;
            if (s_exp_confirm_s >= LEAK_VIVO_EXP_CONFIRM_S)
            {
                s_breath_insp = false;
            }
        }
        else
        {
            s_exp_confirm_s = 0.0f;
        }
    }

    if (s_breath_insp && !s_prev_breath_insp)
    {
        onInspirationFromExpiration(s_ms_since_boot);
    }
    else if (!s_breath_insp && s_prev_breath_insp)
    {
        onExpirationStart(s_ms_since_boot);
    }

    if (s_breath_insp)
    {
        s_insp_press_sum += mask_p_cmh2o;
        s_insp_press_n++;
    }
    else
    {
        s_exp_press_sum += mask_p_cmh2o;
        s_exp_press_n++;
    }

    g_leak_vivo_breath_insp = s_breath_insp ? 1U : 0U;
    s_prev_breath_insp = s_breath_insp;
}

static void cpapBackupIfNeeded(void)
{
    if (s_adaptation_frozen)
    {
        return;
    }

    if ((s_ms_since_boot - s_last_insp_ms) < LEAK_VIVO_BACKUP_MS)
    {
        return;
    }
    g_leak_vivo_mean_ratio = g_leak_vivo_mean_ratio_10s * 10;
    s_last_insp_ms = s_ms_since_boot;
}

static void ratioPathStep(float total_slm, float mask_p_cmh2o)
{
    const uint16_t idx = pressureToIndex(mask_p_cmh2o);
    const int32_t q_mlps = LeakBtps_MeasuredStpdToBtpsMlps(slmToMlps(total_slm));
    const int32_t ratio = calcInstantRatio(idx, q_mlps);
    s_last_instant_ratio = ratio;
    bufferRatio(ratio);

    s_breath_ratio_count++;
    s_breath_ratio_sum += ratio;
    s_mean_breath_ratio = s_breath_ratio_sum / s_breath_ratio_count;
}

/* === Public API === */

void LeakVivo_Init(void)
{
    LeakBtps_Init();
    LeakVivo_Reset();
    g_leak_vivo_patient_lpf_hz = 2.0f;
    g_high_threshold_slm = 24.0f;
}

void LeakVivo_Reset(void)
{
    for (int i = 0; i < LEAK_VIVO_RATIO_BUF; i++)
    {
        s_ratio_ring[i] = 0;
    }
    s_ratio_idx = 0;
    s_ratio_sum_div10 = 0;
    s_breath_ratio_sum = 0;
    s_breath_ratio_count = 0;
    s_mean_breath_ratio = 0;

    g_leak_vivo_mean_ratio = LEAK_VIVO_STANDARD_RATIO;
    g_leak_vivo_mean_ratio_10s = 0;
    s_ratio_tick = 0U;
    s_ms_since_boot = 0U;
    s_last_insp_ms = 0U;

    s_breath_insp = false;
    s_prev_breath_insp = false;
    s_insp_confirm_s = 0.0f;
    s_exp_confirm_s = 0.0f;

    s_live_volume_ml = 0.0f;
    s_rest_volume_ml = 0.0f;
    g_leak_vivo_rest_volume_ml = 0.0f;

    s_insp_start_ms = 0U;
    s_exp_start_ms = 0U;
    s_insp_press_sum = 0.0f;
    s_insp_press_n = 0U;
    s_exp_press_sum = 0.0f;
    s_exp_press_n = 0U;
    s_last_insp_press_idx = 0U;
    s_last_exp_press_idx = 0U;
    s_last_insp_dur_ms = 1000U;
    s_last_exp_dur_ms = 1000U;

    s_total_slm = 0.0f;
    s_vent_ref_slm = 0.0f;
    s_uninten_slm = 0.0f;
    s_instant_excess_slm = 0.0f;
    s_patient_raw_slm = 0.0f;
    s_patient_slm = 0.0f;
    s_reported_leak_slm = 0.0f;
    s_is_high = false;
    g_leak_vivo_flow_leak_slm = 0.0f;
    g_leak_vivo_breath_insp = 0U;

    s_patient_filt_seeded = false;
    s_adaptation_frozen   = false;
    s_last_instant_ratio  = 0;
}

void LeakVivo_SetHighThreshold(float slm)
{
    g_high_threshold_slm = clampf(slm, 5.0f, 60.0f);
}

float LeakVivo_GetHighThreshold(void)
{
    return g_high_threshold_slm;
}

void LeakVivo_SetStandardRatio(int32_t ratio)
{
    if (ratio < 100) ratio = 100;
    if (ratio > LEAK_VIVO_MAX_RATIO_VALUE) ratio = LEAK_VIVO_MAX_RATIO_VALUE;
    g_leak_vivo_mean_ratio = ratio;
}

int32_t LeakVivo_GetInstantRatio(void)
{
    return s_last_instant_ratio;
}

void LeakVivo_SetAdaptationFreeze(bool freeze)
{
    s_adaptation_frozen = freeze;
}

bool LeakVivo_IsAdaptationFrozen(void)
{
    return s_adaptation_frozen;
}

void LeakVivo_Update(float total_q_slm, float mask_pressure_cmh2o,
                     float therapy_setpoint_cmh2o, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    s_therapy_sp_cmh2o = therapy_setpoint_cmh2o;
    s_total_slm = total_q_slm;
    s_ms_since_boot += (uint32_t)(dt_s * 1000.0f);

    const uint16_t idx = pressureToIndex(mask_pressure_cmh2o);
    const int32_t leak_mlps = leakFlowMlps(idx, g_leak_vivo_mean_ratio);
    const int32_t ref_mlps  = tableRefBtpsMlps(idx);

    g_leak_vivo_flow_leak_slm = mlpsToSlm(leak_mlps);
    s_vent_ref_slm = mlpsToSlm(ref_mlps);

    const int32_t total_mlps = slmToMlps(total_q_slm);
    const int32_t patient_mlps = total_mlps - leak_mlps;

    s_patient_raw_slm = mlpsToSlm(patient_mlps);
    s_instant_excess_slm = mlpsToSlm(total_mlps - ref_mlps);

    const float total_leak_slm = g_leak_vivo_flow_leak_slm;
    const float vent_ref_slm = s_vent_ref_slm;
    s_uninten_slm = total_leak_slm - vent_ref_slm;
    if (s_uninten_slm < 0.0f)
    {
        s_uninten_slm = 0.0f;
    }

    const float fc = clampf(g_leak_vivo_patient_lpf_hz, 0.5f, 8.0f);
    const float omega = LEAK_VIVO_TWO_PI * fc;
    const float alpha = (omega * dt_s) / (1.0f + omega * dt_s);
    if (!s_patient_filt_seeded)
    {
        s_patient_slm = s_patient_raw_slm;
        s_patient_filt_seeded = true;
    }
    else
    {
        s_patient_slm = alpha * s_patient_raw_slm + (1.0f - alpha) * s_patient_slm;
    }

    s_live_volume_ml += s_patient_raw_slm * (1000.0f / 60.0f) * dt_s;

    updateBreathFsm(s_patient_raw_slm, mask_pressure_cmh2o, dt_s);
    cpapBackupIfNeeded();

    s_ratio_tick++;
    if (s_ratio_tick >= LEAK_VIVO_RATIO_TICKS)
    {
        s_ratio_tick = 0U;
        ratioPathStep(total_q_slm, mask_pressure_cmh2o);
    }

    const uint16_t set_idx = pressureToIndex(therapy_setpoint_cmh2o);
    const int32_t reported_mlps = leakFlowMlps(set_idx, g_leak_vivo_mean_ratio);
    s_reported_leak_slm = mlpsToSlm(reported_mlps);

    s_is_high = (fabsf(s_uninten_slm) > g_high_threshold_slm);
}

float LeakVivo_GetTotalSlm(void)           { return s_total_slm; }
float LeakVivo_GetVentSlm(void)            { return s_vent_ref_slm; }
float LeakVivo_GetUnintentionalSlm(void)   { return s_uninten_slm; }
float LeakVivo_GetInstantExcessSlm(void)   { return s_instant_excess_slm; }
float LeakVivo_GetPatientSlm(void)         { return s_patient_slm; }
float LeakVivo_GetPatientRawSlm(void)       { return s_patient_raw_slm; }
bool  LeakVivo_IsHigh(void)                { return s_is_high; }
int32_t LeakVivo_GetMeanRatio(void)        { return g_leak_vivo_mean_ratio; }
int32_t LeakVivo_GetMeanRatio10Sec(void)   { return g_leak_vivo_mean_ratio_10s; }
float LeakVivo_GetReportedLeakSlm(void)    { return s_reported_leak_slm; }
