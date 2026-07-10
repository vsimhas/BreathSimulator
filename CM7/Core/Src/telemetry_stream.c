/**
  ******************************************************************************
  * @file    telemetry_stream.c
  * @brief   Live pressure / patient-flow CSV stream over USB CDC VCP.
  ******************************************************************************
  */

#include "telemetry_stream.h"

#if TELEM_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"
#include "therapy.h"
#include "blower_ipc.h"
#include "blower_ctrl.h"
#include "apap_ctrl.h"
#include "epr_ctrl.h"
#include "leak_estimator.h"
#include "tube_comp.h"

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#ifndef TELEM_BUF_SIZE
#define TELEM_BUF_SIZE   2048U
#endif

#ifndef TELEM_TX_CHUNK
#define TELEM_TX_CHUNK   64U
#endif

#ifndef TELEM_FORMAT_BUF
#define TELEM_FORMAT_BUF 64U
#endif

#if (TELEM_BUF_SIZE & (TELEM_BUF_SIZE - 1U)) != 0U
#error "TELEM_BUF_SIZE must be a power of two"
#endif

#define TELEM_BUF_MASK   (TELEM_BUF_SIZE - 1U)
#define TELEM_CMD_QUEUE_LEN  96U

volatile uint8_t  g_telem_enabled      = 1U;
volatile uint32_t g_telem_sample_count = 0U;
volatile uint32_t g_telem_dropped_count = 0U;

extern volatile float   g_blower_ctrl_kp_low;
extern volatile float   g_blower_ctrl_ki_low;
extern volatile float   g_blower_ctrl_kd_low;
extern volatile float   g_blower_ctrl_kp_mid;
extern volatile float   g_blower_ctrl_ki_mid;
extern volatile float   g_blower_ctrl_kd_mid;
extern volatile float   g_blower_ctrl_kp_high;
extern volatile float   g_blower_ctrl_ki_high;
extern volatile float   g_blower_ctrl_kd_high;
extern volatile uint8_t g_blower_ctrl_ff_enable;
extern volatile float   g_blower_ctrl_ff_k_rpm_per_sqrtcmh;
extern volatile uint8_t g_blower_ctrl_flow_ff_enable;
extern volatile int8_t  g_blower_ctrl_flow_ff_inhale_sign;
extern volatile uint8_t g_blower_ctrl_dip_ff_enable;
extern volatile uint8_t g_blower_ctrl_physics_ff_enable;
extern volatile uint8_t g_blower_ctrl_physics_ff_use_total_flow;
extern volatile uint8_t g_blower_ctrl_physics_ff_exhale_mit_enable;
extern volatile float   g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2;
extern volatile float   g_blower_ctrl_physics_ff_flow_lpf_hz;
extern volatile float   g_blower_ctrl_physics_ff_flow_lpf_inhale_hz;
extern volatile float   g_blower_ctrl_physics_ff_flow_lpf_exhale_hz;
extern volatile uint8_t g_blower_ctrl_physics_ff_inhale_mit_enable;
extern volatile uint8_t g_blower_ctrl_physics_ff_onset_enable;
extern volatile float   g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_onset_thresh_slm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_onset_max_rpm;
extern volatile float   g_blower_ctrl_physics_ff_inhale_rise_boost;
extern volatile float   g_blower_ctrl_physics_ff_rise_rpm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s;
extern volatile float   g_blower_ctrl_under_p_gain;
extern volatile float   g_blower_ctrl_under_p_thresh_cmh2o;
extern volatile float   g_blower_ctrl_flow_ff_k_rpm_per_slm;
extern volatile float   g_blower_ctrl_flow_ff_max_rpm;
extern volatile float   g_blower_ctrl_flow_ff_deadband_slm;
extern volatile float   g_blower_ctrl_flow_ff_rise_rpm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_onset_thresh_slm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_onset_max_rpm;
extern volatile uint8_t g_blower_ctrl_flow_ff_onset_enable;
extern volatile float   g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s;
extern volatile float   g_blower_ctrl_dip_ff_thresh_cmh2o_per_s;
extern volatile float   g_blower_ctrl_dip_ff_max_rpm;
extern volatile float   g_blower_ctrl_dip_ff_err_min_cmh2o;
extern volatile int8_t  g_blower_ctrl_flow_ff_inhale_sign;
extern volatile float   g_blower_ctrl_over_p_gain;
extern volatile float   g_blower_ctrl_over_brake_rpm_per_s;
extern volatile float   g_blower_ctrl_slew_cmh2o_per_s;
extern volatile float   g_blower_ctrl_slew_up_cmh2o_per_s;
extern volatile float   g_blower_ctrl_out_filter_alpha;
extern volatile float   g_blower_ctrl_integ_deadband_cmh2o;
extern volatile float   g_blower_ctrl_integ_max_abs_rpm;
extern volatile float   g_blower_ctrl_meas_filter_alpha;
extern volatile float   g_blower_ctrl_deriv_filter_alpha;
extern volatile float   g_leak_patient_lpf_hz;
extern volatile float   g_leak_vivo_patient_lpf_hz;
extern volatile int32_t g_blower_ctrl_rpm_max;
extern volatile uint8_t g_blower_ctrl_enabled;
extern volatile uint8_t g_blower_ctrl_pid_enable;
extern volatile uint8_t g_blower_ctrl_rpm_floor_enable;

static uint8_t            s_buf[TELEM_BUF_SIZE];
static volatile uint32_t  s_head;
static volatile uint32_t  s_tail;
static uint8_t            s_tx_chunk[TELEM_TX_CHUNK];
static volatile uint8_t   s_inited = 0U;
static char               s_rx_line[96];
static uint8_t            s_rx_len = 0U;
static char               s_cmd_queue[TELEM_CMD_QUEUE_LEN][96];
static volatile uint8_t   s_cmd_head = 0U;
static volatile uint8_t   s_cmd_tail = 0U;

static void Telem_BufPush(const uint8_t *data, uint32_t len);
static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen);
static void Telem_QueueCommand(const char *cmd);

static void Telem_PushRaw(const char *text)
{
  size_t len;

  if (text == NULL)
  {
    return;
  }

  len = strnlen(text, 256U);
  Telem_BufPush((const uint8_t *)text, (uint32_t)len);
}

static void Telem_HandleCommand(const char *cmd)
{
  if (cmd == NULL) { return; }

  if ((strcmp(cmd, "telem on") == 0) || (strcmp(cmd, "telem 1") == 0))
  {
    g_telem_enabled = 1U;
    Telem_PushRaw("# telem on\r\n");
  }
  else if ((strcmp(cmd, "telem off") == 0) || (strcmp(cmd, "telem 0") == 0))
  {
    g_telem_enabled = 0U;
    Telem_PushRaw("# telem off\r\n");
  }
  else
  {
    Telem_QueueCommand(cmd);
  }
}

static void Telem_QueueCommand(const char *cmd)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint8_t next = (uint8_t)((s_cmd_head + 1U) % TELEM_CMD_QUEUE_LEN);
  if (next == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    Telem_PushRaw("# err cmd queue full\r\n");
    return;
  }

  (void)strncpy(s_cmd_queue[s_cmd_head], cmd, sizeof(s_cmd_queue[0]) - 1U);
  s_cmd_queue[s_cmd_head][sizeof(s_cmd_queue[0]) - 1U] = '\0';
  s_cmd_head = next;

  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Telem_OnRx(const uint8_t *data, uint32_t len)
{
  if ((data == NULL) || (len == 0U)) { return; }

  for (uint32_t i = 0U; i < len; i++)
  {
    char c = (char)data[i];
    if ((c == '\r') || (c == '\n'))
    {
      if (s_rx_len > 0U)
      {
        s_rx_line[s_rx_len] = '\0';
        Telem_HandleCommand(s_rx_line);
        s_rx_len = 0U;
      }
    }
    else if (s_rx_len < (sizeof(s_rx_line) - 1U))
    {
      s_rx_line[s_rx_len++] = c;
    }
    else
    {
      s_rx_len = 0U;
    }
  }
}

static void Telem_PushAck(const char *cmd)
{
  char line[112];
  int n = snprintf(line, sizeof(line), "# ack %s\r\n", cmd);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static void Telem_PushErr(const char *reason)
{
  char line[80];
  int n = snprintf(line, sizeof(line), "# err %s\r\n", reason);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static void Telem_PushStatus(void)
{
  char line[280];
  uint32_t now = HAL_GetTick();
  const uint8_t mode = Therapy_GetMode();
  const bool autoset = (mode != (uint8_t)THERAPY_MODE_CPAP);
  int n = snprintf(line, sizeof(line),
                   "S,%lu,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f"
                   ",%u,%u,%.3f,%.3f,%.3f,%u,%.3f,%u,%u,%.2f,%u"
                   ",%u,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.1f,%u\r\n",
                   (unsigned long)now,
                   (unsigned)Therapy_GetState(),
                   (double)Therapy_GetTargetPressure(),
                   (double)Therapy_GetSetpointCmh2o(),
                   (double)Therapy_GetMeasuredCmh2o(),
                   (double)Therapy_GetLeakSlm(),
                   Therapy_IsEprEnabled() ? 1U : 0U,
                   Therapy_IsMaskOffActive() ? 1U : 0U,
                   (unsigned)Therapy_GetFault(),
                   (unsigned)mode,
                   (double)Therapy_GetApapMinPressure(),
                   (double)Therapy_GetApapMaxPressure(),
                   autoset ? (unsigned)Apap_GetState() : 0U,
                   autoset && Apap_IsOscillating() ? 1U : 0U,
                   autoset ? (double)Apap_GetFotPressurePpCmh2o() : 0.0,
                   autoset ? (double)Apap_GetFotFlowPpSlm() : 0.0,
                   autoset ? (double)Apap_GetFotImpedanceCmh2oPerSlm() : 0.0,
                   autoset && Apap_IsOsaLikely() ? 1U : 0U,
                   autoset ? (double)Apap_GetOsaFlowPpThreshSlm() : 0.0,
                   autoset && Apap_IsOsaTreating() ? 1U : 0U,
                   (unsigned)Therapy_GetEprLevel(),
                   (double)Epr_GetActiveReliefCmh2o(),
                   (unsigned)Epr_GetPhase(),
                   (unsigned)Leak_GetAlgo(),
                   (long)Leak_GetMeanRatio(),
                   (long)Leak_GetMeanRatio10Sec(),
                   (double)Leak_GetVentSlm(),
                   (double)Leak_GetUnintentionalSlm(),
                   (double)Leak_GetFlowLeakSlm(),
                   (double)Leak_GetReportedLeakSlm(),
                   (double)Leak_GetRestVolumeMl(),
                   (unsigned)Leak_GetBreathInspiration());
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static int Telem_ParseFloat(const char *text, float *out)
{
  char *end = NULL;
  float v = strtof(text, &end);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  *out = v;
  return 1;
}

static int Telem_ParseLong(const char *text, long *out)
{
  char *end = NULL;
  long v = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  *out = v;
  return 1;
}

static int Telem_SetFloatParam(const char *name, float value)
{
  if (strcmp(name, "kp_low") == 0) { g_blower_ctrl_kp_low = value; return 1; }
  if (strcmp(name, "ki_low") == 0) { g_blower_ctrl_ki_low = value; return 1; }
  if (strcmp(name, "kd_low") == 0) { g_blower_ctrl_kd_low = value; return 1; }
  if (strcmp(name, "kp_mid") == 0) { g_blower_ctrl_kp_mid = value; return 1; }
  if (strcmp(name, "ki_mid") == 0) { g_blower_ctrl_ki_mid = value; return 1; }
  if (strcmp(name, "kd_mid") == 0) { g_blower_ctrl_kd_mid = value; return 1; }
  if (strcmp(name, "kp_high") == 0) { g_blower_ctrl_kp_high = value; return 1; }
  if (strcmp(name, "ki_high") == 0) { g_blower_ctrl_ki_high = value; return 1; }
  if (strcmp(name, "kd_high") == 0) { g_blower_ctrl_kd_high = value; return 1; }

  if (strcmp(name, "ff_k") == 0) { g_blower_ctrl_ff_k_rpm_per_sqrtcmh = value; return 1; }
  if (strcmp(name, "physics_ff_c_fan") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 0.001f) { value = 0.001f; }
    g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2 = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_flow_lpf_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 15.0f) { value = 15.0f; }
    g_blower_ctrl_physics_ff_flow_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_flow_lpf_inhale_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 25.0f) { value = 25.0f; }
    g_blower_ctrl_physics_ff_flow_lpf_inhale_hz = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_rise") == 0)
  {
    if (value < 100.0f) { value = 100.0f; }
    if (value > 20000.0f) { value = 20000.0f; }
    g_blower_ctrl_physics_ff_rise_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_fall") == 0)
  {
    if (value < 100.0f) { value = 100.0f; }
    if (value > 20000.0f) { value = 20000.0f; }
    g_blower_ctrl_physics_ff_fall_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_fall_exhale") == 0)
  {
    if (value < 100.0f) { value = 100.0f; }
    if (value > 25000.0f) { value = 25000.0f; }
    g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_flow_lpf_exhale_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 25.0f) { value = 25.0f; }
    g_blower_ctrl_physics_ff_flow_lpf_exhale_hz = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_use_total_flow") == 0)
  {
    g_blower_ctrl_physics_ff_use_total_flow = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_exhale_mit_enable") == 0)
  {
    g_blower_ctrl_physics_ff_exhale_mit_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_inhale_mit_enable") == 0)
  {
    g_blower_ctrl_physics_ff_inhale_mit_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_enable") == 0)
  {
    g_blower_ctrl_physics_ff_onset_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_k") == 0)
  {
    g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_thresh") == 0)
  {
    if (value < 10.0f) { value = 10.0f; }
    if (value > 500.0f) { value = 500.0f; }
    g_blower_ctrl_physics_ff_onset_thresh_slm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_max") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 5000.0f) { value = 5000.0f; }
    g_blower_ctrl_physics_ff_onset_max_rpm = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_inhale_rise_boost") == 0)
  {
    if (value < 1.0f) { value = 1.0f; }
    if (value > 4.0f) { value = 4.0f; }
    g_blower_ctrl_physics_ff_inhale_rise_boost = value;
    return 1;
  }
  if (strcmp(name, "under_p_gain") == 0)
  {
    if (value < 1.0f) { value = 1.0f; }
    if (value > 8.0f) { value = 8.0f; }
    g_blower_ctrl_under_p_gain = value;
    return 1;
  }
  if (strcmp(name, "under_p_thresh") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 1.0f) { value = 1.0f; }
    g_blower_ctrl_under_p_thresh_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "flow_ff_k") == 0) { g_blower_ctrl_flow_ff_k_rpm_per_slm = value; return 1; }
  if (strcmp(name, "flow_ff_max") == 0) { g_blower_ctrl_flow_ff_max_rpm = value; return 1; }
  if (strcmp(name, "flow_ff_deadband") == 0) { g_blower_ctrl_flow_ff_deadband_slm = value; return 1; }
  if (strcmp(name, "flow_ff_rise") == 0) { g_blower_ctrl_flow_ff_rise_rpm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_fall") == 0) { g_blower_ctrl_flow_ff_fall_rpm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_onset_k") == 0) { g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_onset_thresh") == 0) { g_blower_ctrl_flow_ff_onset_thresh_slm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_onset_max") == 0) { g_blower_ctrl_flow_ff_onset_max_rpm = value; return 1; }
  if (strcmp(name, "dip_ff_k") == 0) { g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "dip_ff_thresh") == 0) { g_blower_ctrl_dip_ff_thresh_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "dip_ff_max") == 0) { g_blower_ctrl_dip_ff_max_rpm = value; return 1; }
  if (strcmp(name, "dip_ff_err_min") == 0) { g_blower_ctrl_dip_ff_err_min_cmh2o = value; return 1; }
  if (strcmp(name, "over_p_gain") == 0) { g_blower_ctrl_over_p_gain = value; return 1; }
  if (strcmp(name, "over_brake") == 0) { g_blower_ctrl_over_brake_rpm_per_s = value; return 1; }
  if (strcmp(name, "slew_down") == 0) { g_blower_ctrl_slew_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "slew_up") == 0) { g_blower_ctrl_slew_up_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "out_alpha") == 0) { g_blower_ctrl_out_filter_alpha = value; return 1; }
  if (strcmp(name, "integ_deadband") == 0) { g_blower_ctrl_integ_deadband_cmh2o = value; return 1; }
  if (strcmp(name, "integ_max") == 0) { g_blower_ctrl_integ_max_abs_rpm = value; return 1; }
  if (strcmp(name, "meas_alpha") == 0) { g_blower_ctrl_meas_filter_alpha = value; return 1; }
  if (strcmp(name, "deriv_alpha") == 0) { g_blower_ctrl_deriv_filter_alpha = value; return 1; }
  if (strcmp(name, "patient_lpf_hz") == 0)
  {
    g_leak_patient_lpf_hz = value;
    g_leak_vivo_patient_lpf_hz = value;
    return 1;
  }
  return 0;
}

static int Telem_SetLeakIntParam(const char *name, long value)
{
  if (strcmp(name, "vivo_enable") == 0)
  {
    Leak_SetVivoEnable(value != 0L);
    return 1;
  }
  if (strcmp(name, "standard_ratio") == 0)
  {
    if (value < 100L) { value = 100L; }
    if (value > 32000L) { value = 32000L; }
    Leak_SetStandardRatio((int32_t)value);
    return 1;
  }
  return 0;
}

static int Telem_SetLeakFloatParam(const char *name, float value)
{
  if (strcmp(name, "vent_k") == 0)
  {
    Leak_SetVentCoefficient(value);
    return 1;
  }
  if (strcmp(name, "tau_seconds") == 0)
  {
    Leak_SetTimeConstant(value);
    return 1;
  }
  if (strcmp(name, "high_threshold") == 0)
  {
    Leak_SetHighThreshold(value);
    return 1;
  }
  if (strcmp(name, "patient_lpf_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 8.0f) { value = 8.0f; }
    g_leak_patient_lpf_hz = value;
    g_leak_vivo_patient_lpf_hz = value;
    return 1;
  }
  return 0;
}

static int Telem_SetTubeCompParam(const char *name, float value)
{
  if (strcmp(name, "enable") == 0)
  {
    TubeComp_SetEnable(value != 0.0f);
    return 1;
  }
  if (strcmp(name, "R_lam") == 0)
  {
    TubeComp_SetCoefficients(value, TubeComp_GetRTurb());
    return 1;
  }
  if (strcmp(name, "R_turb") == 0)
  {
    TubeComp_SetCoefficients(TubeComp_GetRLam(), value);
    return 1;
  }
  return 0;
}

static int Telem_SetIntParam(const char *name, long value)
{
  if (strcmp(name, "loop_enable") == 0)
  {
    BlowerCtrl_SetEnable(value != 0L);
    return 1;
  }
  if (strcmp(name, "pid_enable") == 0)
  {
    g_blower_ctrl_pid_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "rpm_floor_enable") == 0)
  {
    g_blower_ctrl_rpm_floor_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "ff_enable") == 0)
  {
    g_blower_ctrl_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "flow_ff_enable") == 0)
  {
    g_blower_ctrl_flow_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "flow_ff_onset_enable") == 0)
  {
    g_blower_ctrl_flow_ff_onset_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "dip_ff_enable") == 0)
  {
    g_blower_ctrl_dip_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "physics_ff_enable") == 0)
  {
    g_blower_ctrl_physics_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "flow_ff_sign") == 0)
  {
    g_blower_ctrl_flow_ff_inhale_sign = (value < 0L) ? (int8_t)-1 : (int8_t)1;
    return 1;
  }
  if (strcmp(name, "rpm_max") == 0)
  {
    if (value < 0L) { value = 0L; }
    if (value > 45000L) { value = 45000L; }
    g_blower_ctrl_rpm_max = (int32_t)value;
    return 1;
  }
  return 0;
}

static int Telem_SetFocParam(const char *name, long value)
{
  BlowerFocGainId_t id;
  int16_t gain;

  if (value < 0L) { value = 0L; }
  if (value > 20000L) { value = 20000L; }
  gain = (int16_t)value;

  if (strcmp(name, "speed_kp") == 0) { id = FOC_GAIN_SPEED_KP; }
  else if (strcmp(name, "speed_ki") == 0) { id = FOC_GAIN_SPEED_KI; }
  else if (strcmp(name, "torque_kp") == 0) { id = FOC_GAIN_TORQUE_KP; }
  else if (strcmp(name, "torque_ki") == 0) { id = FOC_GAIN_TORQUE_KI; }
  else if (strcmp(name, "flux_kp") == 0) { id = FOC_GAIN_FLUX_KP; }
  else if (strcmp(name, "flux_ki") == 0) { id = FOC_GAIN_FLUX_KI; }
  else { return 0; }

  return BlowerIpc_CM7_SetFocGain(id, gain) ? 1 : 0;
}

static int Telem_ParseManualSpeed(const char *args, int32_t *rpm, uint16_t *ramp_ms)
{
  long rpm_l = 0L;
  long ramp_l = 0L;
  char extra = '\0';

  if (sscanf(args, "%ld %ld%c", &rpm_l, &ramp_l, &extra) != 2)
  {
    return 0;
  }
  if (rpm_l < 0L) { rpm_l = 0L; }
  if (rpm_l > 45000L) { rpm_l = 45000L; }
  if (ramp_l < 0L) { ramp_l = 0L; }
  if (ramp_l > 65535L) { ramp_l = 65535L; }
  *rpm = (int32_t)rpm_l;
  *ramp_ms = (uint16_t)ramp_l;
  return 1;
}

static void Telem_BlowerManualStart(int32_t rpm, uint16_t ramp_ms)
{
  if (Therapy_GetState() != (uint8_t)THERAPY_STATE_IDLE)
  {
    Therapy_Stop();
  }
  BlowerCtrl_ManualStart(rpm, ramp_ms);
}

#define TELEM_CMD_BATCH_MAX  16U

static bool Telem_QueueTailStartsWith(const char *prefix, size_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cmd_head == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return false;
  }
  const int eq = (strncmp(s_cmd_queue[s_cmd_tail], prefix, len) == 0);
  if (primask == 0U)
  {
    __enable_irq();
  }
  return (eq != 0);
}

static bool Telem_DequeueNextCommand(char *cmd, size_t cmd_len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cmd_head == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return false;
  }
  (void)strncpy(cmd, s_cmd_queue[s_cmd_tail], cmd_len - 1U);
  cmd[cmd_len - 1U] = '\0';
  s_cmd_tail = (uint8_t)((s_cmd_tail + 1U) % TELEM_CMD_QUEUE_LEN);
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

static void Telem_ProcessOneCommand(const char *cmd);

static void Telem_DrainQueuedTherapySetCommands(void)
{
  char pending[sizeof(s_cmd_queue[0])];

  for (uint8_t n = 0U; n < TELEM_CMD_BATCH_MAX; n++)
  {
    if (!Telem_QueueTailStartsWith("therapy set ", 12U))
    {
      return;
    }
    if (!Telem_DequeueNextCommand(pending, sizeof(pending)))
    {
      return;
    }
    Telem_ProcessOneCommand(pending);
  }
}

static void Telem_ProcessOneCommand(const char *cmd)
{
  if (strcmp(cmd, "therapy start") == 0)
  {
    Telem_DrainQueuedTherapySetCommands();
    Therapy_Start();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy stop") == 0)
  {
    Therapy_Stop();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy status") == 0)
  {
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strcmp(cmd, "therapy set mode cpap") == 0)
  {
    Therapy_SetMode(THERAPY_MODE_CPAP);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy set mode autoset") == 0)
  {
    Therapy_SetMode(THERAPY_MODE_AUTOSET);
    Therapy_SetTargetPressure(Therapy_GetApapMinPressure());
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy set mode autoset_her") == 0)
  {
    Therapy_SetMode(THERAPY_MODE_AUTOSET_FOR_HER);
    Therapy_SetTargetPressure(Therapy_GetApapMinPressure());
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure_min ", 25U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[25], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure_min range 4..25");
      return;
    }
    Therapy_SetApapMinPressure(v);
    if (Therapy_GetMode() != THERAPY_MODE_CPAP)
    {
      Therapy_SetTargetPressure(Therapy_GetApapMinPressure());
    }
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure_max ", 25U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[25], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure_max range 4..25");
      return;
    }
    Therapy_SetApapMaxPressure(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure ", 21U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[21], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure range 4..25");
      return;
    }
    Therapy_SetMode(THERAPY_MODE_CPAP);
    Therapy_SetTargetPressure(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set ramp ", 17U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[17], &v) || (v < 0L) || (v > 45L))
    {
      Telem_PushErr("ramp range 0..45");
      return;
    }
    Therapy_SetRampMinutes((uint8_t)v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr ", 16U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[16], &v) || ((v != 0L) && (v != 1L)))
    {
      Telem_PushErr("epr must be 0 or 1");
      return;
    }
    Therapy_SetEprEnabled(v != 0L);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_relief ", 23U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[23], &v) || (v < 1.0f) || (v > 3.0f))
    {
      Telem_PushErr("epr_relief range 1..3");
      return;
    }
    Therapy_SetEprReliefCmh2o(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_level ", 20U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[20], &v) || (v < 1L) || (v > 3L))
    {
      Telem_PushErr("epr_level must be 1, 2, or 3");
      return;
    }
    Therapy_SetEprLevel((uint8_t)v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_type ", 21U) == 0)
  {
    if (strcmp(&cmd[21], "fulltime") == 0)
    {
      Therapy_SetEprType(0U);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    if (strcmp(&cmd[21], "ramp") == 0)
    {
      Therapy_SetEprType(1U);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    long v;
    if (Telem_ParseLong(&cmd[21], &v) && ((v == 0L) || (v == 1L)))
    {
      Therapy_SetEprType((uint8_t)v);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    Telem_PushErr("epr_type fulltime|ramp|0|1");
    return;
  }

  if (strncmp(cmd, "therapy set maskoff ", 20U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[20], &v) || ((v != 0L) && (v != 1L)))
    {
      Telem_PushErr("maskoff must be 0 or 1");
      return;
    }
    Therapy_SetMaskOffEnabled(v != 0L);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "blower manual start ", 20U) == 0)
  {
    int32_t rpm = 0;
    uint16_t ramp_ms = 0U;
    if (!Telem_ParseManualSpeed(&cmd[20], &rpm, &ramp_ms))
    {
      Telem_PushErr("manual start needs rpm ramp_ms");
      return;
    }
    Telem_BlowerManualStart(rpm, ramp_ms);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "blower manual speed ", 20U) == 0)
  {
    int32_t rpm = 0;
    uint16_t ramp_ms = 0U;
    if (!Telem_ParseManualSpeed(&cmd[20], &rpm, &ramp_ms))
    {
      Telem_PushErr("manual speed needs rpm ramp_ms");
      return;
    }
    BlowerCtrl_ManualSetSpeed(rpm, ramp_ms);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strcmp(cmd, "blower manual stop") == 0)
  {
    BlowerCtrl_ManualStop();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "blower set ", 11U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[11], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("blower set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetIntParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetFloatParam(name, fv))
    {
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown blower param");
    return;
  }

  if (strncmp(cmd, "leak set ", 9U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[9], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("leak set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetLeakIntParam(name, iv))
    {
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetLeakFloatParam(name, fv))
    {
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }

    Telem_PushErr("unknown leak param");
    return;
  }

  if (strncmp(cmd, "tubecomp set ", 13U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[13], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("tubecomp set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetTubeCompParam(name, (float)iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetTubeCompParam(name, fv))
    {
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown tubecomp param");
    return;
  }

  if (strncmp(cmd, "foc set ", 8U) == 0)
  {
    char name[32];
    char value_text[32];
    long iv;
    int matched = sscanf(&cmd[8], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("foc set needs name value");
      return;
    }
    if (!Telem_ParseLong(value_text, &iv))
    {
      Telem_PushErr("foc value must be integer");
      return;
    }
    if (Telem_SetFocParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    Telem_PushErr("unknown foc param");
    return;
  }

  Telem_PushErr("unknown command");
}

void Telem_ProcessCommands(void)
{
  for (uint8_t batch = 0U; batch < TELEM_CMD_BATCH_MAX; batch++)
  {
    char cmd[sizeof(s_cmd_queue[0])];

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_cmd_head == s_cmd_tail)
    {
      if (primask == 0U)
      {
        __enable_irq();
      }
      return;
    }
    (void)strncpy(cmd, s_cmd_queue[s_cmd_tail], sizeof(cmd) - 1U);
    cmd[sizeof(cmd) - 1U] = '\0';
    s_cmd_tail = (uint8_t)((s_cmd_tail + 1U) % TELEM_CMD_QUEUE_LEN);
    if (primask == 0U)
    {
      __enable_irq();
    }

    Telem_ProcessOneCommand(cmd);
  }
}

static void Telem_BufPush(const uint8_t *data, uint32_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t next = (s_head + 1U) & TELEM_BUF_MASK;
    if (next == s_tail)
    {
      s_tail = (s_tail + 1U) & TELEM_BUF_MASK;
      g_telem_dropped_count++;
    }
    s_buf[s_head] = data[i];
    s_head = next;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t head = s_head;
  uint32_t tail = s_tail;
  uint32_t avail;

  if (head >= tail)
  {
    avail = head - tail;
  }
  else
  {
    avail = TELEM_BUF_SIZE - tail;
  }

  if (avail > maxLen)
  {
    avail = maxLen;
  }

  for (uint32_t i = 0U; i < avail; i++)
  {
    dst[i] = s_buf[(tail + i) & TELEM_BUF_MASK];
  }
  s_tail = (tail + avail) & TELEM_BUF_MASK;

  if (primask == 0U)
  {
    __enable_irq();
  }

  return avail;
}

void Telem_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  s_rx_len = 0U;
  g_telem_sample_count = 0U;
  g_telem_dropped_count = 0U;
  s_inited = 1U;
  Telem_PushRaw("# telem ready (T,ms,P_sensor,Q,pid_rpm,mech_rpm,P_mask | R,...)\r\n");
}

void Telem_PushSample(float pressure_sensor_cmh2o, float patient_flow_slm,
                      int32_t pid_rpm_cmd, int32_t mech_rpm,
                      float pressure_mask_cmh2o)
{
  if ((s_inited == 0U) || (g_telem_enabled == 0U))
  {
    return;
  }

  char line[TELEM_FORMAT_BUF];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "T,%lu,%.2f,%.2f,%ld,%ld,%.2f\r\n",
                   (unsigned long)now,
                   (double)pressure_sensor_cmh2o,
                   (double)patient_flow_slm,
                   (long)pid_rpm_cmd,
                   (long)mech_rpm,
                   (double)pressure_mask_cmh2o);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
  g_telem_sample_count++;
}

void Telem_PushBenchRpm(int32_t rpm_cmd, int32_t rpm_act, float pressure_cmh2o)
{
  if (s_inited == 0U)
  {
    return;
  }

  char line[56];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "R,%lu,%ld,%ld,%.2f\r\n",
                   (unsigned long)now,
                   (long)rpm_cmd,
                   (long)rpm_act,
                   (double)pressure_cmh2o);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

void Telem_Pump(void)
{
  if (s_inited == 0U) { return; }
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }

  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) { return; }
  if (hcdc->TxState != 0U) { return; }

  uint32_t n = Telem_BufPopChunk(s_tx_chunk, sizeof(s_tx_chunk));
  if (n == 0U) { return; }

  (void)CDC_Transmit_FS(s_tx_chunk, (uint16_t)n);
}

#endif /* TELEM_ENABLED */
