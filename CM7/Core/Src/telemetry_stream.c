/**
  ******************************************************************************
  * @file    telemetry_stream.c
  * @brief   Breath simulator USB CDC telemetry and commands.
  ******************************************************************************
  */

#include "telemetry_stream.h"

#if TELEM_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"
#include "breath_sim.h"
#include "blower_ipc.h"

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

#define TELEM_BUF_MASK   (TELEM_BUF_SIZE - 1U)
#define TELEM_CMD_QUEUE_LEN  16U
#define TELEM_CMD_MAX_LEN    96U

volatile uint8_t  g_telem_enabled      = 1U;
volatile uint32_t g_telem_sample_count = 0U;
volatile uint32_t g_telem_dropped_count = 0U;

static uint8_t s_telem_buf[TELEM_BUF_SIZE];
static volatile uint32_t s_telem_head;
static volatile uint32_t s_telem_tail;

static char s_rx_line[96];
static uint32_t s_rx_len;

static char s_cmd_queue[TELEM_CMD_QUEUE_LEN][TELEM_CMD_MAX_LEN];
static volatile uint8_t s_cmd_head;
static volatile uint8_t s_cmd_tail;

static void Telem_BufPush(const uint8_t *data, uint32_t len);
static void Telem_PushRaw(const char *text);
static void Telem_PushAck(const char *cmd);
static void Telem_PushErr(const char *reason);
static void Telem_PushStatus(void);
static void Telem_ProcessOneCommand(const char *cmd);

void Telem_Init(void)
{
  s_telem_head = 0U;
  s_telem_tail = 0U;
  s_rx_len = 0U;
  s_cmd_head = 0U;
  s_cmd_tail = 0U;
}

static void Telem_BufPush(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t next = (s_telem_head + 1U) & TELEM_BUF_MASK;
    if (next == s_telem_tail)
    {
      g_telem_dropped_count++;
      return;
    }
    s_telem_buf[s_telem_head] = data[i];
    s_telem_head = next;
  }
}

static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen)
{
  uint32_t n = 0U;
  while ((n < maxLen) && (s_telem_tail != s_telem_head))
  {
    dst[n++] = s_telem_buf[s_telem_tail];
    s_telem_tail = (s_telem_tail + 1U) & TELEM_BUF_MASK;
  }
  return n;
}

static void Telem_PushRaw(const char *text)
{
  if (text == NULL) { return; }
  Telem_BufPush((const uint8_t *)text, (uint32_t)strlen(text));
}

static void Telem_QueueCommand(const char *cmd)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint8_t next = (uint8_t)((s_cmd_head + 1U) % TELEM_CMD_QUEUE_LEN);
  if (next == s_cmd_tail)
  {
    if (primask == 0U) { __enable_irq(); }
    Telem_PushRaw("# err cmd queue full\r\n");
    return;
  }
  (void)strncpy(s_cmd_queue[s_cmd_head], cmd, sizeof(s_cmd_queue[0]) - 1U);
  s_cmd_queue[s_cmd_head][sizeof(s_cmd_queue[0]) - 1U] = '\0';
  s_cmd_head = next;
  if (primask == 0U) { __enable_irq(); }
}

static void Telem_HandleCommand(const char *cmd)
{
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

void Telem_OnRx(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0U; i < len; i++)
  {
    const char c = (char)data[i];
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
  }
}

static void Telem_PushAck(const char *cmd)
{
  char line[96];
  const int n = snprintf(line, sizeof(line), "# ack %s\r\n", cmd);
  if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
}

static void Telem_PushErr(const char *reason)
{
  char line[96];
  const int n = snprintf(line, sizeof(line), "# err %s\r\n", reason);
  if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
}

static int Telem_ParseFloat(const char *text, float *out)
{
  char *end = NULL;
  const float v = strtof(text, &end);
  if ((end == text) || (out == NULL)) { return 0; }
  *out = v;
  return 1;
}

static int Telem_ParseLong(const char *text, long *out)
{
  char *end = NULL;
  const long v = strtol(text, &end, 10);
  if ((end == text) || (out == NULL)) { return 0; }
  *out = v;
  return 1;
}

/* ---------------------------------------------------------------------------
 * Status / sample lines
 *
 *  S, tick_ms, state, running, timing_mode, rate_bpm, period_s, insp_s,
 *     ie_ratio, insp_pause_s, exp_s, exp_pause_s, rpm_base, rpm_amp,
 *     waveform, exp_tau_s, flattening, jitter_pct, seed, flags
 *
 *  B, tick_ms, breath_index, segment, event, flags, rpm_cmd, rpm_act,
 *     phase, envelope
 *
 *  M, tick_ms, breath_index          <- emitted at each breath boundary
 *
 * `flags` is the sticky BREATH_FLAG_* bitfield for the run. A non-zero
 * value means the delivered waveform may not match the requested one, so
 * analysis should treat the affected breaths as suspect.
 * ------------------------------------------------------------------------ */

static void Telem_PushStatus(void)
{
  BreathSimParams_t p;
  BreathSimTiming_t t;
  BreathSimStatus_t st;
  BreathSim_GetParams(&p);
  BreathSim_GetTiming(&t);
  BreathSim_GetStatus(&st);

  char line[224];
  const int n = snprintf(line, sizeof(line),
    "S,%lu,%s,%u,%s,%.2f,%.3f,%.3f,%.2f,%.2f,%.3f,%.2f,%ld,%ld,%s,%.2f,%.2f,%.1f,%lu,0x%04X\r\n",
    (unsigned long)HAL_GetTick(),
    BreathSim_StateName(st.state),
    (unsigned)(BreathSim_IsRunning() ? 1U : 0U),
    (p.timing_mode == (uint8_t)BREATH_TIMING_EXPLICIT) ? "explicit" : "rate_ie",
    (double)t.rate_bpm,
    (double)t.period_s,
    (double)t.insp_s,
    (double)p.ie_ratio_exp,
    (double)t.insp_pause_s,
    (double)t.exp_s,
    (double)t.exp_pause_s,
    (long)p.rpm_base,
    (long)p.rpm_amplitude,
    BreathSim_WaveformName(p.waveform),
    (double)p.exp_tau_s,
    (double)p.flattening,
    (double)p.jitter_pct,
    (unsigned long)p.jitter_seed,
    (unsigned)st.flags);
  if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
}

/* ---------------------------------------------------------------------------
 * Parameter table - the single place the USB interface learns the limits.
 * These used to be hardcoded here as well as in breath_sim.c and in the
 * TouchGFX settings editor; three copies that were free to drift.
 * ------------------------------------------------------------------------ */

typedef enum { TP_FLOAT = 0, TP_INT } TelemParamType_t;

typedef struct
{
  const char      *name;
  TelemParamType_t type;
  float            min_v;
  float            max_v;
  size_t           offset;
} TelemParamDef_t;

static const TelemParamDef_t k_param_defs[] = {
  { "rate",       TP_FLOAT, BREATH_RATE_MIN_BPM,  BREATH_RATE_MAX_BPM,     offsetof(BreathSimParams_t, rate_bpm)      },
  { "insp",       TP_FLOAT, BREATH_INSP_MIN_S,    BREATH_INSP_MAX_S,       offsetof(BreathSimParams_t, insp_time_s)   },
  { "ie",         TP_FLOAT, BREATH_IE_RATIO_MIN,  BREATH_IE_RATIO_MAX,     offsetof(BreathSimParams_t, ie_ratio_exp)  },
  { "insp_pause", TP_FLOAT, 0.0f,                 BREATH_INSP_PAUSE_MAX_S, offsetof(BreathSimParams_t, insp_pause_s)  },
  { "exp_pause",  TP_FLOAT, 0.0f,                 BREATH_EXP_PAUSE_MAX_S,  offsetof(BreathSimParams_t, exp_pause_s)   },
  { "exp_tau",    TP_FLOAT, BREATH_EXP_TAU_MIN_S, BREATH_EXP_TAU_MAX_S,    offsetof(BreathSimParams_t, exp_tau_s)     },
  { "flattening", TP_FLOAT, 0.0f,                 BREATH_FLATTENING_MAX,   offsetof(BreathSimParams_t, flattening)    },
  { "jitter",     TP_FLOAT, 0.0f,                 BREATH_JITTER_MAX_PCT,   offsetof(BreathSimParams_t, jitter_pct)    },
  { "rpm_base",   TP_INT,   (float)BREATH_RPM_BASE_MIN, (float)BREATH_RPM_BASE_MAX, offsetof(BreathSimParams_t, rpm_base)      },
  { "amplitude",  TP_INT,   0.0f,                 (float)BREATH_RPM_AMP_MAX,  offsetof(BreathSimParams_t, rpm_amplitude) },
};
#define TELEM_PARAM_COUNT  (sizeof(k_param_defs) / sizeof(k_param_defs[0]))

/** Apply a staged parameter set and report whether it had to be adjusted. */
static void Telem_ApplyParams(const BreathSimParams_t *p, const char *cmd)
{
  if (!BreathSim_SetParams(p))
  {
    /* Rate, inspiratory time and I:E are over-determined; say so rather
     * than silently running something other than what was asked for. */
    Telem_PushRaw("# warn request adjusted for consistency; see S line\r\n");
  }
  Telem_PushAck(cmd);
  Telem_PushStatus();
}

/** Split "<word> <rest>"; returns rest or NULL. Copies the word into buf. */
static const char *Telem_SplitWord(const char *text, char *buf, size_t buf_len)
{
  const char *sp = strchr(text, ' ');
  size_t n = (sp != NULL) ? (size_t)(sp - text) : strlen(text);
  if (n >= buf_len) { n = buf_len - 1U; }
  memcpy(buf, text, n);
  buf[n] = '\0';
  return (sp != NULL) ? (sp + 1) : NULL;
}

static int Telem_ParseEventName(const char *name, uint8_t *out)
{
  if (strcmp(name, "none") == 0)      { *out = (uint8_t)BREATH_EVENT_NONE;       return 1; }
  if (strcmp(name, "apnea") == 0)     { *out = (uint8_t)BREATH_EVENT_APNEA;      return 1; }
  if (strcmp(name, "hypopnea") == 0)  { *out = (uint8_t)BREATH_EVENT_HYPOPNEA;   return 1; }
  if (strcmp(name, "flowlimit") == 0) { *out = (uint8_t)BREATH_EVENT_FLOW_LIMIT; return 1; }
  if (strcmp(name, "csr") == 0)       { *out = (uint8_t)BREATH_EVENT_CSR;        return 1; }
  return 0;
}

static void Telem_PushHelp(void)
{
  Telem_PushRaw("# sim start | stop | status | help\r\n");
  Telem_PushRaw("# sim set <param> <value>\r\n");
  Telem_PushRaw("#   rate insp ie insp_pause exp_pause exp_tau flattening jitter\r\n");
  Telem_PushRaw("#   rpm_base amplitude seed\r\n");
  Telem_PushRaw("# sim set waveform sine|ramp|square|table\r\n");
  Telem_PushRaw("# sim set timing rate_ie|explicit\r\n");
  Telem_PushRaw("# sim table clear | add <v>.. | show   (values 0..1)\r\n");
  Telem_PushRaw("# sim event <apnea|hypopnea|flowlimit|csr> <secs> [severity%]\r\n");
  Telem_PushRaw("# sim event cancel\r\n");
  Telem_PushRaw("# sim script clear | add <event> <secs> [severity%] | start [loop] | stop\r\n");
  Telem_PushRaw("# foc set <gain> <value>\r\n");
}

/* --------------------------------------------------------------------- */

static void Telem_CmdSet(const char *cmd, const char *args)
{
  char name[24];
  const char *rest = Telem_SplitWord(args, name, sizeof(name));

  if (rest == NULL)
  {
    Telem_PushErr("sim set <param> <value>");
    return;
  }

  BreathSimParams_t p;
  BreathSim_GetParams(&p);

  if (strcmp(name, "waveform") == 0)
  {
    if      (strcmp(rest, "sine") == 0)   { p.waveform = (uint8_t)BREATH_WAVE_SINE; }
    else if (strcmp(rest, "ramp") == 0)   { p.waveform = (uint8_t)BREATH_WAVE_RAMP; }
    else if (strcmp(rest, "square") == 0) { p.waveform = (uint8_t)BREATH_WAVE_SQUARE; }
    else if (strcmp(rest, "table") == 0)
    {
      if (BreathSim_GetTableLength() < 2U)
      {
        Telem_PushErr("no table loaded; use sim table add");
        return;
      }
      p.waveform = (uint8_t)BREATH_WAVE_TABLE;
    }
    else
    {
      Telem_PushErr("waveform sine|ramp|square|table");
      return;
    }
    Telem_ApplyParams(&p, cmd);
    return;
  }

  if (strcmp(name, "timing") == 0)
  {
    if      (strcmp(rest, "rate_ie") == 0)  { p.timing_mode = (uint8_t)BREATH_TIMING_RATE_IE; }
    else if (strcmp(rest, "explicit") == 0) { p.timing_mode = (uint8_t)BREATH_TIMING_EXPLICIT; }
    else { Telem_PushErr("timing rate_ie|explicit"); return; }
    Telem_ApplyParams(&p, cmd);
    return;
  }

  if (strcmp(name, "seed") == 0)
  {
    long v;
    if (!Telem_ParseLong(rest, &v) || (v < 0L)) { Telem_PushErr("seed >= 0"); return; }
    p.jitter_seed = (uint32_t)v;
    Telem_ApplyParams(&p, cmd);
    return;
  }

  for (size_t i = 0U; i < TELEM_PARAM_COUNT; i++)
  {
    const TelemParamDef_t *d = &k_param_defs[i];
    if (strcmp(name, d->name) != 0) { continue; }

    float v;
    if (!Telem_ParseFloat(rest, &v))
    {
      Telem_PushErr("bad value");
      return;
    }
    if ((v < d->min_v) || (v > d->max_v))
    {
      char msg[80];
      (void)snprintf(msg, sizeof(msg), "%s range %.2f..%.2f",
                     d->name, (double)d->min_v, (double)d->max_v);
      Telem_PushErr(msg);
      return;
    }

    uint8_t *base = (uint8_t *)&p;
    if (d->type == TP_FLOAT)
    {
      *(float *)(void *)(base + d->offset) = v;
    }
    else
    {
      *(int32_t *)(void *)(base + d->offset) = (int32_t)v;
    }

    /* The blower cannot be trusted below the sensorless observer minimum;
     * warn at the point of setting rather than only in the run flags. */
    if ((d->offset == offsetof(BreathSimParams_t, rpm_base)) &&
        (v < (float)BREATH_RPM_OBS_MIN_RPM))
    {
      char msg[104];
      (void)snprintf(msg, sizeof(msg),
        "# warn rpm_base %d below observer minimum %d; speed feedback unreliable\r\n",
        (int)v, BREATH_RPM_OBS_MIN_RPM);
      Telem_PushRaw(msg);
    }

    Telem_ApplyParams(&p, cmd);
    return;
  }

  Telem_PushErr("unknown param (try: sim help)");
}

static void Telem_CmdTable(const char *cmd, const char *args)
{
  static float   s_pending[BREATH_TABLE_POINTS];
  static uint8_t s_pending_len;

  if (strcmp(args, "clear") == 0)
  {
    s_pending_len = 0U;
    Telem_PushAck(cmd);
    return;
  }

  if (strcmp(args, "show") == 0)
  {
    char line[64];
    const int n = snprintf(line, sizeof(line), "# table staged=%u active=%u\r\n",
                           (unsigned)s_pending_len, (unsigned)BreathSim_GetTableLength());
    if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
    return;
  }

  if (strncmp(args, "add ", 4U) == 0)
  {
    const char *cursor = &args[4];
    while (*cursor != '\0')
    {
      while (*cursor == ' ') { cursor++; }
      if (*cursor == '\0') { break; }

      char *end = NULL;
      const float v = strtof(cursor, &end);
      if (end == cursor)
      {
        Telem_PushErr("table values must be numeric");
        return;
      }
      if ((v < 0.0f) || (v > 1.0f))
      {
        Telem_PushErr("table values 0..1");
        return;
      }
      if (s_pending_len >= (uint8_t)BREATH_TABLE_POINTS)
      {
        Telem_PushErr("table full");
        return;
      }
      s_pending[s_pending_len++] = v;
      cursor = end;
    }

    if (s_pending_len >= 2U)
    {
      (void)BreathSim_SetTable(s_pending, s_pending_len);
    }
    char line[48];
    const int n = snprintf(line, sizeof(line), "# table %u points\r\n",
                           (unsigned)s_pending_len);
    if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
    Telem_PushAck(cmd);
    return;
  }

  Telem_PushErr("sim table clear|add <v>..|show");
}

static void Telem_CmdEvent(const char *cmd, const char *args)
{
  if (strcmp(args, "cancel") == 0)
  {
    BreathSim_CancelEvent();
    Telem_PushAck(cmd);
    return;
  }

  char name[24];
  const char *rest = Telem_SplitWord(args, name, sizeof(name));

  BreathEvent_t ev;
  if (!Telem_ParseEventName(name, &ev.type))
  {
    Telem_PushErr("event apnea|hypopnea|flowlimit|csr|cancel");
    return;
  }
  if (rest == NULL)
  {
    Telem_PushErr("sim event <type> <secs> [severity%]");
    return;
  }

  char dur_txt[16];
  const char *sev_txt = Telem_SplitWord(rest, dur_txt, sizeof(dur_txt));
  float dur;
  if (!Telem_ParseFloat(dur_txt, &dur) || (dur <= 0.0f) || (dur > 600.0f))
  {
    Telem_PushErr("duration 0..600 s");
    return;
  }

  float sev_pct = 100.0f;
  if ((sev_txt != NULL) && !Telem_ParseFloat(sev_txt, &sev_pct))
  {
    Telem_PushErr("severity 0..100");
    return;
  }
  ev.duration_s = dur;
  ev.severity   = sev_pct * 0.01f;

  if (!BreathSim_TriggerEvent(&ev))
  {
    Telem_PushErr("event rejected (script running?)");
    return;
  }
  Telem_PushAck(cmd);
}

static void Telem_CmdScript(const char *cmd, const char *args)
{
  if (strcmp(args, "clear") == 0)
  {
    BreathSim_ScriptClear();
    Telem_PushAck(cmd);
    return;
  }
  if (strcmp(args, "stop") == 0)
  {
    BreathSim_ScriptStop();
    Telem_PushAck(cmd);
    return;
  }
  if ((strcmp(args, "start") == 0) || (strcmp(args, "start loop") == 0))
  {
    if (!BreathSim_ScriptStart(strcmp(args, "start loop") == 0))
    {
      Telem_PushErr("script empty");
      return;
    }
    Telem_PushAck(cmd);
    return;
  }

  if (strncmp(args, "add ", 4U) == 0)
  {
    char name[24];
    const char *rest = Telem_SplitWord(&args[4], name, sizeof(name));

    BreathScriptStep_t step;
    if (!Telem_ParseEventName(name, &step.event))
    {
      Telem_PushErr("script event none|apnea|hypopnea|flowlimit|csr");
      return;
    }
    if (rest == NULL)
    {
      Telem_PushErr("sim script add <event> <secs> [severity%]");
      return;
    }

    char dur_txt[16];
    const char *sev_txt = Telem_SplitWord(rest, dur_txt, sizeof(dur_txt));
    float dur;
    if (!Telem_ParseFloat(dur_txt, &dur) || (dur <= 0.0f) || (dur > 3600.0f))
    {
      Telem_PushErr("duration 0..3600 s");
      return;
    }
    float sev_pct = 100.0f;
    if ((sev_txt != NULL) && !Telem_ParseFloat(sev_txt, &sev_pct))
    {
      Telem_PushErr("severity 0..100");
      return;
    }
    step.duration_s = dur;
    step.severity   = sev_pct * 0.01f;

    if (!BreathSim_ScriptAppend(&step))
    {
      Telem_PushErr("script full");
      return;
    }
    char line[48];
    const int n = snprintf(line, sizeof(line), "# script %u steps\r\n",
                           (unsigned)BreathSim_ScriptLength());
    if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
    Telem_PushAck(cmd);
    return;
  }

  Telem_PushErr("sim script clear|add|start [loop]|stop");
}

static void Telem_CmdFoc(const char *cmd, const char *args)
{
  char name[24];
  const char *rest = Telem_SplitWord(args, name, sizeof(name));
  if (rest == NULL)
  {
    Telem_PushErr("foc set <name> <value>");
    return;
  }

  long value = 0L;
  if (!Telem_ParseLong(rest, &value))
  {
    Telem_PushErr("foc value");
    return;
  }

  BlowerFocGainId_t id;
  if      (strcmp(name, "speed_kp")  == 0) { id = FOC_GAIN_SPEED_KP; }
  else if (strcmp(name, "speed_ki")  == 0) { id = FOC_GAIN_SPEED_KI; }
  else if (strcmp(name, "torque_kp") == 0) { id = FOC_GAIN_TORQUE_KP; }
  else if (strcmp(name, "torque_ki") == 0) { id = FOC_GAIN_TORQUE_KI; }
  else if (strcmp(name, "flux_kp")   == 0) { id = FOC_GAIN_FLUX_KP; }
  else if (strcmp(name, "flux_ki")   == 0) { id = FOC_GAIN_FLUX_KI; }
  else { Telem_PushErr("unknown foc param"); return; }

  if (!BlowerIpc_CM7_SetFocGain(id, (int16_t)value))
  {
    Telem_PushErr("foc set failed");
    return;
  }
  Telem_PushAck(cmd);
}

static void Telem_ProcessOneCommand(const char *cmd)
{
  if (strcmp(cmd, "sim start") == 0)
  {
    BreathSim_Start();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "sim stop") == 0)
  {
    BreathSim_Stop();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "sim status") == 0)
  {
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if ((strcmp(cmd, "sim help") == 0) || (strcmp(cmd, "help") == 0))
  {
    Telem_PushHelp();
    return;
  }

  if (strncmp(cmd, "sim set ", 8U) == 0)     { Telem_CmdSet(cmd, &cmd[8]);     return; }
  if (strncmp(cmd, "sim table ", 10U) == 0)  { Telem_CmdTable(cmd, &cmd[10]);  return; }
  if (strncmp(cmd, "sim event ", 10U) == 0)  { Telem_CmdEvent(cmd, &cmd[10]);  return; }
  if (strncmp(cmd, "sim script ", 11U) == 0) { Telem_CmdScript(cmd, &cmd[11]); return; }
  if (strncmp(cmd, "foc set ", 8U) == 0)     { Telem_CmdFoc(cmd, &cmd[8]);     return; }

  Telem_PushErr("unknown command (try: sim help)");
}

void Telem_ProcessCommands(void)
{
  char cmd[sizeof(s_cmd_queue[0])];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cmd_head == s_cmd_tail)
  {
    if (primask == 0U) { __enable_irq(); }
    return;
  }
  (void)strncpy(cmd, s_cmd_queue[s_cmd_tail], sizeof(cmd) - 1U);
  cmd[sizeof(cmd) - 1U] = '\0';
  s_cmd_tail = (uint8_t)((s_cmd_tail + 1U) % TELEM_CMD_QUEUE_LEN);
  if (primask == 0U) { __enable_irq(); }
  Telem_ProcessOneCommand(cmd);
}

void Telem_PushBreathSample(const BreathSimStatus_t *st)
{
  static uint32_t s_last_breath;
  static uint8_t  s_have_last;

  if ((g_telem_enabled == 0U) || (st == NULL)) { return; }

  char line[128];

  /* Breath-boundary marker, so a host log can be aligned against the
   * device-under-test's own log without inferring breath starts. */
  if ((s_have_last == 0U) || (st->breath_index != s_last_breath))
  {
    const int m = snprintf(line, sizeof(line), "M,%lu,%lu\r\n",
      (unsigned long)HAL_GetTick(), (unsigned long)st->breath_index);
    if (m > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)m); }
    s_last_breath = st->breath_index;
    s_have_last = 1U;
  }

  const int n = snprintf(line, sizeof(line),
    "B,%lu,%lu,%s,%s,0x%04X,%ld,%ld,%.3f,%.3f\r\n",
    (unsigned long)HAL_GetTick(),
    (unsigned long)st->breath_index,
    BreathSim_SegmentName(st->segment),
    BreathSim_EventName(st->event),
    (unsigned)st->flags,
    (long)st->rpm_cmd,
    (long)st->rpm_act,
    (double)st->phase,
    (double)st->envelope);
  if (n > 0)
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    g_telem_sample_count++;
  }
}

void Telem_Pump(void)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if ((hcdc == NULL) || (hcdc->TxState != 0U)) { return; }

  static uint8_t chunk[TELEM_TX_CHUNK];
  const uint32_t n = Telem_BufPopChunk(chunk, TELEM_TX_CHUNK);
  if (n == 0U) { return; }
  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, chunk, n);
  (void)USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

#endif /* TELEM_ENABLED */
