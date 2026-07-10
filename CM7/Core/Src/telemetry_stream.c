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
#define TELEM_CMD_QUEUE_LEN  32U

volatile uint8_t  g_telem_enabled      = 1U;
volatile uint32_t g_telem_sample_count = 0U;
volatile uint32_t g_telem_dropped_count = 0U;

static uint8_t s_telem_buf[TELEM_BUF_SIZE];
static volatile uint32_t s_telem_head;
static volatile uint32_t s_telem_tail;

static char s_rx_line[96];
static uint32_t s_rx_len;

static char s_cmd_queue[TELEM_CMD_QUEUE_LEN][64];
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

static void Telem_PushStatus(void)
{
  BreathSimParams_t p;
  BreathSim_GetParams(&p);
  char line[160];
  const int n = snprintf(line, sizeof(line),
    "S,%lu,%u,%.2f,%.1f,%ld,%ld,%u,%.2f,%.2f,%u\r\n",
    (unsigned long)HAL_GetTick(),
    (unsigned)p.rate_bpm,
    (double)p.insp_time_s,
    (double)p.ie_ratio_exp,
    (long)p.rpm_base,
    (long)p.rpm_amplitude,
    (unsigned)p.waveform,
    (double)p.insp_pause_s,
    (double)p.exp_pause_s,
    (unsigned)(BreathSim_IsRunning() ? 1U : 0U));
  if (n > 0) { Telem_BufPush((const uint8_t *)line, (uint32_t)n); }
}

static void Telem_ApplyParams(const BreathSimParams_t *p)
{
  BreathSim_SetParams(p);
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

  if (strncmp(cmd, "sim set rate ", 13U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[13], &v) || (v < 4L) || (v > 60L))
    {
      Telem_PushErr("rate range 4..60");
      return;
    }
    BreathSimParams_t p;
    BreathSim_GetParams(&p);
    p.rate_bpm = (float)v;
    Telem_ApplyParams(&p);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "sim set insp ", 13U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[13], &v) || (v < 0.3f) || (v > 4.0f))
    {
      Telem_PushErr("insp range 0.3..4.0");
      return;
    }
    BreathSimParams_t p;
    BreathSim_GetParams(&p);
    p.insp_time_s = v;
    Telem_ApplyParams(&p);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "sim set ie ", 11U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[11], &v) || (v < 1L) || (v > 4L))
    {
      Telem_PushErr("ie range 1..4");
      return;
    }
    BreathSimParams_t p;
    BreathSim_GetParams(&p);
    p.ie_ratio_exp = (float)v;
    Telem_ApplyParams(&p);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "sim set rpm_base ", 17U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[17], &v) || (v < 2000L) || (v > 20000L))
    {
      Telem_PushErr("rpm_base range 2000..20000");
      return;
    }
    BreathSimParams_t p;
    BreathSim_GetParams(&p);
    p.rpm_base = (int32_t)v;
    Telem_ApplyParams(&p);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "sim set amplitude ", 18U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[18], &v) || (v < 0L) || (v > 25000L))
    {
      Telem_PushErr("amplitude range 0..25000");
      return;
    }
    BreathSimParams_t p;
    BreathSim_GetParams(&p);
    p.rpm_amplitude = (int32_t)v;
    Telem_ApplyParams(&p);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "sim set waveform ", 17U) == 0)
  {
    if (strcmp(&cmd[17], "sine") == 0)
    {
      BreathSimParams_t p;
      BreathSim_GetParams(&p);
      p.waveform = (uint8_t)BREATH_WAVE_SINE;
      Telem_ApplyParams(&p);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    if (strcmp(&cmd[17], "ramp") == 0)
    {
      BreathSimParams_t p;
      BreathSim_GetParams(&p);
      p.waveform = (uint8_t)BREATH_WAVE_RAMP;
      Telem_ApplyParams(&p);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    if (strcmp(&cmd[17], "square") == 0)
    {
      BreathSimParams_t p;
      BreathSim_GetParams(&p);
      p.waveform = (uint8_t)BREATH_WAVE_SQUARE;
      Telem_ApplyParams(&p);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    Telem_PushErr("waveform sine|ramp|square");
    return;
  }

  if (strncmp(cmd, "foc set ", 8U) == 0)
  {
    const char *args = &cmd[8];
    const char *sp = strchr(args, ' ');
    if (sp == NULL)
    {
      Telem_PushErr("foc set <name> <value>");
      return;
    }
    char name[24];
    const size_t nlen = (size_t)(sp - args);
    if (nlen >= sizeof(name))
    {
      Telem_PushErr("foc name too long");
      return;
    }
    memcpy(name, args, nlen);
    name[nlen] = '\0';
    long value = 0L;
    if (!Telem_ParseLong(sp + 1, &value))
    {
      Telem_PushErr("foc value");
      return;
    }
    BlowerFocGainId_t id;
    if (strcmp(name, "speed_kp") == 0) { id = FOC_GAIN_SPEED_KP; }
    else if (strcmp(name, "speed_ki") == 0) { id = FOC_GAIN_SPEED_KI; }
    else if (strcmp(name, "torque_kp") == 0) { id = FOC_GAIN_TORQUE_KP; }
    else if (strcmp(name, "torque_ki") == 0) { id = FOC_GAIN_TORQUE_KI; }
    else if (strcmp(name, "flux_kp") == 0) { id = FOC_GAIN_FLUX_KP; }
    else if (strcmp(name, "flux_ki") == 0) { id = FOC_GAIN_FLUX_KI; }
    else
    {
      Telem_PushErr("unknown foc param");
      return;
    }
    if (!BlowerIpc_CM7_SetFocGain(id, (int16_t)value))
    {
      Telem_PushErr("foc set failed");
      return;
    }
    Telem_PushAck(cmd);
    return;
  }

  Telem_PushErr("unknown command");
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

void Telem_PushBreathSample(int32_t rpm_cmd, int32_t rpm_act, float phase, float envelope)
{
  if (g_telem_enabled == 0U) { return; }

  char line[96];
  const int n = snprintf(line, sizeof(line), "B,%lu,%ld,%ld,%.3f,%.3f\r\n",
    (unsigned long)HAL_GetTick(),
    (long)rpm_cmd,
    (long)rpm_act,
    (double)phase,
    (double)envelope);
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
