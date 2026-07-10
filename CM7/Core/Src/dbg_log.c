/**
  ******************************************************************************
  * @file    dbg_log.c
  * @brief   Lightweight ring-buffered logger over the USB CDC VCP.
  ******************************************************************************
  *
  * Architecture:
  *   producers  ->  Dbg_Log() / DBG_x macros
  *                         |
  *                         v
  *                  vsnprintf into a small stack buffer
  *                         |
  *                         v
  *                  ring buffer (DBG_BUF_SIZE bytes, oldest-drop policy)
  *                         |
  *                         v
  *                  Dbg_Pump() (called from main loop)
  *                         |
  *                         v
  *                  CDC_Transmit_FS() (non-blocking)
  *
  * The ring buffer decouples log producers from the USB IN endpoint, so
  * loggers never block waiting for the host. If the host is disconnected
  * the bytes simply pile up and the oldest are discarded once the buffer
  * fills.
  *
  * The transmit path uses a small staging buffer (s_tx_chunk) so that
  * data handed to the USB DMA is not aliased by future producers. We only
  * advance the ring tail after copying out a chunk.
  ******************************************************************************
  */

#include "dbg_log.h"

#include <stdio.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ----------------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------------*/

/* Power-of-two ring size keeps wrap arithmetic to a bitmask. */
#ifndef DBG_BUF_SIZE
#define DBG_BUF_SIZE          4096U
#endif

/* USB CDC FS endpoint MPS is 64 - send one packet per pump call to keep
 * the host responsive and the IN buffer well behaved. */
#ifndef DBG_TX_CHUNK
#define DBG_TX_CHUNK          64U
#endif

/* Per-message stack scratch. Must hold timestamp + tag + body + CRLF. */
#ifndef DBG_FORMAT_BUF
#define DBG_FORMAT_BUF        160U
#endif

#if (DBG_BUF_SIZE & (DBG_BUF_SIZE - 1U)) != 0U
#error "DBG_BUF_SIZE must be a power of two"
#endif

#define DBG_BUF_MASK          (DBG_BUF_SIZE - 1U)

/* ----------------------------------------------------------------------------
 * Public state
 * --------------------------------------------------------------------------*/
volatile uint8_t  g_dbg_runtime_level = DBG_DEFAULT_RUNTIME_LEVEL;
volatile uint8_t  g_dbg_usb_ready     = 0U;
volatile uint32_t g_dbg_emitted_count = 0U;
volatile uint32_t g_dbg_dropped_count = 0U;
volatile uint32_t g_dbg_tx_busy_count = 0U;

/* ----------------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------------*/
static uint8_t            s_buf[DBG_BUF_SIZE];
static volatile uint32_t  s_head;     /* producer index */
static volatile uint32_t  s_tail;     /* consumer index */
static uint8_t            s_tx_chunk[DBG_TX_CHUNK];
static volatile uint8_t   s_inited = 0U;

/* ----------------------------------------------------------------------------
 * Ring-buffer primitives (IRQ safe via PRIMASK guard)
 * --------------------------------------------------------------------------*/
static void Dbg_BufPush(const uint8_t *data, uint32_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t next = (s_head + 1U) & DBG_BUF_MASK;
    if (next == s_tail)
    {
      /* Full: drop oldest byte to make room for the new one. */
      s_tail = (s_tail + 1U) & DBG_BUF_MASK;
      g_dbg_dropped_count++;
    }
    s_buf[s_head] = data[i];
    s_head = next;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint32_t Dbg_BufPopChunk(uint8_t *dst, uint32_t maxLen)
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
    /* Only return the contiguous part up to the wrap point so the
     * USB DMA sees a flat slice of memory. The next pump pulls the rest. */
    avail = DBG_BUF_SIZE - tail;
  }

  if (avail > maxLen)
  {
    avail = maxLen;
  }

  for (uint32_t i = 0U; i < avail; i++)
  {
    dst[i] = s_buf[(tail + i) & DBG_BUF_MASK];
  }
  s_tail = (tail + avail) & DBG_BUF_MASK;

  if (primask == 0U)
  {
    __enable_irq();
  }

  return avail;
}

/* ----------------------------------------------------------------------------
 * Output formatting
 * --------------------------------------------------------------------------*/
static char Dbg_LevelChar(uint8_t level)
{
  switch (level)
  {
    case DBG_LEVEL_ERR:   return 'E';
    case DBG_LEVEL_WARN:  return 'W';
    case DBG_LEVEL_INFO:  return 'I';
    case DBG_LEVEL_DEBUG: return 'D';
    default:              return '?';
  }
}

void Dbg_LogV(uint8_t level, const char *tag, const char *fmt, va_list ap)
{
  if (s_inited == 0U) { return; }
  if ((level == DBG_LEVEL_OFF) || (level > g_dbg_runtime_level)) { return; }

  char buf[DBG_FORMAT_BUF];
  int  n = 0;

  uint32_t now = HAL_GetTick();
  n = snprintf(buf, sizeof(buf), "[%lu.%03lu] %c ",
               (unsigned long)(now / 1000U),
               (unsigned long)(now % 1000U),
               Dbg_LevelChar(level));
  if ((n < 0) || (n >= (int)sizeof(buf))) { return; }

  if ((tag != NULL) && (tag[0] != '\0'))
  {
    int t = snprintf(&buf[n], sizeof(buf) - (size_t)n, "[%s] ", tag);
    if (t < 0) { return; }
    n += t;
    if (n >= (int)sizeof(buf)) { n = (int)sizeof(buf) - 1; }
  }

  if (fmt != NULL)
  {
    int b = vsnprintf(&buf[n], sizeof(buf) - (size_t)n, fmt, ap);
    if (b < 0) { return; }
    n += b;
    if (n > (int)sizeof(buf) - 2) { n = (int)sizeof(buf) - 2; }
  }

  buf[n++] = '\r';
  buf[n++] = '\n';

  Dbg_BufPush((const uint8_t *)buf, (uint32_t)n);
  g_dbg_emitted_count++;
}

void Dbg_Log(uint8_t level, const char *tag, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  Dbg_LogV(level, tag, fmt, ap);
  va_end(ap);
}

void Dbg_LogHex(uint8_t level, const char *tag, const char *prefix,
                const void *data, size_t len)
{
  if (s_inited == 0U) { return; }
  if ((level == DBG_LEVEL_OFF) || (level > g_dbg_runtime_level)) { return; }
  if ((data == NULL) || (len == 0U)) { return; }

  /* Cap to keep one line bounded. Adjust if you need longer dumps. */
  static const size_t kMax = 32U;
  size_t n = (len < kMax) ? len : kMax;

  char  hex[3 * 32 + 4];
  char *p = hex;
  const uint8_t *b = (const uint8_t *)data;
  for (size_t i = 0; i < n; i++)
  {
    int w = snprintf(p, (size_t)(&hex[sizeof(hex)] - p),
                     (i == 0) ? "%02X" : " %02X", b[i]);
    if (w <= 0) { break; }
    p += w;
  }
  if (n < len)
  {
    (void)snprintf(p, (size_t)(&hex[sizeof(hex)] - p), "..");
  }

  Dbg_Log(level, tag, "%s%s%s",
          (prefix != NULL) ? prefix : "",
          (prefix != NULL) ? " "    : "",
          hex);
}

/* ----------------------------------------------------------------------------
 * Pump - drain ring buffer to USB CDC. Call from main loop.
 * --------------------------------------------------------------------------*/
void Dbg_Pump(void)
{
  if (s_inited == 0U) { return; }
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }

  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) { return; }
  if (hcdc->TxState != 0U) { g_dbg_tx_busy_count++; return; }

  uint32_t n = Dbg_BufPopChunk(s_tx_chunk, sizeof(s_tx_chunk));
  if (n == 0U) { return; }

  /* CDC_Transmit_FS does not copy - point it at our staging buffer.
   * If the call returns BUSY we lose this slice; we rely on the host
   * picking up the next pump iteration to keep the log flowing. */
  (void)CDC_Transmit_FS(s_tx_chunk, (uint16_t)n);
}

/* ----------------------------------------------------------------------------
 * Lifetime
 * --------------------------------------------------------------------------*/
void Dbg_SetRuntimeLevel(uint8_t level)
{
  if (level > DBG_LEVEL_DEBUG) { level = DBG_LEVEL_DEBUG; }
  g_dbg_runtime_level = level;
}

void Dbg_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  g_dbg_emitted_count = 0U;
  g_dbg_dropped_count = 0U;
  g_dbg_tx_busy_count = 0U;
  g_dbg_usb_ready = 1U;
  s_inited = 1U;

  /* First line is helpful so a freshly opened terminal sees something. */
  Dbg_Log(DBG_LEVEL_INFO, "SYS", "debug log online (lvl=%u)",
          (unsigned)g_dbg_runtime_level);
}
