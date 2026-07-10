/**
  ******************************************************************************
  * @file    dbg_log.h
  * @brief   Lightweight printf-style debug logger over the USB CDC VCP.
  ******************************************************************************
  *
  * Usage
  * -----
  *   #include "dbg_log.h"
  *
  *   DBG_E("NFC", "init failed rc=%d", rc);
  *   DBG_W("SD",  "card not present");
  *   DBG_I("SYS", "boot ok, fw=%s", FW_VER);
  *   DBG_D("SPI", "tx %u bytes", n);
  *
  * Levels (highest -> lowest verbosity):
  *   DBG_LEVEL_DEBUG  > DBG_LEVEL_INFO > DBG_LEVEL_WARN > DBG_LEVEL_ERR > DBG_LEVEL_OFF
  *
  * Two independent gates:
  *   - DBG_MAX_LEVEL    : compile-time, removes higher-level calls entirely.
  *   - g_dbg_runtime_level : runtime, can be tweaked from the debugger
  *                          watch window or from a future console command.
  *
  * Behaviour:
  *   - Calls are non-blocking. If the ring buffer is full, the OLDEST bytes
  *     are dropped (counter g_dbg_dropped_count is incremented). This keeps
  *     the most recent log lines visible after a burst.
  *   - Bytes are pumped to the host by Dbg_Pump(), which must be called
  *     periodically from the main loop. It is safe to call when USB is
  *     unplugged - it simply parks bytes in the ring buffer.
  *   - All log macros are safe from interrupt context.
  *
  * Disabling completely:
  *   #define DBG_ENABLED 0
  * (place this in your build options or before including dbg_log.h)
  ******************************************************************************
  */

#ifndef DBG_LOG_H
#define DBG_LOG_H

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * Levels
 * --------------------------------------------------------------------------*/
#define DBG_LEVEL_OFF     0U
#define DBG_LEVEL_ERR     1U
#define DBG_LEVEL_WARN    2U
#define DBG_LEVEL_INFO    3U
#define DBG_LEVEL_DEBUG   4U

/* Master compile-time switch. Set to 0 to remove all logging code. */
#ifndef DBG_ENABLED
#define DBG_ENABLED       1
#endif

/* Compile-time max verbosity. Calls above this are removed by the
 * preprocessor (zero code, zero stack). */
#ifndef DBG_MAX_LEVEL
#define DBG_MAX_LEVEL     DBG_LEVEL_DEBUG
#endif

/* Default runtime verbosity at boot. Patch from debugger as needed. */
#ifndef DBG_DEFAULT_RUNTIME_LEVEL
#define DBG_DEFAULT_RUNTIME_LEVEL  DBG_LEVEL_INFO
#endif

/* ----------------------------------------------------------------------------
 * Public state (debugger-visible)
 * --------------------------------------------------------------------------*/
extern volatile uint8_t  g_dbg_runtime_level;   /* live filter, watch window */
extern volatile uint8_t  g_dbg_usb_ready;       /* 1 once Dbg_Init() succeeded */
extern volatile uint32_t g_dbg_emitted_count;   /* messages accepted */
extern volatile uint32_t g_dbg_dropped_count;   /* bytes dropped on full ring */
extern volatile uint32_t g_dbg_tx_busy_count;   /* USB still busy at pump time */

/* ----------------------------------------------------------------------------
 * Lifetime / output API
 * --------------------------------------------------------------------------*/
void Dbg_Init(void);
void Dbg_Pump(void);
void Dbg_SetRuntimeLevel(uint8_t level);
void Dbg_LogV(uint8_t level, const char *tag, const char *fmt, va_list ap);
void Dbg_Log(uint8_t level, const char *tag, const char *fmt, ...);

/* Hex dump helper - dumps `len` bytes from `data` as "AA BB CC ..". */
void Dbg_LogHex(uint8_t level, const char *tag, const char *prefix,
                const void *data, size_t len);

/* ----------------------------------------------------------------------------
 * Convenience macros - tag is a short string like "NFC", "SD", "SYS".
 * Pass NULL as the tag if a message should not have one.
 * --------------------------------------------------------------------------*/
#if (DBG_ENABLED) && (DBG_MAX_LEVEL >= DBG_LEVEL_ERR)
  #define DBG_E(tag, ...)   Dbg_Log(DBG_LEVEL_ERR,   (tag), __VA_ARGS__)
  #define DBG_HEX_E(tag, p, d, n) Dbg_LogHex(DBG_LEVEL_ERR,   (tag), (p), (d), (n))
#else
  #define DBG_E(tag, ...)   ((void)0)
  #define DBG_HEX_E(tag, p, d, n) ((void)0)
#endif

#if (DBG_ENABLED) && (DBG_MAX_LEVEL >= DBG_LEVEL_WARN)
  #define DBG_W(tag, ...)   Dbg_Log(DBG_LEVEL_WARN,  (tag), __VA_ARGS__)
  #define DBG_HEX_W(tag, p, d, n) Dbg_LogHex(DBG_LEVEL_WARN,  (tag), (p), (d), (n))
#else
  #define DBG_W(tag, ...)   ((void)0)
  #define DBG_HEX_W(tag, p, d, n) ((void)0)
#endif

#if (DBG_ENABLED) && (DBG_MAX_LEVEL >= DBG_LEVEL_INFO)
  #define DBG_I(tag, ...)   Dbg_Log(DBG_LEVEL_INFO,  (tag), __VA_ARGS__)
  #define DBG_HEX_I(tag, p, d, n) Dbg_LogHex(DBG_LEVEL_INFO,  (tag), (p), (d), (n))
#else
  #define DBG_I(tag, ...)   ((void)0)
  #define DBG_HEX_I(tag, p, d, n) ((void)0)
#endif

#if (DBG_ENABLED) && (DBG_MAX_LEVEL >= DBG_LEVEL_DEBUG)
  #define DBG_D(tag, ...)   Dbg_Log(DBG_LEVEL_DEBUG, (tag), __VA_ARGS__)
  #define DBG_HEX_D(tag, p, d, n) Dbg_LogHex(DBG_LEVEL_DEBUG, (tag), (p), (d), (n))
#else
  #define DBG_D(tag, ...)   ((void)0)
  #define DBG_HEX_D(tag, p, d, n) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* DBG_LOG_H */
