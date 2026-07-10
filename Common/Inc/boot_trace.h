#ifndef BOOT_TRACE_H
#define BOOT_TRACE_H

#include <stdint.h>
#include "stm32h7xx.h"

/* RTC backup registers — same address space from CM7/CM4 debugger watch. */
#define BOOT_TRACE_BKP_STAGE   RTC->BKP18R
#define BOOT_TRACE_BKP_SYNC    RTC->BKP19R

#define BOOT_TRACE_CM7_BASE    0xC7000000U
#define BOOT_TRACE_CM4_BASE    0xC4000000U

/* Survives __main .bss zero-init (runs after SystemInit, before main). */
#define BOOT_TRACE_NOINIT  __attribute__((section(".noinit")))

/* Fixed RAM mirrors (linker .noinit) — use in Memory window if symbol watch stays 0. */
#define BOOT_TRACE_CM7_RAM_STAGE   (*(volatile uint32_t *)0x2403F804U)
#define BOOT_TRACE_CM7_RAM_BOOT    (*(volatile uint32_t *)0x2403F808U)
#define BOOT_TRACE_CM4_RAM_STAGE   (*(volatile uint32_t *)0x10047804U)
#define BOOT_TRACE_CM4_RAM_BOOT    (*(volatile uint32_t *)0x10047808U)

/* Written from Reset_Handler before any C init (0xFF = assembly entry marker). */
#define BOOT_TRACE_ASM_ENTRY       0x000000FFU

/* Direct RTC read in debugger when RAM mirrors are stale. */
#define BOOT_TRACE_RTC_STAGE   (*(volatile uint32_t *)0x58004098U)
#define BOOT_TRACE_RTC_SYNC    (*(volatile uint32_t *)0x5800409CU)

void BootTraceWrite(uint32_t marker);

extern volatile uint32_t boot_stage;
extern volatile uint32_t boot_trace_stage;
extern volatile uint32_t boot_trace_sync;
extern volatile uint32_t boot_sync_step;

#endif /* BOOT_TRACE_H */
