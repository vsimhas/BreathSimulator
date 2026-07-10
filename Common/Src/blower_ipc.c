/**
  * @file    blower_ipc.c
  * @brief   Dual-core blower mailbox in D3 SRAM4.
  */

#include "blower_ipc.h"
#include "stm32h7xx_hal.h"

#define BLOWER_IPC_SECTION __attribute__((section(".blower_ipc")))

BLOWER_IPC_SECTION BlowerIpcShared_t g_blower_ipc;

void BlowerIpc_EnableSharedRamClock(void)
{
#if defined(RCC_AHB4ENR_SRDSRAMEN)
  __HAL_RCC_SRDSRAM_CLK_ENABLE();
#elif defined(RCC_AHB4ENR_SRAM4EN)
  __HAL_RCC_SRAM4_CLK_ENABLE();
#else
  /* STM32H747: SRAM4 at 0x38000000 has no dedicated AHB4 enable bit. */
#endif
}

void BlowerIpc_InitSharedMemory(void)
{
  if (g_blower_ipc.magic != BLOWER_IPC_MAGIC)
  {
    g_blower_ipc.magic = BLOWER_IPC_MAGIC;
    g_blower_ipc.cm7_cmd_seq = 0U;
    g_blower_ipc.cm4_cmd_ack_seq = 0U;
    g_blower_ipc.cm4_status_seq = 0U;
    g_blower_ipc.command = BLOWER_CMD_NONE;
    g_blower_ipc.target_speed_rpm = 0;
    g_blower_ipc.ramp_duration_ms = 0U;
    g_blower_ipc.reserved_align = 0U;
    g_blower_ipc.mc_state = 0U;
    g_blower_ipc.current_faults = 0U;
    g_blower_ipc.occurred_faults = 0U;
    g_blower_ipc.mech_speed_rpm = 0;
    g_blower_ipc.speed_ref_rpm = 0;
    g_blower_ipc.bus_voltage_v = 0U;
    g_blower_ipc.brake_pwm_permille = 0U;
    g_blower_ipc.brake_active = 0U;
    g_blower_ipc.motor_running = 0U;
    g_blower_ipc.reserved = 0U;
    g_blower_ipc.cm7_foc_tuning_seq = 0U;
    g_blower_ipc.cm4_foc_tuning_ack_seq = 0U;
    g_blower_ipc.foc_speed_kp = 0;
    g_blower_ipc.foc_speed_ki = 0;
    g_blower_ipc.foc_torque_kp = 0;
    g_blower_ipc.foc_torque_ki = 0;
    g_blower_ipc.foc_flux_kp = 0;
    g_blower_ipc.foc_flux_ki = 0;
    g_blower_ipc.foc_reserved = 0U;
  }
}
