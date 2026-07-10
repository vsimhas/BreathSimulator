#ifndef BLOWER_IPC_H
#define BLOWER_IPC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dual-core shared mailbox in D3 SRAM4 (both cores must map the same address). */
#define BLOWER_IPC_SRAM4_BASE       0x38000000U
#define BLOWER_IPC_MAGIC            0x424C5752U /* 'BLWR' */

#define BLOWER_IPC_HSEM_CMD_ID      1U
#define BLOWER_IPC_HSEM_STATUS_ID   2U

/*
 * Hard ceiling exposed to the CM7 caller. The MCWB-generated firmware on
 * CM4 saturates the speed reference at MAX_APPLICATION_SPEED_RPM (currently
 * 36528 in drive_parameters.h). We clamp at that same value on the CM7 side
 * so requests beyond the motor's reachable speed do not behave like faults.
 */
#define BLOWER_IPC_MAX_SPEED_RPM    36528

typedef enum
{
  BLOWER_CMD_NONE = 0,
  BLOWER_CMD_START,
  BLOWER_CMD_STOP,
  BLOWER_CMD_SET_SPEED,
  BLOWER_CMD_ACK_FAULT,
} BlowerIpcCommand_t;

typedef enum
{
  FOC_GAIN_SPEED_KP = 0,
  FOC_GAIN_SPEED_KI,
  FOC_GAIN_TORQUE_KP,
  FOC_GAIN_TORQUE_KI,
  FOC_GAIN_FLUX_KP,
  FOC_GAIN_FLUX_KI,
} BlowerFocGainId_t;

typedef struct __attribute__((aligned(32)))
{
  uint32_t magic;

  volatile uint32_t cm7_cmd_seq;
  volatile uint32_t cm4_cmd_ack_seq;
  volatile uint32_t cm4_status_seq;

  volatile uint32_t command;
  /* RPM is carried as int32 - int16 (max 32767) was overflowing the
   * blower's operating range and tripping MC_SPEED_FDBK on CM4. */
  volatile int32_t  target_speed_rpm;
  volatile uint16_t ramp_duration_ms;
  volatile uint16_t reserved_align;

  volatile uint16_t mc_state;
  volatile uint16_t current_faults;
  volatile uint16_t occurred_faults;
  volatile int32_t  mech_speed_rpm;
  volatile int32_t  speed_ref_rpm;
  volatile uint16_t bus_voltage_v;
  volatile uint16_t brake_pwm_permille;
  volatile uint8_t  brake_active;
  volatile uint8_t  motor_running;
  volatile uint16_t reserved;

  /* CM4 FOC inner-loop PI gains (runtime tunable via CM7 / USB). */
  volatile uint32_t cm7_foc_tuning_seq;
  volatile uint32_t cm4_foc_tuning_ack_seq;
  volatile int16_t  foc_speed_kp;
  volatile int16_t  foc_speed_ki;
  volatile int16_t  foc_torque_kp;
  volatile int16_t  foc_torque_ki;
  volatile int16_t  foc_flux_kp;
  volatile int16_t  foc_flux_ki;
  volatile uint16_t foc_reserved;
} BlowerIpcShared_t;

typedef struct
{
  uint16_t mc_state;
  uint16_t current_faults;
  uint16_t occurred_faults;
  int32_t  mech_speed_rpm;
  int32_t  speed_ref_rpm;
  uint16_t bus_voltage_v;
  uint16_t brake_pwm_permille;
  bool     brake_active;
  bool     motor_running;
  uint32_t status_seq;
  uint32_t cmd_ack_seq;
} BlowerIpcStatus_t;

extern BlowerIpcShared_t g_blower_ipc;

void BlowerIpc_EnableSharedRamClock(void);
void BlowerIpc_InitSharedMemory(void);

#if defined(CORE_CM4)
void BlowerIpc_CM4_Init(void);
void BlowerIpc_CM4_ProcessCommands(void);
void BlowerIpc_CM4_UpdateStatus(void);
void BlowerBrake_Init(void);
void BlowerBrake_Update(void);
#endif

#if defined(CORE_CM7)
void BlowerIpc_CM7_Init(void);
bool BlowerIpc_CM7_Start(void);
bool BlowerIpc_CM7_Stop(void);
bool BlowerIpc_CM7_SetSpeedRpm(int32_t rpm, uint16_t ramp_duration_ms);
/* Same as SetSpeedRpm but with no DBG_I/DBG_W output. Use this from a
 * high-rate control loop (e.g. the pressure PID at 200 Hz) where the
 * logging volume of the verbose variant would flood the USB-CDC link. */
bool BlowerIpc_CM7_SetSpeedRpmSilent(int32_t rpm);
bool BlowerIpc_CM7_AcknowledgeFault(void);
bool BlowerIpc_CM7_GetStatus(BlowerIpcStatus_t *status);
bool BlowerIpc_CM7_SetFocGain(BlowerFocGainId_t id, int16_t value);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BLOWER_IPC_H */
