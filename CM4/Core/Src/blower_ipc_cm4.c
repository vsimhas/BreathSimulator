/**
  * @file    blower_ipc_cm4.c
  * @brief   CM4-side blower IPC command handling and regen brake control.
  */

#include "blower_ipc.h"
#include "main.h"
#include "mc_api.h"
#include "mc_config_common.h"
#include "mc_stm_types.h"
#include "mc_interface.h"
#include "bus_voltage_sensor.h"
#include "mc_config.h"
#include "pid_regulator.h"

extern TIM_HandleTypeDef htim3;

#define BRAKE_PWM_FREQ_HZ           10000U
#define BRAKE_TIM_CLK_HZ            200000000UL
/*
 * Brake thresholds in MILLIVOLTS, and moved up into the headroom opened by
 * OV_VOLTAGE_THRESHOLD_V 24.9 -> 28.0.
 *
 * They were 24 V / 22 V in whole volts against a 23.86 V supply - only
 * 0.14 V of margin, so the brake armed on essentially any regen and the
 * resistor saw far more duty than intended. (The same threshold-sitting-on-
 * the-supply mistake heated R154 on the CPAP board, which is why that
 * project disabled its brake outright.)
 *
 * The brake is KEPT here rather than disabled. A CPAP holds a steady
 * pressure; this rig decelerates the blower once per breath, so a
 * dissipative path is worth more here, not less. It just has to sit above
 * the supply instead of on it.
 *
 * Ladder: supply 23.86 -> brake 25.5 (off 24.5) -> CM7 derate 26.0..27.2
 * -> OV trip 28.0 -> caps 35. Brake first because dissipating is cheaper
 * than distorting the commanded waveform.
 */
/*
 * The brake ACTION can be switched off; publishing the bus voltage cannot.
 * They used to be the same function, and when the brake was disabled by
 * commenting out the BlowerBrake_Update() call in main.c the bus voltage
 * stopped being published with it - silently, because nothing reads 0 V as
 * an error. That left BOTH regen defences inert: the brake, deliberately,
 * and the CM7 slew derate, accidentally, because its input was frozen at 0.
 * The result was an OVER_VOLT trip on the expiratory deceleration
 * (2026-09-28, rpm_cmd 23741 mid-fall from ~35500).
 */
#ifndef BLOWER_BRAKE_ENABLE
#define BLOWER_BRAKE_ENABLE          1
#endif

#define BRAKE_VBUS_ON_MV         25500U
#define BRAKE_VBUS_OFF_MV        24500U
/** Permille of brake PWM per mV above BRAKE_VBUS_ON_MV. */
#define BRAKE_PERMILLE_PER_MV        0.25f
#define BRAKE_DECEL_RPM_PER_MS      50
#define BRAKE_MAX_PERMILLE            900U

static uint32_t s_last_cmd_seq;
static uint32_t s_last_foc_tuning_seq;
static int32_t s_prev_speed_rpm;
static bool s_brake_latched;

static int16_t BlowerIpc_RpmToSpeedUnit(int32_t rpm)
{
  /* SPEED_UNIT/U_RPM is typically 10/60 = 1/6, so even at 36528 RPM
   * (workbench app limit) the result is ~6088, which fits comfortably in
   * an int16. We still do the math in int32 to avoid intermediate overflow. */
  return (int16_t)((rpm * (int32_t)SPEED_UNIT) / (int32_t)U_RPM);
}

static int32_t BlowerIpc_SpeedUnitToRpm(int16_t speed_unit)
{
  return ((int32_t)speed_unit * (int32_t)U_RPM) / (int32_t)SPEED_UNIT;
}

/* ---------------------------------------------------------------------------
 * Restart-while-coasting guard
 *
 * Ported from CPAP_InitialCode 966762c, which runs the same blower and drive.
 * Restarting while the rotor is still coasting produced a "false RUN": the
 * estimate tracked the commanded speed, iq sat near zero, no pressure was
 * produced and no fault was raised - and after an over-voltage trip it gave
 * repeated DP_FAULTs. The breath simulator stops and starts far more often
 * than a CPAP does, so it is more exposed to this than the rig it came from.
 *
 * Sizing (measured on the CPAP rig, from the pressure decay after a stop):
 * the rotor is below ~1.5 krpm 3.3-3.7 s after a stop from 18 k, 4.2-5.2 s
 * from 33 k. The hold-off leaves wide margin for the bearing-friction tail:
 * 6.7 s from 18 k, 9.0 s from 33 k. Restarts that waited 10-12 s all started
 * cleanly (18 starts, 2026-09-25).
 *
 * A start arriving inside the hold-off is not dropped: it is kept pending and
 * issued from the main loop once the hold-off expires and the state machine
 * is in IDLE. STOP cancels it. The remainder is published as
 * restart_holdoff_ms so CM7 can see why a start has not taken effect.
 * ------------------------------------------------------------------------ */
#define RESTART_HOLDOFF_BASE_MS      4000U
#define RESTART_HOLDOFF_MS_PER_KRPM   150U
/* Below this the rotor was never really spun up (alignment, a failed start at
 * crawl speed): nothing to wait for. */
#define RESTART_HOLDOFF_MIN_RPM      1000U

static bool     s_drive_on;          /* PWM was driving the rotor           */
static uint32_t s_drive_rpm;         /* |mech| while it was                 */
static uint32_t s_coast_start_ms;    /* when drive stopped                  */
static uint32_t s_coast_holdoff_ms;  /* hold-off length; 0 = none           */
static bool     s_start_pending;     /* start requested during the hold-off */
/* STOP accepted but the state machine has not left the driven states yet (it
 * acts on the next medium-frequency tick). A START in that window must
 * survive the stop, not be mistaken for "already running". */
static bool     s_stop_in_flight;

static bool BlowerIpc_StateDrives(MCI_State_t st)
{
  return (st == RUN) || (st == START) || (st == SWITCH_OVER) ||
         (st == ALIGNMENT);
}

/* Main-loop context only: it is the sole writer of the coast state. */
static void BlowerIpc_CM4_TrackCoast(void)
{
  const MCI_State_t st = MC_GetSTMStateMotor1();
  if (BlowerIpc_StateDrives(st))
  {
    int32_t mech = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
    if (mech < 0)
    {
      mech = -mech;
    }
    s_drive_rpm = (uint32_t)mech;
    s_drive_on = true;
    s_coast_holdoff_ms = 0U;
  }
  else if (s_drive_on)
  {
    /* Drive just stopped. The estimate froze at the last driven speed, which
     * is what the rotor is coasting down from. */
    s_drive_on = false;
    s_stop_in_flight = false;
    s_coast_start_ms = HAL_GetTick();
    s_coast_holdoff_ms = (s_drive_rpm < RESTART_HOLDOFF_MIN_RPM)
        ? 0U
        : RESTART_HOLDOFF_BASE_MS
          + ((s_drive_rpm * RESTART_HOLDOFF_MS_PER_KRPM) / 1000U);
  }
}

/* Pure read, safe from the medium-frequency hook as well. */
static uint32_t BlowerIpc_CM4_HoldoffRemainingMs(void)
{
  const uint32_t holdoff = s_coast_holdoff_ms;
  if (holdoff == 0U)
  {
    return 0U;
  }
  const uint32_t elapsed = HAL_GetTick() - s_coast_start_ms;
  return (elapsed >= holdoff) ? 0U : (holdoff - elapsed);
}

/* Replaces the direct MC_StartMotor1() call in the START handler. */
static void BlowerIpc_CM4_RequestStart(void)
{
  HAL_GPIO_WritePin(STSPIN_STBY_GPIO_Port, STSPIN_STBY_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOT_POW_EN_GPIO_Port, MOT_POW_EN_Pin, GPIO_PIN_SET);
  s_start_pending = true;
}

/* Issues a pending start once the rotor has had time to stop. Called every
 * main-loop pass. MCI_StartMotor only accepts IDLE, so acknowledge a latched
 * fault first and start on a later pass - the ack takes effect on the next
 * state-machine tick, and a start queued behind it would be refused. */
static void BlowerIpc_CM4_ServicePendingStart(void)
{
  if (!s_start_pending)
  {
    return;
  }
  const MCI_State_t st = MC_GetSTMStateMotor1();
  if (BlowerIpc_StateDrives(st))
  {
    if (!s_stop_in_flight)
    {
      s_start_pending = false;   /* already running */
    }
    /* else: a STOP is about to take effect; keep the start for after it. */
    return;
  }
  if (BlowerIpc_CM4_HoldoffRemainingMs() > 0U)
  {
    return;
  }
  if (st == FAULT_OVER)
  {
    (void)MC_AcknowledgeFaultMotor1();
    return;
  }
  if (st == IDLE)
  {
    s_start_pending = false;
    (void)MC_StartMotor1();
  }
  /* STOP / WAIT_STOP_MOTOR / FAULT_NOW: settling, try again next pass. */
}

static void BlowerBrake_ApplyPermille(uint16_t permille)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
  uint32_t pulse;

  if (permille > 1000U)
  {
    permille = 1000U;
  }

  pulse = ((uint32_t)permille * (period + 1U)) / 1000U;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
  g_blower_ipc.brake_pwm_permille = permille;
  g_blower_ipc.brake_active = (permille > 0U) ? 1U : 0U;
}

void BlowerBrake_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  uint32_t prescaler;
  uint32_t period;

  prescaler = (BRAKE_TIM_CLK_HZ / (BRAKE_PWM_FREQ_HZ * 1000UL)) - 1U;
  if (prescaler == 0xFFFFFFFFUL)
  {
    prescaler = 0U;
  }
  period = 999U;

  htim3.Init.Prescaler = (uint16_t)prescaler;
  htim3.Init.Period = period;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  BlowerBrake_ApplyPermille(0U);
  s_prev_speed_rpm = 0;
  s_brake_latched = false;
}

/*
 * Reads the DC bus, publishes it (always), and drives the regen brake (when
 * BLOWER_BRAKE_ENABLE). Must be called every main-loop pass: everything that
 * protects the bus during deceleration depends on the value it publishes -
 * the brake here, and the slew derate on CM7.
 */
void BlowerBrake_Update(void)
{
  uint16_t vbus_v = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
  int32_t curr_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  int32_t decel_rpm = (int32_t)s_prev_speed_rpm - curr_rpm;
  uint16_t brake_permille = 0U;

  /* Millivolts, used for the brake decision here and published for the CM7
   * slew derate. VBS_GetAvBusVoltage_V is (AvBusVoltage_d *
   * ConversionFactor) / 65536 truncated to whole volts; the same product
   * scaled by 1000 gives mV. 64-bit because d * CF alone already fills a
   * uint32. Whole volts is far too coarse to ladder on: the whole
   * supply-to-trip range is only a few counts. */
  const uint32_t vbus_mv =
      (uint32_t)(((uint64_t)BusVoltageSensor_M1._Super.AvBusVoltage_d *
                  (uint64_t)BusVoltageSensor_M1._Super.ConversionFactor *
                  1000ULL) / 65536ULL);

  g_blower_ipc.bus_voltage_v = vbus_v;
  g_blower_ipc.bus_voltage_mv_low  = (uint16_t)(vbus_mv & 0xFFFFU);
  g_blower_ipc.bus_voltage_mv_high = (uint16_t)(vbus_mv >> 16);

#if BLOWER_BRAKE_ENABLE
  if ((vbus_mv >= BRAKE_VBUS_ON_MV) &&
      ((decel_rpm >= BRAKE_DECEL_RPM_PER_MS) || s_brake_latched))
  {
    const uint32_t over_mv = vbus_mv - BRAKE_VBUS_ON_MV;
    uint32_t permille = (uint32_t)((float)over_mv * BRAKE_PERMILLE_PER_MV);

    s_brake_latched = true;
    if (permille > BRAKE_MAX_PERMILLE)
    {
      permille = BRAKE_MAX_PERMILLE;
    }
    brake_permille = (uint16_t)permille;
  }

  if (vbus_mv <= BRAKE_VBUS_OFF_MV)
  {
    s_brake_latched = false;
    brake_permille = 0U;
  }
#else
  (void)decel_rpm;   /* brake action compiled out; voltage still published */
#endif

  BlowerBrake_ApplyPermille(brake_permille);
  s_prev_speed_rpm = curr_rpm;
}

static void BlowerIpc_CM4_SeedFocTuning(void)
{
  g_blower_ipc.foc_speed_kp = PIDSpeedHandle_M1.hKpGain;
  g_blower_ipc.foc_speed_ki = PIDSpeedHandle_M1.hKiGain;
  g_blower_ipc.foc_torque_kp = PIDIqHandle_M1.hKpGain;
  g_blower_ipc.foc_torque_ki = PIDIqHandle_M1.hKiGain;
  g_blower_ipc.foc_flux_kp = PIDIdHandle_M1.hKpGain;
  g_blower_ipc.foc_flux_ki = PIDIdHandle_M1.hKiGain;
  __DMB();
  s_last_foc_tuning_seq = g_blower_ipc.cm7_foc_tuning_seq;
  g_blower_ipc.cm4_foc_tuning_ack_seq = s_last_foc_tuning_seq;
  __DMB();
}

static int16_t BlowerIpc_PickFocGain(int16_t mailbox, int16_t current)
{
  return (mailbox > 0) ? mailbox : current;
}

static void BlowerIpc_CM4_ApplyFocTuning(void)
{
  uint32_t seq = g_blower_ipc.cm7_foc_tuning_seq;
  if (seq == s_last_foc_tuning_seq)
  {
    return;
  }

  __DMB();
  PID_SetKP(&PIDSpeedHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_speed_kp, PIDSpeedHandle_M1.hKpGain));
  PID_SetKI(&PIDSpeedHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_speed_ki, PIDSpeedHandle_M1.hKiGain));
  PID_SetKP(&PIDIqHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_torque_kp, PIDIqHandle_M1.hKpGain));
  PID_SetKI(&PIDIqHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_torque_ki, PIDIqHandle_M1.hKiGain));
  PID_SetKP(&PIDIdHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_flux_kp, PIDIdHandle_M1.hKpGain));
  PID_SetKI(&PIDIdHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_flux_ki, PIDIdHandle_M1.hKiGain));
  __DMB();

  s_last_foc_tuning_seq = seq;
  g_blower_ipc.cm4_foc_tuning_ack_seq = seq;
  __DMB();
}

void BlowerIpc_CM4_Init(void)
{
  BlowerIpc_EnableSharedRamClock();
  __HAL_RCC_HSEM_CLK_ENABLE();

  BlowerIpc_InitSharedMemory();
  s_last_cmd_seq = g_blower_ipc.cm7_cmd_seq;
  BlowerIpc_CM4_SeedFocTuning();
  BlowerBrake_Init();
}

static void BlowerIpc_CM4_HandleCommand(BlowerIpcCommand_t cmd, int32_t rpm, uint16_t ramp_ms)
{
  switch (cmd)
  {
    case BLOWER_CMD_START:
      /* Clear latched FOC faults before arming - CM7 may post ACK then START
       * back-to-back and only the last IPC write lands. The start itself now
       * waits out any coast-down hold-off. */
      if ((MC_GetSTMStateMotor1() == (MCI_State_t)FAULT_OVER) &&
          (MC_GetCurrentFaultsMotor1() == MC_NO_FAULTS))
      {
        (void)MC_AcknowledgeFaultMotor1();
      }
      if (rpm != 0)
      {
        MC_ProgramSpeedRampMotor1(BlowerIpc_RpmToSpeedUnit(rpm), ramp_ms);
      }
      BlowerIpc_CM4_RequestStart();
      BlowerIpc_CM4_ServicePendingStart();
      break;

    case BLOWER_CMD_STOP:
    {
      /* In FAULT_OVER acknowledge ONLY; otherwise stop, and look at the
       * answer. Ported from CPAP_InitialCode d4096b0 + 9f7d796.
       *
       * MCI_StopMotor refuses while any fault is latched (PastFaults or
       * CurrentFaults) and in IDLE; MCI_FaultAcknowledged only acts in
       * FAULT_OVER. This used to call stop and discard the result, so a STOP
       * arriving in FAULT_OVER was simply refused.
       *
       * The ack must not be followed by a stop: the ack queues
       * MCI_ACK_FAULTS, and a MC_StopMotor1() after it then sees no faults
       * and a non-IDLE state, is accepted, and OVERWRITES the queued command
       * with MCI_STOP - which FAULT_OVER ignores. Measured on the CPAP rig
       * 2026-09-24 after a DP_FAULT at 40 krpm: the drive stayed latched in
       * FAULT_OVER through every STOP. PWM is already off there, so the ack
       * alone is the whole job. The ack is refused while the fault condition
       * is still present, so this cannot clear an active fault.
       *
       * Note for debugging: the angle/speed estimator is disabled in IDLE,
       * FAULT_NOW and FAULT_OVER, so after a fault the reported speed FREEZES
       * at the last driven value while PWM is off and the rotor coasts. Read
       * mc_state and occurred_faults, not mech_speed_rpm, to know whether the
       * power stage is actually on. */
      s_start_pending = false;    /* a start held off for coast-down is void */
      if (MC_GetSTMStateMotor1() == (MCI_State_t)FAULT_OVER)
      {
        (void)MC_AcknowledgeFaultMotor1();
      }
      else if (MC_StopMotor1())
      {
        s_stop_in_flight = BlowerIpc_StateDrives(MC_GetSTMStateMotor1());
      }
      else
      {
        const MCI_State_t st = MC_GetSTMStateMotor1();
        /* Refused. In IDLE, STOP, WAIT_STOP_MOTOR or FAULT_* the power stage
         * is already off or on its way off: nothing to force.
         *
         * Refused while still POWERING the motor should be impossible - a
         * latched fault forces FAULT_OVER and switches PWM off - so this is
         * defence in depth. If it ever happens, do not leave the rotor driven
         * with no report: raise the SDK's own software-error stop (the same
         * call TSK_HardwareFaultTask makes). The safety task then switches
         * PWM off and the next START acknowledges it. */
        if ((st == RUN) || (st == START) || (st == SWITCH_OVER) ||
            (st == ALIGNMENT) || (st == CHARGE_BOOT_CAP) ||
            (st == OFFSET_CALIB))
        {
          MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0U);
        }
      }
      BlowerBrake_ApplyPermille(0U);
      s_brake_latched = false;
      break;
    }

    case BLOWER_CMD_SET_SPEED:
      MC_ProgramSpeedRampMotor1(BlowerIpc_RpmToSpeedUnit(rpm), ramp_ms);
      break;

    case BLOWER_CMD_ACK_FAULT:
      (void)MC_AcknowledgeFaultMotor1();
      break;

    default:
      break;
  }
}

void BlowerIpc_CM4_ProcessCommands(void)
{
  BlowerIpc_CM4_ApplyFocTuning();

  /* Every pass, not only when a new command arrives: a held-off start must
   * still be issued once the rotor has stopped, even if CM7 has gone quiet. */
  BlowerIpc_CM4_TrackCoast();
  BlowerIpc_CM4_ServicePendingStart();

  uint32_t seq;
  BlowerIpcCommand_t cmd;
  int32_t rpm;
  uint16_t ramp_ms;

  seq = g_blower_ipc.cm7_cmd_seq;
  if (seq == s_last_cmd_seq)
  {
    return;
  }

  __DMB();
  cmd = (BlowerIpcCommand_t)g_blower_ipc.command;
  rpm = g_blower_ipc.target_speed_rpm;
  ramp_ms = g_blower_ipc.ramp_duration_ms;
  __DMB();

  BlowerIpc_CM4_HandleCommand(cmd, rpm, ramp_ms);
  s_last_cmd_seq = seq;
  g_blower_ipc.cm4_cmd_ack_seq = seq;
  __DMB();
}

void BlowerIpc_CM4_UpdateStatus(void)
{
  MCI_State_t state = MC_GetSTMStateMotor1();

  g_blower_ipc.mc_state = (uint16_t)state;
  g_blower_ipc.current_faults = MC_GetCurrentFaultsMotor1();
  g_blower_ipc.occurred_faults = MC_GetOccurredFaultsMotor1();
  g_blower_ipc.mech_speed_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  g_blower_ipc.speed_ref_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedReferenceMotor1());
  g_blower_ipc.motor_running = ((state == RUN) || (state == START) || (state == SWITCH_OVER) ||
                                (state == STOP) || (state == WAIT_STOP_MOTOR)) ? 1U : 0U;
  {
    uint32_t hold = BlowerIpc_CM4_HoldoffRemainingMs();
    if (s_start_pending && (hold == 0U))
    {
      hold = 1U;   /* pending, but the state machine is not in IDLE yet */
    }
    g_blower_ipc.restart_holdoff_ms = (uint16_t)((hold > 65535U) ? 65535U : hold);
  }
  __DMB();
  g_blower_ipc.cm4_status_seq++;
  __DMB();
  (void)HAL_HSEM_Release(BLOWER_IPC_HSEM_STATUS_ID, 0U);
}
