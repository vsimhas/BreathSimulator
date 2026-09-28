
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/* Over-modulation. 0 = classic SVPWM (as shipped), 1 = PWMC_SetPhaseVoltage_OVM.
 * Ported from CPAP_InitialCode 71391ea, same blower, motor and power stage.
 *
 * WHAT IT BUYS. The fundamental a three-phase inverter can produce is
 * 0.707*Vbus under linear SVPWM and up to 0.780*Vbus in six-step, so the
 * voltage ceiling moves by about 10%. The drive here is back-EMF limited,
 * not torque limited: measured on the CPAP rig, the rotor plateaued at
 * 37700 rpm on a 23.9 V bus with Iq at only 58-65% of its limit. It has
 * torque in hand and simply runs out of volts. Bench result there was
 * 41000 rpm no-load, where the STSPIN830 raised Driver Protection.
 *
 * WHAT IT BUYS *HERE*, at MAX_APPLICATION_SPEED_RPM 38500. The point is not
 * a higher number - the ceiling is unchanged - it is that 38500 stops being
 * marginal. A breath simulator pushes against whatever pressure the CPAP
 * under test is holding, so the blower needs voltage headroom to SUSTAIN the
 * commanded speed under load, not just brush it unloaded. Raise the ceiling
 * only after a bench sweep shows the observer stays healthy (see below).
 *
 * HAZARD 1 - THE OBSERVER. This drive is sensorless (STO+PLL), and the state
 * observer reconstructs applied phase voltage as Vbus * Valfa_beta, which
 * assumes LINEAR modulation. In over-modulation the real fundamental is up to
 * 10% above what that product implies, so the back-EMF estimate - and with it
 * speed and rotor angle - degrades exactly in the region being entered. Watch
 * for MC_SPEED_FDBK: VARIANCE_THRESHOLD is 0.25 with
 * M1_SS_MEAS_ERRORS_BEFORE_FAULTS = 3.
 *
 * HAZARD 2 - CURRENT SAMPLING. Three-shunt reconstruction needs a minimum
 * duty window, TW_AFTER = HW_DEAD_TIME_NS + TRISE_NS = 550 + 1000 = 1550 ns,
 * which is 3.1% of the 50 us period at PWM_FREQUENCY 20000. Over-modulation
 * works BY pushing two of the three duties toward the rails, so it eats that
 * window as it makes the extra voltage. Partial over-modulation should
 * survive; deep over-modulation approaching six-step probably will not on
 * 3 shunts. The symptom is erratic Iq rather than a clean fault.
 *
 * MAX_MODULE (parameters_conversion.h, a percentage, currently 100) caps how
 * far into over-modulation the drive goes. 95% keeps it below
 * OVM_VREF_MODE2_START, i.e. mode 1 only, which is the conservative setting
 * if either hazard shows up.
 *
 * Also expect more torque and current ripple, and more regenerative energy on
 * deceleration - which this rig produces on every single breath. The
 * protection ladder was widened on 2026-09-28 for exactly that reason
 * (OV 28.0 V, brake 25.5 V, CM7 derate 26.0-27.2 V).
 *
 * HOW TO TEST. Open loop or ST Motor Pilot, not mid-run. Ramp slowly past
 * 37700 and watch speed feedback and Iq. If the observer holds, the gain is
 * real; if it faults or Iq goes erratic, set this to 0 and rebuild - nothing
 * else has to change, the SVPWM path stays compiled in. */
/* 2026-09-28: 0, and it should stay 0. Measured on this rig, not assumed.
 *
 * Over-modulation is supposed to buy speed headroom by raising the achievable
 * fundamental from 0.707 to ~0.780 of Vbus. On this drive it cost more than
 * it bought, because it degrades the sensorless observer (see the hazard note
 * below) and the drive stopped following its speed command.
 *
 * Paired runs at two CPAP pressures, same everything else:
 *
 *              OVM on            OVM off
 *   CPAP  4    408 mL, 39.6 slm  506 mL, 45.2 slm
 *   CPAP 16    151 mL, 19.3 slm  212 mL, 18.2 slm
 *   tracking   TRACK_ERROR on    clean
 *              every breath
 *
 * TRACK_ERROR was set on every breath of every run with this enabled - even
 * at low pressure where delivered volume looked healthy - and cleared
 * completely with it off. Peak flow at CPAP 4 rose 14% despite a LONGER
 * inspiratory time in the OVM-off runs, which normally lowers peak flow; the
 * confound works against that result rather than for it.
 *
 * The volume figures carry a real confound (inspiratory time differed, 1.0 s
 * vs 1.667 s, because the reflash reset the pause parameters), so treat the
 * +24%/+40% as indicative. The tracking result and the peak-flow result do
 * not depend on it.
 *
 * Re-enabling would need a reason and a fresh paired measurement. */
#ifndef FOC_OVERMODULATION_ENABLE
#define FOC_OVERMODULATION_ENABLE           0
#endif


/*** Speed measurement settings ***/
/* 2026-09-28: 36012 -> 38500.
 *
 * 36012 was the ST profiler's measurement of the back-EMF-limited ceiling at
 * profiling time, recorded as if it were a motor rating. It is a property of
 * the drive, not the motor.
 *
 * 38500 is the figure measured on the CPAP_InitialCode rig, which runs the
 * same blower, motor profile and power stage: 38.5 krpm holds clean under
 * load, current control starts breaking down around 39 k (id standard
 * deviation 226 mA, |id| 630 mA) and a DP_FAULT follows at ~39.5 k. Those
 * numbers transfer directly here because this board's VBUS_PARTITIONING_
 * FACTOR is correct (verified 2026-09-28), so neither board is misreporting
 * speed relative to the other.
 *
 * This raises the allowed speed REFERENCE; it does not force the rotor
 * anywhere. If the drive cannot reach the commanded speed it current-limits,
 * and BreathSim's tracking-error flag reports the shortfall rather than the
 * rig silently delivering a different breath.
 *
 * Going beyond this needs FOC over-modulation (CPAP_InitialCode 71391ea,
 * which took that rig from ~37.7 k to ~41 k no-load). Not enabled here.
 *
 * Note the energy: (38500/36012)^2 = +14% stored rotational energy, so ~14%
 * more regen into the bus on every expiratory deceleration. The protection
 * ladder was widened on the same day to absorb it - see
 * OV_VOLTAGE_THRESHOLD_V below and the brake thresholds in blower_ipc_cm4.c. */
#define MAX_APPLICATION_SPEED_RPM           38500 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD                  0.25 /*!< Maximum accepted variance on speed estimates (percentage) */

/* State observer scaling factors F1 */
#define F1                                  8192
#define F2                                  16384
#define F1_LOG                              LOG2((8192))
#define F2_LOG                              LOG2((16384))

/* State observer constants */
#define GAIN1                               -11146
#define GAIN2                               5275

/* Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                         958
#define PLL_KI_GAIN                         34
#define PLL_KPDIV                           16384
#define PLL_KPDIV_LOG                       LOG2((PLL_KPDIV))
#define PLL_KIDIV                           65535
#define PLL_KIDIV_LOG                       LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP                  64 /*!< Depth of the FIFO used  to average mechanical speed in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG              LOG2((64))
#define STO_FIFO_DEPTH_UNIT                 64 /*!< Depth of the FIFO used to average mechanical speed in the unit defined by #SPEED_UNIT */
#define M1_BEMF_CONSISTENCY_TOL             64 /* Parameter for B-emf amplitude-speed consistency */
#define M1_BEMF_CONSISTENCY_GAIN            64 /* Parameter for B-emf amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       20000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           ES_GPIO

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT               2168
#define PID_TORQUE_KI_DEFAULT               806
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 2168
#define PID_FLUX_KI_DEFAULT                 806
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            8192
#define TF_KIDIV                            16384
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((8192))
#define TF_KIDIV_LOG                        LOG2((16384))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

#define PID_SPEED_KP_DEFAULT                2339/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT                367/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            128
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((128))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             1.445

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            12965
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
/* 2026-09-28: 24.9 -> 28.0.
 *
 * The divider on THIS board is the as-drawn 180K/12K - V_MOT across R138
 * measures 1.491 V against a 23.86 V bus, which is 0.0625 exactly. So unlike
 * the CPAP board, nothing here was mis-reporting: 24.9 V really was a 24.9 V
 * trip, leaving only 1.04 V above the supply.
 *
 * That is too little to build a protection ladder in. The breath simulator
 * decelerates the blower on every single breath, so regen into the bus is
 * continuous rather than occasional, and with 1 V of room the brake and the
 * CM7 slew derate both sat effectively on top of the supply. Raising the trip
 * to 28.0 V gives 4.14 V of working range for brake -> derate -> trip to be
 * separated, and still leaves 7 V to the 35 V rating of the lowest-rated caps
 * on the regen path (confirmed same as the CPAP board, 2026-09-28).
 *
 * OVERVOLTAGE_THRESHOLD_d is derived from this and VBUS_PARTITIONING_FACTOR,
 * so both must be right for the trip to land where it says. */
#define OV_VOLTAGE_THRESHOLD_V              28.0 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V              7.0 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          110 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

/* Phase 1 */
#define PHASE1_DURATION                     320 /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT_A              1.345

/* Phase 2 */
#define PHASE2_DURATION                     4322 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT_A              1.345

/* Phase 3 */
#define PHASE3_DURATION                     0 /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT_A              1.345

/* Phase 4 */
#define PHASE4_DURATION                     0 /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT_A              1.345

/* Phase 5 */
#define PHASE5_DURATION                     0 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE5_FINAL_CURRENT_A              1.345

#define ENABLE_SL_ALGO_FROM_PHASE           2

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                  0  /*!< degrees [0...359] */

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM               12965
#define NB_CONSECUTIVE_TESTS                2 /* corresponding to former
                                                 NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE / MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT              17 /*!< It expresses how much estimated speed can exceed forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT              15 /*!< It expresses how much estimated speed can be below forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */

#define TRANSITION_DURATION                 25 /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(64)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(64)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (8 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
