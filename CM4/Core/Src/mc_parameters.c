
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
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

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"
#include "r3_2_h7xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - H7 - Shared Resources
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (5U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(17U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(17U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(17U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(17U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(5U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
  },
  .ADCConfig2 = {
                  (7U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(7U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(7U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(5U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(5U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(7U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCDataReg1 = {
                   ADC1
                  ,ADC1
                  ,ADC1
                  ,ADC1
                  ,ADC1
                  ,ADC1
                 },
  .ADCDataReg2 = {
                   ADC2
                  ,ADC2
                  ,ADC2
                  ,ADC2
                  ,ADC2
                  ,ADC2
                 },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE,
  .TIMx                  = TIM8,

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMPParams           = MC_NULL,

/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,

  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = 0,
  .DAC_OVP_Threshold     = 23830,
};

ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = U_RPM/SPEED_UNIT
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/

