/**
  ******************************************************************************
  * @file    sys_diag.h
  * @brief   Runtime diagnostics for chasing the SensorTask hang.
  ******************************************************************************
  *
  * On 2026-10-09 SensorTask stopped twice: the flow and pressure read counters
  * froze together while the breath and command tasks carried on normally. Both
  * times it happened during a burst of USB `sim set` commands, and the debugger
  * could not be attached to catch it in the act.
  *
  * The leading hypothesis is a stack overflow in SystemTask. It runs every USB
  * command handler (snprintf with many %f, plus large struct copies), and it is
  * created immediately after SensorTask - so with heap_4 and a downward-growing
  * stack, the memory just below SystemTask's stack is SensorTask's TCB. An
  * overflow there would corrupt SensorTask's wake time and leave it asleep for
  * ever while everything else runs: exactly the symptom.
  *
  * This module exists to confirm or kill that, without a debugger:
  *  - every task's stack high-water mark (the deepest use since boot; 0 means
  *    the stack has been overrun), readable BEFORE any hang - one burst of
  *    commands is enough to see whether SystemTask is near its limit;
  *  - a SensorTask stage marker, so the next hang names the call it is in and
  *    for how long, and a loop counter that shows the instant it stopped.
  *
  * FreeRTOS's own overflow check is deliberately NOT enabled: TouchGFX's
  * OSWrappers defines vApplicationStackOverflowHook as while(1), so enabling it
  * would turn a one-task stall into a whole-board freeze.
  */

#ifndef SYS_DIAG_H
#define SYS_DIAG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Where SensorTask is in its loop. Set just before each call that could block. */
typedef enum
{
  SENSOR_STAGE_DELAY = 0,     /**< osDelayUntil - normal resting place */
  SENSOR_STAGE_REINIT,        /**< flow re-init: takes the I2C1 mutex with NO timeout */
  SENSOR_STAGE_FLOW_MUTEX,    /**< taking the I2C1 mutex (20 ms timeout) */
  SENSOR_STAGE_FLOW_IO,       /**< SFM3300 read / start / init on I2C1 */
  SENSOR_STAGE_PRESS,         /**< AMS5935 harvest, tare and next start on SPI4 */
  SENSOR_STAGE_INPUTS,        /**< BreathSim_SetSensorInputs */
  SENSOR_STAGE_TELEM,         /**< pushing the F and P telemetry lines */
  SENSOR_STAGE_COUNT
} SensorStage_t;

extern volatile uint8_t  g_sensor_stage;
extern volatile uint32_t g_sensor_stage_tick;  /**< HAL tick when the stage was entered */
extern volatile uint32_t g_sensor_loops;       /**< SensorTask loop iterations since boot */

const char *SysDiag_SensorStageName(uint8_t stage);

/** Number of tasks SysDiag_Task() reports. */
uint8_t SysDiag_TaskCount(void);

/**
  * @brief One task's diagnostics.
  *
  * Reading another task's state and high-water mark goes through its TCB. If a
  * TCB has been corrupted - which is the hypothesis - that read may itself be
  * unsafe, so SensorTask is reported LAST, after everything else is out.
  *
  * @param stack_size_bytes  as configured
  * @param stack_free_bytes  least free stack since boot; 0 = overrun
  * @param state             osThreadState_t as an int
  */
bool SysDiag_Task(uint8_t idx, const char **name, uint32_t *stack_size_bytes,
                  uint32_t *stack_free_bytes, int32_t *state);

#ifdef __cplusplus
}
#endif

#endif /* SYS_DIAG_H */
