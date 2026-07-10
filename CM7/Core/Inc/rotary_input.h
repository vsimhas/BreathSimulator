/**
 * rotary_input.h - Quadrature encoder + push-switch input driver.
 *
 * The hardware:
 *   - Rotary encoder phases on TIM2 (encoder mode TI12, period = 80).
 *     One mechanical detent = 4 timer counts.
 *   - Push-switch on PB5 / EXTI9_5, configured as GPIO_MODE_IT_RISING in
 *     MX_GPIO_Init(). HAL_GPIO_EXTI_Callback in stm32h7xx_it.c gates the
 *     increment to GPIO_Pin == ROT_SW_Pin so NFC_INT (PI8, line 8 on the
 *     same EXTI9_5 handler) does not contaminate the count. Result:
 *     ulRotSwIntrCnt advances by exactly 1 per physical click.
 *
 * Usage:
 *   - Call RotaryInput_Init() once after HAL_TIM_Encoder_Start(&htim2,...).
 *   - Call RotaryInput_Poll() periodically (main loop, ~ every 1-10 ms) to
 *     accumulate detents/presses in the internal latches.
 *   - Consumers call RotaryInput_PopDelta() to receive the net detent delta
 *     accumulated since the last call (positive = clockwise), and
 *     RotaryInput_PopPress() to receive the number of full button presses
 *     since the last call.
 *
 * The Pop API is safe to call from a single consumer (e.g. the GUI tick).
 */
#ifndef ROTARY_INPUT_H
#define ROTARY_INPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void    RotaryInput_Init(void);
void    RotaryInput_Poll(void);

int8_t  RotaryInput_PopDelta(void);
uint8_t RotaryInput_PopPress(void);

/* Return un-applied presses to the latch (e.g. if the application drained
 * the queue but only applied one of them this tick). The next Pop will see
 * them again. */
void    RotaryInput_PushBackPresses(uint8_t n);

/* Inspect-only (does not consume). Useful for debugging in the watch window. */
int32_t RotaryInput_PeekRawCount(void);
uint32_t RotaryInput_PeekRawSwCount(void);

#ifdef __cplusplus
}
#endif

#endif /* ROTARY_INPUT_H */
