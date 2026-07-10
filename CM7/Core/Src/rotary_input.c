/**
 * rotary_input.c
 *
 * Plain-C driver that converts raw TIM2 encoder counts and ROT_SW interrupt
 * counts into application-friendly detent/press events.
 *
 * Critical detail: TIM2 is configured with Period = 80 (i.e. counter wraps in
 * the range [0..80]). To compute a signed delta across a wrap we use 16-bit
 * arithmetic and treat the 0..80 window as a tiny modulus. With 4 counts per
 * detent and a poll rate of >= 100 Hz the human cannot turn the knob fast
 * enough to alias.
 */

#include "rotary_input.h"

#include "main.h"
#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern volatile uint32_t ulRotSwIntrCnt; /* defined in stm32h7xx_it.c */

#define ROTARY_COUNTS_PER_DETENT  4
/* ROT_SW_Pin is configured as GPIO_MODE_IT_RISING (see MX_GPIO_Init in
 * main.c), so HAL_GPIO_EXTI_Callback fires exactly once per physical click
 * - in the ideal case. In practice the mechanical switch bounces and the
 * EXTI line latches multiple rising edges within a few milliseconds of one
 * physical click. The debounce below collapses those bursts to a single
 * logical press. */
#define ROT_SW_COUNTS_PER_PRESS   1
/* Minimum time between two accepted press events. 80 ms comfortably hides
 * mechanical bounce (typically <10 ms on this encoder) while still letting
 * an attentive user click twice in deliberate succession (>12 Hz). */
#define ROT_SW_DEBOUNCE_MS        80U

static uint16_t s_last_tim2_cnt = 0U;
static int32_t  s_accum_counts = 0;        /* signed accumulator of raw counts */
static int32_t  s_pending_detents = 0;     /* not yet popped */

static uint32_t s_last_sw_cnt = 0U;
static uint32_t s_pending_presses = 0U;
static uint32_t s_last_press_tick = 0U;    /* HAL_GetTick() of last accepted press */

static uint8_t  s_initialised = 0U;

/* Wrap-aware signed delta on a free-running [0..ARR] modulus.
 * Treats movements > ARR/2 as having wrapped the short way. */
static int16_t signed_delta_wrap(uint16_t curr, uint16_t prev, uint16_t arr_plus_1)
{
    int32_t diff = (int32_t)curr - (int32_t)prev;
    int32_t half = (int32_t)arr_plus_1 / 2;
    if (diff > half)
    {
        diff -= (int32_t)arr_plus_1;
    }
    else if (diff < -half)
    {
        diff += (int32_t)arr_plus_1;
    }
    return (int16_t)diff;
}

void RotaryInput_Init(void)
{
    s_last_tim2_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_last_sw_cnt   = ulRotSwIntrCnt;
    s_accum_counts  = 0;
    s_pending_detents = 0;
    s_pending_presses = 0U;
    s_last_press_tick = HAL_GetTick();
    s_initialised   = 1U;
}

void RotaryInput_Poll(void)
{
    if (!s_initialised)
    {
        RotaryInput_Init();
        return;
    }

    /* Encoder delta. TIM2.Init.Period = 80, so the counter cycles 0..80,
     * giving an effective modulus of 81. */
    uint16_t curr   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t period = (uint16_t)(htim2.Init.Period + 1U);
    int16_t  d      = signed_delta_wrap(curr, s_last_tim2_cnt, period);
    s_last_tim2_cnt = curr;

    s_accum_counts += d;

    /* Convert raw counts -> detents (4 counts/detent). Keep the remainder
     * so we never lose a partial detent across polls. */
    while (s_accum_counts >= ROTARY_COUNTS_PER_DETENT)
    {
        s_pending_detents += 1;
        s_accum_counts    -= ROTARY_COUNTS_PER_DETENT;
    }
    while (s_accum_counts <= -ROTARY_COUNTS_PER_DETENT)
    {
        s_pending_detents -= 1;
        s_accum_counts    += ROTARY_COUNTS_PER_DETENT;
    }

    /* Push-switch. We always consume every raw ISR edge so the running count
     * doesn't drift, but a burst of edges that arrive inside the bounce
     * window collapses to exactly one logical press. */
    uint32_t curr_sw = ulRotSwIntrCnt;
    uint32_t sw_diff = curr_sw - s_last_sw_cnt; /* unsigned wrap-safe */
    s_last_sw_cnt = curr_sw;

    if (sw_diff > 0U)
    {
        uint32_t now = HAL_GetTick();
        if ((now - s_last_press_tick) >= ROT_SW_DEBOUNCE_MS)
        {
            s_pending_presses += 1U;
            s_last_press_tick  = now;
        }
        /* else: still inside the debounce window - drop the burst entirely.
         * This is what fixes the "one click sometimes jumps two screens
         * deep" symptom on the actual hardware. */
    }
}

int8_t RotaryInput_PopDelta(void)
{
    int32_t d;

    __disable_irq();
    d = s_pending_detents;
    /* Clamp the report to one byte to keep the API tiny; remainder stays for
     * next pop. */
    if (d > 127)
    {
        s_pending_detents = d - 127;
        d = 127;
    }
    else if (d < -128)
    {
        s_pending_detents = d - (-128);
        d = -128;
    }
    else
    {
        s_pending_detents = 0;
    }
    __enable_irq();

    return (int8_t)d;
}

uint8_t RotaryInput_PopPress(void)
{
    uint8_t p;

    __disable_irq();
    if (s_pending_presses > 255U)
    {
        p = 255U;
        s_pending_presses -= 255U;
    }
    else
    {
        p = (uint8_t)s_pending_presses;
        s_pending_presses = 0U;
    }
    __enable_irq();

    return p;
}

void RotaryInput_PushBackPresses(uint8_t n)
{
    if (n == 0U) return;
    __disable_irq();
    s_pending_presses += (uint32_t)n;
    __enable_irq();
}

int32_t RotaryInput_PeekRawCount(void)
{
    return (int32_t)(uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

uint32_t RotaryInput_PeekRawSwCount(void)
{
    return ulRotSwIntrCnt;
}
