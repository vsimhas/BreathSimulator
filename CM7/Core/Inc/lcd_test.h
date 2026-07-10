#ifndef LCD_TEST_H
#define LCD_TEST_H

#include "stm32h7xx_hal.h"

extern volatile uint32_t lcd_test_step;
extern volatile HAL_StatusTypeDef lcd_test_result;
extern volatile uint32_t lcd_demo_frame;

HAL_StatusTypeDef LCD_LTDC_ApplyPanelConfig(LTDC_HandleTypeDef *hltdc);
HAL_StatusTypeDef LCD_RunBasicTest(LTDC_HandleTypeDef *hltdc);
void LCD_RunDemoFrame(void);

#endif /* LCD_TEST_H */
