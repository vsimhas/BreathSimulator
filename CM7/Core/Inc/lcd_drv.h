#ifndef LCD_DRV_H
#define LCD_DRV_H

#include "lcd_panel.h"
#include "stm32h7xx_hal.h"

/*
 * STM32 LTDC RGB565 framebuffer driver for TM043NDH02 (480x272).
 * Drawing API adapted from concepts in:
 *   - RA6M3_TFT_4_3inch_04082021 (480x272 RGB565 GLCDC)
 *   - SM_7_INCH_BOARD_WITH_AUTO_4_2INCH (480x272 SDRAM framebuffer)
 * Panel timing follows TM043NDH02 V2.0 datasheet.
 */

HAL_StatusTypeDef LCD_Init(LTDC_HandleTypeDef *hltdc);
void LCD_SetBacklight(uint8_t enable);
HAL_StatusTypeDef LCD_DisplayBringUp(LTDC_HandleTypeDef *hltdc);
void LCD_LatchController(LTDC_HandleTypeDef *hltdc);

uint16_t *LCD_GetFrameBuffer(void);
void LCD_FlushFrameBuffer(void);

void LCD_FillScreen(uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawColorBars(void);
void LCD_DrawCheckerboard(uint16_t tile_size, uint16_t color_a, uint16_t color_b);
void LCD_DrawGrid(uint16_t spacing, uint16_t color);
void LCD_DrawHorizontalGradient(uint16_t y0, uint16_t height, uint16_t color_left, uint16_t color_right);
void LCD_DrawString(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg_color, uint8_t scale);

#endif /* LCD_DRV_H */
