#include "lcd_test.h"
#include "lcd_drv.h"
#include "lcd_panel.h"
#include "main.h"

volatile uint32_t lcd_test_step = 0U;
volatile HAL_StatusTypeDef lcd_test_result = HAL_BUSY;
volatile uint32_t lcd_demo_frame = 0U;

HAL_StatusTypeDef LCD_LTDC_ApplyPanelConfig(LTDC_HandleTypeDef *hltdc_local)
{
  return LCD_Init(hltdc_local);
}

HAL_StatusTypeDef LCD_RunBasicTest(LTDC_HandleTypeDef *hltdc_local)
{
  lcd_test_step = 1U;
  if (LCD_Init(hltdc_local) != HAL_OK)
  {
    return HAL_ERROR;
  }

  lcd_test_step = 2U;
  LCD_SetBacklight(1U);
  HAL_Delay(50);

  lcd_test_step = 3U;
  LCD_FillScreen(LCD_COLOR_RED);
  HAL_Delay(400);

  lcd_test_step = 4U;
  LCD_FillScreen(LCD_COLOR_GREEN);
  HAL_Delay(400);

  lcd_test_step = 5U;
  LCD_FillScreen(LCD_COLOR_BLUE);
  HAL_Delay(400);

  lcd_test_step = 6U;
  LCD_DrawColorBars();
  HAL_Delay(800);

  lcd_test_step = 7U;
  LCD_DrawCheckerboard(16U, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
  HAL_Delay(800);

  lcd_test_step = 8U;
  LCD_DrawGrid(24U, LCD_COLOR_CYAN);
  HAL_Delay(800);

  lcd_test_step = 9U;
  LCD_FillScreen(LCD_COLOR_BLACK);
  LCD_DrawRect(10U, 10U, 460U, 252U, LCD_COLOR_WHITE);
  LCD_DrawString(24U, 24U, "CPAP TESTBOARD V0.1", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2U);
  LCD_DrawString(24U, 64U, "TM043NDH02 480X272", LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 2U);
  LCD_DrawString(24U, 104U, "LTDC RGB565 OK", LCD_COLOR_GREEN, LCD_COLOR_BLACK, 2U);
  HAL_Delay(1200);

  lcd_test_step = 10U;
  LCD_DrawHorizontalGradient(0U, LCD_HEIGHT, LCD_COLOR_BLUE, LCD_COLOR_RED);
  HAL_Delay(800);

  lcd_demo_frame = 0U;
  lcd_test_step = 100U;
  return HAL_OK;
}

void LCD_RunDemoFrame(void)
{
  switch (lcd_demo_frame % 6U)
  {
    case 0U:
      LCD_FillScreen(LCD_COLOR_RED);
      break;
    case 1U:
      LCD_FillScreen(LCD_COLOR_GREEN);
      break;
    case 2U:
      LCD_FillScreen(LCD_COLOR_BLUE);
      break;
    case 3U:
      LCD_DrawColorBars();
      break;
    case 4U:
      LCD_DrawCheckerboard(20U, LCD_COLOR_CYAN, LCD_COLOR_MAGENTA);
      break;
    default:
      LCD_DrawHorizontalGradient(0U, LCD_HEIGHT, LCD_COLOR_YELLOW, LCD_COLOR_BLUE);
      break;
  }

  lcd_demo_frame++;
}
