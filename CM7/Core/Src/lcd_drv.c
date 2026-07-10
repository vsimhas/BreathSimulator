#include "lcd_drv.h"
#include "lcd_font.h"
#include "main.h"

static void LCD_DrawGlyph(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t scale)
{
  const uint8_t *glyph = LCD_FontGetGlyph(c);
  uint8_t col;
  uint8_t row;
  uint8_t bit;

  for (col = 0U; col < LCD_FONT_WIDTH; col++)
  {
    uint8_t column_bits = glyph[col];
    for (row = 0U; row < LCD_FONT_HEIGHT; row++)
    {
      bit = (uint8_t)(1U << row);
      if ((column_bits & bit) != 0U)
      {
        uint8_t sx;
        uint8_t sy;
        for (sy = 0U; sy < scale; sy++)
        {
          for (sx = 0U; sx < scale; sx++)
          {
            LCD_DrawPixel((uint16_t)(x + ((uint16_t)col * scale) + sx),
                          (uint16_t)(y + ((uint16_t)row * scale) + sy),
                          color);
          }
        }
      }
      else if (bg_color != color)
      {
        uint8_t sx;
        uint8_t sy;
        for (sy = 0U; sy < scale; sy++)
        {
          for (sx = 0U; sx < scale; sx++)
          {
            LCD_DrawPixel((uint16_t)(x + ((uint16_t)col * scale) + sx),
                          (uint16_t)(y + ((uint16_t)row * scale) + sy),
                          bg_color);
          }
        }
      }
    }
  }
}

uint16_t *LCD_GetFrameBuffer(void)
{
  return (uint16_t *)LCD_FRAME_BUFFER_ADDR;
}

void LCD_FlushFrameBuffer(void)
{
  /* SDRAM framebuffer MPU subregion is non-cacheable — no D-cache maintenance needed. */
}

HAL_StatusTypeDef LCD_Init(LTDC_HandleTypeDef *hltdc_local)
{
  LTDC_LayerCfgTypeDef layer_cfg = {0};

  hltdc_local->Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc_local->Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc_local->Init.DEPolarity = LTDC_DEPOLARITY_AH;
  hltdc_local->Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc_local->Init.HorizontalSync = LCD_LTDC_HSYNC;
  hltdc_local->Init.VerticalSync = LCD_LTDC_VSYNC;
  hltdc_local->Init.AccumulatedHBP = LCD_LTDC_ACCUMULATED_HBP;
  hltdc_local->Init.AccumulatedVBP = LCD_LTDC_ACCUMULATED_VBP;
  hltdc_local->Init.AccumulatedActiveW = LCD_LTDC_ACCUMULATED_ACTIVE_W;
  hltdc_local->Init.AccumulatedActiveH = LCD_LTDC_ACCUMULATED_ACTIVE_H;
  hltdc_local->Init.TotalWidth = LCD_LTDC_TOTAL_WIDTH;
  hltdc_local->Init.TotalHeigh = LCD_LTDC_TOTAL_HEIGHT;
  hltdc_local->Init.Backcolor.Blue = 0U;
  hltdc_local->Init.Backcolor.Green = 0U;
  hltdc_local->Init.Backcolor.Red = 0U;

  if (HAL_LTDC_Init(hltdc_local) != HAL_OK)
  {
    return HAL_ERROR;
  }

  layer_cfg.WindowX0 = 0U;
  layer_cfg.WindowX1 = LCD_WIDTH;
  layer_cfg.WindowY0 = 0U;
  layer_cfg.WindowY1 = LCD_HEIGHT;
  layer_cfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  layer_cfg.Alpha = 255U;
  layer_cfg.Alpha0 = 0U;
  layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  layer_cfg.FBStartAdress = LCD_FRAME_BUFFER_ADDR;
  layer_cfg.ImageWidth = LCD_WIDTH;
  layer_cfg.ImageHeight = LCD_HEIGHT;
  layer_cfg.Backcolor.Blue = 0U;
  layer_cfg.Backcolor.Green = 0U;
  layer_cfg.Backcolor.Red = 0U;

  if (HAL_LTDC_ConfigLayer(hltdc_local, &layer_cfg, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* LTDC scan-out and backlight are enabled in LCD_LatchController after TouchGFX paints. */

  return HAL_OK;
}

void LCD_LatchController(LTDC_HandleTypeDef *hltdc_local)
{
  if ((hltdc_local == NULL) || (hltdc_local->Instance == NULL))
  {
    return;
  }

  if (LTDC_Layer1->CFBAR != LCD_FRAME_BUFFER_ADDR)
  {
    LTDC_Layer1->CFBAR = LCD_FRAME_BUFFER_ADDR;
  }

  LTDC_Layer1->CR |= LTDC_LxCR_LEN;
  __HAL_LTDC_ENABLE(hltdc_local);
  __HAL_LTDC_LAYER_ENABLE(hltdc_local, 0U);
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(hltdc_local);
  LCD_SetBacklight(1U);
}

HAL_StatusTypeDef LCD_DisplayBringUp(LTDC_HandleTypeDef *hltdc_local)
{
  if (hltdc_local == NULL)
  {
    return HAL_ERROR;
  }

  /* Fill before enabling scan-out; turn backlight on last (panel timing settle). */
  LCD_FillScreen(LCD_COLOR_BLUE);
  __HAL_LTDC_LAYER_ENABLE(hltdc_local, 0U);
  __HAL_LTDC_ENABLE(hltdc_local);
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(hltdc_local);
  HAL_Delay(20);
  LCD_SetBacklight(1U);

  return HAL_OK;
}

void LCD_SetBacklight(uint8_t enable)
{
  HAL_GPIO_WritePin(BL_CTL_GPIO_Port, BL_CTL_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  uint16_t *framebuffer;

  if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT))
  {
    return;
  }

  framebuffer = LCD_GetFrameBuffer();
  framebuffer[((uint32_t)y * LCD_WIDTH) + x] = color;
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  uint16_t row;
  uint16_t col;
  uint16_t x_end;
  uint16_t y_end;
  uint16_t *framebuffer;

  if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT) || (w == 0U) || (h == 0U))
  {
    return;
  }

  x_end = (uint16_t)((x + w > LCD_WIDTH) ? LCD_WIDTH : (x + w));
  y_end = (uint16_t)((y + h > LCD_HEIGHT) ? LCD_HEIGHT : (y + h));
  framebuffer = LCD_GetFrameBuffer();

  for (row = y; row < y_end; row++)
  {
    for (col = x; col < x_end; col++)
    {
      framebuffer[((uint32_t)row * LCD_WIDTH) + col] = color;
    }
  }
}

void LCD_FillScreen(uint16_t color)
{
  LCD_FillRect(0U, 0U, LCD_WIDTH, LCD_HEIGHT, color);
  LCD_FlushFrameBuffer();
}

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if ((w == 0U) || (h == 0U))
  {
    return;
  }

  LCD_FillRect(x, y, w, 1U, color);
  LCD_FillRect(x, (uint16_t)(y + h - 1U), w, 1U, color);
  LCD_FillRect(x, y, 1U, h, color);
  LCD_FillRect((uint16_t)(x + w - 1U), y, 1U, h, color);
}

void LCD_DrawColorBars(void)
{
  static const uint16_t bar_colors[8] =
  {
    LCD_COLOR_WHITE,
    LCD_COLOR_YELLOW,
    LCD_COLOR_CYAN,
    LCD_COLOR_GREEN,
    LCD_COLOR_MAGENTA,
    LCD_COLOR_RED,
    LCD_COLOR_BLUE,
    LCD_COLOR_BLACK
  };

  uint16_t bar_width = (uint16_t)(LCD_WIDTH / 8U);
  uint8_t index;

  for (index = 0U; index < 8U; index++)
  {
    LCD_FillRect((uint16_t)(index * bar_width), 0U, bar_width, LCD_HEIGHT, bar_colors[index]);
  }

  LCD_FlushFrameBuffer();
}

void LCD_DrawCheckerboard(uint16_t tile_size, uint16_t color_a, uint16_t color_b)
{
  uint16_t y;
  uint16_t x;

  if (tile_size == 0U)
  {
    tile_size = 16U;
  }

  for (y = 0U; y < LCD_HEIGHT; y++)
  {
    for (x = 0U; x < LCD_WIDTH; x++)
    {
      uint16_t color = (((x / tile_size) + (y / tile_size)) & 1U) ? color_a : color_b;
      LCD_DrawPixel(x, y, color);
    }
  }

  LCD_FlushFrameBuffer();
}

void LCD_DrawGrid(uint16_t spacing, uint16_t color)
{
  uint16_t pos;

  if (spacing == 0U)
  {
    spacing = 20U;
  }

  LCD_FillScreen(LCD_COLOR_BLACK);

  for (pos = 0U; pos < LCD_WIDTH; pos = (uint16_t)(pos + spacing))
  {
    LCD_FillRect(pos, 0U, 1U, LCD_HEIGHT, color);
  }

  for (pos = 0U; pos < LCD_HEIGHT; pos = (uint16_t)(pos + spacing))
  {
    LCD_FillRect(0U, pos, LCD_WIDTH, 1U, color);
  }

  LCD_FlushFrameBuffer();
}

static uint16_t LCD_BlendRgb565(uint16_t c0, uint16_t c1, uint8_t weight)
{
  uint32_t r0 = (c0 >> 11) & 0x1FU;
  uint32_t g0 = (c0 >> 5) & 0x3FU;
  uint32_t b0 = c0 & 0x1FU;
  uint32_t r1 = (c1 >> 11) & 0x1FU;
  uint32_t g1 = (c1 >> 5) & 0x3FU;
  uint32_t b1 = c1 & 0x1FU;
  uint32_t inv = 255U - weight;

  uint32_t r = ((r0 * inv) + (r1 * weight)) / 255U;
  uint32_t g = ((g0 * inv) + (g1 * weight)) / 255U;
  uint32_t b = ((b0 * inv) + (b1 * weight)) / 255U;

  return (uint16_t)((r << 11) | (g << 5) | b);
}

void LCD_DrawHorizontalGradient(uint16_t y0, uint16_t height, uint16_t color_left, uint16_t color_right)
{
  uint16_t x;
  uint16_t y;
  uint16_t y_end;

  if (y0 >= LCD_HEIGHT)
  {
    return;
  }

  y_end = (uint16_t)((y0 + height > LCD_HEIGHT) ? LCD_HEIGHT : (y0 + height));

  for (y = y0; y < y_end; y++)
  {
    for (x = 0U; x < LCD_WIDTH; x++)
    {
      uint8_t weight = (uint8_t)(((uint32_t)x * 255U) / (LCD_WIDTH - 1U));
      LCD_DrawPixel(x, y, LCD_BlendRgb565(color_left, color_right, weight));
    }
  }

  LCD_FlushFrameBuffer();
}

void LCD_DrawString(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg_color, uint8_t scale)
{
  uint16_t cursor_x = x;

  if ((text == NULL) || (scale == 0U))
  {
    return;
  }

  while (*text != '\0')
  {
    LCD_DrawGlyph(cursor_x, y, *text, color, bg_color, scale);
    cursor_x = (uint16_t)(cursor_x + ((LCD_FONT_WIDTH + 1U) * scale));
    text++;
  }

  LCD_FlushFrameBuffer();
}
