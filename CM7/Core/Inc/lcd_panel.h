#ifndef LCD_PANEL_H
#define LCD_PANEL_H

#include <stdint.h>

/* Tianma TM043NDH02: 480x272 RGB parallel panel (HX8257A T-CON), RGB565 output. */
#define LCD_WIDTH                   480U
#define LCD_HEIGHT                  272U
#define LCD_PIXEL_CLOCK_HZ          9000000U

/* Framebuffer in external SDRAM (offset from FMC bank1 base 0xC0000000). */
#define LCD_FRAME_BUFFER_ADDR       (0xC0100000U)
#define LCD_FRAME_BUFFER_SIZE       ((uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT * 2U)

/*
 * TM043NDH02 typical timing (525 x 286 @ ~9 MHz, 60 Hz) — TM043NDH02 V2.0 datasheet.
 * Alternate reference timings (same 480x272 panel, different GLCDC boards):
 *   RA6M3_TFT_4_3inch: total 582 x 286
 *   SM_7_INCH auto 4.2": total 540 x 278
 * If the image is shifted or unstable, try those totals in LTDC_Init.
 */
#define LCD_HSYNC_WIDTH             41U
#define LCD_HBACK_PORCH             2U
#define LCD_HFRONT_PORCH            2U
#define LCD_VSYNC_WIDTH             10U
#define LCD_VBACK_PORCH             2U
#define LCD_VFRONT_PORCH            2U

#define LCD_LTDC_HSYNC              (LCD_HSYNC_WIDTH - 1U)
#define LCD_LTDC_VSYNC              (LCD_VSYNC_WIDTH - 1U)
#define LCD_LTDC_ACCUMULATED_HBP    (LCD_HSYNC_WIDTH + LCD_HBACK_PORCH - 1U)
#define LCD_LTDC_ACCUMULATED_VBP    (LCD_VSYNC_WIDTH + LCD_VBACK_PORCH - 1U)
#define LCD_LTDC_ACCUMULATED_ACTIVE_W (LCD_HSYNC_WIDTH + LCD_HBACK_PORCH + LCD_WIDTH - 1U)
#define LCD_LTDC_ACCUMULATED_ACTIVE_H (LCD_VSYNC_WIDTH + LCD_VBACK_PORCH + LCD_HEIGHT - 1U)
#define LCD_LTDC_TOTAL_WIDTH        (LCD_HSYNC_WIDTH + LCD_HBACK_PORCH + LCD_WIDTH + LCD_HFRONT_PORCH - 1U)
#define LCD_LTDC_TOTAL_HEIGHT       (LCD_VSYNC_WIDTH + LCD_VBACK_PORCH + LCD_HEIGHT + LCD_VFRONT_PORCH - 1U)

#define LCD_RGB565(r, g, b)         ((uint16_t)(((((uint16_t)(r) & 0xF8U) << 8) | (((uint16_t)(g) & 0xFCU) << 3) | (((uint16_t)(b) & 0xF8U) >> 3))))

#define LCD_COLOR_BLACK             LCD_RGB565(0U, 0U, 0U)
#define LCD_COLOR_WHITE             LCD_RGB565(255U, 255U, 255U)
#define LCD_COLOR_RED               LCD_RGB565(255U, 0U, 0U)
#define LCD_COLOR_GREEN             LCD_RGB565(0U, 255U, 0U)
#define LCD_COLOR_BLUE              LCD_RGB565(0U, 0U, 255U)
#define LCD_COLOR_YELLOW            LCD_RGB565(255U, 255U, 0U)
#define LCD_COLOR_CYAN              LCD_RGB565(0U, 255U, 255U)
#define LCD_COLOR_MAGENTA           LCD_RGB565(255U, 0U, 255U)

#endif /* LCD_PANEL_H */
