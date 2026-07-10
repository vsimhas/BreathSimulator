#ifndef LCD_FONT_H
#define LCD_FONT_H

#include <stdint.h>

#define LCD_FONT_WIDTH   5U
#define LCD_FONT_HEIGHT  7U

const uint8_t *LCD_FontGetGlyph(char c);

#endif /* LCD_FONT_H */
