#include "lcd_font.h"

/* 5x7 ASCII subset used by the CPAP board demo (space through Z). */
static const uint8_t s_font_space[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t s_font_dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
static const uint8_t s_font_dot[5] = {0x00U, 0x00U, 0x00U, 0x06U, 0x06U};

static const uint8_t s_font_0[5] = {0x0EU, 0x11U, 0x13U, 0x15U, 0x0EU};
static const uint8_t s_font_1[5] = {0x04U, 0x0CU, 0x04U, 0x04U, 0x0EU};
static const uint8_t s_font_2[5] = {0x0EU, 0x11U, 0x02U, 0x04U, 0x1FU};
static const uint8_t s_font_3[5] = {0x1FU, 0x02U, 0x0CU, 0x02U, 0x1FU};
static const uint8_t s_font_4[5] = {0x12U, 0x12U, 0x1FU, 0x02U, 0x02U};
static const uint8_t s_font_5[5] = {0x1FU, 0x10U, 0x1EU, 0x01U, 0x1EU};
static const uint8_t s_font_6[5] = {0x0EU, 0x10U, 0x1EU, 0x11U, 0x0EU};
static const uint8_t s_font_7[5] = {0x1FU, 0x01U, 0x02U, 0x04U, 0x08U};
static const uint8_t s_font_8[5] = {0x0EU, 0x11U, 0x0EU, 0x11U, 0x0EU};
static const uint8_t s_font_9[5] = {0x0EU, 0x11U, 0x0FU, 0x01U, 0x0EU};

static const uint8_t s_font_A[5] = {0x0EU, 0x11U, 0x1FU, 0x11U, 0x11U};
static const uint8_t s_font_B[5] = {0x1EU, 0x11U, 0x1EU, 0x11U, 0x1EU};
static const uint8_t s_font_C[5] = {0x0EU, 0x11U, 0x10U, 0x11U, 0x0EU};
static const uint8_t s_font_D[5] = {0x1EU, 0x11U, 0x11U, 0x11U, 0x1EU};
static const uint8_t s_font_E[5] = {0x1FU, 0x10U, 0x1EU, 0x10U, 0x1FU};
static const uint8_t s_font_F[5] = {0x1FU, 0x10U, 0x1EU, 0x10U, 0x10U};
static const uint8_t s_font_G[5] = {0x0EU, 0x11U, 0x17U, 0x11U, 0x0EU};
static const uint8_t s_font_H[5] = {0x11U, 0x11U, 0x1FU, 0x11U, 0x11U};
static const uint8_t s_font_I[5] = {0x0EU, 0x04U, 0x04U, 0x04U, 0x0EU};
static const uint8_t s_font_L[5] = {0x10U, 0x10U, 0x10U, 0x10U, 0x1FU};
static const uint8_t s_font_M[5] = {0x11U, 0x1BU, 0x15U, 0x11U, 0x11U};
static const uint8_t s_font_N[5] = {0x11U, 0x19U, 0x15U, 0x13U, 0x11U};
static const uint8_t s_font_O[5] = {0x0EU, 0x11U, 0x11U, 0x11U, 0x0EU};
static const uint8_t s_font_P[5] = {0x1EU, 0x11U, 0x1EU, 0x10U, 0x10U};
static const uint8_t s_font_R[5] = {0x1EU, 0x11U, 0x1EU, 0x14U, 0x13U};
static const uint8_t s_font_S[5] = {0x0FU, 0x10U, 0x0EU, 0x01U, 0x1EU};
static const uint8_t s_font_T[5] = {0x1FU, 0x04U, 0x04U, 0x04U, 0x04U};
static const uint8_t s_font_U[5] = {0x11U, 0x11U, 0x11U, 0x11U, 0x0EU};
static const uint8_t s_font_V[5] = {0x11U, 0x11U, 0x11U, 0x0AU, 0x04U};
static const uint8_t s_font_W[5] = {0x11U, 0x11U, 0x15U, 0x1BU, 0x11U};
static const uint8_t s_font_X[5] = {0x11U, 0x0AU, 0x04U, 0x0AU, 0x11U};
static const uint8_t s_font_Y[5] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x04U};
static const uint8_t s_font_Z[5] = {0x1FU, 0x02U, 0x04U, 0x08U, 0x1FU};

const uint8_t *LCD_FontGetGlyph(char c)
{
  if (c == ' ')
  {
    return s_font_space;
  }
  if (c == '-')
  {
    return s_font_dash;
  }
  if (c == '.')
  {
    return s_font_dot;
  }
  if (c >= '0' && c <= '9')
  {
    static const uint8_t *digits[10] =
    {
      s_font_0, s_font_1, s_font_2, s_font_3, s_font_4,
      s_font_5, s_font_6, s_font_7, s_font_8, s_font_9
    };
    return digits[(uint8_t)(c - '0')];
  }
  if (c >= 'A' && c <= 'Z')
  {
    switch (c)
    {
      case 'A': return s_font_A;
      case 'B': return s_font_B;
      case 'C': return s_font_C;
      case 'D': return s_font_D;
      case 'E': return s_font_E;
      case 'F': return s_font_F;
      case 'G': return s_font_G;
      case 'H': return s_font_H;
      case 'I': return s_font_I;
      case 'L': return s_font_L;
      case 'M': return s_font_M;
      case 'N': return s_font_N;
      case 'O': return s_font_O;
      case 'P': return s_font_P;
      case 'R': return s_font_R;
      case 'S': return s_font_S;
      case 'T': return s_font_T;
      case 'U': return s_font_U;
      case 'V': return s_font_V;
      case 'W': return s_font_W;
      case 'X': return s_font_X;
      case 'Y': return s_font_Y;
      case 'Z': return s_font_Z;
      default: break;
    }
  }
  return s_font_space;
}
