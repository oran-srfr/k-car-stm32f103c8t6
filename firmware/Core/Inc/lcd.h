/* ============================================================================
 * lcd.h —— 绘图接口 + RGB565 颜色定义
 *
 * 字符来源说明：
 *   · ASCII（数字、字母、时间）→ 本文件，用 lcdfont.h 里的本地 8x16 点阵
 *     不依赖字库芯片，屏幕接好就一定能显示，最可靠
 *   · 汉字 → zk.c，走外部字库芯片（15x16）
 * ========================================================================== */

#ifndef __LCD_H
#define __LCD_H

#include "lcd_init.h"

/* ---------------------------- RGB565 颜色 ---------------------------- */
#define WHITE           0xFFFF
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x7FFF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define GRAY            0x8430
#define LGRAY           0xC618      /* 浅灰 */
#define DARKBLUE        0x01CF
#define LIGHTBLUE       0x7D7C
#define BROWN           0xBC40
#define BRRED           0xFC07
#define GRAYBLUE        0x5458
#define LIGHTGREEN      0x841F

/* ---------------------------- 基本绘图 ---------------------------- */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

/* ---------------------------- ASCII 字符 ---------------------------- */
/* sizey 目前只支持 16（8x16 点阵）；传 12/24/32 会按 16 画，避免引用未编入的字模 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num,
                  uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p,
                    uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len,
                    uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len,
                       uint16_t fc, uint16_t bc, uint8_t sizey);

#endif /* __LCD_H */
