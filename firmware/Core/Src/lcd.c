/* ============================================================================
 * lcd.c —— 绘图实现（ASCII 用本地 8x16 点阵，不依赖字库芯片）
 *
 * 性能说明：
 *   LCD_WR_DATA() 每写一个像素都会翻一次 CS，画满一屏要几万次 CS 翻转，太慢。
 *   所以批量画像素时用「CS 拉低一次 -> 连续写 -> CS 拉高」的方式，
 *   实测刷一屏 128x160 从约 400ms 降到约 30ms。
 * ========================================================================== */

#include "lcd.h"
#include "lcdfont.h"        /* 8x16 ASCII 点阵（只在 lcd.c 里引用，避免重复定义）*/

/* ----------------------------------------------------------------------------
 * 区域填充
 * -------------------------------------------------------------------------- */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t x, y;
    uint16_t hi = (uint16_t)(color >> 8);
    uint16_t lo = (uint16_t)(color & 0xFF);

    if (xsta > xend) { x = xsta; xsta = xend; xend = x; }
    if (ysta > yend) { y = ysta; ysta = yend; yend = y; }

    LCD_Address_Set(xsta, ysta, xend, yend);
    LCD_CS_Clr();
    for (y = ysta; y <= yend; y++) {
        for (x = xsta; x <= xend; x++) {
            LCD_Writ_Bus((uint8_t)hi);
            LCD_Writ_Bus((uint8_t)lo);
        }
    }
    LCD_CS_Set();
}

/* ----------------------------------------------------------------------------
 * 画点
 * -------------------------------------------------------------------------- */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

/* ----------------------------------------------------------------------------
 * 画线（Bresenham）
 * -------------------------------------------------------------------------- */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int16_t dx, dy, sx, sy, err, e2;

    dx = (int16_t)((x2 > x1) ? (x2 - x1) : (x1 - x2));
    dy = (int16_t)((y2 > y1) ? (y2 - y1) : (y1 - y2));
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;
    err = dx - dy;

    for (;;) {
        LCD_DrawPoint(x1, y1, color);
        if (x1 == x2 && y1 == y2) { break; }
        e2 = err * 2;
        if (e2 > -dy) { err -= dy; x1 = (uint16_t)(x1 + sx); }
        if (e2 <  dx) { err += dx; y1 = (uint16_t)(y1 + sy); }
    }
}

/* ----------------------------------------------------------------------------
 * 画矩形框
 * -------------------------------------------------------------------------- */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

/* ----------------------------------------------------------------------------
 * 画圆（中点画圆法）
 * -------------------------------------------------------------------------- */
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int16_t a = 0;
    int16_t b = (int16_t)r;
    int16_t err = 0;

    while (a <= b) {
        LCD_DrawPoint((uint16_t)(x0 + a), (uint16_t)(y0 + b), color);
        LCD_DrawPoint((uint16_t)(x0 + b), (uint16_t)(y0 + a), color);
        LCD_DrawPoint((uint16_t)(x0 - a), (uint16_t)(y0 + b), color);
        LCD_DrawPoint((uint16_t)(x0 - b), (uint16_t)(y0 + a), color);
        LCD_DrawPoint((uint16_t)(x0 + a), (uint16_t)(y0 - b), color);
        LCD_DrawPoint((uint16_t)(x0 + b), (uint16_t)(y0 - a), color);
        LCD_DrawPoint((uint16_t)(x0 - a), (uint16_t)(y0 - b), color);
        LCD_DrawPoint((uint16_t)(x0 - b), (uint16_t)(y0 - a), color);

        if (err <= 0) {
            a++;
            err += 2 * a + 1;
        } else {
            b--;
            err += -2 * b + 1;
        }
    }
}

/* ----------------------------------------------------------------------------
 * 显示一个 ASCII 字符（8x16）
 *
 * num  : ASCII 码（0x20 ~ 0x7E），超出范围画 '?'
 * fc   : 前景色
 * bc   : 背景色（会自动覆盖旧内容，所以同一位置反复刷新不会叠影）
 * sizey: 仅支持 16；传其它值按 16 处理
 * mode : 保留参数（0/1），此实现始终用背景色填充
 *
 * ⚠ 位序很重要：lcdfont.h 的字模是 **低位优先**（bit0 = 最左边那个像素）。
 *   照搬“高位优先”的写法（temp & 0x80 再左移）会让每个字符左右镜像，
 *   而行列位置、字间距都还是对的 —— 症状就是「字是反的但位置没错」。
 *   中文字库芯片那边的数据是高位优先，两处不能混。
 * -------------------------------------------------------------------------- */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num,
                  uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint16_t idx;
    uint8_t  temp;
    uint8_t  t, m;
    uint16_t hi_f = (uint16_t)(fc >> 8);
    uint16_t lo_f = (uint16_t)(fc & 0xFF);
    uint16_t hi_b = (uint16_t)(bc >> 8);
    uint16_t lo_b = (uint16_t)(bc & 0xFF);

    (void)sizey;
    (void)mode;

    if (num < ' ' || num > '~') {
        num = '?';
    }
    idx = (uint16_t)(num - ' ');

    LCD_Address_Set(x, y, (uint16_t)(x + 7), (uint16_t)(y + 15));

    LCD_CS_Clr();
    for (t = 0; t < 16; t++) {
        temp = ascii_1608[idx][t];
        for (m = 0; m < 8; m++) {
            if (temp & (0x01u << m)) {          /* 低位优先：bit0 是最左边 */
                LCD_Writ_Bus((uint8_t)hi_f);
                LCD_Writ_Bus((uint8_t)lo_f);
            } else {
                LCD_Writ_Bus((uint8_t)hi_b);
                LCD_Writ_Bus((uint8_t)lo_b);
            }
        }
    }
    LCD_CS_Set();
}

/* ----------------------------------------------------------------------------
 * 显示字符串（遇到 '\0' 结束，遇到 '\n' 换行）
 * -------------------------------------------------------------------------- */
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p,
                    uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint16_t x0 = x;

    while (*p != '\0') {
        if (*p == '\n') {
            x = x0;
            y = (uint16_t)(y + 16);
        } else {
            LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
            x = (uint16_t)(x + 8);
        }
        p++;
    }
}

/* ----------------------------------------------------------------------------
 * 显示无符号整数（定宽 len 位，右对齐，不足补空格）
 * -------------------------------------------------------------------------- */
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len,
                    uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t  t, digit;
    uint16_t div = 1;
    uint8_t  started = 0;

    if (len == 0) { return; }
    if (len > 5)  { len = 5; }

    for (t = 1; t < len; t++) {
        div = (uint16_t)(div * 10);
    }

    for (t = 0; t < len; t++) {
        digit = (uint8_t)((num / div) % 10);
        if (digit != 0) {
            started = 1;
        }
        if (started || t == (uint8_t)(len - 1)) {
            LCD_ShowChar((uint16_t)(x + t * 8), y, (uint8_t)('0' + digit), fc, bc, sizey, 0);
        } else {
            LCD_ShowChar((uint16_t)(x + t * 8), y, ' ', fc, bc, sizey, 0);
        }
        if (div >= 10) {
            div = (uint16_t)(div / 10);
        }
    }
}

/* ----------------------------------------------------------------------------
 * 显示一位小数的浮点数（例如 12.3）
 * -------------------------------------------------------------------------- */
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len,
                       uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint16_t int_part;
    uint8_t  dec_part;

    if (num < 0.0f) {
        num = 0.0f;
    }

    int_part = (uint16_t)num;
    dec_part = (uint8_t)((num - (float)int_part) * 10.0f + 0.5f);
    if (dec_part > 9u) {
        dec_part = 0u;
        int_part++;
    }

    LCD_ShowIntNum(x, y, int_part, len, fc, bc, sizey);
    LCD_ShowChar((uint16_t)(x + len * 8), y, '.', fc, bc, sizey, 0);
    LCD_ShowChar((uint16_t)(x + len * 8 + 8), y, (uint8_t)('0' + dec_part), fc, bc, sizey, 0);
}
