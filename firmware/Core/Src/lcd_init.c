/* ============================================================================
 * lcd_init.c —— ST7735 软件 SPI 底层实现（HAL 版）
 *
 * 从参考包 lcd_init.c 移植，改动：
 *   1. StdPeriph 的 GPIO_SetBits/ResetBits -> HAL_GPIO_WritePin
 *   2. 补开 GPIOA 时钟（参考包只开了 GPIOB，但本站 CS=PA1、DC=PA12 在 A 口，
 *      这是最容易漏的一步，漏了屏幕全白毫无反应）
 *   3. 补发软件复位命令 0x01
 *   4. 去掉 JTAG 重映射（本站没用到 PB4/PB3/PA15）
 * ========================================================================== */

#include "lcd_init.h"

/* ----------------------------------------------------------------------------
 * 引脚初始化
 * -------------------------------------------------------------------------- */
void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 关键：GPIOA 和 GPIOB 都要开时钟！*/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SCLK / MOSI / CS / DC 推挽输出 */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = LCD_SCLK_PIN;
    HAL_GPIO_Init(LCD_SCLK_PORT, &gpio);

    gpio.Pin = LCD_MOSI_PIN;
    HAL_GPIO_Init(LCD_MOSI_PORT, &gpio);

    gpio.Pin = LCD_CS_PIN;
    HAL_GPIO_Init(LCD_CS_PORT, &gpio);

    gpio.Pin = LCD_DC_PIN;
    HAL_GPIO_Init(LCD_DC_PORT, &gpio);

#if LCD_USE_BLK_PIN
    gpio.Pin = LCD_BLK_PIN;
    HAL_GPIO_Init(LCD_BLK_PORT, &gpio);
    HAL_GPIO_WritePin(LCD_BLK_PORT, LCD_BLK_PIN, GPIO_PIN_SET);   /* 背光开 */
#endif

    /* 字库芯片 MISO：上拉输入（模块没接时会读成 1，便于发现接错） */
    gpio.Pin  = ZK_MISO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ZK_MISO_PORT, &gpio);

    /* 默认电平：片选拉低（选中 LCD），时钟拉高，数据线拉高 */
    LCD_CS_Set();
    LCD_SCLK_Set();
    LCD_MOSI_Set();
    LCD_DC_Set();
}

/* ----------------------------------------------------------------------------
 * 软件 SPI 写一个字节
 *
 * 时序：先设 MOSI，再拉高 SCLK（ST7735 在上升沿采样）
 * 注意：这个函数不管 CS，由调用者控制（因为 CS 要和字库芯片复用）
 * -------------------------------------------------------------------------- */
void LCD_Writ_Bus(uint8_t dat)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        LCD_SCLK_Clr();
        if (dat & 0x80) {
            LCD_MOSI_Set();
        } else {
            LCD_MOSI_Clr();
        }
        LCD_SCLK_Set();
        dat <<= 1;
    }
}

void LCD_WR_DATA8(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_Writ_Bus(dat);
    LCD_CS_Set();
}

void LCD_WR_DATA(uint16_t dat)
{
    LCD_CS_Clr();
    LCD_Writ_Bus((uint8_t)(dat >> 8));
    LCD_Writ_Bus((uint8_t)(dat & 0xFF));
    LCD_CS_Set();
}

void LCD_WR_REG(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();           /* 低 = 命令 */
    LCD_Writ_Bus(dat);
    LCD_DC_Set();           /* 高 = 数据 */
    LCD_CS_Set();
}

/* ----------------------------------------------------------------------------
 * 设置显示窗口（连续写像素前必须先调）
 * -------------------------------------------------------------------------- */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WR_REG(0x2A);                   /* 列地址 */
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);

    LCD_WR_REG(0x2B);                   /* 行地址 */
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);

    LCD_WR_REG(0x2C);                   /* 开始写显存 */
}

/* ----------------------------------------------------------------------------
 * 运行时修改扫描方向（MADCTL）。用于开机方向自检，或按需旋转屏幕
 * -------------------------------------------------------------------------- */
void LCD_SetMadctl(uint8_t v)
{
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(v);
}

/* ----------------------------------------------------------------------------
 * ST7735 初始化序列（1.8 寸 128x160 通用）
 * -------------------------------------------------------------------------- */
void LCD_Init(void)
{
    LCD_GPIO_Init();

    /* 软复位：本模块没有 RES 引脚，靠这条命令让驱动芯片回到初始状态 */
    LCD_WR_REG(0x01);
    HAL_Delay(120);

    LCD_WR_REG(0x11);                   /* 退出睡眠 */
    HAL_Delay(120);

    /* 帧率 */
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);

    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);

    LCD_WR_REG(0xB3);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);

    LCD_WR_REG(0xB4);                   /* 点反转 */
    LCD_WR_DATA8(0x03);

    /* 电源序列 */
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x04);

    LCD_WR_REG(0xC1);
    LCD_WR_DATA8(0xC0);

    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0x2A);

    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0xEE);

    LCD_WR_REG(0xC5);                   /* VCOM */
    LCD_WR_DATA8(0x1A);

    /* 扫描方向 + RGB 顺序 */
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(LCD_MADCTL_DEFAULT);

    /* 伽马校正 */
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x22);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x30);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x3A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x13);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x16);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x13);

    LCD_WR_REG(0x3A);                   /* 65K 色 */
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0x29);                   /* 开显示 */
}
