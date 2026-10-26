/* ============================================================================
 * lcd_init.h —— ST7735 软件 SPI 底层（HAL 版）
 *
 * 硬件接线（本项目实际接线，和参考包不同，注意别看错）：
 *     ST7735 模块      STM32F103C8T6
 *     ---------------------------------
 *     VCC         ->   3.3V
 *     GND         ->   GND
 *     CLK (SCL)   ->   PB14
 *     SDI (MOSI)  ->   PB15
 *     CS          ->   PA1     （与字库芯片 CS 共用这一根线）
 *     DC  (RS)    ->   PA12
 *     SDO (MISO)  ->   PB13    （读字库芯片用，必须接）
 *     BLK         ->   3.3V    （背光常亮，不占 MCU 脚）
 *
 * 关键：CS 是复用的
 *     拉低 = 选中 LCD，写字/写命令
 *     拉高 = 选中字库芯片，读汉字点阵
 * ========================================================================== */

#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 屏幕方向：0 或 1 = 竖屏 128x160；2 或 3 = 横屏 160x128 */
#define USE_HORIZONTAL      1

/* ============================================================================
 * 屏幕扫描方向（MADCTL 0x36）—— 每批屏的排线方向不同，必须按实物调
 *
 *   0x00  正常竖屏
 *   0xC0  上下左右都反转（180°旋转）
 *   0x40  左右镜像
 *   0x80  上下镜像
 *   0xA0  旋转 90°（横屏）
 *   0x60  旋转 270°（横屏）
 *
 * 如果开机字是镜像的 / 倒着的，就改这个值。
 * 开着 HMI_BRINGUP_TEST 时，固件会自动把这 4 个竖屏值各试 3 秒，
 * 屏幕上标 M1~M4，直接看哪个是对的。
 * ========================================================================== */
#define LCD_MADCTL_DEFAULT  0x00

#if (USE_HORIZONTAL == 0) || (USE_HORIZONTAL == 1)
#define LCD_W               128
#define LCD_H               160
#else
#define LCD_W               160
#define LCD_H               128
#endif

/* ============================================================================
 * 引脚定义：以后换接线只改这一段
 * ========================================================================== */

#define LCD_SCLK_PORT       GPIOB
#define LCD_SCLK_PIN        GPIO_PIN_14

#define LCD_MOSI_PORT       GPIOB
#define LCD_MOSI_PIN        GPIO_PIN_15

#define LCD_CS_PORT         GPIOA
#define LCD_CS_PIN          GPIO_PIN_1

#define LCD_DC_PORT         GPIOA
#define LCD_DC_PIN          GPIO_PIN_12

/* 字库芯片数据输出线 */
#define ZK_MISO_PORT        GPIOB
#define ZK_MISO_PIN         GPIO_PIN_13

/* 背光：本模块 BLK 直接接 3.3V，不需要 MCU 控制。
 * 如果你把 BLK 接到了 MCU（比如 PB9）想用 PWM 调光，把下面改成 1 并填引脚 */
#define LCD_USE_BLK_PIN     0
#if LCD_USE_BLK_PIN
#define LCD_BLK_PORT        GPIOB
#define LCD_BLK_PIN         GPIO_PIN_9
#endif

/* ============================================================================
 * 引脚操作宏
 * ========================================================================== */

#define LCD_SCLK_Clr()      HAL_GPIO_WritePin(LCD_SCLK_PORT, LCD_SCLK_PIN, GPIO_PIN_RESET)
#define LCD_SCLK_Set()      HAL_GPIO_WritePin(LCD_SCLK_PORT, LCD_SCLK_PIN, GPIO_PIN_SET)

#define LCD_MOSI_Clr()      HAL_GPIO_WritePin(LCD_MOSI_PORT, LCD_MOSI_PIN, GPIO_PIN_RESET)
#define LCD_MOSI_Set()      HAL_GPIO_WritePin(LCD_MOSI_PORT, LCD_MOSI_PIN, GPIO_PIN_SET)

#define LCD_DC_Clr()        HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_Set()        HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET)

/* 选中 LCD */
#define LCD_CS_Clr()        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_Set()        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET)

/* 选中字库芯片（与 LCD 反相：拉高选字库） */
#define ZK_CS_Clr()         HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define ZK_CS_Set()         HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET)

/* 读字库芯片输出位 */
#define ZK_MISO             HAL_GPIO_ReadPin(ZK_MISO_PORT, ZK_MISO_PIN)

/* ============================================================================
 * 接口
 * ========================================================================== */

void LCD_GPIO_Init(void);                                   /* 引脚初始化 */
void LCD_Writ_Bus(uint8_t dat);                             /* 软件 SPI 发一个字节 */
void LCD_WR_DATA8(uint8_t dat);                             /* 写 8 位数据 */
void LCD_WR_DATA(uint16_t dat);                             /* 写 16 位数据（颜色）*/
void LCD_WR_REG(uint8_t dat);                               /* 写命令 */
void LCD_Address_Set(uint16_t x1, uint16_t y1,
                     uint16_t x2, uint16_t y2);             /* 设置显示窗口 */
void LCD_Init(void);                                        /* 屏幕初始化 */
void LCD_SetMadctl(uint8_t v);                              /* 运行时改扫描方向 */

#endif /* __LCD_INIT_H */
