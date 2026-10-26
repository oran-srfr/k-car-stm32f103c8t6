/* ============================================================================
 * zk.h —— 外部字库芯片驱动（GB2312 汉字）
 *
 * 芯片读法：SPI 模式 0x03 读命令 + 24 位地址，然后连续读 N 字节。
 * 片选和 LCD 共用（拉高选字库），所以读写时不能同时进行。
 *
 * 地址公式（缺一不可，很多人只写第二个分支，结果全角标点全乱码）：
 *   汉字（MSB 0xB0~0xF7）：addr = ((MSB-0xB0)*94 + (LSB-0xA1) + 846)*n + BaseAdd
 *   符号（MSB 0xA1~0xA9）：addr = ((MSB-0xA1)*94 + (LSB-0xA1))      *n + BaseAdd
 * ========================================================================== */

#ifndef __ZK_H
#define __ZK_H

#include "lcd_init.h"

/* 一个汉字最多 32x32 = 128 字节，留 130 余量 */
extern uint8_t FontBuf[130];

/* 向字库芯片写一个字节（命令/地址） */
void ZK_command(uint8_t dat);

/* 从字库芯片读一个字节 */
uint8_t get_data_from_ROM(void);

/* 从指定地址读 N 字节到 pBuff */
void get_n_bytes_data_from_ROM(uint8_t AddrHigh, uint8_t AddrMid, uint8_t AddrLow,
                               uint8_t *pBuff, uint8_t DataLen);

/* 把 FontBuf 里的点阵画到 (x,y) */
void Display_GB2312(uint16_t x, uint16_t y, uint8_t zk_num, uint16_t fc, uint16_t bc);

/* 显示 GBK 编码的汉字串
 * zk_num: 1 = 12x12, 2 = 15x16, 3 = 24x24, 4 = 32x32 */
void Display_GB2312_String(uint16_t x, uint16_t y, uint8_t zk_num,
                           uint8_t text[], uint16_t fc, uint16_t bc);

/* 显示 ASCII 串
 * 注意：本实现走本地 lcdfont.h 的 8x16 点阵，不读字库芯片
 *      —— 数字和时间是核心显示内容，不能因为字库芯片异常就全黑 */
void Display_Asc_String(uint16_t x, uint16_t y, uint16_t zk_num,
                        uint8_t text[], uint16_t fc, uint16_t bc);

/* ---------------- 自检用（不参与正常显示） ---------------- */
/* 从字库芯片读一个字的点阵到 dst，返回实际读到的字节数。
 * 用于上板验证字库芯片是否接对：全 0x00 或全 0xFF 说明没读到。 */
uint8_t ZK_Probe(uint8_t msb, uint8_t lsb, uint8_t zk_num, uint8_t *dst, uint8_t cap);

/* 总线诊断：把 MISO 线的 4 种可能问题一次全测出来
 *   r[0] = 片选高（本工程假定"高=选字库"）时读到的字节数
 *   r[1] = 上述数据里非 0xFF 的个数（0 = 全读成高，没数据）
 *   r[2] = 片选低（反相）时读到的字节数
 *   r[3] = 上述数据里非 0xFF 的个数
 *   r[4] = MISO 加下拉后读到的电平（1 = 有东西在驱动高）
 *   r[5] = MISO 加上拉后读到的电平（0 = 有东西在驱动低）
 *   r[6] = 片选高时的非 0x00 个数（排除"全低"情况）
 * 判读：
 *   r[4]=1 且 r[5]=1  → 线悬空（MISO 没接或接错脚），上拉/下拉都能改变电平
 *   r[1]>0             → 片选"高"是对的，芯片正常
 *   r[3]>0 且 r[1]=0   → 片选极性反了，应改成"低=选字库"
 *   r[1]=0 且 r[3]=0 且 r[4]==r[5] → 线被钉死，芯片没供电或型号不符
 */
void ZK_BusTest(uint32_t *r);

/* 反相片选模式下的原始数据（给 ZK_BusTest 用，方便外部读内存查看） */
extern uint8_t ZK_InvertBuf[32];

/* ---------------------------------------------------------------------------
 * 引脚扫描：把 31 个候选引脚逐个当 MISO 读一遍字库，看哪个脚能读到真实数据
 *
 * 用途：SDO 飞线接错脚时，不用拆线也不用猜，直接扫出来是哪一个。
 * dst 需要 31 个 uint16_t，每个元素 = 该脚读到的「非 0xFF 字节数」。
 * 正常（接对且有字库芯片）的那个脚，这个数应该在 8~31 之间；其余全是 0。
 *
 * 下标对应关系：
 *   0..12  -> PA0..PA12
 *   13     -> PA15            (跳过 PA13/PA14，那是 SWD 调试口，动了会断连)
 *   14..29 -> PB0..PB15
 *   30     -> PC13
 * ------------------------------------------------------------------------- */
#define ZK_SCAN_COUNT       31u
void ZK_PinScan(uint16_t *dst);
const char *ZK_ScanPinName(uint8_t idx);

#endif /* __ZK_H */
