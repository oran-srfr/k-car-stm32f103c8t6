/* ============================================================================
 * zk.c —— 外部字库芯片驱动（GB2312 汉字，15x16）
 *
 * 从参考包 zk.c 移植，改动：
 *   1. StdPeriph -> HAL（引脚宏在 lcd_init.h，和 LCD 共用 CS）
 *   2. Display_Asc_String 改为走本地 8x16 点阵（不读芯片），
 *      保证数字/时间在没有字库芯片时也能显示
 *   3. 增加 ZK_Probe() 用于上板自检
 *
 * 时序要点（写和读的相位不同，不能合并成一个函数）：
 *   写：SCLK 拉低 -> 设置 MOSI -> SCLK 拉高
 *   读：SCLK 拉低 -> 读 MISO -> SCLK 拉高
 * ========================================================================== */

#include "zk.h"
#include "lcd.h"

uint8_t FontBuf[130];
uint8_t ZK_InvertBuf[32];

/* 片选极性开关：0 = 拉高选字库（本工程默认，与参考包一致）
 *               1 = 拉低选字库（诊断时用） */
static uint8_t s_zk_cs_invert = 0;

/* ----------------------------------------------------------------------------
 * 向字库芯片写一个字节（命令或地址）
 * -------------------------------------------------------------------------- */
void ZK_command(uint8_t dat)
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

/* ----------------------------------------------------------------------------
 * 从字库芯片读一个字节
 * -------------------------------------------------------------------------- */
uint8_t get_data_from_ROM(void)
{
    uint8_t i;
    uint8_t ret = 0;

    for (i = 0; i < 8; i++) {
        LCD_SCLK_Clr();
        ret <<= 1;
        if (ZK_MISO) {
            ret++;
        }
        LCD_SCLK_Set();
    }
    return ret;
}

/* ----------------------------------------------------------------------------
 * 读 N 个字节
 * -------------------------------------------------------------------------- */
void get_n_bytes_data_from_ROM(uint8_t AddrHigh, uint8_t AddrMid, uint8_t AddrLow,
                               uint8_t *pBuff, uint8_t DataLen)
{
    uint8_t i;

    /* 选中字库芯片。正常模式 CS 拉高；诊断模式 CS 拉低 */
    if (s_zk_cs_invert) {
        LCD_CS_Clr();
    } else {
        ZK_CS_Clr();
    }

    ZK_command(0x03);               /* 读命令 */
    ZK_command(AddrHigh);
    ZK_command(AddrMid);
    ZK_command(AddrLow);
    for (i = 0; i < DataLen; i++) {
        pBuff[i] = get_data_from_ROM();
    }

    /* 取消选中，回到 LCD */
    if (s_zk_cs_invert) {
        LCD_CS_Set();
    } else {
        ZK_CS_Set();
    }
}

/* ----------------------------------------------------------------------------
 * 把 FontBuf 里的点阵画出来
 * zk_num: 1 = 12x12, 2 = 15x16, 3 = 24x24, 4 = 32x32
 * -------------------------------------------------------------------------- */
void Display_GB2312(uint16_t x, uint16_t y, uint8_t zk_num, uint16_t fc, uint16_t bc)
{
    uint8_t  i, k;
    uint16_t n;
    uint8_t  w;
    uint16_t hi_f = (uint16_t)(fc >> 8), lo_f = (uint16_t)(fc & 0xFF);
    uint16_t hi_b = (uint16_t)(bc >> 8), lo_b = (uint16_t)(bc & 0xFF);

    switch (zk_num) {
    case 1:  w = 16; n = 24;  break;    /* 12x12：每行 2 字节，12 行 */
    case 3:  w = 24; n = 72;  break;    /* 24x24：每行 3 字节，24 行 */
    case 4:  w = 32; n = 128; break;    /* 32x32：每行 4 字节，32 行 */
    case 2:
    default: w = 16; n = 32;  break;    /* 15x16：每行 2 字节，16 行（字形 15 宽，末列空）*/
    }

    LCD_Address_Set(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + w - 1));

    LCD_CS_Clr();
    for (i = 0; i < n; i++) {
        for (k = 0; k < 8; k++) {
            if (FontBuf[i] & (0x80 >> k)) {
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
 * 计算一个字在字库芯片里的地址
 * -------------------------------------------------------------------------- */
static uint32_t zk_calc_addr(uint8_t msb, uint8_t lsb, uint8_t zk_num)
{
    uint32_t base;
    uint16_t n;
    uint32_t idx;

    switch (zk_num) {
    case 1:  base = 0x000000UL; n = 24;  break;
    case 3:  base = 0x068190UL; n = 72;  break;
    case 4:  base = 0x0EDF00UL; n = 128; break;
    case 2:
    default: base = 0x02C9D0UL; n = 32;  break;
    }

    if (msb >= 0xA1 && msb <= 0xA9) {
        /* GB2312 符号区：没有 +846 */
        idx = (uint32_t)(msb - 0xA1) * 94UL + (uint32_t)(lsb - 0xA1);
    } else {
        /* GB2312 汉字区：要 +846（前面有 9 个符号区，9*94=846） */
        idx = (uint32_t)(msb - 0xB0) * 94UL + (uint32_t)(lsb - 0xA1) + 846UL;
    }

    return idx * (uint32_t)n + base;
}

/* ----------------------------------------------------------------------------
 * 显示 GBK 汉字串
 * -------------------------------------------------------------------------- */
void Display_GB2312_String(uint16_t x, uint16_t y, uint8_t zk_num,
                           uint8_t text[], uint16_t fc, uint16_t bc)
{
    uint8_t  i = 0;
    uint8_t  d;
    uint16_t n;
    uint32_t addr;
    uint32_t pcx = x;

    switch (zk_num) {
    case 1:  d = 12; n = 24;  break;
    case 3:  d = 24; n = 72;  break;
    case 4:  d = 32; n = 128; break;
    case 2:
    default: d = 16; n = 32;  break;
    }

    while (text[i] != 0) {
        /* 不是合法的 GBK 双字节（比如混了 ASCII），直接跳过 1 字节 */
        if (text[i] < 0xA1 || text[i + 1] == 0) {
            i++;
            continue;
        }

        addr = zk_calc_addr(text[i], text[i + 1], zk_num);
        get_n_bytes_data_from_ROM((uint8_t)((addr & 0xFF0000UL) >> 16),
                                  (uint8_t)((addr & 0x00FF00UL) >> 8),
                                  (uint8_t)(addr & 0x0000FFUL),
                                  FontBuf, (uint8_t)n);
        Display_GB2312(pcx, y, zk_num, fc, bc);

        pcx += d;
        i += 2;
    }
}

/* ----------------------------------------------------------------------------
 * 显示 ASCII 串：走本地 8x16 点阵，不读字库芯片
 * -------------------------------------------------------------------------- */
void Display_Asc_String(uint16_t x, uint16_t y, uint16_t zk_num,
                        uint8_t text[], uint16_t fc, uint16_t bc)
{
    (void)zk_num;                   /* 统一按 8x16 画 */
    LCD_ShowString(x, y, text, fc, bc, 16, 0);
}

/* ============================================================================
 * 引脚扫描：找出 SDO 到底接在哪个脚上
 * ========================================================================== */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} zk_cand_t;

static const zk_cand_t s_cand[ZK_SCAN_COUNT] = {
    { GPIOA, GPIO_PIN_0,  "PA0"  }, { GPIOA, GPIO_PIN_1,  "PA1"  },
    { GPIOA, GPIO_PIN_2,  "PA2"  }, { GPIOA, GPIO_PIN_3,  "PA3"  },
    { GPIOA, GPIO_PIN_4,  "PA4"  }, { GPIOA, GPIO_PIN_5,  "PA5"  },
    { GPIOA, GPIO_PIN_6,  "PA6"  }, { GPIOA, GPIO_PIN_7,  "PA7"  },
    { GPIOA, GPIO_PIN_8,  "PA8"  }, { GPIOA, GPIO_PIN_9,  "PA9"  },
    { GPIOA, GPIO_PIN_10, "PA10" }, { GPIOA, GPIO_PIN_11, "PA11" },
    { GPIOA, GPIO_PIN_12, "PA12" }, { GPIOA, GPIO_PIN_15, "PA15" },
    { GPIOB, GPIO_PIN_0,  "PB0"  }, { GPIOB, GPIO_PIN_1,  "PB1"  },
    { GPIOB, GPIO_PIN_2,  "PB2"  }, { GPIOB, GPIO_PIN_3,  "PB3"  },
    { GPIOB, GPIO_PIN_4,  "PB4"  }, { GPIOB, GPIO_PIN_5,  "PB5"  },
    { GPIOB, GPIO_PIN_6,  "PB6"  }, { GPIOB, GPIO_PIN_7,  "PB7"  },
    { GPIOB, GPIO_PIN_8,  "PB8"  }, { GPIOB, GPIO_PIN_9,  "PB9"  },
    { GPIOB, GPIO_PIN_10, "PB10" }, { GPIOB, GPIO_PIN_11, "PB11" },
    { GPIOB, GPIO_PIN_12, "PB12" }, { GPIOB, GPIO_PIN_13, "PB13" },
    { GPIOB, GPIO_PIN_14, "PB14" }, { GPIOB, GPIO_PIN_15, "PB15" },
    { GPIOC, GPIO_PIN_13, "PC13" },
};

const char *ZK_ScanPinName(uint8_t idx)
{
    if (idx < ZK_SCAN_COUNT) {
        return s_cand[idx].name;
    }
    return "?";
}

/* 用运行时指定的引脚读一个字节 */
static uint8_t zk_read_byte_at(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t i;
    uint8_t r = 0;

    for (i = 0; i < 8; i++) {
        LCD_SCLK_Clr();
        r <<= 1;
        if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_RESET) {
            r++;
        }
        LCD_SCLK_Set();
    }
    return r;
}

void ZK_PinScan(uint16_t *dst)
{
    uint8_t  i, k;
    uint32_t addr;
    uint8_t  buf[32];
    uint8_t  notFF;
    GPIO_InitTypeDef g;

    /* 「第」字 15x16 的地址 */
    addr = zk_calc_addr(0xB5, 0xDA, 2);

    for (i = 0; i < ZK_SCAN_COUNT; i++) {
        dst[i] = 0;

        /* 先恢复 SPI 那几根线的正常状态，再把候选脚设为上拉输入。
         * （候选脚如果本身是 SCLK/MOSI，这一步会把它抢过来，
         *   但那几根脚本来就不是 MISO，扫出来是 0 也无所谓） */
        LCD_GPIO_Init();

        g.Pin   = s_cand[i].pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(s_cand[i].port, &g);

        HAL_Delay(1);

        /* 选中字库芯片（CS 拉高），发 0x03 + 24 位地址，读 32 字节 */
        ZK_CS_Clr();
        ZK_command(0x03);
        ZK_command((uint8_t)((addr & 0xFF0000UL) >> 16));
        ZK_command((uint8_t)((addr & 0x00FF00UL) >> 8));
        ZK_command((uint8_t)(addr & 0x0000FFUL));
        for (k = 0; k < 32; k++) {
            buf[k] = zk_read_byte_at(s_cand[i].port, s_cand[i].pin);
        }
        ZK_CS_Set();

        notFF = 0;
        for (k = 0; k < 32; k++) {
            if (buf[k] != 0xFF) {
                notFF++;
            }
        }
        dst[i] = (uint16_t)notFF;
    }
}

/* ============================================================================
 * 总线诊断
 * ========================================================================== */

/* 重新配置 MISO 的上拉/下拉 */
static void zk_miso_pull(uint32_t pull)
{
    GPIO_InitTypeDef g;

    g.Pin   = ZK_MISO_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = pull;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ZK_MISO_PORT, &g);
}

static uint8_t zk_count_not(uint8_t *buf, uint8_t len, uint8_t value)
{
    uint8_t i, c = 0;
    for (i = 0; i < len; i++) {
        if (buf[i] != value) {
            c++;
        }
    }
    return c;
}

void ZK_BusTest(uint32_t *r)
{
    /* ---- 测试 1：MISO 线是否悬空 ----
     * 悬空时，内部下拉会让它读 0，内部上拉会让它读 1。
     * 如果下拉读到 1、上拉读到 1，说明有东西一直把它拉高（或根本没接，靠上拉）。 */

    zk_miso_pull(GPIO_PULLDOWN);
    HAL_Delay(2);
    r[4] = (uint32_t)ZK_MISO;       /* 下拉时应为 0 */

    zk_miso_pull(GPIO_PULLUP);
    HAL_Delay(2);
    r[5] = (uint32_t)ZK_MISO;       /* 上拉时应为 1 */

    /* ---- 测试 2：正常片选（高=选字库）读「第」字 ---- */
    s_zk_cs_invert = 0;
    r[0] = (uint32_t)ZK_Probe(0xB5, 0xDA, 2, FontBuf, 32);
    r[1] = (uint32_t)zk_count_not(FontBuf, (uint8_t)r[0], 0xFF);
    r[6] = (uint32_t)zk_count_not(FontBuf, (uint8_t)r[0], 0x00);

    /* ---- 测试 3：反相片选（低=选字库）读同一个字 ---- */
    s_zk_cs_invert = 1;
    r[2] = (uint32_t)ZK_Probe(0xB5, 0xDA, 2, ZK_InvertBuf, 32);
    r[3] = (uint32_t)zk_count_not(ZK_InvertBuf, (uint8_t)r[2], 0xFF);

    /* 恢复默认极性，恢复正常显示 */
    s_zk_cs_invert = 0;
    zk_miso_pull(GPIO_PULLUP);
}
uint8_t ZK_Probe(uint8_t msb, uint8_t lsb, uint8_t zk_num, uint8_t *dst, uint8_t cap)
{
    uint16_t n;
    uint32_t addr;
    uint8_t  len;

    switch (zk_num) {
    case 1:  n = 24;  break;
    case 3:  n = 72;  break;
    case 4:  n = 128; break;
    case 2:
    default: n = 32;  break;
    }

    if (n > cap) {
        len = cap;
    } else {
        len = (uint8_t)n;
    }

    addr = zk_calc_addr(msb, lsb, zk_num);
    get_n_bytes_data_from_ROM((uint8_t)((addr & 0xFF0000UL) >> 16),
                              (uint8_t)((addr & 0x00FF00UL) >> 8),
                              (uint8_t)(addr & 0x0000FFUL),
                              dst, len);
    return len;
}
