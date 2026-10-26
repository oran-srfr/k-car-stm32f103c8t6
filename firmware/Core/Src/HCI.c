/* ============================================================================
 * HCI.c —— 人机界面显示（1.8 寸 TFT / ST7735 / 128x160 竖屏）
 *
 * 目标：整段拷进任何工程即可用，需要对接的只有「移植层」那 3 个 wrapper。
 *
 * 屏幕布局（竖屏 128x160，汉字 15x16、ASCII 8x16）：
 *
 *     x=0        48      64    80   96
 *     y=2   [第一圈]      [ 12.3s ]          第一圈耗时
 *     y=22  [第二圈]      [ 11.8s ]          第二圈耗时
 *     y=46  [障碍识别]        [ 3][次]       障碍识别次数
 *     y=68  [倒车入库]        [ 1][次]       倒车入库次数
 *     y=90  [侧方停车]        [ 0][次]       侧方停车次数
 *
 * 设计要点：
 *   1. 5 个显示项就是 5 个全局变量，功能层直接赋值，HMI 不控车、不参与逻辑。
 *   2. 中文标签用 GBK 十六进制转义写死，源文件存 UTF-8 也不会乱码
 *      （字库地址依赖 GB2312 编码，直接写中文容易因编辑器编码不同而算错地址）。
 *   3. 不用 sprintf，手动拼字符，省 Flash。
 *   4. 100ms 节流 + 脏标记：只重绘变化的数值区，一行最多 6 字符 ≈ 48 像素。
 * ========================================================================== */

#include "HCI.h"

#include <string.h>

/* ----------------------------------------------------------------------------
 * 框架对接：下面这句按你的工程改。CubeMX 生成的工程一般是这个头文件。
 * 本文件只用到 HAL_GetTick() 一个函数（做 100ms 节流）。
 * -------------------------------------------------------------------------- */
#include "stm32f1xx_hal.h"

#ifndef HCI_GET_TICK
#define HCI_GET_TICK()      HAL_GetTick()
#endif

/* ============================================================================
 * 可配置参数
 * ========================================================================== */

/* 刷新周期（ms）。软件 SPI 刷屏慢，不要设太小 */
#ifndef HCI_REFRESH_MS
#define HCI_REFRESH_MS      100u
#endif

/* 0 = 不接屏幕，三个 wrapper 变成空函数，方便先在大框架里编译通过
 * 1 = 正常显示 */
#ifndef HCI_HAS_LCD
#define HCI_HAS_LCD         1
#endif

/* RGB565 颜色 */
#ifndef HCI_COLOR_LABEL
#define HCI_COLOR_LABEL     0xC618u     /* 浅灰：中文标签 */
#endif
#ifndef HCI_COLOR_VALUE
#define HCI_COLOR_VALUE     0x07E0u     /* 绿色：数值 */
#endif
#ifndef HCI_COLOR_BG
#define HCI_COLOR_BG        0x0000u     /* 黑色：背景 */
#endif

/* ============================================================================
 * 布局常量
 * ========================================================================== */

#ifndef HCI_W
#define HCI_W               128u
#endif
#ifndef HCI_H
#define HCI_H               160u
#endif

#define HCI_X_LABEL         0u      /* 中文标签左边缘 */
#define HCI_X_LAP_VAL       48u     /* 耗时数值：3 个汉字占 48px，从这里接上 */
#define HCI_X_CNT_VAL       64u     /* 计数数值：4 个汉字占 64px，从这里接上 */
#define HCI_X_UNIT          80u     /* 单位「次」 */

#define HCI_ROW1_Y          2u      /* 第一圈 */
#define HCI_ROW2_Y          22u     /* 第二圈 */
#define HCI_ROW3_Y          46u     /* 障碍识别 */
#define HCI_ROW4_Y          68u     /* 倒车入库 */
#define HCI_ROW5_Y          90u     /* 侧方停车 */

#define HCI_LAP_CHARS       6u      /* " 12.3s" 定宽，保证旧内容被完全覆盖 */
#define HCI_CNT_CHARS       2u      /* 计数 0~99 */

/* 参考包 zk.c 里的点阵编号 */
#define HCI_ZK_HZ           2u      /* Display_GB2312_String: 2 = 15x16 汉字 */
#define HCI_ZK_ASCII        4u      /* Display_Asc_String   : 4 = 8x16 ASCII */

/* ============================================================================
 * 五个全局变量
 * ========================================================================== */

uint32_t HCI_Lap1Ms    = 0u;
uint32_t HCI_Lap2Ms    = 0u;
uint16_t HCI_ObsCount  = 0u;
uint16_t HCI_RevCount  = 0u;
uint16_t HCI_SideCount = 0u;

/* ============================================================================
 * 中文标签：GBK 字节转义
 *
 *   第一圈   = B5DA D2BB C8A6
 *   第二圈   = B5DA B6FE C8A6
 *   障碍识别 = D5CF B0AD CAB6 B1F0
 *   倒车入库 = B5B9 B3B5 C8EB BFE2
 *   侧方停车 = B2E0 B7BD CDA3 B3B5
 *   次       = B4CE
 * ========================================================================== */

static unsigned char LBL_LAP1[] = "\xB5\xDA\xD2\xBB\xC8\xA6";          /* 第一圈 */
static unsigned char LBL_LAP2[] = "\xB5\xDA\xB6\xFE\xC8\xA6";          /* 第二圈 */
static unsigned char LBL_OBS[]  = "\xD5\xCF\xB0\xAD\xCA\xB6\xB1\xF0";  /* 障碍识别 */
static unsigned char LBL_REV[]  = "\xB5\xB9\xB3\xB5\xC8\xEB\xBF\xE2";  /* 倒车入库 */
static unsigned char LBL_SIDE[] = "\xB2\xE0\xB7\xBD\xCD\xA3\xB3\xB5";  /* 侧方停车 */
static unsigned char LBL_UNIT[] = "\xB4\xCE";                          /* 次 */

/* ============================================================================
 * 移植层 —— 整个文件只有这一段需要按你的框架改
 *
 * 下面 3 个 extern 对应参考包 f103_tft_ref 里的 lcd.c / zk.c。
 * 如果你框架里的函数名或参数不同，只改这 3 个 wrapper 的函数体，其余不用动。
 * ========================================================================== */

#if HCI_HAS_LCD

extern void LCD_Fill(uint16_t xsta, uint16_t ysta,
                     uint16_t xend, uint16_t yend,
                     uint16_t color);

extern void Display_GB2312_String(uint16_t x, uint16_t y,
                                  uint8_t zk_num, uint8_t text[],
                                  uint16_t fc, uint16_t bc);

extern void Display_Asc_String(uint16_t x, uint16_t y,
                               uint16_t zk_num, uint8_t text[],
                               uint16_t fc, uint16_t bc);

#endif /* HCI_HAS_LCD */

/* 整屏填充背景色 */
static void hci_clear(uint16_t bg)
{
#if HCI_HAS_LCD
    LCD_Fill(0u, 0u, HCI_W - 1u, HCI_H - 1u, bg);
#else
    (void)bg;
#endif
}

/* 画一串中文（GBK 编码，每字 15x16，步进 16px） */
static void hci_draw_hz(uint16_t x, uint16_t y, unsigned char *gbk, uint16_t color)
{
#if HCI_HAS_LCD
    Display_GB2312_String(x, y, (uint8_t)HCI_ZK_HZ, gbk,
                          color, (uint16_t)HCI_COLOR_BG);
#else
    (void)x; (void)y; (void)gbk; (void)color;
#endif
}

/* 画一串 ASCII（每字 8x16，步进 8px） */
static void hci_draw_ascii(uint16_t x, uint16_t y, char *ascii, uint16_t color)
{
#if HCI_HAS_LCD
    Display_Asc_String(x, y, (uint16_t)HCI_ZK_ASCII, (unsigned char *)ascii,
                       color, (uint16_t)HCI_COLOR_BG);
#else
    (void)x; (void)y; (void)ascii; (void)color;
#endif
}

/* ============================================================================
 * 格式化：手动拼字符，不依赖 sprintf
 * ========================================================================== */

/* 毫秒 -> 定宽 6 字符，右对齐，四舍五入到 0.1 秒
 *     0      -> " --.-s"   （未完成）
 *     12345  -> " 12.3s"
 *     596    -> "  0.6s"
 *     123456 -> "123.5s"
 * 超过 999.9s 按 999.9s 显示（一屏只有 3 位整数） */
static void hci_fmt_time(char *out, uint32_t ms)
{
    uint32_t t10;
    uint32_t ip;
    uint32_t fp;

    if (ms == 0u) {
        out[0] = ' ';
        out[1] = '-';
        out[2] = '-';
        out[3] = '.';
        out[4] = '-';
        out[5] = 's';
        out[6] = '\0';
        return;
    }

    t10 = (ms + 50u) / 100u;            /* 单位 0.1 秒，四舍五入 */
    if (t10 > 9999u) {
        t10 = 9999u;                    /* 饱和到 999.9s：先钳位再拆分，
                                         * 否则整数位截断、小数位残留会显示成错误数值 */
    }
    ip  = t10 / 10u;
    fp  = t10 % 10u;

    out[0] = (ip >= 100u) ? (char)('0' + (char)((ip / 100u) % 10u)) : ' ';
    out[1] = (ip >= 10u)  ? (char)('0' + (char)((ip / 10u)  % 10u)) : ' ';
    out[2] = (char)('0' + (char)(ip % 10u));
    out[3] = '.';
    out[4] = (char)('0' + (char)fp);
    out[5] = 's';
    out[6] = '\0';
}

/* 计数 -> 定宽 2 字符，右对齐，超过 99 按 99 显示
 *     0   -> " 0"
 *     7   -> " 7"
 *     42  -> "42"
 *     120 -> "99" */
static void hci_fmt_cnt(char *out, uint16_t v)
{
    if (v > 99u) {
        v = 99u;
    }
    out[0] = (v >= 10u) ? (char)('0' + (char)(v / 10u)) : ' ';
    out[1] = (char)('0' + (char)(v % 10u));
    out[2] = '\0';
}

/* ============================================================================
 * 脏标记：记录上次画到屏幕上的文本，值没变就不重绘
 * ========================================================================== */

static char     s_lap1[HCI_LAP_CHARS + 1u];
static char     s_lap2[HCI_LAP_CHARS + 1u];
static char     s_obs [HCI_CNT_CHARS + 1u];
static char     s_rev [HCI_CNT_CHARS + 1u];
static char     s_side[HCI_CNT_CHARS + 1u];
static uint32_t s_last_tick;

/* 一个耗时字段：格式化 -> 比对 -> 变了才画 */
static void hci_refresh_time(uint16_t y, uint32_t ms, char *cache)
{
    char buf[HCI_LAP_CHARS + 1u];

    hci_fmt_time(buf, ms);
    if (strcmp(buf, cache) == 0) {
        return;                          /* 值没变，不动屏幕 */
    }
    memcpy(cache, buf, sizeof(buf));
    hci_draw_ascii((uint16_t)HCI_X_LAP_VAL, y, buf, (uint16_t)HCI_COLOR_VALUE);
}

/* 一个计数字段 */
static void hci_refresh_cnt(uint16_t y, uint16_t v, char *cache)
{
    char buf[HCI_CNT_CHARS + 1u];

    hci_fmt_cnt(buf, v);
    if (strcmp(buf, cache) == 0) {
        return;
    }
    memcpy(cache, buf, sizeof(buf));
    hci_draw_ascii((uint16_t)HCI_X_CNT_VAL, y, buf, (uint16_t)HCI_COLOR_VALUE);
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void HCI_Init(void)
{
    /* 1) 清屏，避免 LCD_Init 之后残留花屏 */
    hci_clear((uint16_t)HCI_COLOR_BG);

    /* 2) 5 行中文标签（静态，只画这一次，之后不再重画） */
    hci_draw_hz((uint16_t)HCI_X_LABEL, (uint16_t)HCI_ROW1_Y, LBL_LAP1, (uint16_t)HCI_COLOR_LABEL);
    hci_draw_hz((uint16_t)HCI_X_LABEL, (uint16_t)HCI_ROW2_Y, LBL_LAP2, (uint16_t)HCI_COLOR_LABEL);
    hci_draw_hz((uint16_t)HCI_X_LABEL, (uint16_t)HCI_ROW3_Y, LBL_OBS,  (uint16_t)HCI_COLOR_LABEL);
    hci_draw_hz((uint16_t)HCI_X_LABEL, (uint16_t)HCI_ROW4_Y, LBL_REV,  (uint16_t)HCI_COLOR_LABEL);
    hci_draw_hz((uint16_t)HCI_X_LABEL, (uint16_t)HCI_ROW5_Y, LBL_SIDE, (uint16_t)HCI_COLOR_LABEL);

    /* 3) 三行计数后面的单位「次」 */
    hci_draw_hz((uint16_t)HCI_X_UNIT, (uint16_t)HCI_ROW3_Y, LBL_UNIT, (uint16_t)HCI_COLOR_LABEL);
    hci_draw_hz((uint16_t)HCI_X_UNIT, (uint16_t)HCI_ROW4_Y, LBL_UNIT, (uint16_t)HCI_COLOR_LABEL);
    hci_draw_hz((uint16_t)HCI_X_UNIT, (uint16_t)HCI_ROW5_Y, LBL_UNIT, (uint16_t)HCI_COLOR_LABEL);

    /* 4) 立刻把 5 个变量的当前值画出来 */
    HCI_ForceUpdate();
}

void HCI_ForceUpdate(void)
{
    memset(s_lap1, 0, sizeof(s_lap1));
    memset(s_lap2, 0, sizeof(s_lap2));
    memset(s_obs,  0, sizeof(s_obs));
    memset(s_rev,  0, sizeof(s_rev));
    memset(s_side, 0, sizeof(s_side));

    /* 把上次刷新时刻往前拨一个周期，让下面这次刷新不被节流挡掉 */
    s_last_tick = (uint32_t)HCI_GET_TICK() - (uint32_t)HCI_REFRESH_MS;

    HCI_Update();
}

void HCI_Update(void)
{
    uint32_t now;

    /* 100ms 节流：软件 SPI 刷屏慢，不能每个主循环周期都刷 */
    now = (uint32_t)HCI_GET_TICK();
    if ((uint32_t)(now - s_last_tick) < (uint32_t)HCI_REFRESH_MS) {
        return;
    }
    s_last_tick = now;

    /* 逐个字段刷新，值没变的不重绘（脏标记在 hci_refresh_* 内部） */
    hci_refresh_time((uint16_t)HCI_ROW1_Y, HCI_Lap1Ms, s_lap1);
    hci_refresh_time((uint16_t)HCI_ROW2_Y, HCI_Lap2Ms, s_lap2);
    hci_refresh_cnt((uint16_t)HCI_ROW3_Y, HCI_ObsCount,  s_obs);
    hci_refresh_cnt((uint16_t)HCI_ROW4_Y, HCI_RevCount,  s_rev);
    hci_refresh_cnt((uint16_t)HCI_ROW5_Y, HCI_SideCount, s_side);
}
