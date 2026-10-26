/* ============================================================================
 * MpuLink.c —— MPU6050 串口姿态模块解析实现
 *
 * 分工：
 *   中断上下文：MpuLink_RxCpltCallback() 只做「把 1 个字节推进环形缓冲 + 重新武装接收」
 *   主循环：   MpuLink_Poll() 做状态机 + 校验和 + 浮点换算
 *
 * 这样中断里最坏只有几十条指令，不会影响编码器脉冲采集。
 * ========================================================================== */

#include "MpuLink.h"

#include "stm32f1xx_hal.h"

#include <string.h>

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static uint8_t s_ring[MPU_RING_SIZE];
static volatile uint16_t s_head;      /* 写指针（中断里改）*/
static volatile uint16_t s_tail;      /* 读指针（主循环改）*/

static uint8_t s_buf[MPU_FRAME_LEN];  /* 当前正在拼的帧 */
static uint8_t s_idx;                 /* 已收字节数 */

static float     s_yaw_deg;           /* 最新 Yaw */
static uint32_t  s_yaw_ms;            /* 最新 Yaw 的收帧时刻 */
static uint32_t  s_last_ok_ms;        /* 最近一次校验通过的时刻 */
static uint32_t  s_now_ms;            /* Poll 传入的当前时刻 */

/* Yaw 待取队列：保证每一帧都能被取走，不丢转角 */
static float     s_qy[MPU_YAW_QUEUE];
static uint32_t  s_qt[MPU_YAW_QUEUE];
static uint8_t   s_qhead;
static uint8_t   s_qtail;

static MpuLinkStats s_stats;

static UART_HandleTypeDef *s_huart;
static uint8_t s_rx_byte;             /* HAL 单字节接收缓冲 */

/* ============================================================================
 * 环形缓冲
 * ========================================================================== */

static void ring_push(uint8_t b)
{
    uint16_t next = (uint16_t)((s_head + 1u) & (MPU_RING_SIZE - 1u));

    s_stats.bytes_total++;

    if (next == s_tail) {
        /* 满了：丢掉这个字节。计数便于发现主循环太慢 */
        s_stats.ring_overflow++;
        return;
    }
    s_ring[s_head] = b;
    s_head = next;
}

static bool ring_pop(uint8_t *b)
{
    if (s_tail == s_head) {
        return false;
    }
    *b = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) & (MPU_RING_SIZE - 1u));
    return true;
}

/* ----------------------------------------------------------------------------
 * Yaw 待取队列
 * -------------------------------------------------------------------------- */
static void yawq_push(float yaw, uint32_t t_ms)
{
    uint8_t next = (uint8_t)((s_qhead + 1u) & (MPU_YAW_QUEUE - 1u));

    if (next == s_qtail) {
        /* 队列满：丢掉最旧的一帧，保留最新数据（并计数，便于发现问题） */
        s_qtail = (uint8_t)((s_qtail + 1u) & (MPU_YAW_QUEUE - 1u));
        s_stats.frames_dropped++;
    }
    s_qy[s_qhead] = yaw;
    s_qt[s_qhead] = t_ms;
    s_qhead = next;
}

static bool yawq_pop(float *yaw, uint32_t *t_ms)
{
    if (s_qtail == s_qhead) {
        return false;
    }
    *yaw  = s_qy[s_qtail];
    *t_ms = s_qt[s_qtail];
    s_qtail = (uint8_t)((s_qtail + 1u) & (MPU_YAW_QUEUE - 1u));
    return true;
}

/* ============================================================================
 * 换算
 * ========================================================================== */

float MpuLink_RawToDeg(int16_t raw)
{
    /* raw 范围 -32768 ~ 32767 对应 -180° ~ +180° */
    return ((float)raw / 32768.0f) * 180.0f;
}

/* ============================================================================
 * 帧解析（流式状态机）
 * ========================================================================== */

static void mpu_handle_frame(uint32_t now_ms)
{
    uint8_t i;
    uint8_t sum = 0;
    int16_t raw;

    for (i = 0; i < (MPU_FRAME_LEN - 1u); i++) {
        sum = (uint8_t)(sum + s_buf[i]);
    }

    if (sum != s_buf[MPU_FRAME_LEN - 1u]) {
        s_stats.frames_bad++;
        return;
    }

    /* Yaw 是小端：低位在前 */
    raw = (int16_t)(uint16_t)(((uint16_t)s_buf[7] << 8) | (uint16_t)s_buf[6]);

    s_yaw_deg    = MpuLink_RawToDeg(raw);
    s_yaw_ms     = now_ms;
    s_last_ok_ms = now_ms;
    s_stats.frames_ok++;

    yawq_push(s_yaw_deg, s_yaw_ms);       /* 逐帧入队，保证不丢转角 */
}

static void mpu_parse_byte(uint8_t b, uint32_t now_ms)
{
    if (s_idx == 0u) {
        /* 等帧头 */
        if (b == MPU_HEAD) {
            s_buf[s_idx++] = b;
        }
        return;
    }

    if (s_idx == 1u) {
        if (b == MPU_TYPE_ANGLE) {
            s_buf[s_idx++] = b;
        } else if (b != MPU_HEAD) {
            /* 连续两个 0x55：把第二个当成新的帧头，继续保持 idx=1 */
            s_idx = 0;
        }
        /* b == 0x55 时什么都不做，等它的类型字节 */
        return;
    }

    s_buf[s_idx++] = b;

    if (s_idx >= MPU_FRAME_LEN) {
        s_idx = 0;                      /* 先复位，避免回调里重入出错 */
        mpu_handle_frame(now_ms);
    }
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void MpuLink_Init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    memset((void *)s_ring, 0, sizeof(s_ring));

    s_head = 0u;
    s_tail = 0u;
    s_idx  = 0u;

    s_yaw_deg    = 0.0f;
    s_yaw_ms     = 0u;
    s_last_ok_ms = 0u;
    s_now_ms     = 0u;

    s_qhead = 0u;
    s_qtail = 0u;

    s_huart   = NULL;
    s_rx_byte = 0u;
}

void MpuLink_AttachUart(void *huart)
{
    s_huart = (UART_HandleTypeDef *)huart;
    if (s_huart != NULL) {
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

void MpuLink_RxCpltCallback(void)
{
    /* 中断上下文：只入缓冲，然后立刻重新武装下一次单字节接收 */
    ring_push(s_rx_byte);

    if (s_huart != NULL) {
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

void MpuLink_ErrorCallback(void)
{
    /* 出错（溢出/噪声）后接收会停，这里重新武装。
     * 具体错误标志由 HAL 在进入回调前清过，这里只管恢复接收。 */
    if (s_huart != NULL) {
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

void MpuLink_FeedByte(uint8_t b)
{
    ring_push(b);
}

void MpuLink_Poll(uint32_t now_ms)
{
    uint8_t b;

    s_now_ms = now_ms;

    while (ring_pop(&b)) {
        mpu_parse_byte(b, now_ms);
    }
}

bool MpuLink_TakeYaw(float *yaw_deg, uint32_t *t_ms)
{
    float    y;
    uint32_t t;

    if (!yawq_pop(&y, &t)) {
        return false;
    }

    if (yaw_deg != NULL) {
        *yaw_deg = y;
    }
    if (t_ms != NULL) {
        *t_ms = t;
    }
    return true;
}

void MpuLink_GetStats(MpuLinkStats *out)
{
    if (out == NULL) {
        return;
    }

    *out = s_stats;

    /* live 用「距上次有效帧的时间」判断，避免主循环里再算一遍 */
    if (s_stats.frames_ok == 0u) {
        out->live = false;
    } else {
        out->live = ((uint32_t)(s_now_ms - s_last_ok_ms) < MPU_LIVE_TIMEOUT_MS);
    }
}
