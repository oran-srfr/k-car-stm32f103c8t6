/* MpuLink 离线测试：不接硬件，手工构造 0x55 0x53 帧验证解析 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "MpuLink.h"
#include "stm32f1xx_hal.h"   /* 测试目录下的桩，只有 UART 相关的最小定义 */

/* ---- HAL 桩：记录被调用次数 ---- */
static int g_rx_arm_count;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t size)
{
    (void)h; (void)data; (void)size;
    g_rx_arm_count++;
    return HAL_OK;
}

static int fails;

static void expect_true(const char *what, int cond)
{
    if (!cond) { printf("FAIL %s\n", what); fails++; }
    else       { printf("ok   %s\n", what); }
}

static void expect_near(const char *what, double got, double want, double tol)
{
    double d = got - want;
    if (d < 0) d = -d;
    if (d > tol) {
        printf("FAIL %-32s got=%.2f want=%.2f\n", what, got, want);
        fails++;
    } else {
        printf("ok   %-32s got=%.2f\n", what, got);
    }
}

static void expect_u32(const char *what, uint32_t got, uint32_t want)
{
    if (got != want) {
        printf("FAIL %-32s got=%lu want=%lu\n", what,
               (unsigned long)got, (unsigned long)want);
        fails++;
    } else {
        printf("ok   %-32s %lu\n", what, (unsigned long)got);
    }
}

/* 构造一帧角度包并喂进去。yaw_raw 为 16 位原始值 */
static void feed_frame(int16_t yaw_raw, int16_t roll_raw, int16_t pitch_raw, int corrupt)
{
    uint8_t f[MPU_FRAME_LEN];
    uint8_t i;
    uint8_t sum = 0;

    f[0] = 0x55;
    f[1] = 0x53;
    f[2] = (uint8_t)(roll_raw & 0xFF);
    f[3] = (uint8_t)((roll_raw >> 8) & 0xFF);
    f[4] = (uint8_t)(pitch_raw & 0xFF);
    f[5] = (uint8_t)((pitch_raw >> 8) & 0xFF);
    f[6] = (uint8_t)(yaw_raw & 0xFF);
    f[7] = (uint8_t)((yaw_raw >> 8) & 0xFF);
    f[8] = 0x00;
    f[9] = 0x00;

    for (i = 0; i < (MPU_FRAME_LEN - 1u); i++) {
        sum = (uint8_t)(sum + f[i]);
    }
    f[10] = corrupt ? (uint8_t)(sum ^ 0xFFu) : sum;

    for (i = 0; i < MPU_FRAME_LEN; i++) {
        MpuLink_FeedByte(f[i]);
    }
}

/* 把角度换算成原始值 */
static int16_t deg_to_raw(float deg)
{
    return (int16_t)(deg / 180.0f * 32768.0f);
}

int main(void)
{
    float yaw;
    uint32_t t;
    MpuLinkStats st;
    UART_HandleTypeDef fake_uart;

    /* ---------- 1. 正常帧 ---------- */
    MpuLink_Init();
    feed_frame(deg_to_raw(90.0f), 0, 0, 0);
    MpuLink_Poll(1000u);
    expect_true("normal: got yaw", MpuLink_TakeYaw(&yaw, &t));
    expect_near("normal: yaw == 90", yaw, 90.0, 0.01);
    expect_u32("normal: timestamp", t, 1000u);
    expect_true("normal: no more frame", !MpuLink_TakeYaw(&yaw, &t));

    MpuLink_GetStats(&st);
    expect_u32("normal: frames_ok", st.frames_ok, 1u);
    expect_u32("normal: frames_bad", st.frames_bad, 0u);
    expect_true("normal: live", st.live);

    /* ---------- 2. 校验和错误必须丢弃 ---------- */
    MpuLink_Init();
    feed_frame(deg_to_raw(45.0f), 0, 0, 1);
    MpuLink_Poll(1000u);
    expect_true("bad sum: rejected", !MpuLink_TakeYaw(&yaw, &t));
    MpuLink_GetStats(&st);
    expect_u32("bad sum: frames_bad", st.frames_bad, 1u);
    expect_u32("bad sum: frames_ok", st.frames_ok, 0u);

    /* ---------- 3. 前面塞垃圾字节，必须能重新同步 ---------- */
    MpuLink_Init();
    {
        uint8_t junk[] = { 0x00, 0x12, 0xFF, 0x55, 0x11, 0xAA };
        unsigned i;
        for (i = 0; i < sizeof(junk); i++) {
            MpuLink_FeedByte(junk[i]);
        }
    }
    feed_frame(deg_to_raw(-45.0f), 0, 0, 0);
    MpuLink_Poll(2000u);
    expect_true("junk: resync ok", MpuLink_TakeYaw(&yaw, &t));
    expect_near("junk: yaw == -45", yaw, -45.0, 0.01);

    /* ---------- 4. 连续 0x55 0x55 也要能同步 ---------- */
    MpuLink_Init();
    MpuLink_FeedByte(0x55);
    MpuLink_FeedByte(0x55);
    {
        uint8_t f[MPU_FRAME_LEN];
        int16_t raw = deg_to_raw(30.0f);
        uint8_t i, sum = 0;
        f[0] = 0x55; f[1] = 0x53; f[2] = 0; f[3] = 0; f[4] = 0; f[5] = 0;
        f[6] = (uint8_t)(raw & 0xFF);
        f[7] = (uint8_t)((raw >> 8) & 0xFF);
        f[8] = 0; f[9] = 0;
        for (i = 0; i < 10u; i++) sum = (uint8_t)(sum + f[i]);
        f[10] = sum;
        /* 跳过 f[0] 的 0x55（等效于第二个 0x55 当帧头）*/
        for (i = 1; i < MPU_FRAME_LEN; i++) MpuLink_FeedByte(f[i]);
    }
    MpuLink_Poll(3000u);
    expect_true("double head: parsed", MpuLink_TakeYaw(&yaw, &t));
    expect_near("double head: yaw == 30", yaw, 30.0, 0.01);

    /* ---------- 5. 负角度与边界 ---------- */
    expect_near("raw->deg 32767", MpuLink_RawToDeg(32767), 179.9945, 0.01);
    expect_near("raw->deg 0",     MpuLink_RawToDeg(0),     0.0,     0.001);
    expect_near("raw->deg -32768", MpuLink_RawToDeg((int16_t)-32768), -180.0, 0.01);

    /* ---------- 6. 连续多帧：要能逐帧取完 ---------- */
    MpuLink_Init();
    feed_frame(deg_to_raw(10.0f), 0, 0, 0);
    feed_frame(deg_to_raw(20.0f), 0, 0, 0);
    feed_frame(deg_to_raw(30.0f), 0, 0, 0);
    MpuLink_Poll(4000u);
    {
        int n = 0;
        float last = 0.0f;
        while (MpuLink_TakeYaw(&yaw, &t)) {
            n++;
            last = yaw;
        }
        expect_u32("multi: frame count", (uint32_t)n, 3u);
        expect_near("multi: last yaw == 30", last, 30.0, 0.01);
    }

    /* ---------- 7. 环形缓冲溢出要计数 ---------- */
    MpuLink_Init();
    {
        unsigned i;
        for (i = 0; i < (MPU_RING_SIZE + 50u); i++) {
            MpuLink_FeedByte((uint8_t)i);
        }
        MpuLink_GetStats(&st);
        expect_true("overflow: counted", st.ring_overflow > 0u);
        expect_u32("overflow: bytes_total", st.bytes_total, MPU_RING_SIZE + 50u);
    }

    /* ---------- 8. live 标志 ---------- */
    MpuLink_Init();
    feed_frame(deg_to_raw(0.0f), 0, 0, 0);
    MpuLink_Poll(10000u);
    MpuLink_GetStats(&st);
    expect_true("live: fresh", st.live);

    MpuLink_Poll(10000u + MPU_LIVE_TIMEOUT_MS + 100u);
    MpuLink_GetStats(&st);
    expect_true("live: expired after timeout", !st.live);

    /* ---------- 9. AttachUart 要启动接收 ---------- */
    MpuLink_Init();
    g_rx_arm_count = 0;
    MpuLink_AttachUart(&fake_uart);
    expect_u32("attach: arm rx", (uint32_t)g_rx_arm_count, 1u);

    g_rx_arm_count = 0;
    MpuLink_RxCpltCallback();
    expect_u32("rxcb: re-arm rx", (uint32_t)g_rx_arm_count, 1u);

    printf("\n");
    if (fails) { printf("%d CHECK(S) FAILED\n", fails); }
    else       { printf("ALL CHECKS PASSED\n"); }
    return fails ? 1 : 0;
}
