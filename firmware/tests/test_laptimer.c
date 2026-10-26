/* LapTimer 离线测试：不依赖 HAL，纯算法验证 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "LapTimer.h"

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
        printf("FAIL %-30s got=%.1f want=%.1f tol=%.1f\n", what, got, want, tol);
        fails++;
    } else {
        printf("ok   %-30s got=%.0f want=%.0f\n", what, got, want);
    }
}

static float wrap180(float deg)
{
    while (deg > 180.0f)   deg -= 360.0f;
    while (deg < -180.0f)  deg += 360.0f;
    return deg;
}

/* 以固定角速度转 dur_ms，从时刻 t0 开始，每 10ms 喂一帧（模拟 100Hz 模块） */
static unsigned spin(float dps, unsigned dur_ms, unsigned t0, float *raw)
{
    unsigned t;
    for (t = 10u; t <= dur_ms; t += 10u) {
        *raw += dps * 0.01f;
        Lap_FeedYaw(wrap180(*raw), t0 + t);
    }
    return t0 + dur_ms;
}

int main(void)
{
    LapStatus st;

    /* ---------- 1. 慢速 10°/s：每帧只转 0.1°，考验残余累计 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        float raw = 0.0f;
        unsigned t;
        t = spin(10.0f, 36500u, 0u, &raw);
        Lap_GetStatus(&st);
        expect_near("slow: lap1_ms", st.lap1_ms, 36000.0, 500.0);
        expect_true("slow: laps == 1", st.laps == 1u);
        expect_true("slow: not finished", !st.finished);

        t = spin(10.0f, 36500u, t, &raw);
        Lap_GetStatus(&st);
        expect_near("slow: lap2_ms", st.lap2_ms, 36000.0, 500.0);
        expect_true("slow: finished", st.finished);
    }

    /* ---------- 2. 快速 30°/s ---------- */
    Lap_Init();
    Lap_Start(1000u);
    {
        float raw = 0.0f;
        unsigned t;
        t = spin(30.0f, 12300u, 1000u, &raw);
        t = spin(30.0f, 12300u, t, &raw);
        Lap_GetStatus(&st);
        expect_near("fast: lap1_ms", st.lap1_ms, 12000.0, 300.0);
        expect_near("fast: lap2_ms", st.lap2_ms, 12000.0, 300.0);
        expect_true("fast: finished", st.finished);
    }

    /* ---------- 3. 反向（顺时针）也要算圈 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        float raw = 0.0f;
        (void)spin(-40.0f, 9500u, 0u, &raw);
        Lap_GetStatus(&st);
        expect_near("cw: lap1_ms", st.lap1_ms, 9000.0, 400.0);
        expect_true("cw: accum negative", st.accum_deg < -300.0f);
    }

    /* ---------- 4. 静止抖动：不能判出圈 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        unsigned t;
        float raw = 0.0f;
        for (t = 10u; t <= 120000u; t += 10u) {
            float j = ((t / 10u) & 1u) ? 0.03f : -0.03f;
            raw = wrap180(raw + j);
            Lap_FeedYaw(raw, t);
        }
        Lap_GetStatus(&st);
        expect_true("jitter: accum < 1 deg", fabsf(st.accum_deg) < 1.0f);
        expect_true("jitter: no lap", st.laps == 0u);
    }

    /* ---------- 5. 跑满两圈后继续转，不能再累加 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        float raw = 0.0f;
        unsigned t;
        t = spin(60.0f, 6500u, 0u, &raw);
        t = spin(60.0f, 6500u, t, &raw);
        t = spin(60.0f, 12000u, t, &raw);      /* 再转两圈 */
        Lap_GetStatus(&st);
        expect_true("finish: laps == 2", st.laps == 2u);
        expect_near("finish: lap2 unchanged", st.lap2_ms, 6000.0, 300.0);
    }

    /* ---------- 6. 坏帧 NaN / ±Inf 不能污染状态 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        float raw = 0.0f;
        unsigned t;
        Lap_FeedYaw(NAN, 10u);
        Lap_FeedYaw(INFINITY, 20u);
        Lap_FeedYaw(-INFINITY, 30u);
        t = spin(30.0f, 12300u, 40u, &raw);
        t = spin(30.0f, 12300u, t, &raw);
        Lap_GetStatus(&st);
        expect_true("bad frame: survived", st.laps == 2u && st.finished);
    }

    /* ---------- 7. 断流 600ms：不能算出巨大增量 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        float raw = 0.0f;
        unsigned t;
        t = spin(30.0f, 1000u, 0u, &raw);       /* 转 1 秒 */
        Lap_FeedYaw(wrap180(raw + 170.0f), t + 600u);  /* 断流期间角度跳了 170° */
        t += 610u;
        Lap_GetStatus(&st);
        expect_true("dropout: accum still small", fabsf(st.accum_deg) < 60.0f);
        expect_true("dropout: no false lap", st.laps == 0u);
    }

    /* ---------- 8. ±180° 跳变处必须正确解卷绕 ---------- */
    Lap_Init();
    Lap_Start(0u);
    {
        /* 从 170° 开始，缓慢转到 -170°（跨越 180° 边界）*/
        float raw = 170.0f;
        unsigned t;
        float dps = 20.0f;
        for (t = 10u; t <= 2000u; t += 10u) {
            raw = wrap180(raw + dps * 0.01f);
            Lap_FeedYaw(raw, t);
        }
        Lap_GetStatus(&st);
        /* 20°/s × 2s = 40°，跨边界后累积应仍约 40°（不是 -320°）*/
        expect_near("wrap: accum ~40 deg", fabsf(st.accum_deg), 40.0, 6.0);
    }

    printf("\n");
    if (fails) { printf("%d CHECK(S) FAILED\n", fails); }
    else       { printf("ALL CHECKS PASSED\n"); }
    return fails ? 1 : 0;
}
