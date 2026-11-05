#include "point_beep.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* B/C 点鸣笛判定验证
 *
 * 场地：A→B 直道 85cm，B→C 为 R60 半圆（180°），顺时针（全是右转）。
 * 判定：B = 开始持续转向；C = 累计转过 >=165° 且转向停止。
 */

static uint32_t g_now;
static float g_yaw;

static void begin(void)
{
    PointBeep_Init();
    g_now = 0U;
    g_yaw = 0.0f;
}

/* 以指定角速度（度/秒）转 ms 毫秒，返回这段时间里收到的事件 */
static uint8_t spin(float rate_dps, uint32_t ms, float *emitted)
{
    uint8_t events = 0U;

    for (uint32_t i = 0U; i < ms; ++i)
    {
        g_now += 1U;
        g_yaw += rate_dps / 1000.0f;
        if (g_yaw > 180.0f)
        {
            g_yaw -= 360.0f;
        }
        {
            uint8_t ev = (uint8_t)PointBeep_Update(g_yaw, 1U, g_now);
            if (ev != 0U)
            {
                ++events;
                if (emitted != NULL)
                {
                    *emitted = (float)ev;
                }
            }
        }
    }
    return events;
}

/* 直行（角速度 0）ms 毫秒，返回事件数 */
static uint8_t straight(uint32_t ms)
{
    return spin(0.0f, ms, NULL);
}

int main(void)
{
    PointBeepStatus st;

    /* ---- ① 直道上不动（噪声级角速度 3°/s）→ 不响 ---- */
    begin();
    assert(spin(3.0f, 2000U, NULL) == 0U);
    assert(spin(-3.0f, 2000U, NULL) == 0U);
    PointBeep_GetStatus(&st);
    assert(st.b_done == 0U && st.c_done == 0U);

    /* ---- ② 进入 R60 半圆（24°/s）→ 在进弯后 ~60ms 响 B ---- */
    begin();
    (void)straight(500U);
    {
        float ev = 0.0f;
        uint8_t n = spin(24.0f, 300U, &ev);
        assert(n == 1U);
        assert(ev == (float)POINT_BEEP_B);
    }
    PointBeep_GetStatus(&st);
    assert(st.b_done == 1U);
    assert(st.in_arc == 1U);

    /* ---- ③ 走完半圆（约 180°）后转入直行 → 响 C ---- */
    {
        float ev = 0.0f;
        uint8_t n;

        /* 24°/s 走 7.6 秒 ≈ 182°，但仍在转 → 还不能响 C */
        n = spin(24.0f, 7600U, &ev);
        assert(n == 0U);

        /* 出弯转直行 → 停止持续 80ms 后响 C */
        n = spin(0.0f, 200U, &ev);
        assert(n == 1U);
        assert(ev == (float)POINT_BEEP_C);
    }
    PointBeep_GetStatus(&st);
    assert(st.c_done == 1U);
    assert(st.in_arc == 0U);
    assert(st.arc_deg >= 165U);

    /* ---- ④ C 之后继续直行 → 不再重复响 ---- */
    assert(straight(3000U) == 0U);

    /* ---- ⑤ Reset 后可以重新识别下一圈 ---- */
    PointBeep_Reset();
    PointBeep_GetStatus(&st);
    assert(st.b_done == 0U && st.c_done == 0U && st.in_arc == 0U && st.arc_deg == 0U);
    {
        float ev = 0.0f;
        assert(spin(24.0f, 300U, &ev) == 1U);
        assert(ev == (float)POINT_BEEP_B);
    }

    /* ---- ⑥ 只转了一小段（50°）就直行 → 不响 C（不是半圆） ---- */
    begin();
    (void)spin(24.0f, 300U, NULL);      /* B */
    (void)spin(24.0f, 2100U, NULL);     /* ≈50° */
    assert(straight(1000U) == 0U);      /* 停下来了，但转角不够 */
    PointBeep_GetStatus(&st);
    assert(st.c_done == 0U);

    /* ---- ⑦ 陀螺仪掉线 → 不响，也不锁死 ---- */
    begin();
    (void)spin(24.0f, 300U, NULL);
    for (uint32_t i = 0U; i < 500U; ++i)
    {
        g_now += 1U;
        assert(PointBeep_Update(g_yaw, 0U, g_now) == POINT_BEEP_NONE);
    }

    /* ---- ⑧ 逆时针（左转半圆，容错用）也能识别 ---- */
    begin();
    (void)straight(300U);
    {
        float ev = 0.0f;
        assert(spin(-24.0f, 300U, &ev) == 1U);
        assert(ev == (float)POINT_BEEP_B);
    }
    {
        float ev = 0.0f;
        assert(spin(-24.0f, 7600U, &ev) == 0U);
        assert(spin(0.0f, 200U, &ev) == 1U);
        assert(ev == (float)POINT_BEEP_C);
    }

    puts("PASS: point beep triggers at arc entry (B) and after ~165deg (C), once per lap");
    return 0;
}
