#include "point_beep.h"

#include <math.h>

/* ===== 可调参数（全部属于本模块，不影响循迹/直角）===== */

/* 判定"开始转向"的角速度阈值（度/秒）。半圆 R60、车速 0.25 m/s 时
 * 角速度约 v/R = 0.25/0.6 ≈ 24°/s；直道上的噪声一般 < 8°/s。 */
#define POINT_BEEP_ARC_START_RATE_DPS    (12.0f)
/* 角速度持续超过阈值多久才算真的进弯（滤掉单拍抖动） */
#define POINT_BEEP_ARC_START_CYCLES      (60U)
/* 判定"转向停止"的角速度阈值 */
#define POINT_BEEP_ARC_STOP_RATE_DPS     (8.0f)
/* 停止持续多久才算出弯 */
#define POINT_BEEP_ARC_STOP_CYCLES       (80U)
/* 半圆 180°，留 15° 余量 */
#define POINT_BEEP_ARC_EXIT_TOTAL_DEG    (165.0f)
/* 弯道里最长时间（ms）：超时说明判定异常，直接放弃本次，等下一圈 */
#define POINT_BEEP_ARC_TIMEOUT_MS        (15000U)
/* 单拍 yaw 变化小于这个角度当作噪声（度） */
#define POINT_BEEP_YAW_DEADBAND_DEG      (0.05f)

static PointBeepStatus s_status;
static uint8_t  s_have_yaw;        /* 是否已有上一拍 yaw 作为差分基准 */
static float    s_yaw_prev;
static uint32_t s_yaw_prev_ms;
static uint16_t s_turn_cycles;     /* 连续"在转"的拍数 */
static uint16_t s_stop_cycles;     /* 连续"不转"的拍数 */
static uint32_t s_arc_start_ms;
/* 累计转角用 float 累加：每毫秒只有 0.02~0.05 度，
 * 若每拍都取整会全部变成 0，转角永远不增长（这是实测踩过的坑）。 */
static float    s_arc_deg_acc;

static float PointBeep_NormalizeDelta(float delta)
{
    while (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    while (delta < -180.0f)
    {
        delta += 360.0f;
    }
    return delta;
}

void PointBeep_Init(void)
{
    PointBeep_Reset();
}

void PointBeep_Reset(void)
{
    s_status = (PointBeepStatus){0};
    s_have_yaw = 0U;
    s_yaw_prev = 0.0f;
    s_yaw_prev_ms = 0U;
    s_turn_cycles = 0U;
    s_stop_cycles = 0U;
    s_arc_start_ms = 0U;
    s_arc_deg_acc = 0.0f;
}

PointBeepEvent PointBeep_Update(float yaw_deg, uint8_t live, uint32_t now_ms)
{
    float delta;
    float rate;
    uint8_t turning;

    if ((live == 0U) || !isfinite(yaw_deg))
    {
        /* 陀螺仪不可用：清掉差分基准，等恢复后重新开始 */
        s_have_yaw = 0U;
        return POINT_BEEP_NONE;
    }

    if (s_have_yaw == 0U)
    {
        s_have_yaw = 1U;
        s_yaw_prev = yaw_deg;
        s_yaw_prev_ms = now_ms;
        return POINT_BEEP_NONE;
    }

    {
        uint32_t dt = (uint32_t)(now_ms - s_yaw_prev_ms);

        if (dt == 0U)
        {
            return POINT_BEEP_NONE;
        }
        delta = PointBeep_NormalizeDelta(yaw_deg - s_yaw_prev);
        s_yaw_prev = yaw_deg;
        s_yaw_prev_ms = now_ms;
        rate = fabsf(delta) * 1000.0f / (float)dt;
    }

    /* 注意：死区只能用于"是否在转"的判断，【累加转角必须用原始 delta】。
     * 每毫秒的变化只有 0.02~0.05 度，若先过死区再累加会永远加不上（实测踩过）。 */
    turning = (uint8_t)((rate >= POINT_BEEP_ARC_START_RATE_DPS) ? 1U : 0U);

    /* ---- 还没进弯：等"持续转向" ---- */
    if (s_status.in_arc == 0U)
    {
        if (s_status.b_done != 0U)
        {
            return POINT_BEEP_NONE;   /* 本圈已经报过 B，不再重复 */
        }

        if (turning != 0U)
        {
            if (s_turn_cycles < UINT16_MAX)
            {
                ++s_turn_cycles;
            }
        }
        else
        {
            s_turn_cycles = 0U;
        }

        if (s_turn_cycles >= POINT_BEEP_ARC_START_CYCLES)
        {
            s_status.in_arc = 1U;
            s_status.b_done = 1U;
            s_status.arc_deg = 0U;
            s_arc_deg_acc = 0.0f;
            s_stop_cycles = 0U;
            s_arc_start_ms = now_ms;
            return POINT_BEEP_B;
        }
        return POINT_BEEP_NONE;
    }

    /* ---- 已在弯道里：累计转角，等"转过 165° 且停下" ---- */
    s_arc_deg_acc += fabsf(delta);
    {
        float shown = s_arc_deg_acc;

        if (shown > 999.0f)
        {
            shown = 999.0f;
        }
        s_status.arc_deg = (uint16_t)(shown + 0.5f);
    }

    if (rate < POINT_BEEP_ARC_STOP_RATE_DPS)
    {
        if (s_stop_cycles < UINT16_MAX)
        {
            ++s_stop_cycles;
        }
    }
    else
    {
        s_stop_cycles = 0U;
    }

    if ((s_arc_deg_acc >= POINT_BEEP_ARC_EXIT_TOTAL_DEG) &&
        (s_stop_cycles >= POINT_BEEP_ARC_STOP_CYCLES) &&
        (s_status.c_done == 0U))
    {
        s_status.in_arc = 0U;
        s_status.c_done = 1U;
        s_turn_cycles = 0U;
        return POINT_BEEP_C;
    }

    /* 兜底：弯道卡太久，放弃本次（下一圈 PointBeep_Reset 后重来） */
    if ((uint32_t)(now_ms - s_arc_start_ms) > POINT_BEEP_ARC_TIMEOUT_MS)
    {
        s_status.in_arc = 0U;
        s_turn_cycles = 0U;
        s_arc_deg_acc = 0.0f;
    }

    return POINT_BEEP_NONE;
}

void PointBeep_GetStatus(PointBeepStatus *out)
{
    if (out == NULL)
    {
        return;
    }
    *out = s_status;
}
