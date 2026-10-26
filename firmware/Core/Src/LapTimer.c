/* ============================================================================
 * LapTimer.c —— 圈计时器实现
 *
 * 核心是三件事：
 *   1. 解卷绕：Yaw 在 ±180° 处会跳变，必须按「增量」累加，不能直接比较角度
 *   2. 死区 + 残余累计：慢速转弯每帧增量极小，不能丢，要攒着
 *   3. 防护：坏帧（NaN/Inf）、断流（帧间隔过大）不能污染累积角
 *
 * 全部计算用 float。Cortex-M3 没有 FPU，但本模块只在主循环里跑（100Hz），
 * 每帧几十条浮点指令，72MHz 下约 10~20µs，完全够用。
 * ========================================================================== */

#include "LapTimer.h"

#include <math.h>
#include <string.h>

typedef struct {
    bool     running;
    bool     finished;
    uint8_t  laps;

    bool     has_last;          /* 是否已有上一帧基准 */
    float    last_yaw;          /* 上一帧 Yaw */
    uint32_t t_last_ms;         /* 上一帧时刻 */

    float    accum_deg;         /* 累积转角 */
    float    pending_deg;       /* 待并入的残余角 */
    float    rate_dps;          /* 最近一帧角速度 */

    uint32_t t_start_ms;        /* 发车时刻 */
    uint32_t t_mark_ms;         /* 上一圈完成时刻 */
    uint32_t lap1_ms;
    uint32_t lap2_ms;
    uint32_t elapsed_ms;
} LapState;

static LapState s;

/* ----------------------------------------------------------------------------
 * 清空累积相关状态
 * -------------------------------------------------------------------------- */
static void lap_clear_accum(void)
{
    s.accum_deg   = 0.0f;
    s.pending_deg = 0.0f;
    s.rate_dps    = 0.0f;
    s.has_last    = false;
}

void Lap_Init(void)
{
    memset(&s, 0, sizeof(s));
}

void Lap_Start(uint32_t now_ms)
{
    memset(&s, 0, sizeof(s));
    s.running    = true;
    s.t_start_ms = now_ms;
    s.t_mark_ms  = now_ms;
    lap_clear_accum();
}

void Lap_Stop(void)
{
    s.running  = false;
    s.has_last = false;
}

/* ----------------------------------------------------------------------------
 * 判圈：累积角每满 360° 记一圈
 * -------------------------------------------------------------------------- */
static void lap_check_progress(uint32_t now_ms)
{
    while (s.laps < LAP_TARGET_LAPS) {
        float target = 360.0f * (float)(s.laps + 1u);
        if (fabsf(s.accum_deg) < target) {
            break;
        }

        s.laps++;
        if (s.laps == 1u) {
            s.lap1_ms  = now_ms - s.t_start_ms;     /* 第一圈单独耗时 */
            s.t_mark_ms = now_ms;
        } else {
            s.lap2_ms   = now_ms - s.t_mark_ms;     /* 第二圈单独耗时 */
            s.elapsed_ms = now_ms - s.t_start_ms;
            s.finished  = true;
        }
    }

    if (s.finished) {
        /* 停表：之后不再累积，防止停车时车身晃动被判出第 3 圈 */
        s.running  = false;
        s.has_last = false;
    }
}

void Lap_FeedYaw(float yaw_deg, uint32_t now_ms)
{
    uint32_t dt_ms;
    float    d;
    float    rate;

    if (!s.running) {
        return;
    }

    /* ---- 坏帧保护：必须在任何运算之前 ----
     * · ±Inf 会让下面的 while 循环死循环（Inf - 360 仍是 Inf）
     * · NaN 一旦写进 last_yaw，之后每帧增量都是 NaN，判圈永久失效 */
    if (!(yaw_deg > -LAP_YAW_ABS_MAX && yaw_deg < LAP_YAW_ABS_MAX)) {
        s.has_last = false;                 /* 丢基准，下一帧重建 */
        return;
    }

    if (!s.has_last) {
        s.last_yaw  = yaw_deg;
        s.t_last_ms = now_ms;
        s.has_last  = true;
        return;
    }

    dt_ms = now_ms - s.t_last_ms;
    if (dt_ms == 0u) {
        return;
    }
    if (dt_ms > LAP_DT_MAX_MS) {
        /* 数据断流：这段时间转了多少无法得知，直接以当前角度为新基准 */
        s.last_yaw  = yaw_deg;
        s.t_last_ms = now_ms;
        return;
    }

    /* ---- 解卷绕 ---- */
    d = yaw_deg - s.last_yaw;
    while (d > 180.0f) {
        d -= 360.0f;
    }
    while (d < -180.0f) {
        d += 360.0f;
    }
    if (!(d >= -180.0f && d <= 180.0f)) {
        s.has_last = false;
        return;
    }

    s.last_yaw  = yaw_deg;
    s.t_last_ms = now_ms;

    /* ---- 残余累计 + 死区 ----
     * 关键：单帧增量再小也要攒起来（pending），不能直接丢。
     * 只有确认「真的静止」（角速度远低于阈值）时才把残余慢慢泄掉。 */
    rate = fabsf(d) * 1000.0f / (float)dt_ms;
    s.rate_dps = rate;
    s.pending_deg += d;

    if ((fabsf(s.pending_deg) >= LAP_FLUSH_DEG) || (rate >= LAP_FAST_DPS)) {
        s.accum_deg  += s.pending_deg;
        s.pending_deg = 0.0f;
    } else if (rate < LAP_REST_DPS) {
        s.pending_deg *= LAP_REST_LEAK;
    }

    lap_check_progress(now_ms);
}

void Lap_Tick(uint32_t now_ms)
{
    if (s.running) {
        s.elapsed_ms = now_ms - s.t_start_ms;
    }
}

void Lap_GetStatus(LapStatus *out)
{
    if (out == NULL) {
        return;
    }
    out->running     = s.running;
    out->finished    = s.finished;
    out->laps        = s.laps;
    out->lap1_ms     = s.lap1_ms;
    out->lap2_ms     = s.lap2_ms;
    out->elapsed_ms  = s.elapsed_ms;
    out->accum_deg   = s.accum_deg;
    out->pending_deg = s.pending_deg;
    out->rate_dps    = s.rate_dps;
}

bool Lap_IsFinished(void)
{
    return s.finished;
}
