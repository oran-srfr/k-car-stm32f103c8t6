#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 直角处理（原地转向 v3）验证。
 *
 * 流程：丢线触发 → ①停稳 SETTLE 周期（两轮 0）
 *                  ②原地转向（左正右反或反之，占空比 PIVOT_DUTY）
 *                  ③连续 CONFIRM 周期看到线才算找到 → 恢复循迹
 *                  ④单段 LEG 超时 → 反向再扫一次
 *                  ⑤总时长超限 → 退回前进搜索（不判故障）
 */

enum { LO = 1U, LI = 2U, RI = 4U, RO = 8U, CENTER = LI | RI, LOST = 0U, ALL_BLACK = LI | RI | LO | RO };

static uint8_t g_raw;

LineSensorRaw_t LineSensor_ReadRaw(void)
{
    return (LineSensorRaw_t){ g_raw & 1U, (g_raw >> 1) & 1U,
                            (g_raw >> 2) & 1U, (g_raw >> 3) & 1U };
}

static const PDControllerConfig_t pd = {
    CONTROL_KP_DEFAULT, CONTROL_KD_DEFAULT, CONTROL_PD_OUTPUT_LIMIT_DEFAULT
};
static const MotorCommandConfig_t motor = {
    CONTROL_BASE_SPEED_DEFAULT, CONTROL_MAX_FORWARD_COMMAND_DEFAULT,
    CONTROL_MIN_FORWARD_COMMAND_DEFAULT, CONTROL_STRAIGHT_TRIM_DEFAULT
};
static const LineFollowConfig_t follow = {
    LINE_FOLLOW_CURVE_SPEED_DEFAULT, LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT,
    LINE_FOLLOW_SEARCH_SPEED_DEFAULT, LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT,
    LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT, LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT,
    LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT, LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT
};

static void set_pattern(unsigned mask)
{
    const uint8_t map[4] = {
        LINE_SENSOR_LEFT_OUTER_BIT, LINE_SENSOR_LEFT_INNER_BIT,
        LINE_SENSOR_RIGHT_INNER_BIT, LINE_SENSOR_RIGHT_OUTER_BIT
    };
    uint8_t raw = 0x0FU;

    for (unsigned i = 0; i < 4U; ++i) {
        if ((mask >> i) & 1U) {
            raw &= (uint8_t)~(1U << map[i]);
        }
    }
#if LINE_SENSOR_BLACK_ACTIVE_LOW == 0U
    raw ^= 0x0FU;
#endif
    g_raw = raw;
}

static LineFollowResult_t step(unsigned mask)
{
    set_pattern(mask);
    return LineFollow_Update(&pd, &motor, &follow);
}

static int is_pivot(LineFollowResult_t r, float dir)
{
    return (fabsf(r.left_command - (dir * LINE_FOLLOW_CORNER_PIVOT_DUTY)) < 0.5f) &&
           (fabsf(r.right_command + (dir * LINE_FOLLOW_CORNER_PIVOT_DUTY)) < 0.5f);
}

/* 进入直角前需要满足"丢线持续 ARM 周期"，封装成一个步骤 */
static LineFollowResult_t enter_corner(void)
{
    LineFollowResult_t r = step(LI);            /* 正常循迹，记录历史 */
    (void)r;
    (void)step(LO | LI | RI);                   /* 三灯特征：左拐 */
    (void)step(LOST);
    for (unsigned i = 1; i < LINE_FOLLOW_CORNER_ARM_CYCLES; ++i) {
        r = step(LOST);                         /* 丢线累计到 ARM 周期 */
    }
    return step(LOST);                          /* 本拍应进入直角 */
}

int main(void)
{
    LineFollowResult_t r = {0};

    /* ---- ① 进入直角：左拐特征是【两个内探头 + 左外探头】三灯同时亮 ---- */
    LineFollow_Start();
    r = enter_corner();                 /* 特征 + 丢线达 ARM → 进入直角 */
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.state == LINE_FOLLOW_CORNER);
    assert(r.left_command == 0.0f && r.right_command == 0.0f);   /* 停稳阶段 */

    /* ---- 停稳期间保持停车（enter_corner 已用掉 1 拍停稳） ---- */
    for (unsigned i = 1; i < LINE_FOLLOW_CORNER_SETTLE_CYCLES; ++i) {
        r = step(LOST);
        assert(r.state == LINE_FOLLOW_CORNER);
        assert(r.left_command == 0.0f && r.right_command == 0.0f);
    }

    /* ---- ①b 向前滚阶段（当前配置为 0，即关闭） ---- */
#if LINE_FOLLOW_CORNER_CREEP_CYCLES > 0
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_CREEP_CYCLES; ++i) {
        r = step(LOST);
        assert(r.state == LINE_FOLLOW_CORNER);
        assert(fabsf(r.left_command - LINE_FOLLOW_CORNER_CREEP_DUTY) < 0.5f);
        assert(fabsf(r.right_command - LINE_FOLLOW_CORNER_CREEP_DUTY) < 0.5f);
    }
#endif

    /* ---- ② 滚动结束 → 原地转向，朝"直角特征指示的方向"（左外亮 → 左转） ---- */
    r = step(LOST);
    assert(r.state == LINE_FOLLOW_CORNER);
    assert(is_pivot(r, -1.0f));

    /* ---- ③ 起转后的"闭眼期"：这段时间看到线也绝不退出 ----
     * 触发直角时探头往往还压在旧线附近，若立即认线，25 ms 后就退出，
     * 车只转了 5~10 度 —— 这正是"直角转不彻底"的根因。 */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_MIN_PIVOT_CYCLES; ++i) {
        r = step(CENTER);            /* 闭眼期内一直"看得见线" */
        assert(r.state == LINE_FOLLOW_CORNER);
        assert(is_pivot(r, -1.0f));
    }

    /* ---- ④ 闭眼期结束后，连续看到 CONFIRM 拍且图案在容差内 → 退出 ---- */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_CONFIRM_CYCLES; ++i) {
        r = step(CENTER);
    }
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.state != LINE_FOLLOW_CORNER);
    assert(r.left_command > 0.0f && r.right_command > 0.0f);

    /* ---- ⑤ 单段超时 → 方向翻转（反向再扫） ---- */
    LineFollow_Start();
    (void)enter_corner();
    for (unsigned i = 0;
         i < (LINE_FOLLOW_CORNER_SETTLE_CYCLES + LINE_FOLLOW_CORNER_CREEP_CYCLES +
              LINE_FOLLOW_CORNER_LEG_CYCLES + 2U);
         ++i) {
        r = step(LOST);
    }
    assert(r.state == LINE_FOLLOW_CORNER);
    assert(is_pivot(r, 1.0f));      /* 初始 -1，翻转为 +1 */

    /* ---- ⑥ 总时长超限 → 退回前进搜索，不判故障 ---- */
    LineFollow_Start();
    (void)enter_corner();
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_PIVOT_TOTAL_CYCLES + 5U; ++i) {
        r = step(LOST);
    }
    /* 超时那一拍只把状态切到 SEARCHING 并输出 0，下一拍才由丢线处理产生搜索命令 */
    r = step(LOST);
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.left_command > 0.0f && r.right_command > 0.0f);   /* 前进搜索 */

    /* ---- ⑨ 陀螺仪辅助：转角不足 BLIND_DEG 时即使看见线也不退出 ---- */
    LineFollow_Start();
    LineFollow_SetYaw(0.0f, 1U);        /* 陀螺仪有效，起始角 0 */
    (void)enter_corner();
    for (unsigned i = 0;
         i < (LINE_FOLLOW_CORNER_SETTLE_CYCLES + LINE_FOLLOW_CORNER_CREEP_CYCLES);
         ++i) {
        r = step(LOST);
    }
    r = step(LOST);                     /* 起转：记录 yaw_start = 0 */
    assert(is_pivot(r, -1.0f));

    /* 只转了 30 度 → 仍在闭眼期，看见线也不能退出 */
    LineFollow_SetYaw(30.0f, 1U);
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_CONFIRM_CYCLES + 5U; ++i) {
        r = step(CENTER);
        assert(r.state == LINE_FOLLOW_CORNER);
        assert(is_pivot(r, -1.0f));
    }

    /* 转到 60 度（超过 BLIND_DEG）→ 允许确认并退出 */
    LineFollow_SetYaw(60.0f, 1U);
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_CONFIRM_CYCLES; ++i) {
        r = step(CENTER);
    }
    assert(r.state != LINE_FOLLOW_CORNER);
    assert(r.left_command > 0.0f && r.right_command > 0.0f);

    /* ---- ⑩ 陀螺仪辅助：转角超过 MAX_DEG 仍看不到线 → 放弃转向，退回搜索 ---- */
    LineFollow_Start();
    LineFollow_SetYaw(0.0f, 1U);
    (void)enter_corner();
    for (unsigned i = 0;
         i < (LINE_FOLLOW_CORNER_SETTLE_CYCLES + LINE_FOLLOW_CORNER_CREEP_CYCLES + 2U);
         ++i) {
        r = step(LOST);
    }
    LineFollow_SetYaw(LINE_FOLLOW_CORNER_MAX_DEG + 5.0f, 1U);
    for (unsigned i = 0; i < 5U; ++i) {
        r = step(LOST);
    }
    r = step(LOST);                     /* 状态切到 SEARCHING 的下一拍产生搜索命令 */
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.state != LINE_FOLLOW_CORNER);
    /* ---- ⑬ 出弯冷却：刚转完的 800 ms 内，即使出现"特征 + 持续丢线"也不得再进直角 ----
     * 这是现场问题的回归测试：出直角时车斜着接新线，探头呈现"内+外"同侧图案，
     * 与进直角时一模一样，若不冷却会被误判成"又到一个直角"。 */
    LineFollow_Start();
    LineFollow_SetYaw(0.0f, 1U);
    (void)enter_corner();
    for (unsigned i = 0;
         i < (LINE_FOLLOW_CORNER_SETTLE_CYCLES + LINE_FOLLOW_CORNER_CREEP_CYCLES + 2U);
         ++i) {
        r = step(LOST);
    }
    LineFollow_SetYaw(LINE_FOLLOW_CORNER_TARGET_DEG + 2.0f, 1U);   /* 转够 → 退出直角 */
    r = step(LOST);
    assert(r.state != LINE_FOLLOW_CORNER);

    /* 紧接着制造一次"直角特征 + 长丢线" */
    (void)step(LI);
    (void)step(LO | LI | RI);
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_ARM_CYCLES + 5U; ++i) {
        r = step(LOST);
        assert(r.state != LINE_FOLLOW_CORNER);      /* 冷却期内不得触发 */
    }
    /* ---- ⑦ 没有直角特征的单纯丢线 → 不进直角，走前进搜索 ---- */
    LineFollow_Start();
    (void)step(LI);
    r = step(LOST);            /* 只丢线，没有三灯特征 */
    for (unsigned i = 0; i < 10U; ++i) {
        r = step(LOST);
    }
    assert(r.state != LINE_FOLLOW_CORNER);
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);

    /* ---- ⑧ 无历史就丢线 → 立刻故障，不盲目转身 ---- */
    LineFollow_Start();
    r = step(LOST);
    assert(r.fault == LINE_FOLLOW_FAULT_NO_VALID_HISTORY);
    assert(r.left_command == 0.0f && r.right_command == 0.0f);

    puts("PASS: corner settles, pivots toward the line, confirms, reverses and falls back");
    return 0;
}