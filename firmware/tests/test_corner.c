#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 直角处理（重写版）验证。
 *
 * 判据（三灯）：
 *   看到"两个内探头 + 一侧外探头"同时亮（左外亮=左拐，右外亮=右拐），
 *   且之后 SIG_WINDOW 内丢线、丢线持续 >= ARM、本次没试过、不在冷却期
 * 执行：停稳 -> 原地转向（朝最后看到线的一侧）
 *   完成条件只认【陀螺仪到角】；陀螺仪无数据时退化为固定时长
 *   没有"反向再扫"，也没有"看到线就退出"
 */

enum { LO = 1U, LI = 2U, RI = 4U, RO = 8U, CENTER = LI | RI, LOST = 0U };

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

/* 触发直角：先看到"两内 + 一侧外"的三灯特征，再丢线到 ARM 周期。
 * 左拐用 LO|LI|RI，右拐用 LI|RI|RO。 */
static LineFollowResult_t enter_corner(unsigned three_light_pattern)
{
    LineFollowResult_t r = step(three_light_pattern);
    r = step(three_light_pattern);          /* 连续两拍，确保被记录 */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_ARM_CYCLES + 2U; ++i) {
        r = step(LOST);
    }
    return r;
}

static int is_pivot(LineFollowResult_t r, float dir)
{
    return (fabsf(r.left_command - (dir * LINE_FOLLOW_CORNER_PIVOT_DUTY)) < 0.5f) &&
           (fabsf(r.right_command + (dir * LINE_FOLLOW_CORNER_PIVOT_DUTY)) < 0.5f);
}

int main(void)
{
    LineFollowResult_t r = {0};
    const unsigned PASS_SETTLE = LINE_FOLLOW_CORNER_STOP_CYCLES +
                                 LINE_FOLLOW_CORNER_SETTLE_CYCLES + 2U;

    /* ---- ① 三灯特征（左外+左内+右内）后丢线 → 进入直角，先反向刹车 ---- */
    LineFollow_Start();
    r = enter_corner(LO | LI | RI);       /* 三灯：左拐特征 */
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.state == LINE_FOLLOW_CORNER);
    assert(fabsf(r.left_command + LINE_FOLLOW_CORNER_STOP_DUTY) < 0.5f);
    assert(fabsf(r.right_command + LINE_FOLLOW_CORNER_STOP_DUTY) < 0.5f);

    /* ---- ② 刹车结束 → 静止 → 原地左转（朝最后看到线的一侧） ---- */
    for (unsigned i = 0; i < PASS_SETTLE; ++i) {
        r = step(LOST);
    }
    assert(r.state == LINE_FOLLOW_CORNER);
    assert(is_pivot(r, -1.0f));

    /* ---- ③ 转向期间"看到线"也不能退出（不看线） ---- */
    for (unsigned i = 0; i < 200U; ++i) {
        r = step(CENTER);
        assert(r.state == LINE_FOLLOW_CORNER);
        assert(is_pivot(r, -1.0f));
    }

    /* ---- ④ 无陀螺仪：转够固定时长 → 先刹车（两轮反向）→ 再收工 ----
     * 循环上限给足，且不依赖"已经转了多少拍"（前面用例已经走过一段）。
     * 同时验证确实出现过刹车脉冲（左转时左轮为负）。 */
    {
        int saw_brake = 0;
        const unsigned limit = LINE_FOLLOW_CORNER_TIME_PIVOT_CYCLES +
                               LINE_FOLLOW_CORNER_BRAKE_CYCLES + 400U;

        for (unsigned i = 0; i < limit; ++i) {
            r = step(LOST);
            if ((r.state == LINE_FOLLOW_CORNER) && (r.left_command < 0.0f)) {
                saw_brake = 1;
            }
            if (r.state != LINE_FOLLOW_CORNER) {
                break;
            }
        }
        assert(r.state != LINE_FOLLOW_CORNER);
        assert(saw_brake == 1);
    }

    /* ---- ⑤ 陀螺仪有效：到 TARGET_DEG 收工 ---- */
    LineFollow_Start();
    LineFollow_SetYaw(0.0f, 1U);
    (void)enter_corner(LO | LI | RI);
    for (unsigned i = 0; i < PASS_SETTLE; ++i) {
        r = step(LOST);
    }
    assert(is_pivot(r, -1.0f));
    LineFollow_SetYaw(LINE_FOLLOW_CORNER_TARGET_DEG - 5.0f, 1U);
    r = step(LOST);
    assert(r.state == LINE_FOLLOW_CORNER);          /* 还差 5 度，继续转 */
    LineFollow_SetYaw(LINE_FOLLOW_CORNER_TARGET_DEG + 1.0f, 1U);
    r = step(LOST);
    assert(r.state == LINE_FOLLOW_CORNER);          /* 到角 → 进入刹车 */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_BRAKE_CYCLES + 4U; ++i) {
        r = step(LOST);
    }
    assert(r.state != LINE_FOLLOW_CORNER);          /* 刹车结束 → 收工 */

    /* ---- ⑥ 陀螺仪"卡住不动"（角度永远到不了目标）→ 绝对上限必须收工 ----
     * 这是真实存在的风险：陀螺仪中途停止更新、live 还来不及置 0 时，
     * 角度会冻结。若只有"到角"和"超角"两个条件，就会一直转下去。 */
    LineFollow_Start();
    LineFollow_SetYaw(0.0f, 1U);
    (void)enter_corner(LO | LI | RI);
    for (unsigned i = 0; i < PASS_SETTLE; ++i) {
        r = step(LOST);
    }
    LineFollow_SetYaw(45.0f, 1U);          /* 卡在 45 度不动 */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_MAX_CYCLES + 5U; ++i) {
        r = step(LOST);
    }
    assert(r.state != LINE_FOLLOW_CORNER);  /* 绝对上限收工 */
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);

    /* ---- ⑦ 判据是【三灯】：只出现"两灯"（左外+左内）时不得触发 ---- */
    LineFollow_Start();
    (void)step(LO | LI);
    (void)step(LO | LI);
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_ARM_CYCLES + 10U; ++i) {
        r = step(LOST);
        assert(r.state != LINE_FOLLOW_CORNER);
    }
    assert(r.left_command > 0.0f && r.right_command > 0.0f);

    /* ---- ⑧ 冷却期：刚完成直角后，同样的"峰值 + 丢线"不得再触发 ---- */
    LineFollow_Start();
    LineFollow_SetYaw(0.0f, 1U);
    (void)enter_corner(LO | LI | RI);
    for (unsigned i = 0; i < PASS_SETTLE; ++i) {
        r = step(LOST);
    }
    LineFollow_SetYaw(LINE_FOLLOW_CORNER_TARGET_DEG + 1.0f, 1U);
    (void)step(LOST);
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_BRAKE_CYCLES + 2U; ++i) {
        r = step(LOST);
    }
    assert(r.state != LINE_FOLLOW_CORNER);
    for (unsigned i = 0; i < 50U; ++i) {   /* 重新看见线，解除门闩 */
        r = step(LI);
    }
    (void)step(LO | LI);                    /* 又出现外侧峰值 */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_ARM_CYCLES + 5U; ++i) {
        r = step(LOST);
        assert(r.state != LINE_FOLLOW_CORNER);   /* 冷却期内不得触发 */
    }

    /* ---- ⑨ 无历史就丢线 → 立刻故障 ---- */
    LineFollow_Start();
    r = step(LOST);
    assert(r.fault == LINE_FOLLOW_FAULT_NO_VALID_HISTORY);
    assert(r.left_command == 0.0f && r.right_command == 0.0f);

    puts("PASS: three-light signature triggers corner, pivots without reversing, ends by angle");
    return 0;
}