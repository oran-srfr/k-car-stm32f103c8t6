#include "line_follow.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

enum {LEFT_OUTER=1, LEFT_PAIR=3, LEFT_INNER=2, CENTER=6,
      RIGHT_INNER=4, RIGHT_PAIR=12, RIGHT_OUTER=8, LOST=0, ALL_BLACK=15};
static uint8_t raw_bits;
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

/* Feed actual X1..X4 electrical levels through the production sampling stack. */
LineSensorRaw_t LineSensor_ReadRaw(void)
{
    return (LineSensorRaw_t){raw_bits & 1U, (raw_bits >> 1) & 1U,
                            (raw_bits >> 2) & 1U, (raw_bits >> 3) & 1U};
}

/* bits use a physical left-to-right bitmap: bit0=left outer. */
static uint8_t physical_to_raw(uint8_t black)
{
    const uint8_t mapping[] = {LINE_SENSOR_LEFT_OUTER_BIT, LINE_SENSOR_LEFT_INNER_BIT,
                               LINE_SENSOR_RIGHT_INNER_BIT, LINE_SENSOR_RIGHT_OUTER_BIT};
    uint8_t raw = 0x0F;
    for (unsigned i = 0; i < 4; ++i) {
        if (black & (1U << i)) raw &= ~(1U << mapping[i]);
    }
#if LINE_SENSOR_BLACK_ACTIVE_LOW == 0U
    raw ^= 0x0F;
#endif
    return raw;
}

static LineFollowResult_t step(uint8_t bits)
{
    raw_bits = physical_to_raw(bits);
    return LineFollow_Update(&pd, &motor, &follow);
}

static void moving(LineFollowResult_t r)
{
    /* 上下界引用配置宏：这些值随调参变化（MAX_FORWARD 已经 770 → 900 → 930），
     * 写死数字会让测试在调参后误报。
     * 容差 0.5：MotorCommand_Calculate 里 base 先夹到 min+|c|、再算 base-c，
     * 两次浮点运算后可能得到 399.99997 而非 400.0，语义上等于下限。 */
    const float tol = 0.5f;

    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.left_command >= CONTROL_MIN_FORWARD_COMMAND_DEFAULT - tol);
    assert(r.left_command <= CONTROL_MAX_FORWARD_COMMAND_DEFAULT + tol);
    assert(r.right_command >= CONTROL_MIN_FORWARD_COMMAND_DEFAULT - tol);
    assert(r.right_command <= CONTROL_MAX_FORWARD_COMMAND_DEFAULT + tol);
}

static void stopped(LineFollowResult_t r)
{
    assert(r.left_command == 0.0f && r.right_command == 0.0f);
}

static void test_geometry_and_small_corrections(void)
{
    const uint8_t patterns[] = {LEFT_OUTER, LEFT_PAIR, LEFT_INNER, CENTER, RIGHT_INNER, RIGHT_PAIR, RIGHT_OUTER};
    const float errors[] = {-3, -2, -1, 0, 1, 2, 3};
    for (unsigned i = 0; i < sizeof(patterns); ++i) {
        LineFollow_Start();
        LineFollowResult_t r = step(patterns[i]);
        moving(r);
        assert(r.control.track.deviation == errors[i]);
        if (errors[i] < 0) assert(r.left_command < r.right_command);
        if (errors[i] > 0) assert(r.left_command > r.right_command);
        if (errors[i] == 0) assert(r.left_command == r.right_command);
    }
    /* One inner sensor: immediate partial steering without full saturation.
     * 时间轴上有三个阶段，测试必须选对窗口：
     *   第 1 拍     → 微分项给出有限提前量（增大修正）
     *   第 2~20 拍  → D 按 alpha=0.25 衰减（时间常数约 4 拍，约 15 拍基本归零）
     *   第 50 拍以后 → 同向偏差持续触发"强修正"，修正量 ×1.25（原设计）
     * 所以"单内探头只做部分修正"要在 D 衰减后、强修正触发前检查。 */
    LineFollow_Start();
    (void)step(CENTER);
    LineFollowResult_t r = step(LEFT_INNER);
    assert(r.right_command - r.left_command > 100.0f);
    assert(r.right_command - r.left_command < 300.0f);   /* 跳变瞬间：含 D 项提前量 */
    moving(r);
    for (int i = 0; i < 21; ++i) r = step(LEFT_INNER);
    float d1 = r.right_command - r.left_command;
    assert(d1 > 100.0f);            /* 单内探头确实在修正，不是装死 */
    assert(d1 < 300.0f);            /* 未饱和（最大差速约 530） */
    moving(r);

    /* 分级性：偏差越大修正必须越强。比写死具体数值更能适应增益调参。 */
    LineFollow_Start();
    (void)step(CENTER);
    for (int i = 0; i < 21; ++i) r = step(LEFT_PAIR);
    assert((r.right_command - r.left_command) > d1);
    moving(r);
    r = step(CENTER);
    assert(fabsf(r.control.pd.output) <= CONTROL_PD_D_TERM_LIMIT);
    for (int i = 0; i < 100; ++i) r = step(CENTER);
    assert(fabsf(r.left_command - r.right_command) < 0.01f);
    assert(r.base_speed == CONTROL_BASE_SPEED_DEFAULT);

    /* 居中时两轮必须严格同速 —— 这是实测暴露过的问题：
     * 微分项在 dev=0 时会残留 ±37 的输出，导致两轮差 21%（570/470）。
     * 反复在"偏一侧 / 回中"之间跳变，检查回中后是否还有转速差。 */
    LineFollow_Start();
    for (int i = 0; i < 8; ++i) {
        (void)step(RIGHT_INNER);
        (void)step(CENTER);
    }
    r = step(CENTER);
    assert(r.control.track.deviation == 0.0f);
    assert(fabsf(r.left_command - r.right_command) < 0.01f);
    assert(fabsf(r.control.pd.output) < 0.01f);
}

static void test_recovery(void)
{
    LineFollow_Start();
    (void)step(LEFT_OUTER); /* left outer */
    (void)step(CENTER); /* centered does not erase turn history */
    LineFollowResult_t r = step(LOST);
    moving(r);
    assert(r.last_valid_direction == -1);
    assert(r.left_command < r.right_command);
    r = step(RIGHT_OUTER); /* rediscovered on RIGHT: change direction immediately */
    moving(r);
    assert(r.state == LINE_FOLLOW_REACQUIRE);
    assert(r.left_command > r.right_command);
    assert(r.lost_cycles == 1);
    for (unsigned i = 1; i < follow.reacquire_cycles; ++i) r = step(RIGHT_OUTER);
    assert(r.state == LINE_FOLLOW_TRACKING && r.lost_cycles == 0);

    /* A single spurious detection cannot perpetually reset the search budget.
     * 注意用内探头（|dev|=1）：外探头丢线会走"直角原地转向"分支（见 test_corner.c），
     * 这里要验证的是前进搜索的丢线预算，所以选一个不触发直角的偏差。 */
    LineFollow_Start();
    (void)step(RIGHT_INNER);
    for (unsigned i = 0; i < follow.max_lost_cycles; ++i) {
        moving(step(LOST));
        moving(step(RIGHT_INNER));
    }
    r = step(LOST);
    assert(r.fault == LINE_FOLLOW_FAULT_LOST_TIMEOUT);
    stopped(r);
    stopped(step(CENTER)); /* latched even if a line returns */
}

static void test_progressive_steering(void)
{
    LineFollow_Start();
    LineFollowResult_t r = step(CENTER);
    const float initial_differential = fabsf(r.left_command - r.right_command);

    for (unsigned i = 0; i < LINE_FOLLOW_HARD_TURN_TRIGGER_CYCLES; ++i) {
        r = step(LEFT_INNER);
        moving(r);
    }

    assert(fabsf(r.left_command - r.right_command) > initial_differential);
    assert(r.left_command < r.right_command);
    /* 容差 0.5：强修正时 left = base + c 经过两次浮点运算，可能得 449.99997。
     * 上界同样引用宏，不要写死 800（MAX_FORWARD 已改成 930）。 */
    assert(r.left_command >= LINE_FOLLOW_HARD_TURN_MIN_FORWARD - 0.5f);
    assert(r.right_command <= CONTROL_MAX_FORWARD_COMMAND_DEFAULT + 0.5f);
}

static void test_stops_and_curve_hold(void)
{
    LineFollow_Init();
    stopped(step(CENTER));
    LineFollow_Start();
    assert(step(LOST).fault == LINE_FOLLOW_FAULT_NO_VALID_HISTORY);
    LineFollow_Start();
    (void)step(CENTER);
    /* 全黑不再立刻判故障：先直行穿越（直角胶带重叠、粗线、交叉点），
     * 超过预算仍全黑才停。详见 test_corner.c 的完整覆盖。 */
#if LINE_FOLLOW_ALL_BLACK_CROSSING
    {
        LineFollowResult_t cross = step(ALL_BLACK);
        assert(cross.fault == LINE_FOLLOW_FAULT_NONE);
        assert(cross.state == LINE_FOLLOW_CROSSING);
        assert(cross.left_command == LINE_FOLLOW_ALL_BLACK_CREEP_DUTY);
        for (unsigned i = 0; i < LINE_FOLLOW_ALL_BLACK_MAX_CYCLES; ++i) {
            (void)step(ALL_BLACK);
        }
        stopped(step(ALL_BLACK));   /* 预算用尽 → 判故障停车 */
    }
#else
    stopped(step(ALL_BLACK));
#endif
    LineFollow_Start();
    /* 贴边循迹把 straight_max 放宽到 1.5，所以要用 |dev| = 2 才进入弯道档 */
    moving(step(LEFT_OUTER | LEFT_INNER));
    for (unsigned i = 0; i < LINE_FOLLOW_CURVE_HOLD_CYCLES; ++i) {
        LineFollowResult_t r = step(CENTER);
        moving(r);
        assert(r.base_speed == follow.curve_speed);
    }
    LineFollow_Stop();
    stopped(step(CENTER));
    LineFollow_Start();
    raw_bits = physical_to_raw(CENTER);
    MotorCommandConfig_t bad = motor;
    bad.min_forward_command = bad.max_forward_command + 1.0f;
    LineFollowResult_t r = LineFollow_Update(&pd, &bad, &follow);
    assert(r.fault == LINE_FOLLOW_FAULT_INVALID_INPUT);
    stopped(r);
}

static float segment_distance(float x, float y, float y_line)
{
    float closest_x = fmaxf(0.0f, fminf(x, 0.85f));
    return hypotf(x - closest_x, y - y_line);
}

/* R=0.60 m, AB=CD=0.85 m. Only AB -> BC -> CD is modeled here.
 * Assumed chassis: 14 cm wheel spacing, sensor bar 8 cm ahead of axle,
 * 1 cm sensor spacing, 15 mm black stripe, speed=0.6*duty m/s.
 * This ideal differential-drive model is a regression, NOT hardware proof. */
#define R60_STEP_BUDGET   (16000U)
/* 步数预算推导：路径长 ≈ 0.75(AB) + π·0.60(半圆) + 0.30(CD 到 x<0.55) ≈ 2.94 m。
 * 车速 = 0.6 × 占空比，且弧线上用 curve_speed 而不是基速：
 *   直道 520 → 0.312 m/s；弧线 400 → 0.24 m/s
 *   时间 ≈ 0.75/0.312 + 1.885/0.24 + 0.30/0.312 ≈ 2.4 + 7.9 + 1.0 ≈ 11.3 s
 * 再留 30% 余量（低增益电机更慢），预算取 16000（16 s）。
 * 历史：6000（基速 650 时代估的，从未真正执行过）→ 12000 → 16000。 */

static float track_distance(float x, float y)
{
    float distance = fminf(segment_distance(x, y, 0.0f),
                           segment_distance(x, y, -1.20f));
    if (x >= 0.85f) distance = fminf(distance, fabsf(hypotf(x - 0.85f, y + 0.60f) - 0.60f));
    return distance;
}

static void test_r60_arc(float left_gain, float initial_heading, float mirror)
{
    float x = 0.10f, y = 0.0f, heading = initial_heading;
    float max_distance = 0.0f;
    unsigned cycles;
    LineFollow_Start();
    for (cycles = 0; cycles < R60_STEP_BUDGET; ++cycles) {
        uint8_t black = 0;
        /* Physical left-to-right sensor positions, independent of connector order. */
        for (unsigned bit = 0; bit < 4; ++bit) {
            float lateral = (1.5f - (float)bit) * 0.01f;
            float sx = x + 0.08f * cosf(heading) - lateral * sinf(heading);
            float sy = y + 0.08f * sinf(heading) + lateral * cosf(heading);
            if (track_distance(sx, mirror * sy) <= 0.0075f) black |= (1U << bit);
        }
        raw_bits = physical_to_raw(black);
        LineFollowResult_t r = LineFollow_Update(&pd, &motor, &follow);
        moving(r);
        float vl = 0.6f * r.left_command / 1000.0f * left_gain;
        float vr = 0.6f * r.right_command / 1000.0f;
        float velocity = (vl + vr) * 0.5f;
        heading += (vr - vl) / 0.14f * CONTROL_PERIOD_SECONDS;
        x += velocity * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += velocity * sinf(heading) * CONTROL_PERIOD_SECONDS;
        max_distance = fmaxf(max_distance, track_distance(x, mirror * y));
        if (mirror * y < -1.15f && x < 0.55f) break;
    }
    printf("R60 model: motor gain %.2f, heading %.2f, turn %+.0f: %u steps, max axle error %.1f mm\n",
           left_gain, initial_heading, mirror, cycles, max_distance * 1000.0f);
    assert(cycles < R60_STEP_BUDGET);
    assert(max_distance < 0.04f);
    assert(cosf(heading) < -0.95f);
}

int main(void)
{
    test_geometry_and_small_corrections();
    test_recovery();
    test_progressive_steering();
    test_stops_and_curve_hold();
    test_r60_arc(1.00f, 0.00f, 1.0f);
    test_r60_arc(0.97f, 0.03f, 1.0f);
    test_r60_arc(1.03f, -0.03f, 1.0f);
    test_r60_arc(1.00f, 0.00f, -1.0f);
    test_r60_arc(0.97f, 0.03f, -1.0f);
    test_r60_arc(1.03f, -0.03f, -1.0f);
    puts("PASS: physical sensor direction, duty limits, recovery, stops and R60 model");
    return 0;
}
