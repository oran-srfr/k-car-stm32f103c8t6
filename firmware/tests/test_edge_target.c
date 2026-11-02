#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 验证"贴边循迹"（方案 B）：控制目标从偏差 0 移到 LINE_FOLLOW_TARGET_DEVIATION。
 * ① dev = 目标（默认 -1）→ 两轮严格同速
 * ② dev = 0   → 左轮快   ③ dev = -2 → 右轮快
 * ④ 目标处仍按直行基速跑（不被误判成弯道） */

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

static LineFollowResult_t hold(unsigned mask, unsigned cycles)
{
    LineFollowResult_t r = {0};

    LineFollow_Start();
    set_pattern(mask);
    for (unsigned i = 0; i < cycles; ++i) {
        r = LineFollow_Update(&pd, &motor, &follow);
    }
    return r;
}

int main(void)
{
    const unsigned LEFT_OUTER = 1U;
    const unsigned LEFT_INNER = 2U;
    const unsigned RIGHT_INNER = 4U;

    assert(fabsf(LINE_FOLLOW_TARGET_DEVIATION + 1.0f) < 0.001f);

    LineFollowResult_t at_target = hold(LEFT_INNER, 30U);
    assert(at_target.control.track.deviation == -1.0f);
    assert(at_target.left_command == at_target.right_command);

    LineFollowResult_t right_of = hold(LEFT_INNER | RIGHT_INNER, 30U);
    assert(right_of.control.track.deviation == 0.0f);
    assert(right_of.left_command > right_of.right_command);

    LineFollowResult_t left_of = hold(LEFT_OUTER | LEFT_INNER, 30U);
    assert(left_of.control.track.deviation == -2.0f);
    assert(left_of.left_command < left_of.right_command);

    assert(fabsf(at_target.base_speed - CONTROL_BASE_SPEED_DEFAULT) < 0.5f);

    puts("PASS: edge target holds equal wheel speeds and keeps straight base speed");
    return 0;
}