#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#if LINE_FOLLOW_DISCRETE_MODE != 1U
#error "This test requires the production discrete line-follow mode"
#endif
_Static_assert(CONTROL_PERIOD_MS == 1U, "Discrete control period must be 1 ms");

enum {
    LEFT_OUTER = 1U,
    LEFT_INNER = 2U,
    RIGHT_INNER = 4U,
    RIGHT_OUTER = 8U,
    CENTER = LEFT_INNER | RIGHT_INNER
};

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

LineSensorRaw_t LineSensor_ReadRaw(void)
{
    return (LineSensorRaw_t){raw_bits & 1U, (raw_bits >> 1) & 1U,
                            (raw_bits >> 2) & 1U, (raw_bits >> 3) & 1U};
}

/* Convert a physical left-to-right black bitmap into active-low raw GPIO bits. */
static uint8_t physical_to_raw(uint8_t black)
{
    const uint8_t mapping[] = {
        LINE_SENSOR_LEFT_OUTER_BIT, LINE_SENSOR_LEFT_INNER_BIT,
        LINE_SENSOR_RIGHT_INNER_BIT, LINE_SENSOR_RIGHT_OUTER_BIT
    };
    uint8_t raw = 0x0FU;

    for (unsigned i = 0U; i < 4U; ++i) {
        if ((black & (1U << i)) != 0U) {
            raw &= (uint8_t)~(1U << mapping[i]);
        }
    }
#if LINE_SENSOR_BLACK_ACTIVE_LOW == 0U
    raw ^= 0x0FU;
#endif
    return raw;
}

static LineFollowResult_t step(uint8_t black)
{
    raw_bits = physical_to_raw(black);
    return LineFollow_Update(&pd, &motor, &follow);
}

static void expect(uint8_t black, float left, float right)
{
    LineFollowResult_t result = step(black);

    assert(result.fault == LINE_FOLLOW_FAULT_NONE);
    assert(result.left_command == left);
    assert(result.right_command == right);
}

int main(void)
{
    const float inner_fast = LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT;
    const float inner_slow = LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT;
    const float outer_fast = LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT;
    const float outer_slow = LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT;
    const float straight = CONTROL_BASE_SPEED_DEFAULT;

    LineFollow_Start();
    expect(CENTER, straight, straight);
    expect(LEFT_INNER, inner_slow, inner_fast);
    expect(CENTER, straight, straight);
    expect(RIGHT_INNER, inner_fast, inner_slow);
    expect(CENTER, straight, straight);

    expect(LEFT_OUTER, outer_slow, outer_fast);
    expect(LEFT_OUTER | RIGHT_INNER, outer_slow, outer_fast);
    expect(LEFT_INNER, inner_slow, inner_fast);
    expect(RIGHT_OUTER, outer_fast, outer_slow);
    expect(RIGHT_OUTER | LEFT_INNER, outer_fast, outer_slow);
    expect(CENTER, straight, straight);

    expect(LEFT_OUTER | RIGHT_OUTER, inner_slow, inner_slow);

    /* Four indications off: first move slowly, then search with a differential. */
    expect(0U, CONTROL_MIN_FORWARD_COMMAND_DEFAULT,
           CONTROL_MIN_FORWARD_COMMAND_DEFAULT);
    LineFollowResult_t lost = step(0U);
    for (unsigned i = 0U; i < LINE_FOLLOW_ALL_OFF_SLOW_CYCLES; ++i) {
        lost = step(0U);
    }
    assert(lost.fault == LINE_FOLLOW_FAULT_NONE);
    assert(lost.left_command > 0.0f && lost.right_command > 0.0f);
    assert(fabsf(lost.left_command - lost.right_command) ==
           outer_fast - outer_slow);
    puts("PASS: direct sensor direction and two-wheel differential");
    return 0;
}
