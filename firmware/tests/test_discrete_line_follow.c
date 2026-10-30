#include "line_follow.h"
#include "motor_driver.h"
#include "vehicle_control.h"

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
/* Driver boundary only: all sampling, follower and vehicle gating stay real. */
volatile uint8_t g_motor_driver_enabled;
volatile int16_t g_motor_driver_left_actual;
volatile int16_t g_motor_driver_right_actual;
void MotorDriver_Stop(void)
{
    g_motor_driver_left_actual = g_motor_driver_right_actual = 0;
}
void MotorDriver_Disable(void) { g_motor_driver_enabled = 0U; MotorDriver_Stop(); }
void MotorDriver_Enable(void) { g_motor_driver_enabled = 1U; MotorDriver_Stop(); }
void MotorDriver_SetBoth(int16_t left, int16_t right)
{
    g_motor_driver_left_actual = left;
    g_motor_driver_right_actual = right;
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

static void expect_turn(LineFollowResult_t r, int direction)
{
    assert(r.state == LINE_FOLLOW_TURNING);
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.left_command == (direction < 0 ? 0.0f : LINE_FOLLOW_TURN_COMMAND));
    assert(r.right_command == (direction < 0 ? LINE_FOLLOW_TURN_COMMAND : 0.0f));
}

static void test_corner_sequences(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        uint8_t outer = direction < 0 ? LEFT_OUTER : RIGHT_OUTER;
        LineFollow_Start();
        (void)step(CENTER);
        /* Three probes identify the side of a right-angle departure. */
        expect_turn(step(outer | CENTER), direction);
        for (unsigned i = 1; i < LINE_FOLLOW_TURN_MIN_CYCLES; ++i) {
            expect_turn(step(CENTER), direction); /* old leg must not end the turn */
        }
        expect_turn(step(0U), direction); /* no 25 ms straight-ahead gap */
        expect_turn(step(LEFT_INNER), direction);
        expect_turn(step(0U), direction); /* one inner hit is insufficient */
        LineFollowResult_t r;
        for (unsigned i = 1; i < follow.reacquire_cycles; ++i) {
            expect_turn(step(CENTER), direction);
        }
        r = step(CENTER);
        assert(r.state == LINE_FOLLOW_TRACKING && r.lost_cycles == 0U);
        expect(CENTER, CONTROL_BASE_SPEED_DEFAULT, CONTROL_BASE_SPEED_DEFAULT);

        /* Outer -> lost also works if the corner pattern was missed. */
        LineFollow_Start();
        (void)step(outer);
        expect_turn(step(0U), direction);
        LineFollow_Stop();
        r = step(0U);
        assert(r.state == LINE_FOLLOW_STOPPED && r.left_command == 0.0f && r.right_command == 0.0f);

        LineFollow_Start();
        for (unsigned i = 1; i < LINE_FOLLOW_TURN_TRIGGER_CYCLES; ++i) {
            assert(step(outer).state == LINE_FOLLOW_TRACKING);
        }
        expect_turn(step(outer), direction);
    }
}

static void test_corner_watchdogs(void)
{
    LineFollowResult_t r;
    LineFollow_Start();
    expect_turn(step(LEFT_OUTER | CENTER), -1);
    for (unsigned i = 0; i < LINE_FOLLOW_TURN_ALL_BLACK_CYCLES; ++i) {
        expect_turn(step(15U), -1);
    }
    r = step(15U);
    assert(r.fault == LINE_FOLLOW_FAULT_ALL_BLACK && r.left_command == 0.0f && r.right_command == 0.0f);
    assert(step(CENTER).state == LINE_FOLLOW_FAULT);

    LineFollow_Start();
    r = step(15U); /* no direction: do not guess through an intersection */
    assert(r.fault == LINE_FOLLOW_FAULT_ALL_BLACK);

    LineFollow_Start();
    expect_turn(step(LEFT_OUTER | CENTER), -1);
    for (unsigned i = 1; i < LINE_FOLLOW_TURN_MAX_CYCLES; ++i) {
        expect_turn(step(LEFT_OUTER), -1); /* valid edge cannot reset total timeout */
    }
    r = step(LEFT_OUTER);
    assert(r.fault == LINE_FOLLOW_FAULT_TURN_TIMEOUT && r.left_command == 0.0f && r.right_command == 0.0f);

    LineFollow_Start();
    expect_turn(step(RIGHT_OUTER | CENTER), 1);
    for (unsigned i = 0; i < follow.max_lost_cycles; ++i) {
        expect_turn(step(0U), 1);
    }
    r = step(0U);
    assert(r.fault == LINE_FOLLOW_FAULT_LOST_TIMEOUT && r.left_command == 0.0f && r.right_command == 0.0f);
    LineFollow_Start();
    expect(CENTER, CONTROL_BASE_SPEED_DEFAULT, CONTROL_BASE_SPEED_DEFAULT);
}

static void test_vehicle_turn_gating(void)
{
    MotorCommandConfig_t bad = motor;
    VehicleControl_Init(&pd, &bad, &follow);
    VehicleControl_Start();
    VehicleControl_SetMotorOutputEnabled(true);
    raw_bits = physical_to_raw(LEFT_OUTER | CENTER);
    VehicleControl_Update();
    VehicleControlOutput_t out = VehicleControl_GetOutput();
    assert(out.line_output_valid && out.line_follow.state == LINE_FOLLOW_TURNING);
    assert(out.left_applied_command == 0 && out.right_applied_command == (int16_t)LINE_FOLLOW_TURN_COMMAND);
    raw_bits = physical_to_raw(0U);
    VehicleControl_Update();
    assert(VehicleControl_GetOutput().line_output_valid); /* a corner gap is allowed */

    VehicleControl_SetMotorOutputEnabled(false);
    assert(g_motor_driver_enabled == 0U && g_motor_driver_right_actual == 0);
    VehicleControl_Update();
    out = VehicleControl_GetOutput();
    assert(out.left_applied_command == 0 && out.right_applied_command == 0);
    VehicleControl_SetMotorOutputEnabled(true);
    bad.base_speed = NAN; /* invalid configuration preempts a latched turn */
    VehicleControl_Update();
    out = VehicleControl_GetOutput();
    assert(out.line_follow.fault == LINE_FOLLOW_FAULT_INVALID_INPUT && !out.line_output_valid);
    assert(out.left_applied_command == 0 && out.right_applied_command == 0);
    VehicleControl_Stop();
    assert(VehicleControl_GetOutput().line_follow.state == LINE_FOLLOW_STOPPED);
}

static float corner_distance(float x, float y)
{
    float dx = x - fmaxf(0.0f, fminf(x, 0.5f));
    float dy = y - fmaxf(0.0f, fminf(y, 0.6f));
    return fminf(hypotf(dx, y), hypotf(x - 0.5f, dy));
}

/* Ideal differential-drive regression, not measured chassis dimensions.
 * W=14 cm, probes 8 cm ahead, spacing=1 cm, stripe width=15 mm.
 * Model both departure sides and wheel mismatch through real GPIO mapping. */
static void test_corner_model(float mirror, float left_gain, float heading)
{
    float x = 0.1f, y = 0.0f;
    unsigned turns = 0U, cycles;
    LineFollow_Start();
    for (cycles = 0U; cycles < 6000U; ++cycles) {
        uint8_t black = 0U;
        for (unsigned bit = 0U; bit < 4U; ++bit) {
            float lateral = (1.5f - (float)bit) * 0.01f;
            float sx = x + 0.08f * cosf(heading) - lateral * sinf(heading);
            float sy = y + 0.08f * sinf(heading) + lateral * cosf(heading);
            if (corner_distance(sx, mirror * sy) <= 0.0075f) black |= (1U << bit);
        }
        LineFollowResult_t r = step(black);
        if (r.fault != LINE_FOLLOW_FAULT_NONE) {
            fprintf(stderr, "corner failure: mirror %.0f gain %.2f cycle %u xy %.3f %.3f heading %.3f bits %u fault %u\n",
                    mirror, left_gain, cycles, x, y, heading, black, r.fault);
        }
        assert(r.fault == LINE_FOLLOW_FAULT_NONE);
        if (r.state == LINE_FOLLOW_TURNING) ++turns;
        float vl = 0.6f * r.left_command / 1000.0f * left_gain;
        float vr = 0.6f * r.right_command / 1000.0f;
        heading += (vr - vl) / 0.14f * CONTROL_PERIOD_SECONDS;
        x += 0.5f * (vl + vr) * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += 0.5f * (vl + vr) * sinf(heading) * CONTROL_PERIOD_SECONDS;
        if (mirror * y > 0.3f) break;
    }
    printf("Right angle model: side %+.0f gain %.2f, %u cycles, x %.3f y %.3f heading %.3f, turn ticks %u\n",
           mirror, left_gain, cycles, x, y, heading, turns);
    assert(cycles < 6000U && turns > 0U);
    assert(fabsf(x - 0.5f) < 0.03f);
    assert(mirror * sinf(heading) > 0.95f);
}

static float arc_distance(float x, float y)
{
    float sx = fmaxf(0.0f, fminf(x, 0.85f));
    float d = fminf(hypotf(x - sx, y), hypotf(x - sx, y + 1.2f));
    if (x >= 0.85f) d = fminf(d, fabsf(hypotf(x - 0.85f, y + 0.6f) - 0.6f));
    return d;
}

static void test_arc_model(float mirror, float left_gain, float heading)
{
    float x = 0.1f, y = 0.0f, max_error = 0.0f;
    unsigned cycles;
    LineFollow_Start();
    for (cycles = 0U; cycles < 60000U; ++cycles) {
        uint8_t black = 0U;
        for (unsigned bit = 0U; bit < 4U; ++bit) {
            float lateral = (1.5f - (float)bit) * 0.01f;
            float sx = x + 0.08f * cosf(heading) - lateral * sinf(heading);
            float sy = y + 0.08f * sinf(heading) + lateral * cosf(heading);
            if (arc_distance(sx, mirror * sy) <= 0.0075f) black |= (1U << bit);
        }
        LineFollowResult_t r = step(black);
        assert(r.fault == LINE_FOLLOW_FAULT_NONE);
        float vl = 0.6f * r.left_command / 1000.0f * left_gain;
        float vr = 0.6f * r.right_command / 1000.0f;
        heading += (vr - vl) / 0.14f * CONTROL_PERIOD_SECONDS;
        x += 0.5f * (vl + vr) * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += 0.5f * (vl + vr) * sinf(heading) * CONTROL_PERIOD_SECONDS;
        max_error = fmaxf(max_error, arc_distance(x, mirror * y));
        if (mirror * y < -1.15f && x < 0.55f) break;
    }
    printf("Discrete R60 model: side %+.0f gain %.2f, %u cycles, max axle error %.1f mm\n",
           mirror, left_gain, cycles, 1000.0f * max_error);
    assert(cycles < 60000U && max_error < 0.04f && cosf(heading) < -0.95f);
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
    test_corner_sequences();
    test_corner_watchdogs();
    for (int side = -1; side <= 1; side += 2) {
        test_corner_model((float)side, 1.00f, 0.00f);
        test_corner_model((float)side, 0.97f, 0.03f);
        test_corner_model((float)side, 1.03f, -0.03f);
        test_arc_model((float)side, 1.00f, 0.00f);
        test_arc_model((float)side, 0.97f, 0.03f);
        test_arc_model((float)side, 1.03f, -0.03f);
    }
    test_vehicle_turn_gating();
    puts("PASS: discrete tracking, corner latch, recovery, watchdogs, vehicle gating and geometry models");
    return 0;
}
