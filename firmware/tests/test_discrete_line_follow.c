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
    assert(r.left_command == (direction < 0 ? LINE_FOLLOW_TURN_INNER_COMMAND :
                                             LINE_FOLLOW_TURN_OUTER_COMMAND));
    assert(r.right_command == (direction < 0 ? LINE_FOLLOW_TURN_OUTER_COMMAND :
                                              LINE_FOLLOW_TURN_INNER_COMMAND));
    assert(r.left_command > 0.0f && r.right_command > 0.0f);
}

static void test_inner_steering(void)
{
    const float full_differential = LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT -
                                    LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT;
    for (int direction = -1; direction <= 1; direction += 2) {
        uint8_t side = direction < 0 ? LEFT_INNER : RIGHT_INNER;
        uint8_t other = direction < 0 ? RIGHT_INNER : LEFT_INNER;
        LineFollow_Start();
        (void)step(CENTER);
        LineFollowResult_t r = step(side);
        float gentle = fabsf(r.left_command - r.right_command);
        assert(gentle > 0.0f && gentle <= 0.3f * full_differential);
        assert((r.left_command - r.right_command) * direction > 0.0f);
        float previous = gentle;
        for (unsigned i = 0U; i < 200U; ++i) {
            r = step(side);
            float differential = fabsf(r.left_command - r.right_command);
            assert(differential >= previous && differential <= full_differential);
            assert(r.left_command >= motor.min_forward_command && r.right_command >= motor.min_forward_command);
            previous = differential;
        }
        assert(previous == full_differential); /* retain steady curve authority */
        r = step(other); /* reverse immediately, without retaining a strong old command */
        assert((r.left_command - r.right_command) * direction < 0.0f);
        assert(fabsf(r.left_command - r.right_command) == gentle);
        expect(CENTER, motor.base_speed, motor.base_speed);
        r = step(side);
        assert(fabsf(r.left_command - r.right_command) == gentle);
        (void)step(direction < 0 ? LEFT_OUTER : RIGHT_OUTER);
        r = step(side);
        assert(fabsf(r.left_command - r.right_command) == gentle);
        (void)step(0U);
        r = step(side);
        assert(fabsf(r.left_command - r.right_command) == gentle);
        LineFollow_Stop();
        r = step(side);
        assert(r.left_command == 0.0f && r.right_command == 0.0f);
        LineFollow_Start();
        r = step(side);
        assert(fabsf(r.left_command - r.right_command) == gentle);
    }

    /* An asymmetric drive can now use the existing straight trim in discrete mode. */
    MotorCommandConfig_t trimmed = motor;
    trimmed.straight_trim = 12.0f;
    LineFollow_Start();
    raw_bits = physical_to_raw(CENTER);
    LineFollowResult_t r = LineFollow_Update(&pd, &trimmed, &follow);
    assert(r.left_command == motor.base_speed + 12.0f && r.right_command == motor.base_speed - 12.0f);
    trimmed.min_forward_command = 575.0f;
    trimmed.max_forward_command = 590.0f;
    trimmed.straight_trim = 1000.0f;
    LineFollowConfig_t tight = follow;
    tight.curve_speed = tight.sharp_curve_speed = tight.search_speed = motor.base_speed;
    raw_bits = physical_to_raw(RIGHT_INNER);
    r = LineFollow_Update(&pd, &trimmed, &tight);
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.left_command <= trimmed.max_forward_command && r.right_command >= trimmed.min_forward_command);
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
    assert(out.left_applied_command == (int16_t)LINE_FOLLOW_TURN_INNER_COMMAND);
    assert(out.right_applied_command == (int16_t)LINE_FOLLOW_TURN_OUTER_COMMAND);
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

typedef struct {
    float heading_rms;
    float lateral_rms;
    float command_variation;
} StraightMetrics;

/* Model assumptions: same 14 cm / 8 cm geometry as the corner model, 1 mm
 * threshold jitter and 40..120 ms first-order motor lag. Baseline is branch1pro
 * 4fa7740's inner-only 480/680 law. No measured hardware parameters are implied. */
static StraightMetrics straight_model(bool baseline, float lag, float gain, float initial_heading)
{
    float x = 0.0f, y = 0.0f, heading = initial_heading, vl = 0.0f, vr = 0.0f;
    float previous = 0.0f;
    uint32_t seed = 43U;
    StraightMetrics metrics = {0};
    LineFollow_Start();
    for (unsigned i = 0U; i < 8000U; ++i) {
        uint8_t black = 0U;
        seed = 1664525U * seed + 1013904223U;
        float jitter = ((float)(seed >> 8) / 16777216.0f - 0.5f) * 0.002f;
        for (unsigned bit = 0U; bit < 4U; ++bit) {
            float sy = y + 0.08f * sinf(heading) + (1.5f - (float)bit) * 0.01f * cosf(heading);
            if (fabsf(sy + jitter) <= 0.0075f) black |= (1U << bit);
        }
        assert(black == CENTER || black == LEFT_INNER || black == RIGHT_INNER);
        float left, right;
        if (baseline) {
            float correction = black == LEFT_INNER ? -100.0f : (black == RIGHT_INNER ? 100.0f : 0.0f);
            left = 580.0f + correction;
            right = 580.0f - correction;
        } else {
            LineFollowResult_t r = step(black);
            assert(r.state == LINE_FOLLOW_TRACKING && r.fault == LINE_FOLLOW_FAULT_NONE);
            left = r.left_command;
            right = r.right_command;
        }
        vl += CONTROL_PERIOD_SECONDS / lag * (0.6f * left / 1000.0f * gain - vl);
        vr += CONTROL_PERIOD_SECONDS / lag * (0.6f * right / 1000.0f - vr);
        heading += (vr - vl) / 0.14f * CONTROL_PERIOD_SECONDS;
        x += 0.5f * (vl + vr) * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += 0.5f * (vl + vr) * sinf(heading) * CONTROL_PERIOD_SECONDS;
        assert(fabsf(y) < 0.01f); /* stay within 1 cm even during initial settling */
        float correction = 0.5f * (left - right);
        if (i >= 2000U) {
            metrics.heading_rms += heading * heading;
            metrics.lateral_rms += y * y;
            metrics.command_variation += fabsf(correction - previous);
        }
        previous = correction;
    }
    assert(x > 2.5f); /* reduction must not be achieved by stopping the car */
    metrics.heading_rms = sqrtf(metrics.heading_rms / 6000.0f);
    metrics.lateral_rms = sqrtf(metrics.lateral_rms / 6000.0f);
    metrics.command_variation /= 6000.0f;
    return metrics;
}

static void test_straight_models(void)
{
    float old_heading_total = 0.0f, new_heading_total = 0.0f;
    for (unsigned lag_ms = 40U; lag_ms <= 120U; lag_ms += 40U) {
        for (int side = -1; side <= 1; side += 2) {
            float gain = 1.0f + (float)side * 0.03f;
            float initial_heading = -(float)side * 0.06f;
            StraightMetrics old = straight_model(true, (float)lag_ms / 1000.0f, gain, initial_heading);
            StraightMetrics now = straight_model(false, (float)lag_ms / 1000.0f, gain, initial_heading);
            printf("Straight model: lag %u ms, gain %.2f, heading RMS %.3f -> %.3f mrad, lateral RMS %.2f -> %.2f mm, command variation %.2f -> %.2f\n",
                   lag_ms, gain, old.heading_rms * 1000.0f, now.heading_rms * 1000.0f,
                   old.lateral_rms * 1000.0f, now.lateral_rms * 1000.0f,
                   old.command_variation, now.command_variation);
            old_heading_total += old.heading_rms;
            new_heading_total += now.heading_rms;
            assert(now.command_variation < 0.8f * old.command_variation);
            assert(now.lateral_rms < 0.003f);
            assert(now.heading_rms < 0.003f);
        }
    }
    assert(new_heading_total < 0.9f * old_heading_total);
    for (int side = -1; side <= 1; side += 2) {
        StraightMetrics balanced = straight_model(false, 0.08f, 1.0f, 0.06f * (float)side);
        assert(balanced.lateral_rms < 0.003f && balanced.heading_rms < 0.003f);
    }
}

int main(void)
{
    const float inner_slow = LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT;
    const float outer_fast = LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT;
    const float outer_slow = LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT;
    const float straight = CONTROL_BASE_SPEED_DEFAULT;

    LineFollow_Start();
    expect(CENTER, straight, straight);
    expect(LEFT_INNER, straight - LINE_FOLLOW_INNER_INITIAL_CORRECTION,
           straight + LINE_FOLLOW_INNER_INITIAL_CORRECTION);
    expect(CENTER, straight, straight);
    expect(RIGHT_INNER, straight + LINE_FOLLOW_INNER_INITIAL_CORRECTION,
           straight - LINE_FOLLOW_INNER_INITIAL_CORRECTION);
    expect(CENTER, straight, straight);

    expect(LEFT_OUTER, outer_slow, outer_fast);
    expect(LEFT_OUTER | RIGHT_INNER, outer_slow, outer_fast);
    expect(LEFT_INNER, straight - LINE_FOLLOW_INNER_INITIAL_CORRECTION,
           straight + LINE_FOLLOW_INNER_INITIAL_CORRECTION);
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
    test_inner_steering();
    test_straight_models();
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
    puts("PASS: gentle inner steering, straight stability, corner latch, recovery, watchdogs and vehicle gating");
    return 0;
}
