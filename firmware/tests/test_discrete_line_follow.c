#include "line_follow.h"
#include "motor_driver.h"
#include "vehicle_control.h"
#include "wheel_speed.h"

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
    if (r.turn_phase == LINE_FOLLOW_TURN_SEEK) {
        assert(r.left_command == (direction < 0 ? LINE_FOLLOW_TURN_INNER_COMMAND : LINE_FOLLOW_TURN_OUTER_COMMAND));
        assert(r.right_command == (direction < 0 ? LINE_FOLLOW_TURN_OUTER_COMMAND : LINE_FOLLOW_TURN_INNER_COMMAND));
    }
    assert(r.left_command >= -1000.0f && r.right_command >= -1000.0f);
    assert(r.left_command <= 1000.0f && r.right_command <= 1000.0f);
}

static void test_unified_steering(void)
{
    LineFollow_Init();
    for (int direction = -1; direction <= 1; direction += 2) {
        uint8_t inner = direction < 0 ? LEFT_INNER : RIGHT_INNER;
        uint8_t outer = direction < 0 ? LEFT_OUTER : RIGHT_OUTER;
        LineFollow_Start();
        (void)step(CENTER);
        LineFollowResult_t r = step(inner);
        assert((r.left_command - r.right_command) * direction > 0.0f);
        assert(r.base_speed <= motor.base_speed);
        for (unsigned i = 0U; i < 200U; ++i) r = step(inner);
        float steady = fabsf(r.left_command - r.right_command);
        for (unsigned i = 0U; i < 1000U; ++i) r = step(inner);
        assert(fabsf(fabsf(r.left_command - r.right_command) - steady) < 0.01f);
        assert(steady < 160.0f); /* persistent small error never grows to R6's 200 */
        for (unsigned i = 0U; i < 1000U; ++i) {
            r = step(outer);
            assert(r.state == LINE_FOLLOW_TRACKING && r.fault == LINE_FOLLOW_FAULT_NONE);
            assert(r.base_speed <= motor.base_speed);
        }
        assert(fabsf(r.left_command - r.right_command) > steady);
        for (unsigned i = 0U; i < 100U; ++i) r = step(CENTER);
        assert(fabsf(r.left_command - r.right_command) < 0.01f);
        LineFollow_Stop();
        r = step(outer);
        assert(r.left_command == 0.0f && r.right_command == 0.0f);
    }
    MotorCommandConfig_t trimmed = motor;
    trimmed.straight_trim = 12.0f;
    LineFollow_Start();
    raw_bits = physical_to_raw(CENTER);
    LineFollowResult_t r = LineFollow_Update(&pd, &trimmed, &follow);
    assert(r.left_command == motor.base_speed + 12.0f && r.right_command == motor.base_speed - 12.0f);
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
        LineFollowResult_t r = step(CENTER);
        for (unsigned i = 1; i < LINE_FOLLOW_TURN_ALIGN_MIN_CYCLES + LINE_FOLLOW_TURN_REACQUIRE_CYCLES - 1U; ++i) {
            if (r.state != LINE_FOLLOW_TURNING) break;
            r = step(CENTER);
        }
        assert(r.state == LINE_FOLLOW_TRACKING && r.lost_cycles == 0U);
        assert(r.base_speed < CONTROL_BASE_SPEED_DEFAULT);
        float previous_base = r.base_speed;
        for (unsigned i = 0; i < 600U; ++i) {
            r = step(CENTER);
            assert(r.base_speed >= previous_base);
            assert(r.base_speed <= previous_base + LINE_FOLLOW_TURN_EXIT_STEP + 0.01f);
            previous_base = r.base_speed;
        }
        expect(CENTER, CONTROL_BASE_SPEED_DEFAULT, CONTROL_BASE_SPEED_DEFAULT);

        /* Outer -> lost also works if the corner pattern was missed. */
        LineFollow_Start();
        (void)step(outer);
        expect_turn(step(0U), direction);
        LineFollow_Stop();
        r = step(0U);
        assert(r.state == LINE_FOLLOW_STOPPED && r.left_command == 0.0f && r.right_command == 0.0f);

        LineFollow_Start();
        for (unsigned i = 1; i < 100U; ++i) {
            assert(step(outer).state == LINE_FOLLOW_TRACKING);
        }
        assert(step(outer).state == LINE_FOLLOW_TRACKING);
        expect_turn(step(outer | CENTER), direction);
    }
}

static void test_turn_alignment(void)
{
    LineFollow_Start();
    (void)step(LEFT_OUTER | CENTER);
    /* The old straight leg can remain centered well beyond the old 60+10 ms
     * exit test. It cannot qualify as a completed turn without a departure. */
    for (unsigned i = 0; i < 500U; ++i) {
        LineFollowResult_t r = step(CENTER);
        assert(r.state == LINE_FOLLOW_TURNING && r.fault == LINE_FOLLOW_FAULT_NONE);
    }
    (void)step(0U);
    LineFollowResult_t r = step(RIGHT_INNER);
    assert(r.turn_phase == LINE_FOLLOW_TURN_ALIGN);
    assert(r.left_command > r.right_command); /* follow the current right-side line */
    r = step(0U);
    assert(r.left_command > r.right_command); /* soft search right, not the entry left */
    assert(fabsf(r.left_command - r.right_command) <= 2.0f * LINE_FOLLOW_TURN_ALIGN_GAP_CORRECTION);
    r = step(LEFT_INNER);
    assert(r.left_command < r.right_command);
    r = step(0U);
    assert(r.left_command < r.right_command);
    for (unsigned i = 0; i < LINE_FOLLOW_TURN_ALIGN_SOFT_GAP_CYCLES; ++i) r = step(0U);
    assert(r.left_command == LINE_FOLLOW_TURN_INNER_COMMAND);
    assert(r.right_command == LINE_FOLLOW_TURN_OUTER_COMMAND);

    /* A slow vehicle still turning at 1.2 s must not trip the old watchdog. */
    LineFollow_Start();
    (void)step(LEFT_OUTER | CENTER);
    for (unsigned i = 0; i < 1500U; ++i) {
        r = step(LEFT_OUTER);
        assert(r.state == LINE_FOLLOW_TURNING && r.fault == LINE_FOLLOW_FAULT_NONE);
    }
    LineFollow_Stop();
    r = step(CENTER);
    assert(r.turn_phase == LINE_FOLLOW_TURN_NONE && r.turn_cycles == 0U);
}

static void test_exit_edge_is_not_another_corner(void)
{
    LineFollow_SetYaw(0.0f, 0U, 0U, false);
    LineFollow_Start();
    (void)step(LEFT_OUTER | CENTER);
    (void)step(0U);
    LineFollowResult_t r = step(CENTER);
    for (unsigned i = 0U; i < 400U && r.state == LINE_FOLLOW_TURNING; ++i) r = step(CENTER);
    assert(r.state == LINE_FOLLOW_TRACKING);
    for (unsigned i = 0U; i < 20U; ++i) {
        r = step(LEFT_OUTER);
        assert(r.state == LINE_FOLLOW_TRACKING);
        assert(fabsf(r.left_command - r.right_command) <= 2.0f * LINE_FOLLOW_TURN_ALIGN_LIMIT);
        assert(r.left_command >= motor.min_forward_command && r.right_command >= motor.min_forward_command);
    }
    r = step(0U);
    assert(r.state != LINE_FOLLOW_TURNING && r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.left_command > 0.0f && r.right_command > 0.0f);
    r = step(RIGHT_OUTER | CENTER); /* a distinct wide corner may still start immediately */
    assert(r.state == LINE_FOLLOW_TURNING);
    LineFollow_Stop();
}

static float wrap_yaw(float yaw)
{
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

static void test_yaw_braking(void)
{
    LineFollow_SetYaw(179.0f, 1000U, 1000U, true);
    LineFollow_Start();
    LineFollowResult_t r = step(LEFT_OUTER | CENTER);
    assert(r.turn_yaw_assist && fabsf(r.turn_angle_deg) < 0.01f);
    /* A positive-yaw installation is valid even for a logical left turn. */
    for (unsigned angle = 1U; angle <= 90U; ++angle) {
        LineFollow_SetYaw(wrap_yaw(179.0f + (float)angle), 1000U + angle * 10U,
                           1000U + angle * 10U, true);
        r = step(angle < 80U ? LEFT_OUTER : LEFT_INNER);
        assert(r.turn_yaw_assist);
    }
    assert(fabsf(r.turn_angle_deg - 90.0f) < 0.01f);
    assert(r.left_command > r.right_command); /* stop the high residual yaw rate */
    LineFollow_SetYaw(wrap_yaw(179.0f + 90.0f), 1910U, 1910U, true);
    r = step(LEFT_INNER);
    assert(r.left_command < r.right_command); /* static offset still needs capture */
    r = step(LEFT_OUTER);
    assert(r.left_command < r.right_command); /* outside probe needs lateral capture */
    r = step(CENTER);
    assert(r.left_command >= r.right_command); /* no renewed left turn at a static 90-degree center */
    assert(r.left_command <= 1000.0f && r.right_command >= 0.0f);

    LineFollow_SetYaw(-91.0f, 1900U, 2001U, true); /* stale despite caller saying valid */
    assert(!step(CENTER).turn_yaw_assist);
    LineFollow_SetYaw(-91.0f, 2002U, 2002U, true);
    assert(!step(CENTER).turn_yaw_assist); /* do not latch a new origin mid-turn */
    LineFollow_Stop();
    assert(!step(CENTER).turn_yaw_assist);

    LineFollow_SetYaw(0.0f, 3000U, 3000U, true);
    LineFollow_Start();
    (void)step(RIGHT_OUTER | CENTER);
    LineFollow_SetYaw(90.0f, 3010U, 3010U, true); /* impossible one-frame step */
    assert(!step(CENTER).turn_yaw_assist);
    LineFollow_SetYaw(NAN, 3020U, 3020U, true);
    assert(!step(CENTER).turn_yaw_assist);

    LineFollow_SetYaw(179.0f, UINT32_MAX - 5U, UINT32_MAX - 5U, true);
    LineFollow_Start();
    (void)step(RIGHT_OUTER | CENTER);
    LineFollow_SetYaw(-179.0f, 4U, 4U, true); /* yaw and millisecond wrap */
    r = step(RIGHT_OUTER);
    assert(r.turn_yaw_assist && fabsf(r.turn_angle_deg - 2.0f) < 0.01f);
    LineFollow_SetYaw(INFINITY, 14U, 14U, true);
    assert(!step(CENTER).turn_yaw_assist);

    LineFollow_SetYaw(0.0f, 4000U, 4000U, true);
    LineFollow_Start();
    for (unsigned i = 0U; i < 10U; ++i) r = step(LEFT_OUTER);
    assert(r.state == LINE_FOLLOW_TRACKING && !r.turn_yaw_assist); /* edge alone is not a corner */
    LineFollow_SetYaw(0.0f, 0U, 0U, false);
    LineFollow_Stop();
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
    for (unsigned i = 0; i < LINE_FOLLOW_TURN_MAX_LOST_CYCLES; ++i) {
        expect_turn(step(0U), 1);
    }
    r = step(0U);
    assert(r.fault == LINE_FOLLOW_FAULT_LOST_TIMEOUT && r.left_command == 0.0f && r.right_command == 0.0f);
    LineFollow_Start();
    expect(CENTER, CONTROL_BASE_SPEED_DEFAULT, CONTROL_BASE_SPEED_DEFAULT);
}

static void test_power_on_and_pause(void)
{
    for (unsigned trial = 0U; trial < 4U; ++trial) {
        VehicleControl_Init(&pd, &motor, &follow);
        VehicleControl_Start();
        for (unsigned i = 0U; i < 5000U; ++i) {
            raw_bits = physical_to_raw((i + trial) % 3U == 0U ? 0U :
                                       ((i + trial) % 3U == 1U ? 15U : LEFT_OUTER | CENTER));
            VehicleControl_Update();
            VehicleControlOutput_t out = VehicleControl_GetOutput();
            assert(out.line_follow.state == LINE_FOLLOW_TRACKING);
            assert(out.line_follow.fault == LINE_FOLLOW_FAULT_NONE);
            assert(out.line_follow.turn_cycles == 0U && out.line_follow.lost_cycles == 0U);
            assert(out.left_applied_command == 0 && out.right_applied_command == 0);
        }
        raw_bits = physical_to_raw(CENTER);
        VehicleControl_Update();
        VehicleControlOutput_t out = VehicleControl_GetOutput();
        assert(out.line_output_valid && out.line_follow.control.track.status == LINE_TRACK_STATUS_NORMAL);
        VehicleControl_SetMotorOutputEnabled(true);
        raw_bits = physical_to_raw(LEFT_OUTER | CENTER);
        VehicleControl_Update();
        out = VehicleControl_GetOutput();
        assert(out.line_follow.state == LINE_FOLLOW_TURNING && out.line_follow.turn_cycles == 1U);
        VehicleControl_SetMotorOutputEnabled(false);
        for (unsigned i = 0U; i < 5000U; ++i) VehicleControl_Update();
        out = VehicleControl_GetOutput();
        assert(out.line_follow.turn_cycles == 1U && out.line_follow.fault == LINE_FOLLOW_FAULT_NONE);
        assert(out.left_applied_command == 0 && out.right_applied_command == 0);
        VehicleControl_SetMotorOutputEnabled(true);
        VehicleControl_Update();
        out = VehicleControl_GetOutput();
        assert(out.line_follow.turn_cycles == 2U && out.line_follow.fault == LINE_FOLLOW_FAULT_NONE);
        VehicleControl_Stop();
    }
}

static void test_glitches_and_recovery_budget(void)
{
    LineFollow_SetYaw(0.0f, 0U, 0U, false);
    LineFollow_Start();
    (void)step(LEFT_INNER);
    for (unsigned i = 0U; i < LINE_FOLLOW_BLACK_CONFIRM_CYCLES; ++i) {
        assert(step(15U).fault == LINE_FOLLOW_FAULT_NONE);
    }
    assert(step(15U).fault == LINE_FOLLOW_FAULT_ALL_BLACK);
    assert(step(CENTER).state == LINE_FOLLOW_FAULT);
    LineFollow_Start();
    (void)step(LEFT_INNER);
    (void)step(15U);
    assert(step(CENTER).fault == LINE_FOLLOW_FAULT_NONE);
    for (unsigned i = 0U; i < 20U; ++i) {
        assert(step(0U).fault == LINE_FOLLOW_FAULT_NONE);
    }
    LineFollowResult_t r = step(RIGHT_INNER);
    assert(r.state == LINE_FOLLOW_REACQUIRE && r.lost_cycles == 20U);
    r = step(0U);
    assert(r.lost_cycles == 21U); /* one black sample does not forgive the budget */
    for (unsigned i = 0U; i < 5U; ++i) r = step(CENTER);
    assert(step(15U).fault == LINE_FOLLOW_FAULT_NONE);
    for (unsigned i = 0U; i < 5U; ++i) r = step(CENTER);
    assert(r.state == LINE_FOLLOW_REACQUIRE && r.lost_cycles == 21U);
    for (unsigned i = 0U; i < 5U; ++i) r = step(CENTER);
    assert(r.state == LINE_FOLLOW_TRACKING && r.lost_cycles == 0U);
    for (unsigned i = 0U; i <= follow.max_lost_cycles; ++i) r = step(0U);
    assert(r.fault == LINE_FOLLOW_FAULT_LOST_TIMEOUT);
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
    float dy = y - fmaxf(0.0f, fminf(y, 1.2f));
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

/* User-measured wheel track 16.8 cm, probes about 10 cm ahead. Also test
 * 8/12 cm offsets, 0..550 motor dead zones, first-order lag 40/80 ms and
 * signed wheel response. Speeds, sensor spacing and stripe width remain
 * model assumptions. A turn must finish without a full rotation. */
static unsigned max_turn, max_lost;
static float model_max_overshoot, model_max_body_offset;
static float model_dead_zone;
static float model_exit_shift;

static float probe_speed_command(float command, float scale)
{
    float magnitude = fmaxf(0.0f, fabsf(command) - model_dead_zone);
    return copysignf(scale * magnitude / (1000.0f - model_dead_zone), command);
}

static bool dynamic_corner_model(float sensor_offset, float scale, float lag,
                                  float left_gain, float mirror, int yaw_polarity)
{
    float x = 0.1f, y = 0.0f, heading = 0.0f, vl = 0.0f, vr = 0.0f;
    unsigned turn_samples = 0U;
    bool shifted = false;
    LineFollow_Init(); /* each model represents an independent power-on */
    LineFollow_SetYaw(179.0f, 0U, 0U, yaw_polarity != 0);
    LineFollow_Start();
    for (unsigned cycle = 0U; cycle < 200000U; ++cycle) {
        if (!shifted && mirror * y > 0.08f) {
            x += model_exit_shift;
            shifted = true;
        }
        uint8_t black = 0U;
        for (unsigned bit = 0U; bit < 4U; ++bit) {
            float lateral = (1.5f - (float)bit) * 0.01f;
            float sx = x + sensor_offset * cosf(heading) - lateral * sinf(heading);
            float sy = y + sensor_offset * sinf(heading) + lateral * cosf(heading);
            if (corner_distance(sx, mirror * sy) <= 0.0075f) black |= 1U << bit;
        }
        if (yaw_polarity != 0 && cycle % 10U == 0U) {
            LineFollow_SetYaw(wrap_yaw(179.0f + (float)yaw_polarity * heading * 57.2957795f),
                               cycle, cycle, true);
        }
        LineFollowResult_t r = step(black);
        if (r.turn_cycles > max_turn) max_turn = r.turn_cycles;
        if (r.lost_cycles > max_lost) max_lost = r.lost_cycles;
        if (r.fault != LINE_FOLLOW_FAULT_NONE || fabsf(heading) > 3.14f) {
            fprintf(stderr, "Corner dynamics failure: offset %.2f speed %.2f lag %.2f gain %.2f side %+.0f cycle %u xy %.3f %.3f angle %.2f bits %u fault %u phase %u\n",
                    sensor_offset, scale, lag, left_gain, mirror, cycle, x, y,
                    heading, black, r.fault, r.turn_phase);
            return false;
        }
        if (r.state == LINE_FOLLOW_TURNING) ++turn_samples;
        vl += CONTROL_PERIOD_SECONDS / lag * (probe_speed_command(r.left_command, scale) * left_gain - vl);
        vr += CONTROL_PERIOD_SECONDS / lag * (probe_speed_command(r.right_command, scale) - vr);
        heading += (vr - vl) / 0.168f * CONTROL_PERIOD_SECONDS;
        x += 0.5f * (vl + vr) * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += 0.5f * (vl + vr) * sinf(heading) * CONTROL_PERIOD_SECONDS;
        if (mirror * y > 0.03f) model_max_overshoot = fmaxf(model_max_overshoot, mirror * heading - 1.570796327f);
        if (mirror * y > 0.10f) model_max_body_offset = fmaxf(model_max_body_offset, fabsf(x - 0.5f));
        if (mirror * y > 0.6f) {
            bool complete = fabsf(x - 0.5f) < 0.015f &&
                            mirror * sinf(heading) > 0.95f && turn_samples > 0U &&
                            r.state == LINE_FOLLOW_TRACKING;
            if (!complete) {
                fprintf(stderr, "Corner dynamics missed exit: offset %.2f speed %.2f lag %.2f gain %.2f side %+.0f xy %.3f %.3f angle %.2f\n",
                        sensor_offset, scale, lag, left_gain, mirror, x, y, heading);
            }
            return complete;
        }
    }
    return false;
}

static void test_dynamic_corner_models(int yaw_polarity)
{
    const float offsets[] = {0.08f, 0.10f, 0.12f};
    const float scales[] = {0.12f, 0.24f, 0.60f};
    const float dead_zones[] = {0.0f, 250.0f, 350.0f, 450.0f, 500.0f, 550.0f};
    unsigned total = 0U;
    max_turn = max_lost = 0U;
    model_max_overshoot = model_max_body_offset = 0.0f;
    for (unsigned a = 0U; a < 3U; ++a) {
        for (unsigned v = 0U; v < 3U; ++v) {
            for (unsigned dz = 0U; dz < 6U; ++dz) {
                model_dead_zone = dead_zones[dz];
                for (unsigned lag = 0U; lag < 2U; ++lag) {
                    for (unsigned gain = 0U; gain < 2U; ++gain) {
                        for (int side = -1; side <= 1; side += 2) {
                            assert(dynamic_corner_model(offsets[a], scales[v], lag ? 0.08f : 0.04f,
                                                        gain ? 1.03f : 0.97f, (float)side, yaw_polarity));
                            ++total;
                        }
                    }
                }
            }
        }
    }
    printf("Dynamic corner models: yaw polarity %d, %u passed; max turn %u ticks, overshoot %.2f deg, body offset %.1f mm\n",
           yaw_polarity, total, max_turn, model_max_overshoot * 57.2957795f, model_max_body_offset * 1000.0f);
    assert(model_max_body_offset < 0.04f);
    /* Keep R5's lateral capture and tighten the allowed temporary overshoot
     * from about 16 degrees to 13.5 with rate damping on offset samples. */
    if (yaw_polarity != 0) assert(model_max_overshoot * 57.2957795f < 13.5f);
    LineFollow_SetYaw(0.0f, 0U, 0U, false);
}

static void test_displaced_corner_exit(void)
{
    unsigned total = 0U;
    model_dead_zone = 250.0f;
    for (int polarity = -1; polarity <= 1; polarity += 2) {
        for (int side = -1; side <= 1; side += 2) {
            for (int shift = -1; shift <= 1; shift += 2) {
                model_exit_shift = (float)shift * 0.02f;
                for (unsigned lag = 0U; lag < 2U; ++lag) {
                    for (unsigned gain = 0U; gain < 2U; ++gain) {
                        assert(dynamic_corner_model(0.10f, 0.60f, lag ? 0.08f : 0.04f,
                                                    gain ? 1.03f : 0.97f, (float)side, polarity));
                        ++total;
                    }
                }
            }
        }
    }
    printf("Displaced corner exits: %u passed, 20 mm disturbance in both directions\n", total);
    model_exit_shift = 0.0f;
    LineFollow_Init();
}

static float arc_distance(float x, float y)
{
    float sx = fmaxf(0.0f, fminf(x, 0.85f));
    float d = fminf(hypotf(x - sx, y), hypotf(x - sx, y + 1.2f));
    if (x >= 0.85f) d = fminf(d, fabsf(hypotf(x - 0.85f, y + 0.6f) - 0.6f));
    return d;
}

static bool arc_yaw_enabled;

static void test_arc_model(float mirror, float left_gain, float heading,
                           float wheel_track, float sensor_offset)
{
    float x = 0.1f, y = 0.0f, max_error = 0.0f;
    unsigned cycles;
    LineFollow_SetYaw(wrap_yaw(heading * 57.2957795f), 0U, 0U, arc_yaw_enabled);
    LineFollow_Start();
    for (cycles = 0U; cycles < 60000U; ++cycles) {
        uint8_t black = 0U;
        for (unsigned bit = 0U; bit < 4U; ++bit) {
            float lateral = (1.5f - (float)bit) * 0.01f;
            float sx = x + sensor_offset * cosf(heading) - lateral * sinf(heading);
            float sy = y + sensor_offset * sinf(heading) + lateral * cosf(heading);
            if (arc_distance(sx, mirror * sy) <= 0.0075f) black |= (1U << bit);
        }
        if (arc_yaw_enabled && cycles % 10U == 0U) {
            LineFollow_SetYaw(wrap_yaw(heading * 57.2957795f), cycles, cycles, true);
        }
        LineFollowResult_t r = step(black);
        if (arc_yaw_enabled) assert(!r.turn_yaw_assist);
        assert(r.fault == LINE_FOLLOW_FAULT_NONE);
        float vl = 0.6f * r.left_command / 1000.0f * left_gain;
        float vr = 0.6f * r.right_command / 1000.0f;
        heading += (vr - vl) / wheel_track * CONTROL_PERIOD_SECONDS;
        x += 0.5f * (vl + vr) * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += 0.5f * (vl + vr) * sinf(heading) * CONTROL_PERIOD_SECONDS;
        max_error = fmaxf(max_error, arc_distance(x, mirror * y));
        if (mirror * y < -1.15f && x < 0.55f) break;
    }
    printf("Discrete R60 model: yaw %u, wheel track %.1f cm, probes %.1f cm, side %+.0f gain %.2f, %u cycles, max axle error %.1f mm\n",
           arc_yaw_enabled ? 1U : 0U, wheel_track * 100.0f, sensor_offset * 100.0f, mirror, left_gain, cycles, 1000.0f * max_error);
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
static int straight_yaw_polarity;
static float caster_delay;
static bool wheel_model;

static void learn_yaw_polarity(int polarity)
{
    LineFollow_Init();
    LineFollow_SetYaw(0.0f, 10000U, 10000U, true);
    LineFollow_Start();
    (void)step(LEFT_OUTER | CENTER);
    for (unsigned i = 1U; i <= 10U; ++i) {
        LineFollow_SetYaw((float)polarity * (float)i, 10000U + i * 10U,
                           10000U + i * 10U, true);
        (void)step(LEFT_OUTER);
    }
    LineFollow_Stop();
    LineFollow_SetYaw(0.0f, 0U, 0U, false);
}

/* A 20-degree heading error with the probes initially over the line models
 * the sideways approach seen after a corner. Track width/probe offset use
 * the measured 16.8/10 cm geometry. Add actuator lag and 100 Hz quantization. */
static StraightMetrics heading_settling_model(float lag, float gain, float initial_heading, int polarity)
{
    float heading = initial_heading, y = -0.10f * sinf(heading), x = 0.0f;
    float vl = 0.0f, vr = 0.0f, previous = 0.0f, max_tail_angle = 0.0f, yaw_rate = 0.0f;
    float edges_l = 0.0f, edges_r = 0.0f;
    bool feedback_seen = false;
    StraightMetrics metrics = {0};
    uint32_t seed = 43U;
    learn_yaw_polarity(polarity);
    WheelSpeed_Init();
    LineFollow_Start();
    for (unsigned i = 0U; i < 12000U; ++i) {
        uint8_t black = 0U;
        seed = 1664525U * seed + 1013904223U;
        float jitter = ((float)(seed >> 8) / 16777216.0f - 0.5f) * 0.002f;
        for (unsigned bit = 0U; bit < 4U; ++bit) {
            float sy = y + 0.10f * sinf(heading) + (1.5f - (float)bit) * 0.01f * cosf(heading);
            if (fabsf(sy + jitter) <= 0.0075f) black |= 1U << bit;
        }
        if (i % 10U == 0U) {
            float yaw = (float)polarity * heading * 57.2957795f;
            yaw = roundf(yaw * 32768.0f / 180.0f) * 180.0f / 32768.0f;
            LineFollow_SetYaw(wrap_yaw(yaw), i, i, true);
        }
        if (wheel_model) WheelSpeed_Sample((uint32_t)edges_l, (uint32_t)edges_r, i, true);
        LineFollowResult_t r = step(black);
        float command_l = r.left_command, command_r = r.right_command;
        if (wheel_model) {
            WheelSpeed_Apply(&r, &motor, &command_l, &command_r);
            feedback_seen |= g_wheel_speed_status.active;
            assert(command_l + command_r <= r.left_command + r.right_command + 0.01f);
        }
        if (r.fault != LINE_FOLLOW_FAULT_NONE || fabsf(y) >= 0.06f) {
            fprintf(stderr, "Heading settling failed: lag %.2f gain %.2f polarity %d angle %.3f y %.3f bits %u fault %u at %u\n",
                    lag, gain, polarity, heading, y, black, r.fault, i);
            assert(false);
        }
        vl += CONTROL_PERIOD_SECONDS / lag * (0.6f * command_l / 1000.0f * gain - vl);
        vr += CONTROL_PERIOD_SECONDS / lag * (0.6f * command_r / 1000.0f - vr);
        /* Pulse density is a test assumption, not an encoder specification. */
        edges_l += fabsf(vl) * CONTROL_PERIOD_SECONDS * 2000.0f;
        edges_r += fabsf(vr) * CONTROL_PERIOD_SECONDS * 2000.0f;
        float target_rate = (vr - vl) / 0.168f;
        if (caster_delay > 0.0f) yaw_rate += CONTROL_PERIOD_SECONDS / caster_delay * (target_rate - yaw_rate);
        else yaw_rate = target_rate;
        heading += yaw_rate * CONTROL_PERIOD_SECONDS;
        x += 0.5f * (vl + vr) * cosf(heading) * CONTROL_PERIOD_SECONDS;
        y += 0.5f * (vl + vr) * sinf(heading) * CONTROL_PERIOD_SECONDS;
        float correction = 0.5f * (r.left_command - r.right_command);
        if (i >= 2000U) {
            metrics.heading_rms += heading * heading;
            metrics.lateral_rms += y * y;
            metrics.command_variation += fabsf(correction - previous);
            max_tail_angle = fmaxf(max_tail_angle, fabsf(heading));
        }
        previous = correction;
    }
    assert(x > 3.5f); /* must keep driving through the settling interval */
    if (wheel_model) assert(feedback_seen);
    metrics.heading_rms = sqrtf(metrics.heading_rms / 10000.0f);
    metrics.lateral_rms = sqrtf(metrics.lateral_rms / 10000.0f);
    metrics.command_variation /= 10000.0f;
    printf("Heading settling: speed feedback %u caster %.2f lag %.2f gain %.2f start %+.1f polarity %d, RMS %.3f mrad, tail peak %.2f deg, lateral %.2f mm\n",
           wheel_model ? 1U : 0U, caster_delay, lag, gain, initial_heading * 57.2957795f, polarity, metrics.heading_rms * 1000.0f,
           max_tail_angle * 57.2957795f, metrics.lateral_rms * 1000.0f);
    return metrics;
}

static void test_heading_settling_models(void)
{
    for (int polarity = -1; polarity <= 1; polarity += 2) {
        for (unsigned lag_ms = 80U; lag_ms <= 160U; lag_ms += 40U) {
            for (int side = -1; side <= 1; side += 2) {
                for (int gain = -1; gain <= 1; gain += 2) {
                    StraightMetrics m = heading_settling_model((float)lag_ms / 1000.0f,
                        1.0f + 0.03f * (float)gain, 0.34906585f * (float)side, polarity);
                    assert(m.heading_rms < 0.003f && m.lateral_rms < 0.003f);
                }
            }
        }
    }
    LineFollow_Init();
}

static void test_caster_settling_models(void)
{
    /* Extra yaw lag is a sensitivity model of passive rear support/inertia,
     * not a measured caster mechanical model. Keep the original 24 tests. */
    for (unsigned delay = 30U; delay <= 60U; delay += 30U) {
        caster_delay = (float)delay * 0.001f;
        for (int polarity = -1; polarity <= 1; polarity += 2) {
            for (int side = -1; side <= 1; side += 2) {
                for (int gain = -1; gain <= 1; gain += 2) {
                    StraightMetrics m = heading_settling_model(0.12f, 1.0f + 0.03f * (float)gain,
                                                               0.34906585f * (float)side, polarity);
                    assert(m.heading_rms < 0.01f && m.lateral_rms < 0.006f);
                }
            }
        }
    }
    caster_delay = 0.0f;
    LineFollow_Init();
}

static StraightMetrics straight_model(bool baseline, float lag, float gain, float initial_heading)
{
    float x = 0.0f, y = 0.0f, heading = initial_heading, vl = 0.0f, vr = 0.0f;
    float previous = 0.0f;
    uint32_t seed = 43U;
    StraightMetrics metrics = {0};
    if (straight_yaw_polarity != 0) learn_yaw_polarity(straight_yaw_polarity);
    else LineFollow_Init();
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
        if (straight_yaw_polarity != 0 && i % 10U == 0U) {
            /* Left correction produces positive physical heading here. */
            LineFollow_SetYaw(wrap_yaw((float)straight_yaw_polarity * heading * 57.2957795f),
                               i, i, true);
        }
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
            assert(now.command_variation < old.command_variation);
            assert(now.lateral_rms < 0.003f);
            assert(now.heading_rms < 0.003f);
        }
    }
    printf("Aggregate straight heading RMS: fixed steering %.3f, unified %.3f mrad\n",
           old_heading_total * 1000.0f, new_heading_total * 1000.0f);
    for (int side = -1; side <= 1; side += 2) {
        StraightMetrics balanced = straight_model(false, 0.08f, 1.0f, 0.06f * (float)side);
        assert(balanced.lateral_rms < 0.003f && balanced.heading_rms < 0.003f);
    }
    for (int polarity = -1; polarity <= 1; polarity += 2) {
        learn_yaw_polarity(polarity);
        LineFollow_Start();
        LineFollow_SetYaw(0.0f, 0U, 0U, true);
        (void)step(CENTER);
        LineFollow_SetYaw((float)polarity * 0.1f, 10U, 10U, true);
        LineFollowResult_t r = step(CENTER);
        assert(r.left_command > r.right_command); /* oppose positive physical yaw rate */
        r = step(LEFT_INNER);
        assert(fabsf(0.5f * (r.left_command - r.right_command)) < (10.0f * LINE_FOLLOW_RATE_GAIN));
        r = step(RIGHT_INNER);
        assert(0.5f * (r.left_command - r.right_command) > (10.0f * LINE_FOLLOW_RATE_GAIN));
        for (unsigned i = 0U; i < 100U; ++i) (void)step(CENTER);
        LineFollow_SetYaw((float)polarity * 0.1f, 10U, 111U, true);
        r = step(CENTER);
        assert(r.left_command == r.right_command); /* stale gyro must not steer */
        LineFollow_Init();
        LineFollow_Start();
        LineFollow_SetYaw(0.0f, 0U, 0U, true);
        LineFollow_SetYaw(0.1f, 10U, 10U, true);
        r = step(CENTER);
        assert(r.left_command == r.right_command); /* no guessed boot-time polarity */
        for (unsigned lag_ms = 40U; lag_ms <= 120U; lag_ms += 40U) {
            for (int side = -1; side <= 1; side += 2) {
                float gain = 1.0f + (float)side * 0.03f;
                straight_yaw_polarity = 0;
                StraightMetrics no_yaw = straight_model(false, (float)lag_ms / 1000.0f, gain, -(float)side * 0.06f);
                straight_yaw_polarity = polarity;
                StraightMetrics yaw = straight_model(false, (float)lag_ms / 1000.0f, gain, -(float)side * 0.06f);
                printf("Straight yaw model: polarity %d lag %u gain %.2f heading %.3f -> %.3f mrad, command variation %.2f -> %.2f\n",
                       polarity, lag_ms, gain, no_yaw.heading_rms * 1000.0f, yaw.heading_rms * 1000.0f,
                       no_yaw.command_variation, yaw.command_variation);
                assert(yaw.heading_rms < no_yaw.heading_rms);
                assert(yaw.command_variation <= 1.05f * no_yaw.command_variation);
                assert(yaw.lateral_rms < 0.003f);
            }
        }
    }
    straight_yaw_polarity = 0;
    LineFollow_Init();
}

int main(void)
{
    LineFollow_Init();
    LineFollow_Start();
    expect(CENTER, CONTROL_BASE_SPEED_DEFAULT, CONTROL_BASE_SPEED_DEFAULT);
    (void)step(RIGHT_INNER);
    for (unsigned i = 0U; i < LINE_FOLLOW_WHITE_GRACE_CYCLES + 2U; ++i) {
        LineFollowResult_t r = step(0U);
        assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    }
    test_unified_steering();
    test_straight_models();
    test_heading_settling_models();
    test_caster_settling_models();
    wheel_model = true;
    test_caster_settling_models();
    wheel_model = false;
    test_corner_sequences();
    test_corner_watchdogs();
    test_turn_alignment();
    test_exit_edge_is_not_another_corner();
    test_yaw_braking();
    test_dynamic_corner_models(0);
    test_dynamic_corner_models(1);
    test_dynamic_corner_models(-1);
    test_displaced_corner_exit();
    for (int side = -1; side <= 1; side += 2) {
        test_corner_model((float)side, 1.00f, 0.00f);
        test_corner_model((float)side, 0.97f, 0.03f);
        test_corner_model((float)side, 1.03f, -0.03f);
        test_arc_model((float)side, 1.00f, 0.00f, 0.14f, 0.08f);
        test_arc_model((float)side, 1.00f, 0.00f, 0.168f, 0.10f);
        test_arc_model((float)side, 0.97f, 0.03f, 0.14f, 0.08f);
        test_arc_model((float)side, 0.97f, 0.03f, 0.168f, 0.10f);
        test_arc_model((float)side, 1.03f, -0.03f, 0.14f, 0.08f);
        test_arc_model((float)side, 1.03f, -0.03f, 0.168f, 0.10f);
    }
    arc_yaw_enabled = true;
    for (int side = -1; side <= 1; side += 2) {
        test_arc_model((float)side, 1.00f, 0.00f, 0.168f, 0.10f);
        test_arc_model((float)side, 0.97f, 0.03f, 0.168f, 0.10f);
        test_arc_model((float)side, 1.03f, -0.03f, 0.168f, 0.10f);
    }
    LineFollow_SetYaw(0.0f, 0U, 0U, false);
    test_power_on_and_pause();
    test_glitches_and_recovery_budget();
    test_vehicle_turn_gating();
    puts("PASS: unified steering, straight/caster/speed coupling, corners, recovery, watchdogs and gates");
    return 0;
}
