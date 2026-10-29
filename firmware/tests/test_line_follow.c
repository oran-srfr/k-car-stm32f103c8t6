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
    assert(r.fault == LINE_FOLLOW_FAULT_NONE);
    assert(r.left_command >= LINE_FOLLOW_HARD_TURN_MIN_FORWARD);
    assert(r.left_command <= 800.0f);
    assert(r.right_command >= LINE_FOLLOW_HARD_TURN_MIN_FORWARD);
    assert(r.right_command <= 800.0f);
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
    /* One inner sensor: immediate partial steering without full saturation. */
    LineFollow_Start();
    (void)step(CENTER);
    LineFollowResult_t r = step(LEFT_INNER);
    assert(r.right_command - r.left_command > 100.0f);
    assert(r.right_command - r.left_command < 200.0f);
    moving(r);
    r = step(CENTER);
    assert(fabsf(r.control.pd.output) <= CONTROL_PD_D_TERM_LIMIT);
    for (int i = 0; i < 100; ++i) r = step(CENTER);
    assert(fabsf(r.left_command - r.right_command) < 0.01f);
    assert(r.base_speed == CONTROL_BASE_SPEED_DEFAULT);
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

    /* A single spurious detection cannot perpetually reset the search budget. */
    LineFollow_Start();
    (void)step(RIGHT_OUTER);
    for (unsigned i = 0; i < follow.max_lost_cycles; ++i) {
        moving(step(LOST));
        moving(step(RIGHT_OUTER));
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
    assert(r.left_command >= LINE_FOLLOW_HARD_TURN_MIN_FORWARD);
    assert(r.right_command <= 800.0f);
}

static void test_stops_and_curve_hold(void)
{
    LineFollow_Init();
    stopped(step(CENTER));
    LineFollow_Start();
    assert(step(LOST).fault == LINE_FOLLOW_FAULT_NO_VALID_HISTORY);
    LineFollow_Start();
    (void)step(CENTER);
    stopped(step(ALL_BLACK));
    LineFollow_Start();
    moving(step(LEFT_INNER));
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
    for (cycles = 0; cycles < 6000; ++cycles) {
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
    assert(cycles < 6000);
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
