#include "line_track.h"
#include "line_follow.h"
#include "motor_command.h"
#include "pd_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_line_track(void)
{
    LineTrackResult_t r;

    r = LineTrack_Calculate(0x0FU);
    assert(r.status == LINE_TRACK_STATUS_LOST);
    assert(r.black_bits == 0U);

    r = LineTrack_Calculate(0x00U);
    assert(r.status == LINE_TRACK_STATUS_ALL_BLACK);

    /* X1 and X3 are the two inner sensors, so their pair is centered. */
    r = LineTrack_Calculate(0x0AU);
    assert(r.status == LINE_TRACK_STATUS_NORMAL);
    assert(r.black_bits == 0x06U);
    assert(r.deviation == 0.0f);

    r = LineTrack_Calculate(0x0DU);
    assert(r.black_bits == 0x01U);
    assert(r.deviation == -3.0f);

    r = LineTrack_Calculate(0x07U);
    assert(r.black_bits == 0x08U);
    assert(r.deviation == 3.0f);
}

static void test_pd(void)
{
    PDControllerConfig_t config = { 2.0f, 10.0f, 100.0f };
    PDControllerResult_t r;

    PDController_Reset();
    r = PDController_Calculate(LINE_TRACK_STATUS_NORMAL, 1.0f, &config);
    assert(r.status == LINE_TRACK_STATUS_NORMAL);
    assert(r.output == 2.0f);
    assert(r.derivative == 0.0f);

    r = PDController_Calculate(LINE_TRACK_STATUS_NORMAL, 2.0f, &config);
    assert(r.derivative > 0.0f);
    assert(r.output > 2.0f);
    assert(r.output <= config.output_limit);

    r = PDController_Calculate(LINE_TRACK_STATUS_LOST, 5.0f, &config);
    assert(r.output == 0.0f);
}

static void test_motor_command(void)
{
    MotorCommandConfig_t config = { 20.0f, 100.0f };
    MotorCommandResult_t r;

    r = MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL, 5.0f, &config);
    assert(r.left_command == 25.0f);
    assert(r.right_command == 15.0f);

    r = MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL, 200.0f, &config);
    assert(r.left_command == 100.0f);
    assert(r.right_command == 0.0f);

    r = MotorCommand_Calculate(LINE_TRACK_STATUS_LOST, 5.0f, &config);
    assert(r.left_command == 0.0f);
    assert(r.right_command == 0.0f);
}

static LineControlResult_t normal_sample(float deviation)
{
    LineControlResult_t control = {0};

    control.track.status = LINE_TRACK_STATUS_NORMAL;
    control.track.deviation = deviation;
    control.pd.status = LINE_TRACK_STATUS_NORMAL;
    control.pd.output = deviation;
    control.control_status = LINE_CONTROL_STATUS_ACTIVE;
    return control;
}

static LineControlResult_t lost_sample(void)
{
    LineControlResult_t control = {0};

    control.track.status = LINE_TRACK_STATUS_LOST;
    control.pd.status = LINE_TRACK_STATUS_LOST;
    control.control_status = LINE_CONTROL_STATUS_LOST_STOP;
    return control;
}

static LineControlResult_t cross_sample(void)
{
    LineControlResult_t control = {0};

    control.track.status = LINE_TRACK_STATUS_ALL_BLACK;
    control.track.black_bits = 0x0FU;
    control.pd.status = LINE_TRACK_STATUS_ALL_BLACK;
    control.control_status = LINE_CONTROL_STATUS_ALL_BLACK_STOP;
    return control;
}

/* Provide the hardware-facing sample function so line_follow.c can link on host. */
LineControlResult_t LineControl_Sample(const PDControllerConfig_t *config)
{
    (void)config;
    return normal_sample(0.0f);
}

static void test_line_follow_search_and_crossing(void)
{
    MotorCommandConfig_t motor = { 400.0f, 600.0f };
    LineFollowConfig_t follow = {
        .curve_speed = 300.0f,
        .sharp_curve_speed = 250.0f,
        .search_speed = 200.0f,
        .search_inner_speed = -80.0f,
        .straight_max_deviation = 0.5f,
        .curve_max_deviation = 1.5f,
        .max_lost_cycles = 10U,
        .search_reverse_cycles = 2U,
        .max_cross_cycles = 2U,
        .reacquire_cycles = 2U
    };
    LineControlResult_t control;
    LineFollowResult_t r;

    LineFollow_Start();
    control = normal_sample(3.0f);
    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.state == LINE_FOLLOW_TRACKING);

    control = lost_sample();
    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.state == LINE_FOLLOW_LINE_LOST);
    assert(r.left_command == follow.search_speed);
    assert(r.right_command == follow.search_inner_speed);

    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.state == LINE_FOLLOW_SEARCHING);
    assert(r.left_command == follow.search_speed);

    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.left_command == follow.search_inner_speed);
    assert(r.right_command == follow.search_speed);

    LineFollow_Start();
    control = cross_sample();
    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.state == LINE_FOLLOW_CROSSING);
    assert(r.left_command == motor.base_speed);
    assert(r.right_command == motor.base_speed);

    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.state == LINE_FOLLOW_CROSSING);

    r = LineFollow_UpdateWithControl(&control, &motor, &follow);
    assert(r.state == LINE_FOLLOW_FAULT);
    assert(r.fault == LINE_FOLLOW_FAULT_ALL_BLACK);
}

int main(void)
{
    test_line_track();
    test_pd();
    test_motor_command();
    test_line_follow_search_and_crossing();
    puts("PASS: tracking geometry, PD, motor mixing, search, and crossing");
    return 0;
}
