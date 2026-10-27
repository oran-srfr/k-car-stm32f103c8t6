#include "line_track.h"
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

int main(void)
{
    test_line_track();
    test_pd();
    test_motor_command();
    puts("PASS: tracking geometry, PD, and motor mixing");
    return 0;
}
