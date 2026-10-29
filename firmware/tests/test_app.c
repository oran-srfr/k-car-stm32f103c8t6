#include "app.h"
#include "app_modules.h"
#include <assert.h>
#include <stdio.h>

static AppInputs input;
static AppDrive drive;
static unsigned sounds[4];
static unsigned display_count;

void Modules_Init(void) {}
void Modules_Poll(uint32_t now, AppInputs *out) { (void)now; *out = input; }
void Modules_RunStarted(AppMode mode, uint32_t now) { (void)mode; (void)now; }
void Modules_RunStopped(void) {}
void Modules_Drive(AppDrive command) { drive = command; }
void Modules_Sound(AppSound event) { ++sounds[event]; }
void Modules_Display(const AppStatus *s) { (void)s; ++display_count; }

static void step(uint32_t now)
{
    input.sonar_sample_ms = now;
    App_Task(now);
    input.start_pressed = input.stop_pressed = false;
    input.point_event = APP_POINT_NONE;
    input.parking_space_event = input.left_line_event = false;
}

static void begin(AppMode mode, uint32_t now)
{
    input = (AppInputs){ .line_valid = true, .line_start_ready = true, .sonar_valid = true,
                        .distance_mm = 500, .start_pressed = true, .selected_mode = mode };
    for (unsigned i = 0; i < 4; ++i) { sounds[i] = 0; }
    App_Init(now - APP_AUTO_START_STABLE_MS);
    assert(drive == APP_DRIVE_STOP);
    for (unsigned dt = 0; dt <= APP_AUTO_START_STABLE_MS; dt += 10) {
        step(now - APP_AUTO_START_STABLE_MS + dt);
    }
    assert(App_GetStatus().state == APP_FOLLOW_LINE);
}

static void waypoint(AppPoint point, uint32_t now)
{
    input.point_event = point;
    step(now);
}

static void test_laps(void)
{
    begin(APP_MODE_LAPS, 0);
    waypoint(APP_POINT_A, 10); /* Start-area duplicate cannot count a lap. */
    waypoint(APP_POINT_D, 20); /* Missing B/C also cannot count a lap. */
    waypoint(APP_POINT_A, 30);
    assert(App_GetStatus().laps == 0);
    for (unsigned lap = 0; lap < 2; ++lap) {
        uint32_t base = lap * 1000;
        waypoint(APP_POINT_B, base + 100);
        waypoint(APP_POINT_C, base + 200);
        waypoint(APP_POINT_D, base + 300);
        waypoint(APP_POINT_A, base + 1000);
    }
    AppStatus s = App_GetStatus();
    assert(s.laps == 2 && s.lap_ms[0] == 1000 && s.lap_ms[1] == 1000);
    assert(s.state == APP_FINISHED && drive == APP_DRIVE_STOP);
    assert(sounds[APP_SOUND_B] == 2 && sounds[APP_SOUND_C] == 2);
    waypoint(APP_POINT_A, 3000);
    assert(App_GetStatus().laps == 2);
    for (unsigned now = 3010; now <= 3500; now += 10) { step(now); }
    assert(App_GetStatus().state == APP_FINISHED && drive == APP_DRIVE_STOP);
    assert(!App_GetStatus().auto_start_armed);
    /* Even a legacy button event cannot silently restart a finished task. */
    input.start_pressed = true;
    step(3510);
    assert(App_GetStatus().state == APP_FINISHED);
    g_app_control_request = APP_CONTROL_REARM;
    step(3520);
    assert(drive == APP_DRIVE_STOP && App_GetStatus().auto_start_armed);
    for (unsigned now = 3530; now <= 3730; now += 10) { step(now); }
    assert(App_GetStatus().state == APP_FOLLOW_LINE && App_GetStatus().laps == 0);
}

static void test_obstacle(void)
{
    begin(APP_MODE_LAPS, 0);
    input.distance_mm = 80;
    step(10);
    assert(App_GetStatus().state == APP_OBSTACLE_HOLD && drive == APP_DRIVE_STOP);
    step(20);
    input.distance_mm = 90;
    step(30);
    assert(App_GetStatus().obstacles == 1 && sounds[APP_SOUND_OBSTACLE] == 1);
    assert(drive == APP_DRIVE_STOP);
    input.sonar_valid = false;
    step(40);
    assert(App_GetStatus().state == APP_FOLLOW_LINE && drive == APP_DRIVE_FOLLOW);
    input.sonar_valid = true;
    input.distance_mm = 101;
    step(50);
    assert(App_GetStatus().state == APP_FOLLOW_LINE && drive == APP_DRIVE_FOLLOW);
    input.distance_mm = 70;
    step(60);
    assert(App_GetStatus().obstacles == 2);
}

static void test_invalid_and_stop(void)
{
    App_Init(0);
    input = (AppInputs){ .start_pressed = true };
    step(0);
    assert(App_GetStatus().state == APP_WAIT_START && drive == APP_DRIVE_STOP);
    begin(APP_MODE_LAPS, 0);
    /* A missing/stale sonar sample must not gate line following. */
    App_Task(201);
    assert(App_GetStatus().state == APP_FOLLOW_LINE && drive == APP_DRIVE_FOLLOW);
    step(202);
    input.line_valid = false;
    step(203);
    assert(App_GetStatus().state == APP_FAULT && drive == APP_DRIVE_STOP);
    input.line_valid = true;
    step(204);
    assert(App_GetStatus().state == APP_WAIT_START && drive == APP_DRIVE_STOP);
    assert(App_GetStatus().error == APP_ERROR_NONE);
    assert(App_GetStatus().auto_start_armed);
    for (unsigned now = 214; now <= 404; now += 10) { step(now); }
    /* Rearm happens at 204; the first start sample is 214. 404 is only 190 ms. */
    assert(App_GetStatus().state == APP_WAIT_START && drive == APP_DRIVE_STOP);
    step(414);
    assert(App_GetStatus().state == APP_FOLLOW_LINE && drive == APP_DRIVE_FOLLOW);
}

static void valid_wait(uint32_t now)
{
    input = (AppInputs){ .line_valid = true, .line_start_ready = true,
                        .sonar_valid = true, .distance_mm = 500 };
    App_Init(now);
}

static void test_auto_start(void)
{
    valid_wait(0);
    for (unsigned now = 0; now <= 190; now += 10) { step(now); }
    assert(App_GetStatus().state == APP_WAIT_START && drive == APP_DRIVE_STOP);
    step(199);
    assert(App_GetStatus().start_stable_ms == 199);
    step(200);
    assert(App_GetStatus().state == APP_FOLLOW_LINE && drive == APP_DRIVE_FOLLOW);
    assert(!App_GetStatus().auto_start_armed);
}

static void test_start_resets(void)
{
    valid_wait(0);
    for (unsigned now = 0; now <= 190; now += 10) { step(now); }
    /* Search/all-black may have valid controller outputs but cannot start. */
    input.line_start_ready = false;
    step(200);
    assert(App_GetStatus().start_stable_ms == 0);
    input.line_start_ready = true;
    for (unsigned now = 210; now <= 400; now += 10) { step(now); }
    assert(drive == APP_DRIVE_STOP);
    step(410);
    assert(drive == APP_DRIVE_FOLLOW);

    valid_wait(0);
    step(0);
    step(500); /* Long blind gap cannot qualify as stable sensing. */
    assert(App_GetStatus().start_stable_ms == 0 && drive == APP_DRIVE_STOP);
    for (unsigned now = 510; now <= 690; now += 10) { step(now); }
    input.selected_mode = APP_MODE_SIDE_PARK;
    step(700); /* A mode change also requires a new qualification interval. */
    assert(App_GetStatus().start_stable_ms == 0 && drive == APP_DRIVE_STOP);
    for (unsigned now = 710; now <= 900; now += 10) { step(now); }
    assert(App_GetStatus().mode == APP_MODE_SIDE_PARK && drive == APP_DRIVE_FOLLOW);
}

static void test_start_ignores_sonar(void)
{
    valid_wait(0);
    input.sonar_valid = false;
    input.distance_mm = 0;
    for (unsigned now = 0; now <= 200; now += 10) { step(now); }
    assert(App_GetStatus().state == APP_FOLLOW_LINE && drive == APP_DRIVE_FOLLOW);
}

static void test_stop_rearm_and_wrap(void)
{
    valid_wait(UINT32_MAX - 99U);
    for (unsigned dt = 0; dt <= 200; dt += 10) { step(UINT32_MAX - 99U + dt); }
    assert(drive == APP_DRIVE_FOLLOW);
    g_app_control_request = APP_CONTROL_STOP;
    step(111);
    for (unsigned now = 121; now <= 411; now += 10) { step(now); }
    assert(App_GetStatus().state == APP_WAIT_START && !App_GetStatus().auto_start_armed);
    assert(drive == APP_DRIVE_STOP && g_app_control_request == APP_CONTROL_NONE);
    input.stop_pressed = true;
    g_app_control_request = APP_CONTROL_REARM;
    step(421);
    assert(!App_GetStatus().auto_start_armed); /* Stop wins. */
    g_app_control_request = APP_CONTROL_REARM;
    step(431);
    assert(App_GetStatus().auto_start_armed && drive == APP_DRIVE_STOP);
    for (unsigned now = 441; now <= 631; now += 10) { step(now); }
    assert(drive == APP_DRIVE_STOP);
    step(641);
    assert(drive == APP_DRIVE_FOLLOW);
}

static void park_start(AppMode mode)
{
    begin(mode, 0);
    input.parking_space_event = true;
    step(10);
    assert(App_GetStatus().state == APP_PARKING);
    assert(drive == (mode == APP_MODE_SIDE_PARK ? APP_DRIVE_SIDE : APP_DRIVE_REVERSE));
}

static void test_parking(void)
{
    for (AppMode mode = APP_MODE_SIDE_PARK; mode <= APP_MODE_REVERSE_PARK; ++mode) {
        park_start(mode);
        input.parking_space_event = true;
        input.left_line_event = true;
        step(20);
        input.left_line_event = true;
        step(30);
        assert(sounds[APP_SOUND_LEAVE_LINE] == 1);
        AppStatus s = App_GetStatus();
        assert(s.side_spaces + s.reverse_spaces == 1);
        input.park_result = APP_PARK_DONE;
        step(40);
        assert(App_GetStatus().state == APP_FINISHED && drive == APP_DRIVE_STOP);
    }
    park_start(APP_MODE_SIDE_PARK);
    step(30010);
    assert(App_GetStatus().error == APP_ERROR_PARK_TIMEOUT && drive == APP_DRIVE_STOP);
    park_start(APP_MODE_SIDE_PARK);
    input.park_result = APP_PARK_FAILED;
    step(20);
    assert(App_GetStatus().error == APP_ERROR_PARK_FAILED && drive == APP_DRIVE_STOP);
    park_start(APP_MODE_REVERSE_PARK);
    input.sonar_valid = false;
    step(20);
    assert(App_GetStatus().error == APP_ERROR_PARK_SENSOR && drive == APP_DRIVE_STOP);
}

static void test_wrap_and_display(void)
{
    begin(APP_MODE_LAPS, UINT32_MAX - 500U);
    unsigned old = display_count;
    step(UINT32_MAX - 450U);
    assert(display_count == old);
    waypoint(APP_POINT_B, UINT32_MAX - 400U);
    assert(display_count == old + 1);
    waypoint(APP_POINT_C, UINT32_MAX - 300U);
    waypoint(APP_POINT_D, UINT32_MAX - 200U);
    waypoint(APP_POINT_A, 499U);
    assert(App_GetStatus().lap_ms[0] == 1000);
}

int main(void)
{
    test_laps();
    test_obstacle();
    test_invalid_and_stop();
    test_parking();
    test_wrap_and_display();
    test_auto_start();
    test_start_resets();
    test_start_ignores_sonar();
    test_stop_rearm_and_wrap();
    puts("PASS: 9 application scenario groups");
    return 0;
}
