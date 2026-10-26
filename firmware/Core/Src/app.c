#include "app.h"
#include "app_modules.h"
#include <limits.h>

#define SONAR_MAX_AGE_MS 200U
#define OBSTACLE_STOP_MM 80U
#define OBSTACLE_CLEAR_MM 100U
#define PARK_TIMEOUT_MS 30000U /* Provisional watchdog; tune during board tests. */

static AppStatus status;
static uint32_t lap_start_ms;
static uint32_t park_start_ms;
static uint32_t display_ms;
static AppPoint expected_point;
static bool obstacle_latched;
static bool departure_sounded;

static void increment(uint16_t *counter)
{
    if (*counter < UINT16_MAX) { ++*counter; }
}

static void fail(AppError error)
{
    status.error = error;
    status.state = APP_FAULT;
}

void App_Init(uint32_t now_ms)
{
    status = (AppStatus){ .state = APP_WAIT_START };
    lap_start_ms = park_start_ms = display_ms = now_ms;
    expected_point = APP_POINT_B;
    obstacle_latched = departure_sounded = false;
    Modules_Init();
    Modules_Drive(APP_DRIVE_STOP);
}

AppStatus App_GetStatus(void)
{
    return status;
}

static void begin_run(AppMode mode, uint32_t now_ms)
{
    status = (AppStatus){ .state = APP_FOLLOW_LINE, .mode = mode };
    lap_start_ms = now_ms;
    expected_point = APP_POINT_B;
    obstacle_latched = departure_sounded = false;
}

static void follow(const AppInputs *in, uint32_t now_ms)
{
    if (!in->line_valid) { fail(APP_ERROR_LINE_LOST); return; }

    /* B/C tones apply to all modes. Only the ordered B,C,D,A sequence closes a lap. */
    if (in->point_event == APP_POINT_B) { Modules_Sound(APP_SOUND_B); }
    if (in->point_event == APP_POINT_C) { Modules_Sound(APP_SOUND_C); }
    if (status.mode == APP_MODE_LAPS && in->point_event == expected_point) {
        if (expected_point == APP_POINT_A) {
            status.lap_ms[status.laps] = now_ms - lap_start_ms;
            ++status.laps;
            lap_start_ms = now_ms;
            expected_point = APP_POINT_B;
            if (status.laps == 2U) { status.state = APP_FINISHED; }
        } else {
            expected_point = expected_point == APP_POINT_D ? APP_POINT_A :
                             (AppPoint)(expected_point + 1);
        }
    }
    if (status.mode != APP_MODE_LAPS && in->parking_space_event) {
        if (status.mode == APP_MODE_SIDE_PARK) { increment(&status.side_spaces); }
        else { increment(&status.reverse_spaces); }
        status.state = APP_PARKING;
        park_start_ms = now_ms;
        departure_sounded = false;
    }
}

void App_Task(uint32_t now_ms)
{
    AppInputs in;
    Modules_Poll(now_ms, &in);
    bool sonar_fresh = in.sonar_valid &&
                       (uint32_t)(now_ms - in.sonar_sample_ms) <= SONAR_MAX_AGE_MS;
    bool active = status.state == APP_FOLLOW_LINE ||
                  status.state == APP_OBSTACLE_HOLD || status.state == APP_PARKING;

    /* Stop overrides all progress and start events. Restart requires a new press. */
    if (in.stop_pressed) {
        status.state = APP_WAIT_START;
    } else if (!active && in.start_pressed && (unsigned)in.selected_mode <= APP_MODE_REVERSE_PARK &&
               in.line_valid && sonar_fresh && in.distance_mm > OBSTACLE_STOP_MM) {
        begin_run(in.selected_mode, now_ms);
    } else if (active) {
        if (sonar_fresh && in.distance_mm <= OBSTACLE_STOP_MM && !obstacle_latched) {
            obstacle_latched = true;
            increment(&status.obstacles);
            Modules_Sound(APP_SOUND_OBSTACLE);
        }
        if (sonar_fresh && in.distance_mm > OBSTACLE_CLEAR_MM) { obstacle_latched = false; }

        if (status.state == APP_FOLLOW_LINE || status.state == APP_OBSTACLE_HOLD) {
            if (!sonar_fresh || obstacle_latched) { status.state = APP_OBSTACLE_HOLD; }
            else {
                status.state = APP_FOLLOW_LINE;
                follow(&in, now_ms);
            }
        } else if (status.state == APP_PARKING) {
            /* Parking needs its own motion safety model; no blind auto-resume. */
            if (!sonar_fresh || obstacle_latched) { fail(APP_ERROR_PARK_SENSOR); }
            else if (in.park_result == APP_PARK_FAILED) { fail(APP_ERROR_PARK_FAILED); }
            else if ((uint32_t)(now_ms - park_start_ms) >= PARK_TIMEOUT_MS) {
                fail(APP_ERROR_PARK_TIMEOUT);
            } else if (sonar_fresh && !obstacle_latched) {
                if (in.left_line_event && !departure_sounded) {
                    Modules_Sound(APP_SOUND_LEAVE_LINE);
                    departure_sounded = true;
                }
                if (in.park_result == APP_PARK_DONE) { status.state = APP_FINISHED; }
            }
        }
    }

    AppDrive drive = APP_DRIVE_STOP;
    if (sonar_fresh && !obstacle_latched) {
        if (status.state == APP_FOLLOW_LINE) { drive = APP_DRIVE_FOLLOW; }
        else if (status.state == APP_PARKING) {
            drive = status.mode == APP_MODE_SIDE_PARK ? APP_DRIVE_SIDE : APP_DRIVE_REVERSE;
        }
    }
    Modules_Drive(drive);
    if ((uint32_t)(now_ms - display_ms) >= 100U) {
        display_ms = now_ms;
        Modules_Display(&status);
    }
}
