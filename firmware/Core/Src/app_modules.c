#include "app_modules.h"

static void HMI_Poll(AppInputs *inputs)
{
    (void)inputs; /* TODO: start/stop button edges and selected_mode. */
}

static void Vision_Poll(AppInputs *inputs)
{
    (void)inputs; /* TODO: K230 frame parsing and parking_space_event. */
}

static void Sonar_Poll(uint32_t now_ms, AppInputs *inputs)
{
    (void)now_ms;
    (void)inputs; /* TODO: valid distance and acquisition timestamp, no busy wait. */
}

static void Tracking_Poll(AppInputs *inputs)
{
    (void)inputs; /* TODO: line_valid + qualified geometric waypoint events. */
}

static void Parking_Poll(AppInputs *inputs)
{
    (void)inputs; /* TODO: left_line_event and locally verified park_result. */
}

void Modules_Init(void)
{
    /* TODO: motor safe-stop, line sensors, encoders, UARTs, sonar, buzzer, HMI.
     * Keep motors stopped until all required inputs have been validated. */
}

void Modules_Poll(uint32_t now_ms, AppInputs *inputs)
{
    (void)now_ms;
    /* Safe placeholders: no start event, no valid sensors, no invented results.
     * TODO HMI: edge-triggered start/stop and mode selection.
     * TODO line/attitude: line validity and qualified A/B/C/D waypoint events.
     * TODO vision: parse K230 UART in foreground, validate and deduplicate frames;
     * publish parking_space_event for the selected mode.
     * TODO sonar: distance_mm + actual sample timestamp; timeout => invalid.
     * TODO parking: nonblocking maneuver progress, DONE only after verified stop.
     * Interrupt handlers should only capture edges/bytes, never block here. */
    *inputs = (AppInputs){ .park_result = APP_PARK_BUSY };
    HMI_Poll(inputs);
    Vision_Poll(inputs);
    Sonar_Poll(now_ms, inputs);
    Tracking_Poll(inputs);
    Parking_Poll(inputs);
}

void Modules_Drive(AppDrive command)
{
    (void)command;
    /* TODO motor controller: STOP, line following, side/reverse maneuver step.
     * STOP must be idempotent and cancel/reset any parking maneuver progress.
     * No GPIO/PWM is enabled by this scaffold. */
}

void Modules_Sound(AppSound event)
{
    (void)event;
    /* TODO enqueue a tone pattern; PWM timing must not block the main loop. */
}

void Modules_Display(const AppStatus *status)
{
    (void)status;
    /* TODO render the two lap times and three identification counters.
     * Called every 100 ms; repaint only changed fields. HMI must not drive motors. */
}
