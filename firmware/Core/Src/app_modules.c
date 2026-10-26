#include "app_modules.h"
#include "Attitude.h"
#include "HCI.h"
#include "board_pins.h"
#include "lcd_init.h"
#include "ultrasonic.h"
#include "stm32f1xx_hal.h"

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
    Ulr_Task(now_ms);
    UlrReading sample = Ulr_GetReading();
    inputs->sonar_valid = sample.status == ULR_OK || sample.status == ULR_TOO_CLOSE;
    inputs->distance_mm = sample.status == ULR_TOO_CLOSE ? 0U : sample.distance_mm;
    inputs->sonar_sample_ms = sample.sample_ms;
}

static void Mpu_Poll(uint32_t now_ms)
{
    Attitude_Poll(now_ms);
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
    (void)Ulr_Init(HAL_GetTick());
    LCD_Init();
    HCI_Init();
    (void)Attitude_Init(ATTITUDE_BAUD_RATE);
    /* TODO: motor safe-stop, line sensors, encoders, buzzer.
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
     * Sonar publishes actual sample timestamps; failure => invalid.
     * TODO parking: nonblocking maneuver progress, DONE only after verified stop.
     * Interrupt handlers should only capture edges/bytes, never block here. */
    *inputs = (AppInputs){ .park_result = APP_PARK_BUSY };
    HMI_Poll(inputs);
    Vision_Poll(inputs);
    Mpu_Poll(now_ms);
    Sonar_Poll(now_ms, inputs);
    Tracking_Poll(inputs);
    Parking_Poll(inputs);
}

void Modules_RunStarted(AppMode mode, uint32_t now_ms)
{
    if (mode == APP_MODE_LAPS) {
        Attitude_StartLap(now_ms);
    } else {
        Attitude_StopLap();
    }
}

void Modules_RunStopped(void)
{
    Attitude_StopLap();
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
    AttitudeStatus attitude;

    Attitude_GetStatus(&attitude);
    HCI_Lap1Ms = attitude.lap.lap1_ms;
    HCI_Lap2Ms = attitude.lap.lap2_ms;
    HCI_ObsCount = status->obstacles;
    HCI_RevCount = status->reverse_spaces;
    HCI_SideCount = status->side_spaces;
    HCI_Update();
}
