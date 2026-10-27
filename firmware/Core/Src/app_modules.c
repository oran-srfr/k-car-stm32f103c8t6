#include "app_modules.h"
#include "Attitude.h"
#include "HCI.h"
#include "board_pins.h"
#include "buzzer.h"
#include "control_config.h"
#include "lcd.h"
#include "lcd_init.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include "vehicle_control.h"
#include "stm32f1xx_hal.h"

static PDControllerConfig_t s_pd_config = {
    CONTROL_KP_DEFAULT,
    CONTROL_KD_DEFAULT,
    CONTROL_PD_OUTPUT_LIMIT_DEFAULT
};
static MotorCommandConfig_t s_motor_command_config = {
    CONTROL_BASE_SPEED_DEFAULT,
    CONTROL_MAX_FORWARD_COMMAND_DEFAULT
};
static LineFollowConfig_t s_line_follow_config = {
    LINE_FOLLOW_CURVE_SPEED_DEFAULT,
    LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT,
    LINE_FOLLOW_SEARCH_SPEED_DEFAULT,
    LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT,
    LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT,
    LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT,
    LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT,
    LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT
};
static uint32_t s_last_control_ms;
static char s_debug_line[3][17];

volatile LineTrackStatus_t g_tracking_status = LINE_TRACK_STATUS_LOST;
volatile uint8_t g_tracking_bits = 0U;
volatile float g_tracking_deviation = 0.0f;

static void HMI_Poll(AppInputs *inputs)
{
    inputs->selected_mode = g_app_selected_mode;
    /* Automatic launch needs no button. Physical stop/mode UI can be added here. */
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

static void Tracking_Poll(uint32_t now_ms, AppInputs *inputs)
{
    VehicleControlOutput_t vehicle;

    if ((uint32_t)(now_ms - s_last_control_ms) >= CONTROL_PERIOD_MS) {
        s_last_control_ms = now_ms;
        VehicleControl_Update();
    }

    vehicle = VehicleControl_GetOutput();
    if ((vehicle.line_follow.state == LINE_FOLLOW_FAULT) &&
        (vehicle.line_follow.control.track.status == LINE_TRACK_STATUS_NORMAL)) {
        /* The board may be powered before it is placed on the track. */
        VehicleControl_Start();
        VehicleControl_Update();
        vehicle = VehicleControl_GetOutput();
    }
    inputs->line_valid = vehicle.line_output_valid;
    inputs->line_start_ready = vehicle.line_output_valid &&
        vehicle.line_follow.control.track.status == LINE_TRACK_STATUS_NORMAL;
    g_tracking_status = vehicle.line_follow.control.track.status;
    g_tracking_bits = vehicle.line_follow.control.track.black_bits;
    g_tracking_deviation = vehicle.line_follow.control.track.deviation;
}

static void Parking_Poll(AppInputs *inputs)
{
    (void)inputs; /* TODO: left_line_event and locally verified park_result. */
}

void Modules_Init(void)
{
    (void)Buzzer_Init();
    (void)Ulr_Init(HAL_GetTick());
    LineSensor_Init();
    MotorDriver_Init();
    VehicleControl_Init(&s_pd_config, &s_motor_command_config, &s_line_follow_config);
    VehicleControl_Start();
    s_last_control_ms = HAL_GetTick();
    LCD_Init();
    HCI_Init();
    (void)Attitude_Init(ATTITUDE_BAUD_RATE);
    /* Motor output remains gated until the application requests FOLLOW. */
}

void Modules_Poll(uint32_t now_ms, AppInputs *inputs)
{
    Buzzer_Task(now_ms);
    /* Line validity comes from the four-sensor PD follower. Position events,
     * physical HMI controls and vision remain placeholders; do not invent them.
     * HMI mode comes from the debugger until physical controls are wired.
     * TODO position: qualified A/B/C/D waypoint events.
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
    Tracking_Poll(now_ms, inputs);
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
    VehicleControl_SetMotorOutputEnabled(false);
    Attitude_StopLap();
}

void Modules_Drive(AppDrive command)
{
    if (command == APP_DRIVE_FOLLOW) {
        VehicleControl_SetMotorOutputEnabled(true);
    } else {
        VehicleControl_SetMotorOutputEnabled(false);
    }
}

void Modules_Sound(AppSound event)
{
    BuzzerEvent sound;
    switch (event) {
    case APP_SOUND_B: sound = BUZZER_POINT_B; break;
    case APP_SOUND_C: sound = BUZZER_POINT_C; break;
    case APP_SOUND_OBSTACLE: sound = BUZZER_OBSTACLE; break;
    case APP_SOUND_LEAVE_LINE: sound = BUZZER_LEAVE_LINE; break;
    default: return;
    }
    (void)Buzzer_Play(sound, HAL_GetTick());
}

static void DebugLine_Clear(char *line)
{
    uint8_t i;

    for (i = 0U; i < 16U; ++i) {
        line[i] = ' ';
    }
    line[16] = '\0';
}

static void DebugLine_Append(char *line, const char *text)
{
    uint8_t pos = 0U;

    while (pos < 16U && line[pos] != ' ') {
        ++pos;
    }
    while (*text != '\0' && pos < 16U) {
        line[pos++] = *text++;
    }
}

static void DebugLine_AppendU16(char *line, uint16_t value, uint8_t width)
{
    uint8_t pos = 0U;
    uint8_t i;

    while (pos < 16U && line[pos] != ' ') {
        ++pos;
    }
    if ((uint16_t)pos + width > 16U) {
        return;
    }
    for (i = width; i > 0U; --i) {
        line[pos + i - 1U] = (char)('0' + value % 10U);
        value /= 10U;
    }
}

static void DebugLine_AppendHex8(char *line, uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";

    uint8_t pos = 0U;
    while (pos < 16U && line[pos] != ' ') {
        ++pos;
    }
    if (pos + 2U <= 16U) {
        line[pos] = digits[(value >> 4U) & 0x0FU];
        line[pos + 1U] = digits[value & 0x0FU];
    }
}

static void DebugLine_Show(uint16_t y, uint8_t index, const char *text,
                           uint16_t color)
{
    if (s_debug_line[index][0] != text[0] ||
        s_debug_line[index][1] != text[1] ||
        s_debug_line[index][2] != text[2] ||
        s_debug_line[index][3] != text[3] ||
        s_debug_line[index][4] != text[4] ||
        s_debug_line[index][5] != text[5] ||
        s_debug_line[index][6] != text[6] ||
        s_debug_line[index][7] != text[7] ||
        s_debug_line[index][8] != text[8] ||
        s_debug_line[index][9] != text[9] ||
        s_debug_line[index][10] != text[10] ||
        s_debug_line[index][11] != text[11] ||
        s_debug_line[index][12] != text[12] ||
        s_debug_line[index][13] != text[13] ||
        s_debug_line[index][14] != text[14] ||
        s_debug_line[index][15] != text[15]) {
        uint8_t i;
        for (i = 0U; i < 16U; ++i) {
            s_debug_line[index][i] = text[i];
        }
        LCD_ShowString(0U, y, (const uint8_t *)text, color, BLACK, 16U, 0U);
    }
}

static void Debug_Display(const AppStatus *status)
{
    char line[17];

    DebugLine_Clear(line);
    DebugLine_Append(line, "S");
    DebugLine_AppendU16(line, (uint16_t)status->state, 1U);
    DebugLine_Append(line, "E");
    DebugLine_AppendU16(line, (uint16_t)status->error, 1U);
    DebugLine_Append(line, " L");
    DebugLine_AppendU16(line, (uint16_t)g_tracking_status, 1U);
    DebugLine_Append(line, " B");
    DebugLine_AppendHex8(line, g_tracking_bits);
    DebugLine_Append(line, " O");
    DebugLine_AppendU16(line, g_motor_output_enabled ? 1U : 0U, 1U);
    DebugLine_Append(line, "D");
    DebugLine_AppendU16(line, g_motor_driver_enabled ? 1U : 0U, 1U);
    DebugLine_Show(112U, 0U, line, WHITE);

    DebugLine_Clear(line);
    DebugLine_Append(line, "A");
    DebugLine_AppendU16(line, (uint16_t)g_motor_driver_left_actual, 4U);
    DebugLine_Append(line, "/");
    DebugLine_AppendU16(line, (uint16_t)g_motor_driver_right_actual, 4U);
    DebugLine_Show(128U, 1U, line, GREEN);

    DebugLine_Clear(line);
    DebugLine_Append(line, "P");
    DebugLine_AppendU16(line, (uint16_t)g_motor_driver_left_pwm_compare, 4U);
    DebugLine_Append(line, "/");
    DebugLine_AppendU16(line, (uint16_t)g_motor_driver_right_pwm_compare, 4U);
    DebugLine_Show(144U, 2U, line, YELLOW);
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
    Debug_Display(status);
}
