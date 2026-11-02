#include "app_modules.h"
#include "Attitude.h"
#include "HCI.h"
#include "board_pins.h"
#include "buzzer.h"
#include "control_config.h"
#include "encoder.h"
#include "lcd.h"
#include "lcd_init.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include "vehicle_control.h"
#include "stm32f1xx_hal.h"

/* 说明：原先这里有一个"状态检测"调试行（S/E/L/O，画在 y=112），
 * 它和编码器读数的 L 行使用同一个纵坐标，互相覆盖。
 * 现在状态检测已不需要（循迹状态用调试器看 g_tracking_status 即可），
 * 已整段删除，y=112 / y=132 全部让给编码器测速读数。 */

static PDControllerConfig_t s_pd_config = {
    CONTROL_KP_DEFAULT,
    CONTROL_KD_DEFAULT,
    CONTROL_PD_OUTPUT_LIMIT_DEFAULT
};
static MotorCommandConfig_t s_motor_command_config = {
    CONTROL_BASE_SPEED_DEFAULT,
    CONTROL_MAX_FORWARD_COMMAND_DEFAULT,
    CONTROL_MIN_FORWARD_COMMAND_DEFAULT,
    CONTROL_STRAIGHT_TRIM_DEFAULT
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
static char s_debug_line[2][17];

volatile LineTrackStatus_t g_tracking_status = LINE_TRACK_STATUS_LOST;
/* Electrical X1..X4 levels BEFORE spatial mapping; use to identify SW->X. */
volatile uint8_t g_tracking_raw_bits = 0x0FU;
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
    g_tracking_raw_bits = vehicle.line_follow.control.raw_bits;
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
    /* 编码器测速：只计数，不参与控制。用来判断"同样命令下左右轮实际转速是否一致"。
     * 放在 LCD/HCI 之后初始化，避免占用的 EXTI 影响显示引脚扫描。 */
    Encoder_Init();
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
    Encoder_Poll(now_ms);
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

static void DebugLine_Show(uint16_t y, uint8_t index, const char *text,
                           uint16_t color)
{
    uint8_t i;

    for (i = 0U; i < 16U; ++i) {
        if (s_debug_line[index][i] != text[i]) {
            for (i = 0U; i < 16U; ++i) {
                s_debug_line[index][i] = text[i];
            }
            LCD_ShowString(0U, y, (const uint8_t *)text, color, BLACK, 16U, 0U);
            return;
        }
    }
}

/* ============================================================================
 * 编码器测速读数（y=112 / y=132，1 Hz 刷新）
 *
 * 用途：判断"同样命令下左右轮实际转速是否一致"。
 * 之前摆头到底是控制参数问题还是电机个体差异一直无法定论，
 * 因为有编码器之前只能靠肉眼看轮子转得快慢 —— 人眼分辨不了。
 *
 * 显示格式：
 *   y=112:  L <左轮最近 100ms 脉冲数>
 *   y=132:  R <右轮最近 100ms 脉冲数>
 * 两轮命令相同时（两个内探头都压线），这两个数应当接近相等；
 * 长期相差 10% 以上就是电机/减速箱的开环不对称 → 用 straight_trim 补偿。
 *
 * 刷新率 1 Hz：两行共约 12 个字符，每次约 16 ms，摊到 1 秒是 1.6% CPU，
 * 换来看得见的实时数据是值得的。车子静止时数字不变，脏标记会挡住重绘。
 * ========================================================================== */
#define ENC_READOUT_MS   (1000U)
static uint32_t s_enc_readout_ms;

static void Encoder_Readout(uint32_t now_ms)
{
    char line[17];

    if ((uint32_t)(now_ms - s_enc_readout_ms) < ENC_READOUT_MS) {
        return;
    }
    s_enc_readout_ms = now_ms;

    DebugLine_Clear(line);
    DebugLine_Append(line, "L");
    DebugLine_AppendU16(line, Encoder_GetLeftDelta(), 5U);
    DebugLine_Show(112U, 0U, line, CYAN);

    DebugLine_Clear(line);
    DebugLine_Append(line, "R");
    DebugLine_AppendU16(line, Encoder_GetRightDelta(), 5U);
    DebugLine_Show(132U, 1U, line, MAGENTA);
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
    Encoder_Readout(HAL_GetTick());
}
