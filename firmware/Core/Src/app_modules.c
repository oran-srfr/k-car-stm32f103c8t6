#include "app_modules.h"
#include <stddef.h>
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
/* 来自 line_follow.c 的角速度镜像（黑匣子用） */
extern volatile float g_lf_yaw_rate_dps;
extern volatile float g_lf_yaw_rate_slow;
static uint32_t s_last_control_ms;
static char s_debug_line[2][17];

volatile LineTrackStatus_t g_tracking_status = LINE_TRACK_STATUS_LOST;
/* Electrical X1..X4 levels BEFORE spatial mapping; use to identify SW->X. */
volatile uint8_t g_tracking_raw_bits = 0x0FU;
volatile uint8_t g_tracking_bits = 0U;
volatile float g_tracking_deviation = 0.0f;

/* Inspect these symbols together when an unexpected alarm occurs. */
volatile uint8_t g_sonar_status;
volatile uint16_t g_sonar_distance_mm;
volatile uint32_t g_sonar_sample_ms;
volatile uint32_t g_sound_event_count;
volatile uint8_t g_sound_last_event;

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
    g_sonar_status = (uint8_t)sample.status;
    g_sonar_distance_mm = sample.distance_mm;
    g_sonar_sample_ms = sample.sample_ms;
    inputs->sonar_valid = sample.status == ULR_OK || sample.status == ULR_TOO_CLOSE;
    inputs->distance_mm = sample.status == ULR_TOO_CLOSE ? 0U : sample.distance_mm;
    inputs->sonar_sample_ms = sample.sample_ms;
}

static void Mpu_Poll(uint32_t now_ms)
{
    Attitude_Poll(now_ms);
}

/* ===== 完赛判定：跑满两圈 + 过完下一个直角（A 点）→ 300 ms 减速停车 =====
 *
 * 圈数由 LapTimer 按陀螺仪累积转角统计（一圈 360°，两圈 720°）。
 * 场地是顺时针、两个直角（A、D），从起点跑满 720° 时正好回到起点附近，
 * 之后遇到的下一个直角就是 A 点 —— 过完这个直角再停，符合赛题要求。
 *
 * 只读现有状态，不改循迹逻辑；停车由 VehicleControl 的输出级斜坡完成。 */
#define FINISH_RAMP_MS          (300U)   /* 减速停车时长 */
#define FINISH_CORNER_WAIT      (1U)     /* 满圈后再过几个直角才停（A 点 = 1） */

static uint8_t  s_finish_armed = 0U;        /* 已满两圈，等待 A 点直角 */
static uint8_t  s_finish_corner_seen = 0U;  /* 已看到几个直角完成 */
static uint8_t  s_finish_was_corner = 0U;   /* 上一拍是否在直角状态 */
static uint8_t  s_finish_stopped = 0U;      /* 已经请求过停车 */

static void Finish_Poll(uint32_t now_ms)
{
    AttitudeStatus attitude;
    uint8_t is_corner;
    uint8_t corner_reason;
    LineFollow_GetCornerDiag(NULL, &corner_reason, NULL);

    (void)now_ms;

    if (s_finish_stopped != 0U) {
        return;
    }

    Attitude_GetStatus(&attitude);
    if ((s_finish_armed == 0U) && attitude.lap.finished) {
        s_finish_armed = 1U;                    /* 已跑满两圈 */
        s_finish_corner_seen = 0U;
    }
    if (s_finish_armed == 0U) {
        return;
    }

    /* 直角"完成"= 状态从 CORNER 回到其它状态 */
    is_corner = (g_vehicle_control_state == LINE_FOLLOW_CORNER) ? 1U : 0U;
    if ((s_finish_was_corner != 0U) &&
        (g_vehicle_control_state == LINE_FOLLOW_TRACKING) && corner_reason <= 1U) {
        ++s_finish_corner_seen;
        if (s_finish_corner_seen >= FINISH_CORNER_WAIT) {
            VehicleControl_RequestFinishStop(FINISH_RAMP_MS);
            s_finish_stopped = 1U;
        }
    }
    s_finish_was_corner = is_corner;
}

/* ===== line_valid 丢帧保持（治"一颤一颤 / 直线走不动"）=====
 *
 * 现场与主机模拟都确认：探头会偶尔丢一两毫秒的帧。而 App 层有两条硬要求：
 *   ① follow() 只要一帧 line_valid=false 就判故障 → 停车
 *   ② 故障后重新起步要求"连续 200 ms 稳定" → 丢帧会把计时不断清零
 * 两者相乘的结果：丢帧间隔短于 200 ms 时，车永远起不来（模拟里 1000 ms 内
 * 停车 1001 ms，电机 100% 断电）。
 *
 * 所以在【信号源头】给 line_valid 加一段保持窗口：只要 LINE_VALID_HOLD_MS 内
 * 曾经看到过线，就仍然认为"有线"。这一处滤波同时解决了故障、重新武装、
 * 驱动层门控三处的抖动问题，因为它们的输入都是这个信号。
 *
 * 安全：真冲出线路时，超过保持窗口就会照常判故障，不影响保护。 */
#define LINE_VALID_HOLD_MS   (100U)
static uint32_t s_line_valid_ms;

/* ===== 只写内存的抖动诊断（不占屏、不影响控制）=====
 * 每次 line_output_valid 由 1 变 0，记录：次数、那一刻的故障码/循迹状态/
 * 丢线拍数/传感器状态/时刻。跑完后用调试器读这几个全局量即可定位问题，
 * 不需要看屏幕，也不占用 LCD 的阻塞时间。
 * 这些变量是【非 static】，方便按符号名读取。 */
volatile uint16_t g_diag_dropouts;
volatile uint8_t  g_diag_last_fault;
volatile uint8_t  g_diag_last_state;
volatile uint16_t g_diag_last_lost_cycles;
volatile uint8_t  g_diag_last_track;
volatile uint8_t  g_diag_last_bits;
volatile uint32_t g_diag_last_ms;
static uint8_t    s_prev_raw_valid;
static uint32_t s_line_normal_ms;

/* ===== 黑匣子（RAM 环形缓冲，跑完用调试器读出）=====
 * 每 20 ms 一条，512 条 ≈ 10 秒历史。只写内存，不影响任何控制量。
 * 解码：dev = raw/16，diff = raw*8，base = raw*8，yaw = raw*2，state 直接是枚举值。 */
#define REC_PERIOD_MS   (20U)
#define REC_LEN         (512U)
/* 电机门控关闭超过这么多毫秒 → 冻结记录，保留"卡住前那 10 秒" */
#define REC_FREEZE_MS   (300U)
volatile uint16_t g_rec_idx;
volatile uint8_t  g_rec_frozen;
volatile int8_t   g_rec_dev[REC_LEN];
volatile int8_t   g_rec_diff[REC_LEN];
volatile int8_t   g_rec_base[REC_LEN];
volatile int8_t   g_rec_state[REC_LEN];
volatile int8_t   g_rec_yaw[REC_LEN];
static uint32_t s_rec_ms;
static uint32_t s_rec_off_ms;
static bool s_rec_was_running;
static bool s_rec_waiting_off;

static int8_t Rec_Quantize(float value, float scale)
{
    float q = value * scale;

    if (q > 127.0f) {
        q = 127.0f;
    } else if (q < -127.0f) {
        q = -127.0f;
    }
    return (int8_t)((q >= 0.0f) ? (q + 0.5f) : (q - 0.5f));
}

static void Tracking_Poll(uint32_t now_ms, AppInputs *inputs)
{
    VehicleControlOutput_t vehicle;
    uint8_t raw_valid;
    uint8_t line_valid;
    uint8_t normal_recent;

    if ((uint32_t)(now_ms - s_last_control_ms) >= CONTROL_PERIOD_MS) {
        uint32_t elapsed_ms = now_ms - s_last_control_ms;
        s_last_control_ms = now_ms;
        VehicleControl_UpdateElapsed(elapsed_ms);
    }

    vehicle = VehicleControl_GetOutput();
    if ((vehicle.line_follow.state == LINE_FOLLOW_FAULT) &&
        (vehicle.line_follow.control.track.status == LINE_TRACK_STATUS_NORMAL)) {
        /* The board may be powered before it is placed on the track. */
        VehicleControl_Start();
        VehicleControl_Update();
        vehicle = VehicleControl_GetOutput();
    }

    raw_valid = (vehicle.line_output_valid != false) ? 1U : 0U;
    if (raw_valid != 0U) {
        s_line_valid_ms = now_ms;
    }
    if (vehicle.line_follow.control.track.status == LINE_TRACK_STATUS_NORMAL) {
        s_line_normal_ms = now_ms;
    }

    line_valid = (uint8_t)((raw_valid != 0U) ||
                           ((uint32_t)(now_ms - s_line_valid_ms) <= LINE_VALID_HOLD_MS));
    normal_recent = (uint8_t)(
        (vehicle.line_follow.control.track.status == LINE_TRACK_STATUS_NORMAL) ||
        ((uint32_t)(now_ms - s_line_normal_ms) <= LINE_VALID_HOLD_MS));

    inputs->line_valid = (line_valid != 0U);
    inputs->line_start_ready = (line_valid != 0U) && (normal_recent != 0U);
    g_tracking_status = vehicle.line_follow.control.track.status;
    g_tracking_raw_bits = vehicle.line_follow.control.raw_bits;
    g_tracking_bits = vehicle.line_follow.control.track.black_bits;
    g_tracking_deviation = vehicle.line_follow.control.track.deviation;

    /* ===== 黑匣子：每 20 ms 记一条，512 条 ≈ 10 秒历史 =====
     * 只写 RAM，不改任何控制量。跑完用调试器读出后按符号解码即可还原过程。
     * 记录内容：偏差、两轮差速、基速、循迹状态、带符号角速度。
     *
     * ★ 冻结机制：车停下（电机门控关闭）超过 300 ms 就永久停止记录，
     *   这样卡住/故障前那 10 秒会被保留下来。原来不冻结，车停下后仍继续
     *   采样，10 秒后卡住那段就被"故障空转"覆盖了（第一次实测就是这样丢的）。 */
    if (g_rec_frozen == 0U) {
        if (g_motor_output_enabled != false) {
            s_rec_was_running = true;
            s_rec_waiting_off = false;
            if ((uint32_t)(now_ms - s_rec_ms) >= REC_PERIOD_MS) {
                uint16_t i = g_rec_idx;
                float diff = vehicle.line_follow.left_command - vehicle.line_follow.right_command;
                float dev = vehicle.line_follow.control.track.deviation;

                s_rec_ms = now_ms;
                g_rec_dev[i]   = Rec_Quantize(dev, 16.0f);
                g_rec_diff[i]  = Rec_Quantize(diff, 0.125f);
                g_rec_base[i]  = Rec_Quantize(vehicle.line_follow.base_speed, 0.125f);
                g_rec_state[i] = (int8_t)vehicle.line_follow.state;
                g_rec_yaw[i]   = Rec_Quantize(g_lf_yaw_rate_dps, 0.5f);
                g_rec_idx = (uint16_t)((i + 1U) % REC_LEN);
            }
        } else if (s_rec_was_running) {
            if (!s_rec_waiting_off) {
                s_rec_waiting_off = true;
                s_rec_off_ms = now_ms;
            } else if ((uint32_t)(now_ms-s_rec_off_ms) >= REC_FREEZE_MS) {
                g_rec_frozen = 1U;
            }
        }
    }

    /* ===== 只写内存的抖动诊断（不占屏、不影响控制）=====
     * 每次 line_output_valid 由 1 变 0，记录：次数、那一刻的故障码/状态/丢线拍数/
     * 传感器状态。跑完用调试器读这几个全局量即可定位，不用看屏幕。 */
    if ((s_prev_raw_valid != 0U) && (raw_valid == 0U)) {
        if (g_diag_dropouts < UINT16_MAX) {
            ++g_diag_dropouts;
        }
        g_diag_last_fault       = (uint8_t)vehicle.line_follow.fault;
        g_diag_last_state       = (uint8_t)vehicle.line_follow.state;
        g_diag_last_lost_cycles = vehicle.line_follow.lost_cycles;
        g_diag_last_track       = (uint8_t)vehicle.line_follow.control.track.status;
        g_diag_last_bits        = (uint8_t)vehicle.line_follow.control.track.black_bits;
        g_diag_last_ms          = now_ms;
    }
    s_prev_raw_valid = raw_valid;
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
    Finish_Poll(now_ms);
    VehicleControlOutput_t finished = VehicleControl_GetOutput();
    inputs->run_finished = VehicleControl_IsFinishStopping() &&
                           finished.left_applied_command == 0 &&
                           finished.right_applied_command == 0;
}

void Modules_RunStarted(AppMode mode, uint32_t now_ms)
{
    VehicleControl_Start();
    s_finish_armed = s_finish_corner_seen = s_finish_was_corner = s_finish_stopped = 0U;
    g_rec_frozen = 0U;
    g_rec_idx = 0U;
    s_rec_was_running = s_rec_waiting_off = false;
    s_rec_ms = now_ms;
    if (mode == APP_MODE_LAPS) {
        VehicleControl_ClearFinishStop();
        Attitude_StartLap(now_ms);
    } else {
        Attitude_StopLap();
    }
}

void Modules_EmergencyStop(void)
{
    MotorDriver_Stop();
    MotorDriver_Disable();
    Buzzer_Stop();
}

void Modules_RunStopped(void)
{
    Buzzer_Stop();
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
    g_sound_last_event = (uint8_t)event;
    ++g_sound_event_count;
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

/* 屏幕诊断读数（y=112 / y=132）
 *
 *   y=112:  GY:<状态> A<帧龄> T<直角转角>
 *             OK=陀螺仪正常  ??=掉线  --=从没收到过
 *   y=132:  S<循迹状态> <直角结束原因> N<直角次数> Y<yaw>
 *
 * ★ 刷新周期从 1 s 改成 5 s：LCD 是【软件 SPI】逐像素写，
 *   一个字符 16×8 像素 × 2 字节 = 256 次总线写，16 个字符一屏约 4000 次，
 *   一次重绘会阻塞主循环约 10~15 ms。1 s 一次就是每秒一次 10 ms 的"卡"；
 *   之前做 10 Hz 诊断时更严重（每 100 ms 卡 15 ms），当时"颤动频率变高"就是这个。
 *   5 s 一次把影响降到原来的 1/5，同时仍能读到 T 等诊断值。
 * ========================================================================== */
#define ENC_READOUT_MS   (5000U)
static uint32_t s_enc_readout_ms;

static void Encoder_Readout(uint32_t now_ms)
{
    char line[17];
    AttitudeGyroHealth gyro;
    float corner_deg;
    uint8_t corner_reason;
    uint16_t corner_episodes;

    if ((uint32_t)(now_ms - s_enc_readout_ms) < ENC_READOUT_MS) {
        return;
    }
    s_enc_readout_ms = now_ms;

    Attitude_GetGyroHealth(now_ms, &gyro);
    LineFollow_GetCornerDiag(&corner_deg, &corner_reason, &corner_episodes);

    /* ---- 上排：陀螺仪状态 + 帧龄 + 直角转角 ---- */
    DebugLine_Clear(line);
    DebugLine_Append(line, "GY:");
    DebugLine_Append(line, (gyro.code == ATTITUDE_GYRO_OK) ? "OK" :
                            ((gyro.code == ATTITUDE_GYRO_STALE) ? "??" : "--"));
    DebugLine_Append(line, " A");
    DebugLine_AppendU16(line,
                        (uint16_t)((gyro.age_ms > 999U) ? 999U : gyro.age_ms), 3U);
    DebugLine_Append(line, " T");
    DebugLine_AppendU16(line, (uint16_t)(corner_deg + 0.5f), 3U);
    DebugLine_Show(112U, 0U, line, CYAN);

    /* ---- 下排：循迹状态 + 结束原因 + 直角次数 + 当前 yaw ---- */
    DebugLine_Clear(line);
    DebugLine_Append(line, "S");
    DebugLine_AppendU16(line, (uint16_t)g_vehicle_control_state, 1U);
    DebugLine_Append(line, (corner_reason == 0U) ? " A" :
                           ((corner_reason == 1U) ? " T" : " C"));
    DebugLine_Append(line, " N");
    DebugLine_AppendU16(line, corner_episodes % 100U, 2U);
    {
        int32_t yaw_i = (int32_t)(gyro.yaw_deg +
                                  ((gyro.yaw_deg >= 0.0f) ? 0.5f : -0.5f));
        int32_t abs_yaw = (yaw_i < 0) ? -yaw_i : yaw_i;

        if (abs_yaw > 999) {
            abs_yaw = 999;
        }
        DebugLine_Append(line, (yaw_i < 0) ? " Y-" : " Y+");
        DebugLine_AppendU16(line, (uint16_t)abs_yaw, 3U);
    }
    DebugLine_Show(132U, 1U, line, MAGENTA);
}
void Modules_Display(const AppStatus *status)
{
    /* Software SPI is blocking: draw only while the motors are gated off. */
    if (g_motor_output_enabled) { return; }
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
