#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 直角处理（重写版）验证。
 *
 * 判据（三灯）：
 *   看到"两个内探头 + 一侧外探头"同时亮（左外亮=左拐，右外亮=右拐），
 *   且之后 SIG_WINDOW 内丢线、丢线持续 >= ARM、本次没试过、不在冷却期
 * 执行：停稳 -> 原地转向（朝最后看到线的一侧）
 *   完成条件只认【陀螺仪到角】；陀螺仪无数据时退化为固定时长
 *   没有"反向再扫"，也没有"看到线就退出"
 */

enum { LO = 1U, LI = 2U, RI = 4U, RO = 8U, CENTER = LI | RI, LOST = 0U };

static uint8_t g_raw;

LineSensorRaw_t LineSensor_ReadRaw(void)
{
    return (LineSensorRaw_t){ g_raw & 1U, (g_raw >> 1) & 1U,
                            (g_raw >> 2) & 1U, (g_raw >> 3) & 1U };
}

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

static void set_pattern(unsigned mask)
{
    const uint8_t map[4] = {
        LINE_SENSOR_LEFT_OUTER_BIT, LINE_SENSOR_LEFT_INNER_BIT,
        LINE_SENSOR_RIGHT_INNER_BIT, LINE_SENSOR_RIGHT_OUTER_BIT
    };
    uint8_t raw = 0x0FU;

    for (unsigned i = 0; i < 4U; ++i) {
        if ((mask >> i) & 1U) {
            raw &= (uint8_t)~(1U << map[i]);
        }
    }
#if LINE_SENSOR_BLACK_ACTIVE_LOW == 0U
    raw ^= 0x0FU;
#endif
    g_raw = raw;
}

#include "vehicle_control.h"
#include "Attitude.h"
#include "motor_driver.h"
volatile uint8_t g_motor_driver_enabled;
volatile int16_t g_motor_driver_left_actual, g_motor_driver_right_actual;
void MotorDriver_Stop(void) { g_motor_driver_left_actual = g_motor_driver_right_actual = 0; }
void MotorDriver_Disable(void) { g_motor_driver_enabled = 0; }
void MotorDriver_Enable(void) { g_motor_driver_enabled = 1; }
void MotorDriver_SetBoth(int16_t l, int16_t r) { g_motor_driver_left_actual=l; g_motor_driver_right_actual=r; }
float Encoder_GetBalance(void) { return 0; }
void Encoder_SetCommandContext(float l, float r) { (void)l; (void)r; }
void Attitude_GetStatus(AttitudeStatus *s) { *s=(AttitudeStatus){0}; }

#include "app.h"
#include "app_modules.h"
static uint32_t sim_now, last_poll;
static uint16_t sim_distance = 500;
static unsigned obstacle_sounds;
void Modules_Init(void) {
    VehicleControl_Init(&pd,&motor,&follow); VehicleControl_Start();
    last_poll=sim_now;
}
void Modules_Poll(uint32_t now, AppInputs *in) {
    if(now!=last_poll) VehicleControl_UpdateElapsed(now-last_poll);
    last_poll=now;
    VehicleControlOutput_t v=VehicleControl_GetOutput();
    *in=(AppInputs){.selected_mode=APP_MODE_LAPS,
       .line_valid=v.line_output_valid,
       .line_start_ready=v.line_follow.control.track.status==LINE_TRACK_STATUS_NORMAL,
       .sonar_valid=true,.distance_mm=sim_distance,.sonar_sample_ms=(now/60U)*60U};
}
void Modules_RunStarted(AppMode mode,uint32_t now) {
    (void)mode; (void)now; VehicleControl_Start();
}
void Modules_RunStopped(void) { VehicleControl_SetMotorOutputEnabled(false); }
void Modules_Drive(AppDrive drive) { VehicleControl_SetMotorOutputEnabled(drive==APP_DRIVE_FOLLOW); }
void Modules_Sound(AppSound sound) { if(sound==APP_SOUND_OBSTACLE) ++obstacle_sounds; }
void Modules_Display(const AppStatus *status) { (void)status; }
static void run_ms(unsigned ms) {
    for(unsigned i=0;i<ms;i++) { ++sim_now; App_Task(sim_now); }
}
static void start_on(unsigned side) {
    sim_now=0;sim_distance=500;obstacle_sounds=0;set_pattern(side);App_Init(0);
    run_ms(300);assert(App_GetStatus().state==APP_FOLLOW_LINE);
}
int main(void) {
    for(unsigned side=LI;side<=RI;side*=2U) {
        start_on(side);
        sim_distance=70;run_ms(180);
        assert(App_GetStatus().state==APP_OBSTACLE_HOLD);
        assert(obstacle_sounds==1);
        set_pattern(LOST);run_ms(3000);
        assert(App_GetStatus().state==APP_OBSTACLE_HOLD);
        assert(!g_motor_output_enabled);
        sim_distance=500;run_ms(181);
        assert(App_GetStatus().state==APP_FOLLOW_LINE);
        VehicleControlOutput_t v=VehicleControl_GetOutput();
        assert(v.line_output_valid && v.line_follow.state==LINE_FOLLOW_SEARCHING);
        assert(v.left_target_command<=570.01f && v.right_target_command<=570.01f);
        if(side==LI) assert(v.left_target_command<v.right_target_command);
        else assert(v.left_target_command>v.right_target_command);
        set_pattern(side);run_ms(30);
        assert(App_GetStatus().state==APP_FOLLOW_LINE);
        assert(VehicleControl_GetOutput().line_follow.control.track.status==LINE_TRACK_STATUS_NORMAL);
        /* A second obstacle must work too, with a fresh recovery budget. */
        sim_distance=70;run_ms(180);set_pattern(LOST);run_ms(2000);
        assert(App_GetStatus().state==APP_OBSTACLE_HOLD);
        sim_distance=500;run_ms(181);
        assert(App_GetStatus().state==APP_FOLLOW_LINE);
        run_ms(LINE_FOLLOW_RESUME_SEARCH_MS+200U);
        assert(App_GetStatus().state==APP_FAULT && !g_motor_output_enabled);
        assert(g_motor_driver_left_actual==0 && g_motor_driver_right_actual==0);
    }
    puts("PASS: real App/Vehicle/Follow obstacle hold, left/right recovery, repeat hold, timeout stop");
    return 0;
}
