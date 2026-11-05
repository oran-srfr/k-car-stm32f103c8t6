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

static LineFollowResult_t step(unsigned mask)
{
    set_pattern(mask);
    return LineFollow_Update(&pd, &motor, &follow);
}

/* 触发直角：先看到"两内 + 一侧外"的三灯特征，再丢线到 ARM 周期。
 * 左拐用 LO|LI|RI，右拐用 LI|RI|RO。 */
static LineFollowResult_t enter_corner(unsigned three_light_pattern)
{
    LineFollowResult_t r = step(three_light_pattern);
    r = step(three_light_pattern);          /* 连续两拍，确保被记录 */
    for (unsigned i = 0; i < LINE_FOLLOW_CORNER_ARM_CYCLES + 2U; ++i) {
        r = step(LOST);
    }
    return r;
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
int main(void)
{
    LineFollowResult_t result;
    LineFollow_SetElapsedMs(1);
    LineFollow_Start(); LineFollow_SetYaw(170,1);
    enter_corner(LO|LI|RI);
    for(unsigned i=0;i<LINE_FOLLOW_CORNER_STOP_CYCLES+LINE_FOLLOW_CORNER_SETTLE_CYCLES+3;i++) step(LOST);
    float yaw=170;
    unsigned i;
    for(i=0;i<400;i++) {
        yaw+=0.2f; if(yaw>180) yaw-=360;
        LineFollow_SetYaw(yaw,1); result=step(LOST);
        if(result.left_command > 0) break; /* left-turn reverse braking */
    }
    unsigned expected_lead_ms = (unsigned)((LINE_FOLLOW_CORNER_TARGET_DEG-24.0f)/0.2f);
    assert(i>expected_lead_ms-10U && i<expected_lead_ms+10U); /* lead at ~61 degrees, including yaw wrap */

    LineFollow_Start(); LineFollow_SetYaw(0,1); step(LI);
    yaw=0;
    for(i=0;i<500;i++) {yaw+=0.04f;LineFollow_SetYaw(yaw,1);step(LI);}
    for(i=0;i<60;i++) {yaw+=0.04f;LineFollow_SetYaw(yaw,1);step(CENTER);}
    for(i=0;i<500;i++) {yaw+=0.04f;LineFollow_SetYaw(yaw,1);result=step(LI);}
    assert(fabsf(result.left_command-result.right_command)<100); /* old: 240 */

    LineFollow_Start(); LineFollow_SetYaw(0,0); step(LI);
    LineFollow_SetElapsedMs(25); result=step(LOST); result=step(LOST);
    assert(result.lost_cycles==50); /* two calls represent 50 ms */
    LineFollow_SetElapsedMs(1);

    /* A completed corner opens the straight-exit window; failed pivots do not. */
    LineFollow_Start(); LineFollow_SetYaw(0,0);
    enter_corner(LO|LI|RI);
    for(i=0;i<3000;i++) {
        result=step(LOST);
        if(result.state!=LINE_FOLLOW_CORNER) break;
    }
    assert(i<3000 && result.state==LINE_FOLLOW_TRACKING);
    unsigned elapsed_after_corner = 1U; /* completion update consumes 1 ms */
    for(i=0;i<LINE_FOLLOW_POST_CORNER_GENTLE_MS;i++) {
        result=step(CENTER);
        float difference=fabsf(result.left_command-result.right_command);
        if(elapsed_after_corner<LINE_FOLLOW_POST_CORNER_GENTLE_MS-LINE_FOLLOW_POST_CORNER_FADE_MS)
            assert(difference<=LINE_FOLLOW_POST_CORNER_DIFF_MAX+0.01f);
        assert(difference<=480.01f);
        ++elapsed_after_corner;
        if(i==500) { LineFollow_Resume(); } /* a brief stop must retain the exit cap */
        if(i==1000) {
            result=step(LO|LI);
            ++elapsed_after_corner;
            assert(result.right_command-result.left_command>80); /* outer correction retained */
        }
    }
    for(i=0;i<100;i++) result=step(CENTER);
    assert(result.left_command-result.right_command>400); /* normal curve authority restored */
    LineFollow_Start(); result=step(CENTER);
    assert(result.left_command-result.right_command>80); /* no corner, no restriction */

    VehicleControl_Init(&pd,&motor,&follow); VehicleControl_Start(); set_pattern(CENTER);
    for(i=0;i<300;i++) VehicleControl_Update();
    assert(g_motor_driver_left_actual==0 && g_motor_driver_right_actual==0);
    VehicleControl_SetMotorOutputEnabled(true); VehicleControl_Update();
    VehicleControlOutput_t v=VehicleControl_GetOutput();
    assert(fabsf(v.left_target_command-v.right_target_command)<150); /* no parked FF */
    VehicleControl_UpdateElapsed(20); v=VehicleControl_GetOutput();
    assert(v.left_applied_command==126 && v.right_applied_command==126); /* 6/ms slew */
    VehicleControl_SetMotorOutputEnabled(false);
    for(i=0;i<300;i++) VehicleControl_Update();
    VehicleControl_SetMotorOutputEnabled(true); VehicleControl_Update();
    v=VehicleControl_GetOutput();
    assert(fabsf(v.left_target_command-v.right_target_command)<150);
    /* Obstacle hold may let the chassis coast off a single-inner pattern.
     * Time spent parked must not erase the last side or consume recovery time. */
    for(unsigned side=LI;side<=RI;side*=2U) {
        VehicleControl_Init(&pd,&motor,&follow); VehicleControl_Start();
        set_pattern(side); VehicleControl_SetMotorOutputEnabled(true);
        VehicleControl_Update();
        VehicleControl_SetMotorOutputEnabled(false); set_pattern(LOST);
        for(i=0;i<2000;i++) VehicleControl_Update();
        v=VehicleControl_GetOutput();
        assert(v.left_applied_command==0 && v.right_applied_command==0);
        assert(v.line_follow.fault==LINE_FOLLOW_FAULT_NONE);
        VehicleControl_SetMotorOutputEnabled(true); VehicleControl_Update();
        v=VehicleControl_GetOutput();
        assert(v.line_output_valid && v.line_follow.state==LINE_FOLLOW_SEARCHING);
        if(side==LI) assert(v.left_target_command<v.right_target_command);
        else assert(v.left_target_command>v.right_target_command);
        for(i=0;i<100;i++) VehicleControl_Update();
        set_pattern(side); VehicleControl_Update();
        assert(VehicleControl_GetOutput().line_output_valid);
    }
    VehicleControl_SetMotorOutputEnabled(false); set_pattern(LOST);
    for(i=0;i<2000;i++) VehicleControl_Update();
    VehicleControl_SetMotorOutputEnabled(true);
    for(i=0;i<LINE_FOLLOW_RESUME_SEARCH_MS+10U;i++) VehicleControl_Update();
    v=VehicleControl_GetOutput();
    assert(v.line_follow.fault==LINE_FOLLOW_FAULT_LOST_TIMEOUT);
    assert(v.left_applied_command==0 && v.right_applied_command==0);
    VehicleControl_Init(&pd,&motor,&follow); VehicleControl_Start(); set_pattern(LOST);
    VehicleControl_SetMotorOutputEnabled(true); VehicleControl_Update();
    assert(!VehicleControl_GetOutput().line_output_valid); /* no blind cold start */
    VehicleControl_Stop(); VehicleControl_Update();
    assert(VehicleControl_GetOutput().line_follow.state==LINE_FOLLOW_STOPPED);
    /* Separate short all-black episodes must not accumulate into one timeout. */
    LineFollow_Start(); LineFollow_SetElapsedMs(1);
    for(i=0;i<10;i++) {
        step(LI);
        for(unsigned k=0;k<LINE_FOLLOW_ALL_BLACK_MAX_CYCLES/2U;k++) {
            result=step(15U); assert(result.fault==LINE_FOLLOW_FAULT_NONE);
        }
    }
    puts("PASS: corner lead/wrap, rotating FF release, elapsed time, parked start/resume");
    return 0;
}
