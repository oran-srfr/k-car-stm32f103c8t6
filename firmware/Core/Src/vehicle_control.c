#include "vehicle_control.h"

#include <math.h>
#include <stddef.h>

#include "motor_driver.h"

volatile bool g_tracking_started = false;
volatile bool g_motor_output_enabled = false;
volatile float g_left_target_command = 0.0f;
volatile float g_right_target_command = 0.0f;
volatile int16_t g_left_applied_command = 0;
volatile int16_t g_right_applied_command = 0;
volatile LineFollowState_t g_vehicle_control_state = LINE_FOLLOW_STOPPED;
volatile uint32_t g_control_call_count = 0U;

static const PDControllerConfig_t *s_pd_config = NULL;
static const MotorCommandConfig_t *s_motor_config = NULL;
static const LineFollowConfig_t *s_follow_config = NULL;
static VehicleControlOutput_t s_latest_output;

static bool VehicleControl_StateAllowsMotion(LineFollowState_t state)
{
  return (state == LINE_FOLLOW_TRACKING) ||
         (state == LINE_FOLLOW_LINE_LOST) ||
         (state == LINE_FOLLOW_SEARCHING) ||
         (state == LINE_FOLLOW_REACQUIRE);
}

static bool VehicleControl_CommandIsValid(float command)
{
  return isfinite(command) &&
         (command >= -(float)MOTOR_DRIVER_COMMAND_MAX) &&
         (command <= (float)MOTOR_DRIVER_COMMAND_MAX);
}

/** 把已验证范围的浮点算法命令四舍五入为驱动整数命令。 */
static int16_t VehicleControl_ToDriverCommand(float command)
{
  return (command >= 0.0f) ? (int16_t)(command + 0.5f)
                           : (int16_t)(command - 0.5f);
}

static void VehicleControl_DisableMotor(void)
{
  MotorDriver_Stop();
  MotorDriver_Disable();
  g_left_applied_command = 0;
  g_right_applied_command = 0;
}

static void VehicleControl_SaveOutput(const LineFollowResult_t *follow,
                                      bool line_output_valid)
{
  s_latest_output.line_follow = *follow;
  s_latest_output.tracking_started = g_tracking_started;
  s_latest_output.motor_output_enabled = g_motor_output_enabled;
  s_latest_output.line_output_valid = line_output_valid;
  s_latest_output.left_target_command = g_left_target_command;
  s_latest_output.right_target_command = g_right_target_command;
  s_latest_output.left_applied_command = g_left_applied_command;
  s_latest_output.right_applied_command = g_right_applied_command;
}

void VehicleControl_Init(const PDControllerConfig_t *pd_config,
                         const MotorCommandConfig_t *motor_config,
                         const LineFollowConfig_t *follow_config)
{
  s_pd_config = pd_config;
  s_motor_config = motor_config;
  s_follow_config = follow_config;

  g_tracking_started = false;
  g_motor_output_enabled = false;
  g_left_target_command = 0.0f;
  g_right_target_command = 0.0f;
  g_vehicle_control_state = LINE_FOLLOW_STOPPED;
  g_control_call_count = 0U;
  s_latest_output = (VehicleControlOutput_t){0};
  s_latest_output.line_follow.state = LINE_FOLLOW_STOPPED;

  LineFollow_Init();
  VehicleControl_DisableMotor();
}

void VehicleControl_Start(void)
{
  LineFollow_Start();
  g_tracking_started = true;
  g_vehicle_control_state = LINE_FOLLOW_TRACKING;
}

void VehicleControl_Stop(void)
{
  LineFollow_Stop();
  g_tracking_started = false;
  g_motor_output_enabled = false;
  g_left_target_command = 0.0f;
  g_right_target_command = 0.0f;
  g_vehicle_control_state = LINE_FOLLOW_STOPPED;
  VehicleControl_DisableMotor();
  s_latest_output = (VehicleControlOutput_t){0};
  s_latest_output.line_follow = LineFollow_GetOutput();
  s_latest_output.line_follow.state = LINE_FOLLOW_STOPPED;
}

void VehicleControl_SetMotorOutputEnabled(bool enabled)
{
  g_motor_output_enabled = enabled;

  /* 关闭门控必须立即生效；打开门控留到下一次Update判断状态。 */
  if (!enabled)
  {
    VehicleControl_DisableMotor();
  }
}

void VehicleControl_Update(void)
{
  LineFollowResult_t follow;
  bool output_valid;
  int16_t left_command;
  int16_t right_command;

  if ((s_pd_config == NULL) ||
      (s_motor_config == NULL) ||
      (s_follow_config == NULL))
  {
    g_left_target_command = 0.0f;
    g_right_target_command = 0.0f;
    g_vehicle_control_state = LINE_FOLLOW_FAULT;
    VehicleControl_DisableMotor();
    ++g_control_call_count;
    return;
  }

  /* 一次Update只执行一次完整循迹计算，再读取一次缓存输出。 */
  (void)LineFollow_Update(s_pd_config, s_motor_config, s_follow_config);
  follow = LineFollow_GetOutput();

  g_left_target_command = follow.left_command;
  g_right_target_command = follow.right_command;
  g_vehicle_control_state = follow.state;
  ++g_control_call_count;

  output_valid = g_tracking_started &&
                 (follow.fault == LINE_FOLLOW_FAULT_NONE) &&
                 VehicleControl_StateAllowsMotion(follow.state) &&
                 VehicleControl_CommandIsValid(follow.left_command) &&
                 VehicleControl_CommandIsValid(follow.right_command);

  if (!g_motor_output_enabled || !output_valid)
  {
    VehicleControl_DisableMotor();
    VehicleControl_SaveOutput(&follow, output_valid);
    return;
  }

  left_command = VehicleControl_ToDriverCommand(follow.left_command);
  right_command = VehicleControl_ToDriverCommand(follow.right_command);

  if (g_motor_driver_enabled == 0U)
  {
    MotorDriver_Enable();
  }
  MotorDriver_SetBoth(left_command, right_command);
  g_left_applied_command = g_motor_driver_left_actual;
  g_right_applied_command = g_motor_driver_right_actual;
  VehicleControl_SaveOutput(&follow, output_valid);
}

VehicleControlOutput_t VehicleControl_GetOutput(void)
{
  return s_latest_output;
}
