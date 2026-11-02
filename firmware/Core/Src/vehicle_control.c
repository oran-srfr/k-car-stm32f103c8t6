#include "vehicle_control.h"

#include <math.h>
#include <stddef.h>

#include "motor_driver.h"
#include "encoder.h"
#include "Attitude.h"

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
  /* CORNER（直角原地转向）与 CROSSING（全黑穿越）同样必须放行，
   * 否则状态机算出了命令、这里却把电机门控关掉，表现为"到直角就停"。 */
  return (state == LINE_FOLLOW_TRACKING) ||
         (state == LINE_FOLLOW_LINE_LOST) ||
         (state == LINE_FOLLOW_SEARCHING) ||
         (state == LINE_FOLLOW_REACQUIRE) ||
         (state == LINE_FOLLOW_CORNER) ||
         (state == LINE_FOLLOW_CROSSING);
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

/* 单轮命令的斜率限制状态（单位：驱动命令/毫秒） */
static float s_slew_left = 0.0f;
static float s_slew_right = 0.0f;

/**
 * 命令斜率限制（slew rate limit）。
 *
 * 目的：抑制"一顿一顿"的加减速冲击，但**完全不动稳态值**。
 * 位置选在最后一级（送进电机驱动之前），因此能一次覆盖所有突变来源：
 *   ① 四路数字量的状态跳变（520/520 → 460/340 是 120 的阶跃）
 *   ② 强修正放大的切入/退出（差速 120 ↔ 300）
 *   ③ 直角原地转向的启动（520 → +330/-330）
 * 之前几轮一直改控制律内部（Kp、差速、倍率斜坡），每次都会牵动稳态或别的场景；
 * 斜率限制只约束"变化多快"，稳态该给多少还是给多少。
 *
 * 代价：修正的建立时间从 0 变成 |Δ|/SLEW 毫秒。
 *   以 120 的阶跃、6/ms 为例 ≈ 20 ms —— 相对机械响应（几十到上百 ms）可忽略。
 */
static float VehicleControl_Slew(float current, float target)
{
#if CONTROL_SLEW_ENABLE
  float delta = target - current;

  if (delta > CONTROL_SLEW_PER_MS)
  {
    delta = CONTROL_SLEW_PER_MS;
  }
  else if (delta < -CONTROL_SLEW_PER_MS)
  {
    delta = -CONTROL_SLEW_PER_MS;
  }
  return current + delta;
#else
  (void)current;
  return target;
#endif
}

static void VehicleControl_DisableMotor(void)
{
  MotorDriver_Stop();
  MotorDriver_Disable();
  s_slew_left = 0.0f;
  s_slew_right = 0.0f;
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

  /* 把陀螺仪 yaw 与有效性交给循迹状态机：直角转向用它测量实际转角，
   * 数据无效时自动退回按时间的方式。 */
  {
    AttitudeStatus attitude;

    Attitude_GetStatus(&attitude);
    LineFollow_SetYaw(attitude.yaw_deg, attitude.live ? 1U : 0U);
  }

  /* 一次Update只执行一次完整循迹计算，再读取一次缓存输出。 */
  (void)LineFollow_Update(s_pd_config, s_motor_config, s_follow_config);
  follow = LineFollow_GetOutput();

  g_left_target_command = follow.left_command;
  g_right_target_command = follow.right_command;
  g_vehicle_control_state = follow.state;
  ++g_control_call_count;

  /* 把"控制器的意图"同步给编码器模块，供其判断何时该做配平。
   * 放在门控判断之前：即使电机被关掉，配平模块看到的命令也应当是当前真实意图，
   * 否则会拿上一次的旧命令做判断。 */
  Encoder_SetCommandContext(follow.left_command, follow.right_command);

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

  /* 编码器闭环配平（只作用于驱动层，不改变 LineFollow 的决策）。
   *
   * 背景：架空时左右脉冲一致，落地后差异明显 —— 负载（轮胎摩擦、减速箱阻力、
   * 重量分布）不对称。控制器只能等偏差出现才修正，于是"被拖偏→修正→再被拖偏"，
   * 表现就是直线摆头。这里直接让两轮的实际转速相等，把扰动在驱动层抵消掉。
   *
   * 方向说明：MotorDriver_SetBoth(left,right) 在 MOTOR_DRIVER_SWAP_LEFT_RIGHT=1
   * 时会交换通道（SetLeft(right)、SetRight(left)），而物理左轮是 MOTOR_B、
   * 编码器 PA8 也接在左轮。所以"物理左轮"实际由这里的 right 参数驱动：
   * 补偿量要加在 left、减在 right 上。符号若错，落地会越跑越偏
   * （把 ENCODER_BALANCE_SIGN 取反即可）。 */
  {
    float balance = Encoder_GetBalance();

    /* 先加配平，再做斜率限制：配平是慢变量（100 ms 窗口积分），
     * 斜率限制作用于最终的"这一毫秒该给多少"。 */
    s_slew_left = VehicleControl_Slew(s_slew_left,
                                      follow.left_command + balance);
    s_slew_right = VehicleControl_Slew(s_slew_right,
                                       follow.right_command - balance);

    left_command = VehicleControl_ToDriverCommand(s_slew_left);
    right_command = VehicleControl_ToDriverCommand(s_slew_right);
  }

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
