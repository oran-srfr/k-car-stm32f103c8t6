#include "line_follow.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

static LineFollowState_t s_state = LINE_FOLLOW_STOPPED;
static LineFollowFault_t s_fault = LINE_FOLLOW_FAULT_NONE;
static float s_last_valid_deviation = 0.0f;
static int8_t s_last_valid_direction = 0;
static uint8_t s_has_valid_history = 0U;
static uint16_t s_lost_cycles = 0U;
static uint16_t s_reacquire_cycles = 0U;
static uint16_t s_cross_cycles = 0U;
static float s_previous_base_speed = 0.0f;
static float s_previous_left_command = 0.0f;
static float s_previous_right_command = 0.0f;
static float s_reacquire_start_base = 0.0f;
static float s_reacquire_start_left = 0.0f;
static float s_reacquire_start_right = 0.0f;
static LineFollowResult_t s_latest_output;

static void LineFollow_ResetState(LineFollowState_t state)
{
  s_state = state;
  s_fault = LINE_FOLLOW_FAULT_NONE;
  s_last_valid_deviation = 0.0f;
  s_last_valid_direction = 0;
  s_has_valid_history = 0U;
  s_lost_cycles = 0U;
  s_reacquire_cycles = 0U;
  s_cross_cycles = 0U;
  s_previous_base_speed = 0.0f;
  s_previous_left_command = 0.0f;
  s_previous_right_command = 0.0f;
  s_reacquire_start_base = 0.0f;
  s_reacquire_start_left = 0.0f;
  s_reacquire_start_right = 0.0f;

  s_latest_output = (LineFollowResult_t){0};
  s_latest_output.state = state;
  s_latest_output.control.track.status = LINE_TRACK_STATUS_LOST;
  s_latest_output.control.pd.status = LINE_TRACK_STATUS_LOST;
  s_latest_output.control.control_status = LINE_CONTROL_STATUS_DISABLED_STOP;
  PDController_Reset();
}

static uint8_t LineFollow_ConfigIsValid(const MotorCommandConfig_t *motor_config,
                                        const LineFollowConfig_t *follow_config)
{
  if ((motor_config == NULL) || (follow_config == NULL))
  {
    return 0U;
  }

  if (!isfinite(motor_config->base_speed) ||
      !isfinite(motor_config->max_forward_command) ||
      !isfinite(follow_config->curve_speed) ||
      !isfinite(follow_config->sharp_curve_speed) ||
      !isfinite(follow_config->search_speed) ||
      !isfinite(follow_config->search_inner_speed) ||
      !isfinite(follow_config->straight_max_deviation) ||
      !isfinite(follow_config->curve_max_deviation))
  {
    return 0U;
  }

  if ((motor_config->base_speed < 0.0f) ||
      (motor_config->max_forward_command < 0.0f) ||
      (follow_config->curve_speed < 0.0f) ||
      (follow_config->sharp_curve_speed < 0.0f) ||
      (follow_config->search_speed < 0.0f) ||
      (follow_config->straight_max_deviation < 0.0f) ||
      (follow_config->curve_max_deviation < follow_config->straight_max_deviation) ||
      (follow_config->search_reverse_cycles == 0U))
  {
    return 0U;
  }

  return 1U;
}

static float LineFollow_SelectBaseSpeed(float deviation,
                                        const MotorCommandConfig_t *motor_config,
                                        const LineFollowConfig_t *follow_config)
{
  float absolute_deviation = fabsf(deviation);

  if (absolute_deviation <= follow_config->straight_max_deviation)
  {
    return motor_config->base_speed;
  }
  if (absolute_deviation <= follow_config->curve_max_deviation)
  {
    return follow_config->curve_speed;
  }
  return follow_config->sharp_curve_speed;
}

static MotorCommandResult_t LineFollow_MakeCommand(float base_speed,
                                                    float correction,
                                                    const MotorCommandConfig_t *motor_config)
{
  MotorCommandConfig_t command_config = *motor_config;

  command_config.base_speed = base_speed;
  /* 找线命令由状态机授权，仍复用已有左右轮混合和限幅函数。 */
  return MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL,
                                correction,
                                &command_config);
}

static float LineFollow_ClampCommand(float command, float maximum)
{
  if (command > maximum)
  {
    return maximum;
  }
  if (command < -maximum)
  {
    return -maximum;
  }
  return command;
}

static void LineFollow_MakeSearchCommand(float outer_speed,
                                         float inner_speed,
                                         int8_t direction,
                                         const MotorCommandConfig_t *motor_config,
                                         LineFollowResult_t *result)
{
  if (direction < 0)
  {
    result->left_command = LineFollow_ClampCommand(inner_speed,
                                                    motor_config->max_forward_command);
    result->right_command = LineFollow_ClampCommand(outer_speed,
                                                     motor_config->max_forward_command);
  }
  else
  {
    result->left_command = LineFollow_ClampCommand(outer_speed,
                                                    motor_config->max_forward_command);
    result->right_command = LineFollow_ClampCommand(inner_speed,
                                                     motor_config->max_forward_command);
  }
  result->base_speed = outer_speed;
}

static void LineFollow_SetFault(LineFollowFault_t fault,
                                LineFollowResult_t *result)
{
  s_state = LINE_FOLLOW_FAULT;
  s_fault = fault;
  s_reacquire_cycles = 0U;
  result->base_speed = 0.0f;
  result->left_command = 0.0f;
  result->right_command = 0.0f;
}

static void LineFollow_UpdateLastDeviation(float deviation)
{
  s_last_valid_deviation = deviation;
  s_has_valid_history = 1U;

  if (deviation < -0.5f)
  {
    s_last_valid_direction = -1;
  }
  else if (deviation > 0.5f)
  {
    s_last_valid_direction = 1;
  }
}

static void LineFollow_HandleLost(const MotorCommandConfig_t *motor_config,
                                  const LineFollowConfig_t *follow_config,
                                  LineFollowResult_t *result)
{
  int8_t search_direction;
  uint16_t search_phase;

  if (s_has_valid_history == 0U)
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_NO_VALID_HISTORY, result);
    return;
  }

  if (s_lost_cycles < UINT16_MAX)
  {
    ++s_lost_cycles;
  }

  if (s_lost_cycles > follow_config->max_lost_cycles)
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_LOST_TIMEOUT, result);
    return;
  }

  s_state = (s_lost_cycles == 1U) ? LINE_FOLLOW_LINE_LOST
                                  : LINE_FOLLOW_SEARCHING;
  s_reacquire_cycles = 0U;
  search_direction = (s_last_valid_direction != 0) ? s_last_valid_direction : 1;
  search_phase = (uint16_t)((s_lost_cycles - 1U) /
                            follow_config->search_reverse_cycles);
  if ((search_phase & 1U) != 0U)
  {
    search_direction = (int8_t)-search_direction;
  }
  LineFollow_MakeSearchCommand(follow_config->search_speed,
                               follow_config->search_inner_speed,
                               search_direction,
                               motor_config,
                               result);
}

static void LineFollow_HandleNormal(const MotorCommandConfig_t *motor_config,
                                    const LineFollowConfig_t *follow_config,
                                    LineFollowResult_t *result)
{
  MotorCommandResult_t motor;
  float target_base;
  float alpha;

  target_base = LineFollow_SelectBaseSpeed(result->control.track.deviation,
                                           motor_config,
                                           follow_config);
  motor = LineFollow_MakeCommand(target_base,
                                 result->control.pd.output,
                                 motor_config);

  if ((s_state == LINE_FOLLOW_LINE_LOST) ||
      (s_state == LINE_FOLLOW_SEARCHING))
  {
    s_state = LINE_FOLLOW_REACQUIRE;
    s_reacquire_cycles = 0U;
    s_reacquire_start_base = s_previous_base_speed;
    s_reacquire_start_left = s_previous_left_command;
    s_reacquire_start_right = s_previous_right_command;
  }

  s_lost_cycles = 0U;
  s_cross_cycles = 0U;
  LineFollow_UpdateLastDeviation(result->control.track.deviation);

  if (s_state != LINE_FOLLOW_REACQUIRE)
  {
    s_state = LINE_FOLLOW_TRACKING;
    result->base_speed = target_base;
    result->left_command = motor.left_command;
    result->right_command = motor.right_command;
    return;
  }

  if (follow_config->reacquire_cycles == 0U)
  {
    alpha = 1.0f;
  }
  else
  {
    if (s_reacquire_cycles < UINT16_MAX)
    {
      ++s_reacquire_cycles;
    }
    alpha = (float)s_reacquire_cycles /
            (float)follow_config->reacquire_cycles;
    if (alpha > 1.0f)
    {
      alpha = 1.0f;
    }
  }

  result->base_speed = s_reacquire_start_base +
                       alpha * (target_base - s_reacquire_start_base);
  result->left_command = s_reacquire_start_left +
                         alpha * (motor.left_command - s_reacquire_start_left);
  result->right_command = s_reacquire_start_right +
                          alpha * (motor.right_command - s_reacquire_start_right);

  if ((follow_config->reacquire_cycles == 0U) ||
      (s_reacquire_cycles >= follow_config->reacquire_cycles))
  {
    s_state = LINE_FOLLOW_TRACKING;
  }
}

static void LineFollow_HandleCrossing(const MotorCommandConfig_t *motor_config,
                                      const LineFollowConfig_t *follow_config,
                                      LineFollowResult_t *result)
{
  MotorCommandResult_t motor;

  if (s_cross_cycles < UINT16_MAX)
  {
    ++s_cross_cycles;
  }
  if (s_cross_cycles > follow_config->max_cross_cycles)
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_ALL_BLACK, result);
    return;
  }

  s_state = LINE_FOLLOW_CROSSING;
  motor = LineFollow_MakeCommand(motor_config->base_speed, 0.0f, motor_config);
  result->base_speed = motor_config->base_speed;
  result->left_command = motor.left_command;
  result->right_command = motor.right_command;
}

/** 初始化后保持停止，防止复位后产生非零命令。 */
void LineFollow_Init(void)
{
  LineFollow_ResetState(LINE_FOLLOW_STOPPED);
}

/** 清除状态机历史并等待下一次有效采样。 */
void LineFollow_Start(void)
{
  LineFollow_ResetState(LINE_FOLLOW_TRACKING);
}

/** 停止状态机并清除所有可输出的软件命令历史。 */
void LineFollow_Stop(void)
{
  LineFollow_ResetState(LINE_FOLLOW_STOPPED);
}

/** 执行一次循迹状态机，不访问 PWM、方向 GPIO 或电机驱动。 */
LineFollowResult_t LineFollow_Update(const PDControllerConfig_t *pd_config,
                                     const MotorCommandConfig_t *motor_config,
                                     const LineFollowConfig_t *follow_config)
{
  LineControlResult_t control = LineControl_Sample(pd_config);

  return LineFollow_UpdateWithControl(&control, motor_config, follow_config);
}

LineFollowResult_t LineFollow_UpdateWithControl(const LineControlResult_t *control,
                                                const MotorCommandConfig_t *motor_config,
                                                const LineFollowConfig_t *follow_config)
{
  LineFollowResult_t result;

  result.control = (control != NULL) ? *control : (LineControlResult_t){0};
  if (control == NULL)
  {
    result.control.track.status = LINE_TRACK_STATUS_LOST;
    result.control.pd.status = LINE_TRACK_STATUS_LOST;
    result.control.control_status = LINE_CONTROL_STATUS_INVALID_INPUT_STOP;
  }
  result.base_speed = 0.0f;
  result.left_command = 0.0f;
  result.right_command = 0.0f;

  if (s_state == LINE_FOLLOW_STOPPED)
  {
    /* 保持采样供调试观察，但停止状态绝不生成轮命令。 */
  }
  else if (s_state == LINE_FOLLOW_FAULT)
  {
    /* 故障锁存，必须重新 Start 或先 Stop 后再 Start。 */
  }
  else if (result.control.track.status == LINE_TRACK_STATUS_ALL_BLACK)
  {
    if (LineFollow_ConfigIsValid(motor_config, follow_config) == 0U)
    {
      LineFollow_SetFault(LINE_FOLLOW_FAULT_INVALID_INPUT, &result);
    }
    else
    {
      LineFollow_HandleCrossing(motor_config, follow_config, &result);
    }
  }
  else if ((LineFollow_ConfigIsValid(motor_config, follow_config) == 0U) ||
           (result.control.control_status == LINE_CONTROL_STATUS_INVALID_INPUT_STOP))
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_INVALID_INPUT, &result);
  }
  else if (result.control.track.status == LINE_TRACK_STATUS_LOST)
  {
    LineFollow_HandleLost(motor_config, follow_config, &result);
  }
  else if (result.control.track.status == LINE_TRACK_STATUS_NORMAL)
  {
    LineFollow_HandleNormal(motor_config, follow_config, &result);
  }
  else
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_INVALID_INPUT, &result);
  }

  s_previous_base_speed = result.base_speed;
  s_previous_left_command = result.left_command;
  s_previous_right_command = result.right_command;

  result.state = s_state;
  result.fault = s_fault;
  result.last_valid_deviation = s_last_valid_deviation;
  result.last_valid_direction = s_last_valid_direction;
  result.lost_cycles = s_lost_cycles;
  result.reacquire_cycles = s_reacquire_cycles;
  s_latest_output = result;
  return result;
}

/** 读取缓存结果，不触发新的传感器采样或控制运算。 */
LineFollowResult_t LineFollow_GetOutput(void)
{
  return s_latest_output;
}
