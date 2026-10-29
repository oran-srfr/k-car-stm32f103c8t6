#include "line_follow.h"

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

static LineFollowState_t s_state = LINE_FOLLOW_STOPPED;
static LineFollowFault_t s_fault = LINE_FOLLOW_FAULT_NONE;
static float s_last_valid_deviation = 0.0f;
static int8_t s_last_valid_direction = 0;
static uint8_t s_has_valid_history = 0U;
static uint16_t s_lost_cycles = 0U;
static uint16_t s_reacquire_cycles = 0U;
static float s_previous_base_speed = 0.0f;
static float s_reacquire_start_base = 0.0f;
static uint16_t s_curve_hold_cycles = 0U;
#if !LINE_FOLLOW_DISCRETE_MODE
static int8_t s_error_direction = 0;
static uint8_t s_error_direction_cycles = 0U;
#endif
static LineFollowResult_t s_latest_output;

#define SENSOR_LEFT_OUTER_MASK   (1U << 0U)
#define SENSOR_LEFT_INNER_MASK   (1U << 1U)
#define SENSOR_RIGHT_INNER_MASK  (1U << 2U)
#define SENSOR_RIGHT_OUTER_MASK  (1U << 3U)

static void LineFollow_ResetState(LineFollowState_t state)
{
  s_state = state;
  s_fault = LINE_FOLLOW_FAULT_NONE;
  s_last_valid_deviation = 0.0f;
  s_last_valid_direction = 0;
  s_has_valid_history = 0U;
  s_lost_cycles = 0U;
  s_reacquire_cycles = 0U;
  s_previous_base_speed = 0.0f;
  s_reacquire_start_base = 0.0f;
  s_curve_hold_cycles = 0U;
#if !LINE_FOLLOW_DISCRETE_MODE
  s_error_direction = 0;
  s_error_direction_cycles = 0U;
#endif

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
      !isfinite(motor_config->min_forward_command) ||
      !isfinite(motor_config->straight_trim) ||
      !isfinite(follow_config->curve_speed) ||
      !isfinite(follow_config->sharp_curve_speed) ||
      !isfinite(follow_config->search_speed) ||
      !isfinite(follow_config->straight_max_deviation) ||
      !isfinite(follow_config->curve_max_deviation) ||
      !isfinite(follow_config->search_turn_correction))
  {
    return 0U;
  }

  if ((motor_config->min_forward_command <= 0.0f) ||
      (motor_config->max_forward_command <= motor_config->min_forward_command) ||
      (motor_config->base_speed < motor_config->min_forward_command) ||
      (motor_config->base_speed > motor_config->max_forward_command) ||
      (follow_config->curve_speed < motor_config->min_forward_command) ||
      (follow_config->curve_speed > motor_config->base_speed) ||
      (follow_config->sharp_curve_speed < motor_config->min_forward_command) ||
      (follow_config->sharp_curve_speed > follow_config->curve_speed) ||
      (follow_config->search_speed < motor_config->min_forward_command) ||
      (follow_config->search_speed > motor_config->max_forward_command) ||
      (follow_config->straight_max_deviation < 0.0f) ||
      (follow_config->curve_max_deviation < follow_config->straight_max_deviation) ||
      (follow_config->search_turn_correction < 0.0f))
  {
    return 0U;
  }

  return 1U;
}

#if !LINE_FOLLOW_DISCRETE_MODE
static float LineFollow_SelectBaseSpeed(float deviation,
                                        const MotorCommandConfig_t *motor_config,
                                        const LineFollowConfig_t *follow_config)
{
  float absolute_deviation = fabsf(deviation);

  if (absolute_deviation <= follow_config->straight_max_deviation)
  {
    /* Centered samples occur throughout an arc; do not accelerate on each. */
    if (s_curve_hold_cycles > 0U)
    {
      --s_curve_hold_cycles;
      return follow_config->curve_speed;
    }
    return motor_config->base_speed;
  }
  s_curve_hold_cycles = LINE_FOLLOW_CURVE_HOLD_CYCLES;
  if (absolute_deviation <= follow_config->curve_max_deviation)
  {
    return follow_config->curve_speed;
  }
  return follow_config->sharp_curve_speed;
}
#endif

static MotorCommandResult_t LineFollow_MakeCommand(float base_speed,
                                                    float correction,
                                                    bool hard_turn,
                                                    const MotorCommandConfig_t *motor_config)
{
  MotorCommandConfig_t command_config = *motor_config;

  command_config.base_speed = base_speed;
  if (hard_turn) {
    correction *= LINE_FOLLOW_HARD_TURN_GAIN;
    if ((LINE_FOLLOW_HARD_TURN_MIN_FORWARD > 0.0f) &&
        (LINE_FOLLOW_HARD_TURN_MIN_FORWARD <
         command_config.max_forward_command)) {
      command_config.min_forward_command =
          LINE_FOLLOW_HARD_TURN_MIN_FORWARD;
    }
  }
  /* 找线命令由状态机授权，仍复用已有左右轮混合和限幅函数。 */
  return MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL,
                                correction,
                                &command_config);
}

#if !LINE_FOLLOW_DISCRETE_MODE
static bool LineFollow_ShouldUseHardTurn(float deviation)
{
  int8_t direction = (deviation < 0.0f) ? -1 :
                     ((deviation > 0.0f) ? 1 : 0);

  if (direction == 0) {
    s_error_direction = 0;
    s_error_direction_cycles = 0U;
    return false;
  }

  if (direction == s_error_direction) {
    if (s_error_direction_cycles < UINT8_MAX) {
      ++s_error_direction_cycles;
    }
  } else {
    s_error_direction = direction;
    s_error_direction_cycles = 1U;
  }

  return (fabsf(deviation) >= LINE_FOLLOW_HARD_TURN_DEVIATION) ||
         (s_error_direction_cycles >= LINE_FOLLOW_HARD_TURN_TRIGGER_CYCLES);
}
#endif

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

  if (deviation < 0.0f)
  {
    s_last_valid_direction = -1;
  }
  else if (deviation > 0.0f)
  {
    s_last_valid_direction = 1;
  }
  /* A centered sample has no turn information. Preserve the last nonzero
   * direction for short gaps on an arc; Start/Stop explicitly clear it. */
}

static void LineFollow_HandleLost(const MotorCommandConfig_t *motor_config,
                                  const LineFollowConfig_t *follow_config,
                                  LineFollowResult_t *result)
{
  MotorCommandResult_t motor;
  float search_correction;

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
#if !LINE_FOLLOW_DISCRETE_MODE
  s_error_direction = 0;
  s_error_direction_cycles = 0U;
#endif
  result->base_speed = follow_config->search_speed;
  search_correction = follow_config->search_turn_correction *
                      (float)s_last_valid_direction;
  motor = LineFollow_MakeCommand(result->base_speed,
                                 search_correction,
                                 true,
                                 motor_config);
  result->left_command = motor.left_command;
  result->right_command = motor.right_command;
}

static void LineFollow_HandleNormal(const MotorCommandConfig_t *motor_config,
                                    const LineFollowConfig_t *follow_config,
                                    LineFollowResult_t *result)
{
#if LINE_FOLLOW_DISCRETE_MODE
  uint8_t bits = result->control.track.black_bits;
  bool left_outer = (bits & SENSOR_LEFT_OUTER_MASK) != 0U;
  bool left_inner = (bits & SENSOR_LEFT_INNER_MASK) != 0U;
  bool right_inner = (bits & SENSOR_RIGHT_INNER_MASK) != 0U;
  bool right_outer = (bits & SENSOR_RIGHT_OUTER_MASK) != 0U;
  float fast = motor_config->max_forward_command;
  float straight = motor_config->base_speed;

  (void)follow_config;
  LineFollow_UpdateLastDeviation(result->control.track.deviation);
  s_state = LINE_FOLLOW_TRACKING;
  s_fault = LINE_FOLLOW_FAULT_NONE;
  s_lost_cycles = 0U;
  s_reacquire_cycles = 0U;
  s_curve_hold_cycles = 0U;

  if (left_outer || right_outer) {
    /* Outer probe wins. Stop the lit side; keep driving toward the line. */
    if (left_outer && right_outer) {
      result->left_command = 0.0f;
      result->right_command = 0.0f;
    } else if (left_outer) {
      result->left_command = 0.0f;
      result->right_command = fast;
    } else {
      result->left_command = fast;
      result->right_command = 0.0f;
    }
  } else if (left_inner && right_inner) {
    result->left_command = straight;
    result->right_command = straight;
  } else if (left_inner) {
    /* Left inner still on: stop the opposite (right) motor. */
    result->left_command = fast;
    result->right_command = 0.0f;
  } else if (right_inner) {
    /* Right inner still on: stop the opposite (left) motor. */
    result->left_command = 0.0f;
    result->right_command = fast;
  } else {
    result->left_command = 0.0f;
    result->right_command = 0.0f;
  }

  result->base_speed = 0.5f * (result->left_command + result->right_command);
  return;
#else
  MotorCommandResult_t motor;
  float target_base;
  float alpha;
  bool hard_turn;

  target_base = LineFollow_SelectBaseSpeed(result->control.track.deviation,
                                           motor_config,
                                           follow_config);
  hard_turn = LineFollow_ShouldUseHardTurn(result->control.track.deviation);
  motor = LineFollow_MakeCommand(target_base,
                                 result->control.pd.output,
                                 hard_turn,
                                 motor_config);

  if ((s_state == LINE_FOLLOW_LINE_LOST) ||
      (s_state == LINE_FOLLOW_SEARCHING))
  {
    s_state = LINE_FOLLOW_REACQUIRE;
    s_reacquire_cycles = 0U;
    s_reacquire_start_base = s_previous_base_speed;
  }

  LineFollow_UpdateLastDeviation(result->control.track.deviation);

  if (s_state != LINE_FOLLOW_REACQUIRE)
  {
    s_state = LINE_FOLLOW_TRACKING;
    s_lost_cycles = 0U;
    if ((s_previous_base_speed > 0.0f) &&
        (target_base > s_previous_base_speed + LINE_FOLLOW_BASE_STEP))
    {
      target_base = s_previous_base_speed + LINE_FOLLOW_BASE_STEP;
      motor = LineFollow_MakeCommand(target_base, result->control.pd.output,
                                      hard_turn,
                                      motor_config);
    }
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
  /* Smooth only forward speed. Blending old wheel commands can keep turning
   * toward the lost-line side even after the line reappears on the other side. */
  motor = LineFollow_MakeCommand(result->base_speed, result->control.pd.output,
                                  hard_turn,
                                  motor_config);
  result->left_command = motor.left_command;
  result->right_command = motor.right_command;

  if ((follow_config->reacquire_cycles == 0U) ||
      (s_reacquire_cycles >= follow_config->reacquire_cycles))
  {
    s_state = LINE_FOLLOW_TRACKING;
    s_lost_cycles = 0U;
  }
#endif
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
  LineFollowResult_t result;

  result.control = LineControl_Sample(pd_config);
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
    LineFollow_SetFault(LINE_FOLLOW_FAULT_ALL_BLACK, &result);
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
