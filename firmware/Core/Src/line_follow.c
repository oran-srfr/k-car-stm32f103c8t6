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
#if LINE_FOLLOW_DISCRETE_MODE
static int8_t s_turn_direction = 0;
static int8_t s_outer_direction = 0;
static uint16_t s_outer_cycles = 0U;
static uint16_t s_turn_cycles = 0U;
static uint16_t s_turn_all_black_cycles = 0U;
static int8_t s_inner_direction = 0;
static float s_inner_correction = 0.0f;
static uint16_t s_turn_align_cycles = 0U;
static LineFollowTurnPhase_t s_turn_phase = LINE_FOLLOW_TURN_NONE;
static float s_resume_base = 0.0f;
static bool s_turn_departed = false;
static int8_t s_align_search_direction = 0;
static uint16_t s_align_lost_cycles = 0U;
static bool s_yaw_valid = false;
static float s_yaw_deg = 0.0f;
static float s_yaw_rate = 0.0f;
static bool s_yaw_sign_known = false;
static float s_yaw_command_sign = 0.0f;
static uint32_t s_yaw_ms = 0U;
static bool s_turn_yaw_valid = false;
static bool s_turn_corner = false;
static float s_turn_yaw_start = 0.0f;
static uint16_t s_exit_hold_cycles = 0U;
static int8_t s_yaw_turn_direction = 0;
static uint16_t s_black_cycles = 0U;
static void LineFollow_ApplyYawBrake(float base, LineFollowResult_t *result);
#endif
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
#if LINE_FOLLOW_DISCRETE_MODE
  s_turn_direction = 0;
  s_outer_direction = 0;
  s_outer_cycles = 0U;
  s_turn_cycles = 0U;
  s_turn_all_black_cycles = 0U;
  s_inner_direction = 0;
  s_inner_correction = 0.0f;
  s_turn_align_cycles = 0U;
  s_turn_phase = LINE_FOLLOW_TURN_NONE;
  s_resume_base = 0.0f;
  s_turn_departed = false;
  s_align_search_direction = 0;
  s_align_lost_cycles = 0U;
  s_turn_yaw_valid = false;
  s_turn_corner = false;
  s_turn_yaw_start = 0.0f;
  s_exit_hold_cycles = 0U;
  s_yaw_turn_direction = 0;
  s_black_cycles = 0U;
#endif
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

#if !LINE_FOLLOW_DISCRETE_MODE
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
#endif

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

#if LINE_FOLLOW_DISCRETE_MODE
static void LineFollow_ResetInnerSteering(void)
{
  s_inner_direction = 0;
  s_inner_correction = 0.0f;
}

static void LineFollow_HandleInner(int8_t direction,
                                   const MotorCommandConfig_t *motor_config,
                                   LineFollowResult_t *result)
{
  float limit = 0.5f * (LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT -
                        LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT);
  MotorCommandResult_t motor;
  MotorCommandConfig_t config = *motor_config;

  if (direction == 0) {
    /* Remove steering immediately at center; filtering old commands here
     * would keep pushing the car across the line. */
    LineFollow_ResetInnerSteering();
  } else if (direction != s_inner_direction) {
    s_inner_direction = direction;
    s_inner_correction = LINE_FOLLOW_INNER_INITIAL_CORRECTION;
  } else {
    s_inner_correction += LINE_FOLLOW_INNER_CORRECTION_STEP;
  }
  if (s_inner_correction > limit) {
    s_inner_correction = limit;
  }
  /* Reuse limits and straight trim for both centered and inner-probe input. */
  if (s_resume_base >= config.base_speed) {
    s_resume_base = 0.0f;
  } else if (s_resume_base > 0.0f) {
    s_resume_base += LINE_FOLLOW_TURN_EXIT_STEP;
    if (s_resume_base < config.base_speed) {
      config.base_speed = fmaxf(s_resume_base, config.min_forward_command);
    } else {
      s_resume_base = 0.0f;
    }
  }
  float correction = (float)direction * s_inner_correction;
  if ((direction == 0) && s_yaw_valid && s_yaw_sign_known) {
    correction = -s_yaw_rate * s_yaw_command_sign * LINE_FOLLOW_STRAIGHT_RATE_GAIN;
    correction = fmaxf(-LINE_FOLLOW_STRAIGHT_RATE_LIMIT,
                       fminf(correction, LINE_FOLLOW_STRAIGHT_RATE_LIMIT));
  }
  motor = MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL, correction, &config);
  result->left_command = motor.left_command;
  result->right_command = motor.right_command;
  if (s_exit_hold_cycles > 0U) {
    --s_exit_hold_cycles;
    LineFollow_ApplyYawBrake(0.5f * (motor.left_command + motor.right_command), result);
    motor = MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL,
                                   0.5f * (result->left_command - result->right_command) -
                                   config.straight_trim, &config);
    result->left_command = motor.left_command;
    result->right_command = motor.right_command;
  }
}

static float LineFollow_WrapAngle(float angle)
{
  if (angle > 180.0f) angle -= 360.0f;
  if (angle < -180.0f) angle += 360.0f;
  return angle;
}

static float LineFollow_TurnAngle(void)
{
  return fabsf(LineFollow_WrapAngle(s_yaw_deg - s_turn_yaw_start));
}

static void LineFollow_ApplyYawBrake(float base, LineFollowResult_t *result)
{
  if (!s_turn_yaw_valid) return;
  float relative = LineFollow_WrapAngle(s_yaw_deg - s_turn_yaw_start);
  if (!s_yaw_sign_known && (fabsf(relative) >= 5.0f)) {
    s_yaw_command_sign = ((relative < 0.0f) ? -1.0f : 1.0f) * (float)s_yaw_turn_direction;
    s_yaw_sign_known = true;
  }
  if (LineFollow_TurnAngle() >= LINE_FOLLOW_YAW_SLOW_DEG) s_turn_corner = true;
  if (!s_turn_corner) return;
  if (result->control.track.status != LINE_TRACK_STATUS_NORMAL) return;
  float deviation = result->control.track.deviation;
  if (fabsf(deviation) > 0.0f) return; /* lateral capture takes priority */
  float rate = (relative < 0.0f) ? -s_yaw_rate : s_yaw_rate;
  float predicted = fabsf(relative) + fmaxf(rate, 0.0f) * LINE_FOLLOW_YAW_COAST_SECONDS;
  if (fabsf(relative) < LINE_FOLLOW_YAW_SLOW_DEG) return;
  float limit = (LINE_FOLLOW_YAW_TARGET_DEG - predicted) * LINE_FOLLOW_YAW_GAIN;
  limit = fmaxf(-LINE_FOLLOW_YAW_BRAKE_LIMIT,
                 fminf(limit, LINE_FOLLOW_TURN_ALIGN_LIMIT));
  float correction = 0.5f * (result->left_command - result->right_command);
  if (correction * (float)s_yaw_turn_direction > limit) {
    correction = limit * (float)s_yaw_turn_direction;
    result->left_command = base + correction;
    result->right_command = base - correction;
  }
}

static void LineFollow_BeginTurn(int8_t direction, bool corner)
{
  LineFollow_ResetInnerSteering();
  s_state = LINE_FOLLOW_TURNING;
  s_turn_direction = direction;
  s_turn_cycles = 0U;
  s_turn_all_black_cycles = 0U;
  s_reacquire_cycles = 0U;
  s_lost_cycles = 0U;
  s_turn_phase = LINE_FOLLOW_TURN_SEEK;
  s_turn_align_cycles = 0U;
  s_resume_base = 0.0f;
  s_turn_departed = false;
  s_align_search_direction = direction;
  s_align_lost_cycles = 0U;
  s_turn_corner = corner;
  s_turn_yaw_valid = s_yaw_valid;
  s_turn_yaw_start = s_yaw_deg;
  s_yaw_turn_direction = direction;
  s_exit_hold_cycles = 0U;
}

/* Keep the entry direction while seeking; once the line appears, align
 * using current sensor feedback before restoring the straight-line speed. */
static void LineFollow_HandleTurn(const LineFollowConfig_t *follow_config,
                                   LineFollowResult_t *result)
{
  uint8_t bits = result->control.track.black_bits;
  float align_base = fminf(LINE_FOLLOW_TURN_ALIGN_COMMAND,
                           follow_config->curve_speed);
  bool inner_only = (bits & (SENSOR_LEFT_INNER_MASK |
                            SENSOR_RIGHT_INNER_MASK)) != 0U &&
                  (bits & (SENSOR_LEFT_OUTER_MASK |
                            SENSOR_RIGHT_OUTER_MASK)) == 0U;

  if (++s_turn_cycles > LINE_FOLLOW_TURN_MAX_CYCLES) {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_TURN_TIMEOUT, result);
    return;
  }
  if ((bits & (SENSOR_LEFT_INNER_MASK | SENSOR_RIGHT_INNER_MASK)) !=
      (SENSOR_LEFT_INNER_MASK | SENSOR_RIGHT_INNER_MASK)) {
    s_turn_departed = true;
  }
  if (bits == 0x07U || bits == 0x0EU) s_turn_corner = true;
  if (result->control.track.status == LINE_TRACK_STATUS_ALL_BLACK) {
    if (++s_turn_all_black_cycles > LINE_FOLLOW_TURN_ALL_BLACK_CYCLES) {
      LineFollow_SetFault(LINE_FOLLOW_FAULT_ALL_BLACK, result);
      return;
    }
  } else {
    s_turn_all_black_cycles = 0U;
  }
  if (result->control.track.status == LINE_TRACK_STATUS_LOST) {
    s_turn_corner = true;
    if (++s_lost_cycles > LINE_FOLLOW_TURN_MAX_LOST_CYCLES) {
      LineFollow_SetFault(LINE_FOLLOW_FAULT_LOST_TIMEOUT, result);
      return;
    }
  }

  if (s_turn_yaw_valid && s_turn_phase == LINE_FOLLOW_TURN_ALIGN &&
      LineFollow_TurnAngle() >= LINE_FOLLOW_YAW_SLOW_DEG) {
    /* A sharp corner can stay continuously visible and never show 3 probes
     * or white. A single alignment maneuver reaching this angle is also
     * corner evidence; a normal arc finishes its small correction earlier. */
    s_turn_corner = true;
  }
  bool yaw_assist = s_turn_yaw_valid && s_turn_corner;
  bool yaw_line_capture = yaw_assist &&
                          result->control.track.status == LINE_TRACK_STATUS_NORMAL &&
                          LineFollow_TurnAngle() >= LINE_FOLLOW_YAW_SLOW_DEG;
  if ((inner_only || yaw_line_capture) && s_turn_cycles >= LINE_FOLLOW_TURN_MIN_CYCLES) {
    s_turn_phase = LINE_FOLLOW_TURN_ALIGN;
  }
  if (s_turn_phase == LINE_FOLLOW_TURN_ALIGN) {
    ++s_turn_align_cycles;
    if (result->control.track.status == LINE_TRACK_STATUS_LOST) {
      ++s_align_lost_cycles;
    } else {
      s_align_lost_cycles = 0U;
    }
  }

  bool heading_ready = !yaw_assist || LineFollow_TurnAngle() <=
                       LINE_FOLLOW_YAW_TARGET_DEG + LINE_FOLLOW_YAW_EXIT_MARGIN_DEG;
  bool line_aligned = inner_only;
  if (line_aligned && heading_ready && s_turn_departed &&
      s_turn_align_cycles >= LINE_FOLLOW_TURN_ALIGN_MIN_CYCLES) {
    if (s_reacquire_cycles < UINT16_MAX) {
      ++s_reacquire_cycles;
    }
    if (s_reacquire_cycles >= LINE_FOLLOW_TURN_REACQUIRE_CYCLES) {
      s_state = LINE_FOLLOW_TRACKING;
      s_turn_direction = 0;
      s_outer_direction = 0;
      s_outer_cycles = 0U;
      s_lost_cycles = 0U;
      s_reacquire_cycles = 0U;
      s_last_valid_direction = s_align_search_direction;
      LineFollow_UpdateLastDeviation(result->control.track.deviation);
      s_turn_phase = LINE_FOLLOW_TURN_NONE;
      s_resume_base = align_base;
      s_exit_hold_cycles = LINE_FOLLOW_TURN_SETTLE_CYCLES;
      result->left_command = s_resume_base;
      result->right_command = s_resume_base;
      result->base_speed = s_resume_base;
      return;
    }
  } else {
    s_reacquire_cycles = 0U;
  }

  if (s_turn_phase == LINE_FOLLOW_TURN_ALIGN &&
      result->control.track.status == LINE_TRACK_STATUS_NORMAL) {
    /* Once the outgoing line appears, steer using its CURRENT position.
     * Continuing the entry pivot here sweeps across it; immediately restoring
     * straight speed instead can throw the car off before it is aligned. */
    float correction = LINE_FOLLOW_TURN_ALIGN_CORRECTION *
                       result->control.track.deviation +
                       LINE_FOLLOW_TURN_ALIGN_KD * result->control.pd.derivative;
    correction = fmaxf(-LINE_FOLLOW_TURN_ALIGN_LIMIT,
                       fminf(correction, LINE_FOLLOW_TURN_ALIGN_LIMIT));
    if (result->control.track.deviation != 0.0f) {
      s_align_search_direction = (result->control.track.deviation < 0.0f) ? -1 : 1;
    }
    result->left_command = align_base + correction;
    result->right_command = align_base - correction;
  } else if (s_turn_phase == LINE_FOLLOW_TURN_ALIGN &&
             (result->control.track.status == LINE_TRACK_STATUS_ALL_BLACK ||
              s_align_lost_cycles <= LINE_FOLLOW_TURN_ALIGN_SOFT_GAP_CYCLES)) {
    /* A short gap after capture must not restore a full pivot pulse. */
    float correction = (result->control.track.status == LINE_TRACK_STATUS_LOST) ?
                        (float)s_align_search_direction *
                        LINE_FOLLOW_TURN_ALIGN_GAP_CORRECTION : 0.0f;
    result->left_command = align_base + correction;
    result->right_command = align_base - correction;
  } else {
    int8_t search_direction = (s_turn_phase == LINE_FOLLOW_TURN_ALIGN) ?
                              s_align_search_direction : s_turn_direction;
    result->left_command = (search_direction < 0) ?
                         LINE_FOLLOW_TURN_INNER_COMMAND :
                         LINE_FOLLOW_TURN_OUTER_COMMAND;
    result->right_command = (search_direction < 0) ?
                          LINE_FOLLOW_TURN_OUTER_COMMAND :
                          LINE_FOLLOW_TURN_INNER_COMMAND;
  }
  if (yaw_assist && (s_turn_phase == LINE_FOLLOW_TURN_ALIGN ||
                    result->control.track.status == LINE_TRACK_STATUS_NORMAL)) {
    LineFollow_ApplyYawBrake(align_base, result);
  }
  result->base_speed = 0.5f * (result->left_command + result->right_command);
}
#endif

static void LineFollow_HandleLost(const MotorCommandConfig_t *motor_config,
                                  const LineFollowConfig_t *follow_config,
                                  LineFollowResult_t *result)
{
#if !LINE_FOLLOW_DISCRETE_MODE
  MotorCommandResult_t motor;
  float search_correction;
#endif

#if !LINE_FOLLOW_DISCRETE_MODE
  if (s_has_valid_history == 0U)
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_NO_VALID_HISTORY, result);
    return;
  }
#endif

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
#if LINE_FOLLOW_DISCRETE_MODE
  {
    LineFollow_ResetInnerSteering();
    /* The outer probe saw the departure side just before the gap. Do not
     * insert a straight-ahead interval or forget that side at a corner. */
    if (s_outer_direction != 0 && s_exit_hold_cycles == 0U) {
      LineFollow_BeginTurn(s_outer_direction, true);
      LineFollow_HandleTurn(follow_config, result);
      return;
    }
    if (s_exit_hold_cycles > 0U) --s_exit_hold_cycles;
    int8_t direction = s_last_valid_direction;
    float fast = LINE_FOLLOW_RECOVERY_OUTER_COMMAND;

    if (s_lost_cycles <= LINE_FOLLOW_WHITE_GRACE_CYCLES) {
      result->left_command = fminf(motor_config->base_speed, follow_config->curve_speed);
      result->right_command = result->left_command;
    } else {
      if (s_has_valid_history == 0U || s_last_valid_direction == 0) {
        LineFollow_SetFault(LINE_FOLLOW_FAULT_NO_VALID_HISTORY, result);
        return;
      }
      float slow = 0.0f;
      result->left_command = (direction < 0) ? slow : fast;
      result->right_command = (direction < 0) ? fast : slow;
    }
    result->base_speed = 0.5f * (result->left_command + result->right_command);
  }
#else
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
#endif
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
  float inner_slow = LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT;
  float outer_fast = LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT;
  float outer_slow = LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT;

  LineFollow_UpdateLastDeviation(result->control.track.deviation);
  if (left_outer != right_outer) {
    int8_t direction = left_outer ? -1 : 1;
    if (direction == s_outer_direction) {
      if (s_outer_cycles < UINT16_MAX) {
        ++s_outer_cycles;
      }
    } else {
      s_outer_direction = direction;
      s_outer_cycles = 1U;
    }
    if ((left_inner && right_inner) ||
        (s_exit_hold_cycles == 0U && s_outer_cycles >= LINE_FOLLOW_TURN_TRIGGER_CYCLES)) {
      LineFollow_BeginTurn(direction, left_inner && right_inner);
      LineFollow_HandleTurn(follow_config, result);
      return;
    }
  } else {
    s_outer_direction = 0;
    s_outer_cycles = 0U;
  }

  bool recovering = s_lost_cycles > 0U;
  if (recovering) {
    ++s_reacquire_cycles;
    if (s_reacquire_cycles >= follow_config->reacquire_cycles) {
      s_lost_cycles = s_reacquire_cycles = 0U;
      recovering = false;
    }
  } else {
    s_reacquire_cycles = 0U;
  }
  s_state = recovering ? LINE_FOLLOW_REACQUIRE : LINE_FOLLOW_TRACKING;
  s_fault = LINE_FOLLOW_FAULT_NONE;
  s_curve_hold_cycles = 0U;

  if (left_outer || right_outer) {
    LineFollow_ResetInnerSteering();
    if (s_exit_hold_cycles > 0U) {
      /* An outgoing-line edge is not a second corner. Settle on that line
       * before allowing another edge-only full-pivot trigger. */
      --s_exit_hold_cycles;
      MotorCommandConfig_t config = *motor_config;
      config.base_speed = fminf(config.base_speed, LINE_FOLLOW_TURN_ALIGN_COMMAND);
      float correction = LINE_FOLLOW_TURN_ALIGN_CORRECTION * result->control.track.deviation +
                         LINE_FOLLOW_TURN_ALIGN_KD * result->control.pd.derivative;
      correction = fmaxf(-LINE_FOLLOW_TURN_ALIGN_LIMIT,
                         fminf(correction, LINE_FOLLOW_TURN_ALIGN_LIMIT));
      result->left_command = config.base_speed + correction;
      result->right_command = config.base_speed - correction;
      LineFollow_ApplyYawBrake(config.base_speed, result);
      MotorCommandResult_t command = MotorCommand_Calculate(LINE_TRACK_STATUS_NORMAL,
          0.5f * (result->left_command - result->right_command), &config);
      result->left_command = command.left_command;
      result->right_command = command.right_command;
    } else if (left_outer && right_outer) {
      result->left_command = inner_slow;
      result->right_command = inner_slow;
    } else if (left_outer) {
      result->left_command = outer_slow;
      result->right_command = outer_fast;
    } else {
      result->left_command = outer_fast;
      result->right_command = outer_slow;
    }
  } else if (left_inner || right_inner) {
    int8_t direction = (int8_t)((right_inner ? 1 : 0) -
                                (left_inner ? 1 : 0));
    LineFollow_HandleInner(direction, motor_config, result);
  } else {
    result->left_command = inner_slow;
    result->right_command = inner_slow;
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
#if LINE_FOLLOW_DISCRETE_MODE
  s_yaw_valid = false;
  s_yaw_sign_known = false;
  s_yaw_command_sign = 0.0f;
#endif
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

void LineFollow_SetYaw(float yaw_deg, uint32_t sample_ms, uint32_t now_ms, bool valid)
{
#if LINE_FOLLOW_DISCRETE_MODE
  if (!valid || !isfinite(yaw_deg) || fabsf(yaw_deg) > 180.0f ||
      (uint32_t)(now_ms - sample_ms) > LINE_FOLLOW_YAW_MAX_AGE_MS) {
    s_yaw_valid = false;
    s_turn_yaw_valid = false;
    s_yaw_rate = 0.0f;
    return;
  }
  if (s_yaw_valid && sample_ms != s_yaw_ms) {
    uint32_t elapsed = sample_ms - s_yaw_ms;
    float change = LineFollow_WrapAngle(yaw_deg - s_yaw_deg);
    if (elapsed > LINE_FOLLOW_YAW_MAX_AGE_MS ||
        fabsf(change) > LINE_FOLLOW_YAW_MAX_STEP_DEG) {
      s_turn_yaw_valid = false;
      s_yaw_rate = 0.0f;
    } else {
      s_yaw_rate = change * 1000.0f / (float)elapsed;
    }
  } else if (!s_yaw_valid) {
    s_yaw_rate = 0.0f;
  }
  s_yaw_deg = yaw_deg;
  s_yaw_ms = sample_ms;
  s_yaw_valid = true;
#else
  (void)yaw_deg;
  (void)sample_ms;
  (void)now_ms;
  (void)valid;
#endif
}

/** 执行一次循迹状态机，不访问 PWM、方向 GPIO 或电机驱动。 */
LineFollowResult_t LineFollow_Update(const PDControllerConfig_t *pd_config,
                                     const MotorCommandConfig_t *motor_config,
                                     const LineFollowConfig_t *follow_config)
{
  LineFollowResult_t result;

  result.control = LineControl_Sample(pd_config);
#if LINE_FOLLOW_DISCRETE_MODE
  if (result.control.track.status != LINE_TRACK_STATUS_ALL_BLACK) s_black_cycles = 0U;
#endif
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
  else if ((LineFollow_ConfigIsValid(motor_config, follow_config) == 0U) ||
           (result.control.control_status == LINE_CONTROL_STATUS_INVALID_INPUT_STOP))
  {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_INVALID_INPUT, &result);
  }
#if LINE_FOLLOW_DISCRETE_MODE
  else if (s_state == LINE_FOLLOW_TURNING)
  {
    LineFollow_HandleTurn(follow_config, &result);
  }
#endif
  else if (result.control.track.status == LINE_TRACK_STATUS_ALL_BLACK)
  {
#if LINE_FOLLOW_DISCRETE_MODE
    s_reacquire_cycles = 0U;
    if (s_has_valid_history != 0U && ++s_black_cycles <= LINE_FOLLOW_BLACK_CONFIRM_CYCLES) {
      result.base_speed = fminf(motor_config->base_speed, follow_config->curve_speed);
      result.left_command = result.right_command = result.base_speed;
    } else
#endif
    {
      LineFollow_SetFault(LINE_FOLLOW_FAULT_ALL_BLACK, &result);
    }
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
#if LINE_FOLLOW_DISCRETE_MODE
  result.turn_phase = s_turn_phase;
  result.turn_cycles = s_turn_cycles;
  result.turn_yaw_assist = (s_state == LINE_FOLLOW_TURNING ||
                           ((s_state == LINE_FOLLOW_TRACKING || s_state == LINE_FOLLOW_REACQUIRE) &&
                            s_exit_hold_cycles > 0U)) &&
                           s_turn_yaw_valid && s_turn_corner;
  result.turn_angle_deg = result.turn_yaw_assist ? LineFollow_TurnAngle() : 0.0f;
#else
  result.turn_phase = LINE_FOLLOW_TURN_NONE;
  result.turn_cycles = 0U;
  result.turn_yaw_assist = false;
  result.turn_angle_deg = 0.0f;
#endif
  s_latest_output = result;
  return result;
}

/** 读取缓存结果，不触发新的传感器采样或控制运算。 */
LineFollowResult_t LineFollow_GetOutput(void)
{
  return s_latest_output;
}
