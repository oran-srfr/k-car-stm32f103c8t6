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
static uint16_t s_all_black_cycles = 0U;   /* 连续全黑的周期数（直角/交叉点穿越计时） */
static uint16_t s_corner_recover_cycles = 0U; /* 出直角后的修正量软启动倒计时 */
#if LINE_FOLLOW_CORNER_ENABLE
/* 直角处理状态（原地转向 v3）：
 *   s_corner_dir          转向方向：+1 = 线在右侧（右转），-1 = 线在左侧（左转）
 *   s_corner_phase        阶段已用周期（停稳 / 当前这段转向）
 *   s_corner_total        直角处理总周期（用于总超时）
 *   s_corner_confirm      连续看到线的周期数（找到线的确认）
 *   s_corner_leg          第几段转向（0 = 按初始方向，1 = 已反向扫过一次） */
static int8_t s_corner_dir = 1;
static uint16_t s_corner_phase = 0U;
static uint16_t s_corner_total = 0U;
static uint16_t s_corner_confirm = 0U;
static uint16_t s_corner_leg_cycles = 0U;  /* 当前这段转向已用周期（不含停稳） */
static uint8_t s_corner_leg = 0U;
/* 门闩：本次丢线已经尝试过直角处理，只有重新看见线才允许再次触发。
 * 没有它的话，超时退回搜索后陈旧的历史偏差仍满足触发条件，
 * 车会在原地反复重新进入直角。 */
static uint8_t s_corner_done = 0U;
/* 直角特征记忆：距上次看到"两个内探头 + 一侧外探头"的周期数，以及是哪一侧。
 * 现场确认：左拐时中间两个 + 左边那个亮；右拐时中间两个 + 右边那个亮。 */
static uint16_t s_corner_sig_cycles = 0U;
static int8_t s_corner_sig_dir = 0;
/* 陀螺仪角度：起转时的 yaw 参考值，用于测量实际转过的角度 */
static float s_corner_yaw_start = 0.0f;
static uint8_t s_corner_yaw_valid = 0U;
#endif
/* 陀螺仪输入（外部每周期喂入；live=0 时直角退回按时间的方式） */
static float s_yaw_deg = 0.0f;
static uint8_t s_yaw_live = 0U;
#if !LINE_FOLLOW_DISCRETE_MODE
static int8_t s_error_direction = 0;
static uint8_t s_error_direction_cycles = 0U;
static uint8_t s_hard_turn_active = 0U;      /* 强修正迟滞状态 */
static uint16_t s_hard_turn_idle_cycles = 0U; /* 居中/反向持续多久后退出强修正 */
#if LINE_FOLLOW_DAMP_ENABLE
static float s_dev_filtered = 0.0f;           /* 低通后的偏差 */
static float s_dev_filtered_prev = 0.0f;      /* 上一拍的低通偏差（求变化量用） */
#endif
#endif
#if LINE_FOLLOW_DISCRETE_MODE
/* 逐位去抖：四路循迹每一位独立做"泄漏积分"，连续 LINE_FOLLOW_BIT_DEBOUNCE_CYCLES
 * 个周期都与当前值不同才翻转；只要有一拍一致，计数器就回落。
 * 用逐位而不是整字节，是为了避免"两个状态来回跳、任何状态都凑不满确认时间"导致
 * 卡在旧状态不动的死锁。 */
static uint8_t s_bits_filtered = 0U;
static uint8_t s_bits_counters[4] = {0U, 0U, 0U, 0U};
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
  s_all_black_cycles = 0U;
  s_corner_recover_cycles = 0U;
#if LINE_FOLLOW_CORNER_ENABLE
  s_corner_dir = 1;
  s_corner_phase = 0U;
  s_corner_total = 0U;
  s_corner_confirm = 0U;
  s_corner_leg_cycles = 0U;
  s_corner_leg = 0U;
  s_corner_done = 0U;
  s_corner_sig_cycles = 0U;
  s_corner_sig_dir = 0;
#endif
#if !LINE_FOLLOW_DISCRETE_MODE
  s_error_direction = 0;
  s_error_direction_cycles = 0U;
  s_hard_turn_active = 0U;
  s_hard_turn_idle_cycles = 0U;
#if LINE_FOLLOW_DAMP_ENABLE
  s_dev_filtered = 0.0f;
  s_dev_filtered_prev = 0.0f;
#endif
#endif
#if LINE_FOLLOW_DISCRETE_MODE
  s_bits_filtered = 0U;
  s_bits_counters[0] = 0U;
  s_bits_counters[1] = 0U;
  s_bits_counters[2] = 0U;
  s_bits_counters[3] = 0U;
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

  /* 带迟滞的强修正开关。
   *
   * 原来：dev == 0 就把计数器清零并立即撤销放大。而弧线上偏差会在 0 与 ±1
   * 之间反复跳（居中↔偏一格），于是"放大 / 不放大"以 2.5 倍幅度来回切换，
   * 电机表现为一顿一顿的卡顿。
   *
   * 现在：一旦进入放大就保持，只有在"居中或反向"持续 RELEASE 个周期后才退出；
   * 中途的短暂居中不清方向计时，只累计空闲时间。这样弧线上只在入弯时切一次、
   * 出弯后延迟一小段退出，中间不再抖动。 */
  if (direction == 0)
  {
    if (s_error_direction_cycles > 0U)
    {
      if (s_error_direction_cycles < UINT8_MAX)
      {
        --s_error_direction_cycles;     /* 居中：慢慢泄放，不立刻清零 */
      }
    }
    if (s_hard_turn_active != 0U)
    {
      if (s_hard_turn_idle_cycles < UINT16_MAX)
      {
        ++s_hard_turn_idle_cycles;
      }
      if (s_hard_turn_idle_cycles >= LINE_FOLLOW_HARD_TURN_RELEASE_CYCLES)
      {
        s_hard_turn_active = 0U;
        s_hard_turn_idle_cycles = 0U;
      }
    }
    return (s_hard_turn_active != 0U);
  }

  if (direction != s_error_direction)
  {
    s_error_direction = direction;
    s_error_direction_cycles = 1U;
    s_hard_turn_idle_cycles = 0U;
  }
  else
  {
    if (s_error_direction_cycles < UINT8_MAX)
    {
      ++s_error_direction_cycles;
    }
    s_hard_turn_idle_cycles = 0U;
  }

  if ((s_hard_turn_active == 0U) &&
      ((fabsf(deviation) >= LINE_FOLLOW_HARD_TURN_DEVIATION) ||
       (s_error_direction_cycles >= LINE_FOLLOW_HARD_TURN_TRIGGER_CYCLES)))
  {
    s_hard_turn_active = 1U;
  }

  return (s_hard_turn_active != 0U);
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

#if LINE_FOLLOW_CORNER_ENABLE
/* 记录"直角特征"：两个内探头 + 一侧外探头同时压线（现场确认的拐角图案）。
 * 左拐时亮的是左外探头（偏差为负），右拐时亮的是右外探头（偏差为正）。
 * 这里只做"记账"，不动输出；真正触发在丢线处理里，且要求该特征刚刚出现过。
 *
 * 注：曾试过放宽成"同侧一对亮即可"（允许对面内探头丢线），
 * 结果在【出直角】时误判——车斜着接上新线时探头自然呈现"内+外"图案，
 * 于是刚转完又触发一次直角。已回退到严格图案。 */
static void LineFollow_NoteCornerSignature(uint8_t bits)
{
  const uint8_t inners = (uint8_t)(SENSOR_LEFT_INNER_MASK | SENSOR_RIGHT_INNER_MASK);

  if ((bits & inners) != inners)
  {
    return;
  }

  if ((bits & SENSOR_LEFT_OUTER_MASK) != 0U)
  {
    s_corner_sig_dir = -1;      /* 左外探头亮 → 左拐 */
    s_corner_sig_cycles = 0U;
  }
  else if ((bits & SENSOR_RIGHT_OUTER_MASK) != 0U)
  {
    s_corner_sig_dir = 1;       /* 右外探头亮 → 右拐 */
    s_corner_sig_cycles = 0U;
  }
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

  /* ===== 直角识别（原地转向 v3）=====
   *
   * 判据 = 【三灯特征出现过（窗口内）】AND【丢线已持续 ARM 周期】。
   * 现场确认：拐弯处线会同时压住【两个内探头 + 一侧外探头】，
   *   左拐压左外、右拐压右外，方向由亮的是哪一侧外探头决定。
   *
   * 为什么用"与"：
   *   只看三灯特征 → 弧线上偶尔也会出现，误触发；
   *   只看丢线时长 → 直线抖动/弧线短暂丢线也满足，误触发；
   *   两者同时成立 → 弧线（线连续、不会持续丢线）与直线（无三灯特征）都不会触发，
   *                  而直角（有特征 + 线真的断了）一定触发。
   *
   * 注意窗口必须够宽（400 周期）：直角处图案在"三灯 / 两内"之间来回跳，
   * 窗口太小时"最后一次看到三灯"到"真正全白"之间就过期了，
   * 表现为"识别不到直角、直接开过去"。
   *
   * 触发后不立刻转向，而是进入 LINE_FOLLOW_CORNER 状态，由
   * LineFollow_HandleCorner() 按"停稳 → 向前滚 → 原地转 → 收工"执行。 */
#if LINE_FOLLOW_CORNER_ENABLE
  if ((s_corner_sig_dir != 0) &&
      (s_corner_sig_cycles <= LINE_FOLLOW_CORNER_SIG_WINDOW_CYCLES) &&
      (s_lost_cycles >= LINE_FOLLOW_CORNER_ARM_CYCLES) &&
      (s_corner_done == 0U) &&
      (s_lost_cycles <= LINE_FOLLOW_CORNER_PIVOT_TOTAL_CYCLES))
  {
    s_corner_dir = s_corner_sig_dir;
    s_corner_phase = 0U;
    s_corner_total = 0U;
    s_corner_confirm = 0U;
    s_corner_leg_cycles = 0U;
    s_corner_leg = 0U;
    s_corner_done = 1U;
    s_state = LINE_FOLLOW_CORNER;
    s_reacquire_cycles = 0U;
    /* 本拍先停车（停稳阶段），下一拍开始转向 */
    result->base_speed = 0.0f;
    result->left_command = 0.0f;
    result->right_command = 0.0f;
    result->state = LINE_FOLLOW_CORNER;
    return;
  }
#endif

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
    int8_t direction = (s_last_valid_direction != 0) ?
                       s_last_valid_direction : 1;
    float fast = LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT;

    result->base_speed = follow_config->search_speed;
    if (s_lost_cycles <= LINE_FOLLOW_ALL_OFF_SLOW_CYCLES) {
      result->left_command = motor_config->min_forward_command;
      result->right_command = motor_config->min_forward_command;
    } else {
      float slow = LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT;
      result->left_command = (direction < 0) ? slow : fast;
      result->right_command = (direction < 0) ? fast : slow;
    }
  }
#else
#if !LINE_FOLLOW_DISCRETE_MODE
  s_error_direction = 0;
  s_error_direction_cycles = 0U;
  s_hard_turn_active = 0U;
  s_hard_turn_idle_cycles = 0U;
#if LINE_FOLLOW_DAMP_ENABLE
  s_dev_filtered = 0.0f;
  s_dev_filtered_prev = 0.0f;
#endif
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

#if LINE_FOLLOW_DISCRETE_MODE
/**
 * 四路循迹逐位去抖。
 *
 * 黑线卡在相邻两个探头之间时，raw bits 会在两个值之间以毫秒级来回翻转，
 * 离散状态机就跟着每 1 ms 改一次左右轮输出，车表现为高频抖动（摆头）。
 *
 * 这里对每一位独立做泄漏积分：与当前生效值不同的采样让计数器 +1，
 * 相同的采样让计数器 -1；计数器攒到 LINE_FOLLOW_BIT_DEBOUNCE_CYCLES 才翻转该位。
 * 逐位处理不会出现"两个状态交替、谁都不满确认时间"的死锁。
 */
static uint8_t LineFollow_FilterBits(uint8_t raw)
{
  uint8_t out = s_bits_filtered;
  uint8_t i;

  for (i = 0U; i < 4U; ++i)
  {
    uint8_t mask = (uint8_t)(1U << i);

    if ((raw & mask) == (s_bits_filtered & mask))
    {
      if (s_bits_counters[i] > 0U)
      {
        --s_bits_counters[i];
      }
    }
    else
    {
      if (s_bits_counters[i] < LINE_FOLLOW_BIT_DEBOUNCE_CYCLES)
      {
        ++s_bits_counters[i];
      }
      if (s_bits_counters[i] >= LINE_FOLLOW_BIT_DEBOUNCE_CYCLES)
      {
        out ^= mask;
        s_bits_counters[i] = 0U;
      }
    }
  }

  s_bits_filtered = out;
  return out;
}
#endif

static void LineFollow_HandleAllBlack(const MotorCommandConfig_t *motor_config,
                                      LineFollowResult_t *result)
{
  (void)motor_config;

  if (s_all_black_cycles < UINT16_MAX)
  {
    ++s_all_black_cycles;
  }

#if LINE_FOLLOW_ALL_BLACK_CROSSING
  /* 直角处两条胶带重叠、或赛道上的粗线/交叉点，会让四路同时读黑。
   * 原先立即判故障停车，现象就是"到直角就死"。
   * 这里先直行穿越一小段（不转向，避免在粗线上打转）；
   * 超过预算仍然全黑，才认定是真的压在黑面上，判故障保护。 */
  if (s_all_black_cycles <= LINE_FOLLOW_ALL_BLACK_MAX_CYCLES)
  {
    result->base_speed = LINE_FOLLOW_ALL_BLACK_CREEP_DUTY;
    result->left_command = LINE_FOLLOW_ALL_BLACK_CREEP_DUTY;
    result->right_command = LINE_FOLLOW_ALL_BLACK_CREEP_DUTY;
    s_state = LINE_FOLLOW_CROSSING;
    return;
  }
#endif

  LineFollow_SetFault(LINE_FOLLOW_FAULT_ALL_BLACK, result);
}

static void LineFollow_HandleNormal(const MotorCommandConfig_t *motor_config,
                                    const LineFollowConfig_t *follow_config,
                                    LineFollowResult_t *result)
{
#if LINE_FOLLOW_DISCRETE_MODE
  uint8_t bits = LineFollow_FilterBits(result->control.track.black_bits);
  bool left_outer = (bits & SENSOR_LEFT_OUTER_MASK) != 0U;
  bool left_inner = (bits & SENSOR_LEFT_INNER_MASK) != 0U;
  bool right_inner = (bits & SENSOR_RIGHT_INNER_MASK) != 0U;
  bool right_outer = (bits & SENSOR_RIGHT_OUTER_MASK) != 0U;
  float inner_fast = LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT;
  float inner_slow = LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT;
  float outer_fast = LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT;
  float outer_slow = LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT;
  float straight = motor_config->base_speed;

  (void)follow_config;
  LineFollow_UpdateLastDeviation(result->control.track.deviation);
#if LINE_FOLLOW_CORNER_ENABLE
  s_corner_done = 0U;   /* 重新看见线 → 允许下一次直角触发 */
  LineFollow_NoteCornerSignature(result->control.track.black_bits);
#endif
  s_state = LINE_FOLLOW_TRACKING;
  s_fault = LINE_FOLLOW_FAULT_NONE;
  s_lost_cycles = 0U;
  s_reacquire_cycles = 0U;
  s_curve_hold_cycles = 0U;
  s_all_black_cycles = 0U;

  if (left_outer || right_outer) {
    /* Outer probe wins: use a larger two-wheel differential, never zero. */
    if (left_outer && right_outer) {
      result->left_command = inner_slow;
      result->right_command = inner_slow;
    } else if (left_outer) {
      result->left_command = outer_slow;
      result->right_command = outer_fast;
    } else {
      result->left_command = outer_fast;
      result->right_command = outer_slow;
    }
  } else if (left_inner && right_inner) {
    result->left_command = straight;
    result->right_command = straight;
  } else if (left_inner) {
    /* Line on the left: left wheel slower, right wheel faster. */
    result->left_command = inner_slow;
    result->right_command = inner_fast;
  } else if (right_inner) {
    /* Line on the right: left wheel faster, right wheel slower. */
    result->left_command = inner_fast;
    result->right_command = inner_slow;
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
  float correction;
  bool hard_turn;
  float deviation_adjusted;

  /* 刚从直角原地转向恢复：修正量软启动。
   * 出直角瞬间车身朝向误差可能很大，偏差常常直接落在 2~3 格上，
   * 而 |dev| >= 1.5 会立即触发强修正，车猛地甩出去、再反打过冲 ——
   * 实测"直角之后直线摆头极其严重"就是这个。
   * 这里让修正量从 35% 线性升到 100%（约 250 ms），并且期间不叠加强修正放大。 */
  if (s_state == LINE_FOLLOW_CORNER)
  {
    s_corner_recover_cycles = LINE_FOLLOW_CORNER_RECOVER_CYCLES;
  }

  /* 贴边循迹：速度分档与强修正判据都改用"误差"，
   * 否则车长期停在 dev = TARGET（例如 -1）会被误判成一直在弯道里。 */
  deviation_adjusted = result->control.track.deviation -
                       LINE_FOLLOW_TARGET_DEVIATION;

  target_base = LineFollow_SelectBaseSpeed(deviation_adjusted,
                                           motor_config,
                                           follow_config);
  hard_turn = LineFollow_ShouldUseHardTurn(deviation_adjusted);

  correction = result->control.pd.output;

#if LINE_FOLLOW_DAMP_ENABLE
  /* 阻尼项：先低通偏差，再取其每周期变化量。
   * 偏差严格为 0 时不施加 —— 保证"居中两轮同速"这条硬要求不被破坏。 */
  {
    float deviation = deviation_adjusted;

    s_dev_filtered += LINE_FOLLOW_DAMP_ALPHA * (deviation - s_dev_filtered);
    if (fabsf(deviation) > 0.01f)
    {
      float rate = s_dev_filtered - s_dev_filtered_prev;
      float damping = LINE_FOLLOW_DAMP_GAIN * rate;

      if (damping > LINE_FOLLOW_DAMP_LIMIT)
      {
        damping = LINE_FOLLOW_DAMP_LIMIT;
      }
      else if (damping < -LINE_FOLLOW_DAMP_LIMIT)
      {
        damping = -LINE_FOLLOW_DAMP_LIMIT;
      }
      correction += damping;
    }
    s_dev_filtered_prev = s_dev_filtered;
  }
#endif

  if (s_corner_recover_cycles > 0U)
  {
    float progress = 1.0f - ((float)s_corner_recover_cycles /
                             (float)LINE_FOLLOW_CORNER_RECOVER_CYCLES);
    float scale = LINE_FOLLOW_CORNER_RECOVER_MIN +
                  ((1.0f - LINE_FOLLOW_CORNER_RECOVER_MIN) * progress);

    correction *= scale;
    hard_turn = false;
    --s_corner_recover_cycles;
  }

  motor = LineFollow_MakeCommand(target_base,
                                 correction,
                                 hard_turn,
                                 motor_config);

  if ((s_state == LINE_FOLLOW_LINE_LOST) ||
      (s_state == LINE_FOLLOW_SEARCHING) ||
      (s_state == LINE_FOLLOW_CORNER))
  {
    s_state = LINE_FOLLOW_REACQUIRE;
    s_reacquire_cycles = 0U;
    s_reacquire_start_base = s_previous_base_speed;
  }

  LineFollow_UpdateLastDeviation(result->control.track.deviation);
#if LINE_FOLLOW_CORNER_ENABLE
  s_corner_done = 0U;   /* 重新看见线 → 允许下一次直角触发 */
  LineFollow_NoteCornerSignature(result->control.track.black_bits);
#endif

  if (s_state != LINE_FOLLOW_REACQUIRE)
  {
    s_state = LINE_FOLLOW_TRACKING;
    s_lost_cycles = 0U;
    if ((s_previous_base_speed > 0.0f) &&
        (target_base > s_previous_base_speed + LINE_FOLLOW_BASE_STEP))
    {
      target_base = s_previous_base_speed + LINE_FOLLOW_BASE_STEP;
      motor = LineFollow_MakeCommand(target_base, correction,
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
  motor = LineFollow_MakeCommand(result->base_speed, correction,
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

/** 陀螺仪 yaw 角与有效性（live=0 时直角自动退回按时间的方式）。 */
void LineFollow_SetYaw(float yaw_deg, uint8_t live)
{
  s_yaw_deg = yaw_deg;
  s_yaw_live = (live != 0U) ? 1U : 0U;
}

#if LINE_FOLLOW_CORNER_ENABLE
/** 归一到 (-180, 180] 再取绝对值：得到"转过多少度"，与转向方向的正负约定无关。 */
static float LineFollow_YawRotated(float now_deg, float start_deg)
{
  float delta = now_deg - start_deg;

  while (delta > 180.0f)
  {
    delta -= 360.0f;
  }
  while (delta < -180.0f)
  {
    delta += 360.0f;
  }
  return fabsf(delta);
}
#endif

/** 停止状态机并清除所有可输出的软件命令历史。 */
void LineFollow_Stop(void)
{
  LineFollow_ResetState(LINE_FOLLOW_STOPPED);
}

#if LINE_FOLLOW_CORNER_ENABLE
/* 直角处理：停稳 → 原地转向 → 确认找到线。
 *
 * 为什么分三段：
 *   ① 车会以全速（520）冲进拐角，带着惯性直接原地转会让前轮打滑、车身甩出去，
 *      所以先停车 SETTLE 周期把惯性卸掉（需求 B1 允许停车）。
 *   ② 原地转向期间不断检查探头：连续 CONFIRM 周期看到线才算真的找到，
 *      避免"扫到一下 → 退出 → 又丢"的来回抽搐（这是 v1 卡在直角的主因之一）。
 *   ③ 一段转向 LEG 周期还看不到线，就反向再扫一次；总时长超上限则退回前进搜索，
 *      不判故障，也不主动倒车（需求 B4）。
 *
 * 方向约定：dev > 0 表示线在车右侧 → 需要右转 → 左轮正转、右轮反转。 */
static void LineFollow_HandleCorner(uint8_t track_status,
                                    const MotorCommandConfig_t *motor_config,
                                    const LineFollowConfig_t *follow_config,
                                    LineFollowResult_t *result)
{
  float duty = LINE_FOLLOW_CORNER_PIVOT_DUTY;

  if (s_corner_total < UINT16_MAX)
  {
    ++s_corner_total;
  }
  if (s_corner_phase < UINT16_MAX)
  {
    ++s_corner_phase;
  }

  /* ---- ① 停稳阶段：两轮归零，卸掉冲进拐角的惯性 ---- */
  if (s_corner_phase <= LINE_FOLLOW_CORNER_SETTLE_CYCLES)
  {
    result->base_speed = 0.0f;
    result->left_command = 0.0f;
    result->right_command = 0.0f;
    s_corner_confirm = 0U;
    return;
  }

  /* ---- ①b 向前滚一小段：补掉"探头离新线 6 cm"的几何偏移 ----
   *
   * 探头在驱动轴前方约 8 cm，拐角丢线时刚越过拐角约 2 cm，
   * 直接原地转 90° 后探头离新线还有约 6 cm，而阵列总宽只有 4 cm → 够不到。
   * 先直行 CREEP 周期（约 5 cm）再转，探头就能压在新线上。 */
  if (s_corner_phase <=
      (LINE_FOLLOW_CORNER_SETTLE_CYCLES + LINE_FOLLOW_CORNER_CREEP_CYCLES))
  {
    result->base_speed = LINE_FOLLOW_CORNER_CREEP_DUTY;
    result->left_command = LINE_FOLLOW_CORNER_CREEP_DUTY;
    result->right_command = LINE_FOLLOW_CORNER_CREEP_DUTY;
    s_corner_confirm = 0U;
    return;
  }

  /* ---- ② 起转瞬间记下陀螺仪参考角 ---- */
  if (s_corner_leg_cycles == 0U)
  {
    s_corner_yaw_start = s_yaw_deg;
    s_corner_yaw_valid = s_yaw_live;
  }

  /* ---- ③ "闭眼期"：这段时间不承认任何"看见线" ----
   *
   * 直角触发时探头往往还压在旧线附近，若一起转就认线，CONFIRM 周期后就会退出，
   * 车只转了 5~10 度 —— 实测的"直角转不彻底"就是这个。
   *
   * 有了陀螺仪就用【转过的角度】判断，比按时间可靠得多：
   *   角度 < BLIND_DEG(55°) → 继续闭眼转
   * 陀螺仪没有数据时退回按时间（MIN_PIVOT 周期）。 */
  {
    uint8_t blind;

    if (s_corner_yaw_valid != 0U)
    {
      blind = (LineFollow_YawRotated(s_yaw_deg, s_corner_yaw_start) <
               LINE_FOLLOW_CORNER_BLIND_DEG) ? 1U : 0U;
    }
    else
    {
      blind = (s_corner_leg_cycles < LINE_FOLLOW_CORNER_MIN_PIVOT_CYCLES)
                  ? 1U : 0U;
    }

    if (blind != 0U)
    {
      s_corner_confirm = 0U;
    }
    else if (track_status == LINE_TRACK_STATUS_NORMAL)
    {
      /* 只承认"目标图案"：偏差落在 目标 ± 容差 之内。
       * 目标 = 贴边循迹设定值（-1，即只有一个内探头压线）；
       * 容差 1 格，覆盖"左内+左外"（转得略少）和"两个内探头"（转得略多）。
       * 右侧图案一律不认——那说明还没转到 90 度。 */
      float target = LINE_FOLLOW_TARGET_DEVIATION;

      if (fabsf(result->control.track.deviation - target) <=
          LINE_FOLLOW_CORNER_EXIT_TOLERANCE)
      {
        if (s_corner_confirm < UINT16_MAX)
        {
          ++s_corner_confirm;
        }
      }
      else
      {
        s_corner_confirm = 0U;
      }
    }
    else
    {
      s_corner_confirm = 0U;
    }
  }

  /* ---- 陀螺仪：转到目标角度就结束转向，不再等"看见线" ----
   *
   * 几何原因：原地转 90° 后探头正好落在新线上（前一步的向前滚动已经补掉偏移），
   * 但转向过程中探头扫过新线的窗口很短，靠"看见线"退出必然转不满。
   * 所以陀螺仪可用时【按角度收工】：转够 TARGET_DEG 就交给循迹，
   * 即使这一拍看不见线也没关系——下一拍若仍无线，会走正常的前进搜索。 */
  if (s_corner_yaw_valid != 0U)
  {
    float rotated = LineFollow_YawRotated(s_yaw_deg, s_corner_yaw_start);

    if (rotated >= LINE_FOLLOW_CORNER_TARGET_DEG)
    {
      s_corner_recover_cycles = LINE_FOLLOW_CORNER_RECOVER_CYCLES;
      s_lost_cycles = 0U;
      s_reacquire_cycles = 0U;
      s_state = LINE_FOLLOW_TRACKING;
      result->base_speed = 0.0f;
      result->left_command = 0.0f;
      result->right_command = 0.0f;
      return;
    }

    if (rotated >= LINE_FOLLOW_CORNER_MAX_DEG)
    {
      s_state = LINE_FOLLOW_SEARCHING;
      s_reacquire_cycles = 0U;
      result->base_speed = 0.0f;
      result->left_command = 0.0f;
      result->right_command = 0.0f;
      return;
    }
  }

  /* ---- 陀螺仪不可用时：确认找到线才退出 ---- */
  if (s_corner_confirm >= LINE_FOLLOW_CORNER_CONFIRM_CYCLES)
  {
    s_corner_recover_cycles = LINE_FOLLOW_CORNER_RECOVER_CYCLES;
    s_lost_cycles = 0U;
    s_reacquire_cycles = 0U;
    s_state = LINE_FOLLOW_TRACKING;
    /* 本拍就按正常循迹输出，避免多一次停顿 */
    LineFollow_HandleNormal(motor_config, follow_config, result);
    return;
  }

  /* ---- ③ 单段超时：反向再扫一次（只允许一次，且不重新停稳） ---- */
  if ((s_corner_leg_cycles >= LINE_FOLLOW_CORNER_LEG_CYCLES) &&
      (s_corner_leg == 0U))
  {
    s_corner_dir = (int8_t)(-(int32_t)s_corner_dir);
    s_corner_leg = 1U;
    s_corner_leg_cycles = 0U;
  }

  /* ---- ④ 总超时：退回前进搜索，不判故障、不倒车 ---- */
  if (s_corner_total >= LINE_FOLLOW_CORNER_PIVOT_TOTAL_CYCLES)
  {
    s_state = LINE_FOLLOW_SEARCHING;
    s_reacquire_cycles = 0U;
    /* 交给下一拍的 HandleLost 走前进搜索（它会累加丢线预算） */
    result->base_speed = 0.0f;
    result->left_command = 0.0f;
    result->right_command = 0.0f;
    return;
  }

  /* ---- ⑤ 输出原地转向 ---- */
  if (s_corner_leg_cycles < UINT16_MAX)
  {
    ++s_corner_leg_cycles;
  }
  result->base_speed = 0.0f;
  result->left_command = (float)s_corner_dir * duty;
  result->right_command = -(float)s_corner_dir * duty;
}
#endif

/** 执行一次循迹状态机，不访问 PWM、方向 GPIO 或电机驱动。 */
LineFollowResult_t LineFollow_Update(const PDControllerConfig_t *pd_config,
                                     const MotorCommandConfig_t *motor_config,
                                     const LineFollowConfig_t *follow_config)
{
  LineFollowResult_t result;

  result.control = LineControl_Sample(pd_config);

#if LINE_FOLLOW_CORNER_ENABLE
  /* 直角特征计时：每拍 +1，看到特征时被清零 */
  if (s_corner_sig_cycles < UINT16_MAX)
  {
    ++s_corner_sig_cycles;
  }
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
#if LINE_FOLLOW_CORNER_ENABLE
  else if (s_state == LINE_FOLLOW_CORNER)
  {
    /* 直角处理优先级高于其他分支：转向途中即使遇到全黑（胶带重叠）
     * 或重新看到线，都由它自己按"停稳→转向→确认"的流程处理。 */
    LineFollow_HandleCorner(result.control.track.status,
                            motor_config,
                            follow_config,
                            &result);
  }
#endif
  else if (result.control.track.status == LINE_TRACK_STATUS_ALL_BLACK)
  {
    LineFollow_HandleAllBlack(motor_config, &result);
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
