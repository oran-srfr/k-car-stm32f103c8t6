#include "line_follow.h"

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Counters retain their historical names, but now measure milliseconds. */
static uint32_t s_elapsed_ms = 1U;
void LineFollow_SetElapsedMs(uint32_t ms)
{
  s_elapsed_ms = ms ? ms : 1U;
  PDController_SetElapsedMs(s_elapsed_ms);
}
static uint16_t AddMs(uint16_t value, uint16_t maximum)
{
  return s_elapsed_ms >= (uint32_t)(maximum - value) ? maximum :
         (uint16_t)(value + s_elapsed_ms);
}
static uint16_t SubMs(uint16_t value)
{
  return s_elapsed_ms >= value ? 0U : (uint16_t)(value - s_elapsed_ms);
}
static float TimeAlpha(float alpha)
{
  return s_elapsed_ms == 1U ? alpha :
         1.0f - powf(1.0f - alpha, (float)s_elapsed_ms);
}

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
static uint16_t s_post_corner_gentle_ms = 0U;
static bool s_resume_search_active = false;
static uint16_t s_resume_search_ms = 0U;
static uint16_t s_corner_recover_cycles = 0U; /* 出直角后的修正量软启动倒计时 */
#if LINE_FOLLOW_CORNER_ENABLE
/* ===== 直角处理状态（重写版：只有两个阶段）=====
 *   stage = 0 空闲；1 停稳；2 原地转向
 * 转向过程中【不看线】：探头扫过新线的窗口只有几十毫秒，靠看到线退出必然提前；
 * 完成条件只认两个 —— 陀螺仪到角，或（无陀螺仪时）转够固定时间。
 * 没有反向再扫：原来的单段超时后反向正是转 30 度后反着转的根源。 */
static uint8_t s_corner_stage = 0U;
static uint16_t s_corner_timer = 0U;
static int8_t s_corner_dir = 1;
static float s_corner_yaw_start = 0.0f;
static uint8_t s_corner_yaw_valid = 0U;
static uint16_t s_corner_cooldown = 0U;   /* 直角后冷却，防出弯误判 */
static uint8_t s_corner_done = 0U;        /* 本次丢线已尝试过直角 */
static int8_t s_corner_sig_dir = 0;       /* 三灯特征方向：-1 左拐，+1 右拐 */
static uint16_t s_corner_sig_cycles = 0U; /* 距上次看到三灯特征的周期数 */

/* ===== 只读诊断量（不参与任何控制判断，仅供屏幕观察）=====
 *   last_deg  最近一次直角"决定停止转向"时的实际转角（度）
 *   reason    结束原因：0 = 陀螺仪到角；1 = 陀螺仪失效改按时间；2 = 绝对上限放弃
 *   episodes  直角触发累计次数（同一个拐角如果 +2，说明退出后又重新触发了一次） */
static float s_corner_last_deg = 0.0f;
static uint8_t s_corner_last_reason = 0U;
static uint16_t s_corner_episodes = 0U;

/* ===== 闭环刹车用的角速度估计（只读观察量，不影响循迹/弧线）=====
 * 用 yaw 的变化量除以经过的周期数得到角速度（度/秒）。
 * 陀螺仪大约每 20 ms 一帧，所以只在"yaw 真的变了"时更新，
 * 并且用 free-running 的 tick 计数，避免阶段切换时把时间差清零。 */
static float s_corner_yaw_sample = 0.0f;
static uint32_t s_corner_yaw_tick = 0U;
static uint32_t s_corner_rate_tick = 0U;
static float s_corner_rate_dps = 0.0f;
#endif
/* 陀螺仪输入（外部每周期喂入；live=0 时直角退回按时间的方式） */
static float s_yaw_deg = 0.0f;
static uint8_t s_yaw_live = 0U;

/* ===== 偏航角速度（带符号），供循迹阻尼用 =====
 * 光靠 4 路探头只能看到"横向偏移"，看不到"车头歪了多少"。
 * 现场视频里车身与胶带斜交 30~40°，而探头仍显示 dev=-1（目标值），
 * 于是控制器认为没问题 → 带着航向误差走 → 偏移变大才猛修 → 车头甩过头 → 摆动。
 * 这里用陀螺仪算出带符号的角速度，在循迹输出上做负阻尼，压住这种"绕轴摆动"。
 *
 * 计算方式与直角那段一致：只在 yaw 真的变了(>= MIN_DEG)时更新，
 * 用经过的拍数换算成 度/秒，再做轻度低通。 */
/* 黑匣子只读镜像（供 app_modules 记录） */
volatile float g_lf_yaw_rate_dps = 0.0f;
volatile float g_lf_yaw_rate_slow = 0.0f;

static float s_yaw_rate_dps = 0.0f;
static float s_yaw_rate_slow = 0.0f;   /* 慢平均：用于分离"稳态转弯"与"摆动" */
static float s_yaw_rate_sample = 0.0f;
static uint32_t s_yaw_rate_tick = 0U;
static uint8_t s_yaw_rate_seeded = 0U;
#if !LINE_FOLLOW_DISCRETE_MODE
static float s_ff = 0.0f;   /* 自适应曲率前馈（弯道补偿量） */
static int8_t s_error_direction = 0;
static uint8_t s_error_direction_cycles = 0U;
static uint8_t s_hard_turn_active = 0U;      /* 强修正迟滞状态 */
static uint16_t s_hard_turn_idle_cycles = 0U; /* 居中/反向持续多久后退出强修正 */
#if LINE_FOLLOW_DAMP_ENABLE
static float s_dev_filtered = 0.0f;           /* 低通后的偏差 */
static float s_dev_filtered_prev = 0.0f;      /* 上一拍的低通偏差（求变化量用） */
static uint8_t s_dev_filtered_valid = 0U;     /* 0 = 复位后还没采到第一帧（防阻尼假尖峰） */
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
  s_yaw_rate_seeded = 0U;
  s_yaw_rate_dps = s_yaw_rate_slow = 0.0f;
  s_yaw_rate_tick = 0U;
#if LINE_FOLLOW_CORNER_ENABLE
  s_corner_rate_tick = s_corner_yaw_tick = 0U;
  s_corner_rate_dps = 0.0f;
#endif
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
  s_post_corner_gentle_ms = 0U;
  s_resume_search_active = false;
  s_resume_search_ms = 0U;
#if LINE_FOLLOW_CORNER_ENABLE
  s_corner_stage = 0U;
  s_corner_timer = 0U;
  s_corner_dir = 1;
  s_corner_yaw_start = 0.0f;
  s_corner_yaw_valid = 0U;
  s_corner_cooldown = 0U;
  s_corner_done = 0U;
  s_corner_sig_dir = 0;
  s_corner_sig_cycles = 0U;
#endif
#if !LINE_FOLLOW_DISCRETE_MODE
  s_error_direction = 0;
  s_error_direction_cycles = 0U;
  s_hard_turn_active = 0U;
  s_hard_turn_idle_cycles = 0U;
  s_ff = 0.0f;
#if LINE_FOLLOW_DAMP_ENABLE
  s_dev_filtered = 0.0f;
  s_dev_filtered_prev = 0.0f;
  s_dev_filtered_valid = 0U;
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
      s_curve_hold_cycles = SubMs(s_curve_hold_cycles);
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
        s_error_direction_cycles = (uint8_t)SubMs(s_error_direction_cycles);     /* 居中：慢慢泄放，不立刻清零 */
      }
    }
    if (s_hard_turn_active != 0U)
    {
      if (s_hard_turn_idle_cycles < UINT16_MAX)
      {
        s_hard_turn_idle_cycles = AddMs(s_hard_turn_idle_cycles, UINT16_MAX);
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
    s_error_direction_cycles = (uint8_t)AddMs(0U, UINT8_MAX);
    s_hard_turn_idle_cycles = 0U;
  }
  else
  {
    if (s_error_direction_cycles < UINT8_MAX)
    {
      s_error_direction_cycles = (uint8_t)AddMs(s_error_direction_cycles, UINT8_MAX);
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
/* 记录"直角三灯特征"（现场确认的拐角图案）：
 *   左拐：两个内探头 + 左外探头 同时亮
 *   右拐：两个内探头 + 右外探头 同时亮
 * 只做记账，不动输出；真正触发在丢线处理里，且要求该特征刚刚出现过。 */
static void LineFollow_NoteCornerSignature(uint8_t bits)
{
  const uint8_t inners = (uint8_t)(SENSOR_LEFT_INNER_MASK | SENSOR_RIGHT_INNER_MASK);

  if ((bits & inners) != inners)
  {
    return;
  }

  if ((bits & SENSOR_LEFT_OUTER_MASK) != 0U)
  {
    s_corner_sig_dir = -1;      /* 左外亮 → 左拐 */
    s_corner_sig_cycles = 0U;
  }
  else if ((bits & SENSOR_RIGHT_OUTER_MASK) != 0U)
  {
    s_corner_sig_dir = 1;       /* 右外亮 → 右拐 */
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

  if (s_resume_search_active) {
    if (s_resume_search_ms == 0U || s_elapsed_ms >= s_resume_search_ms) {
      s_resume_search_ms = 0U;
      LineFollow_SetFault(LINE_FOLLOW_FAULT_LOST_TIMEOUT, result);
      return;
    }
    s_resume_search_ms = SubMs(s_resume_search_ms);
    s_lost_cycles = AddMs(s_lost_cycles, UINT16_MAX);
    s_state = LINE_FOLLOW_SEARCHING;
    result->base_speed = LINE_FOLLOW_RESUME_SEARCH_BASE;
    MotorCommandConfig_t recovery = *motor_config;
    recovery.base_speed = LINE_FOLLOW_RESUME_SEARCH_BASE;
    MotorCommandResult_t recovery_motor = MotorCommand_Calculate(
        LINE_TRACK_STATUS_NORMAL,
        LINE_FOLLOW_RESUME_SEARCH_CORRECTION * (float)s_last_valid_direction,
        &recovery);
    result->left_command = recovery_motor.left_command;
    result->right_command = recovery_motor.right_command;
    return;
  }

  if (s_lost_cycles < UINT16_MAX)
  {
    s_lost_cycles = AddMs(s_lost_cycles, UINT16_MAX);
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
   * LineFollow_HandleCorner() 按"反向刹车 → 静止 → 原地转 → 刹车脉冲"执行。 */
#if LINE_FOLLOW_CORNER_ENABLE
  /* ===== 直角判定（重写版：只有一条规则）=====
   *   ① 线丢了，且已持续 ARM 周期（不是一次抖动）；
   *   ② 丢线前 |偏差| 的峰值曾达到 PEAK —— 说明线是被"扫向一侧"之后消失的，
   *      直线上的偶发掉线没有这个过程；
   *   ③ 本次丢线还没试过直角（门闩）；④ 不在冷却期（防出弯误判）。
   * 满足即进入直角状态，转向方向 = 最后看到线的一侧。 */
  if ((s_corner_stage == 0U) &&
      (s_corner_done == 0U) &&
      (s_corner_cooldown == 0U) &&
      (s_corner_sig_dir != 0) &&
      (s_corner_sig_cycles <= LINE_FOLLOW_CORNER_SIG_WINDOW_CYCLES) &&
      (s_lost_cycles >= LINE_FOLLOW_CORNER_ARM_CYCLES))
  {
    s_corner_dir = s_corner_sig_dir;
    s_corner_stage = 1U;
    s_corner_timer = 0U;
    s_corner_done = 1U;
    if (s_corner_episodes < UINT16_MAX)
    {
      ++s_corner_episodes;      /* 诊断：本次触发累计次数 */
    }
    s_state = LINE_FOLLOW_CORNER;
    s_reacquire_cycles = 0U;
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
  s_ff = 0.0f;
#if LINE_FOLLOW_DAMP_ENABLE
  s_dev_filtered = 0.0f;
  s_dev_filtered_prev = 0.0f;
  s_dev_filtered_valid = 0U;
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
        s_bits_counters[i] = SubMs(s_bits_counters[i]);
      }
    }
    else
    {
      if (s_bits_counters[i] < LINE_FOLLOW_BIT_DEBOUNCE_CYCLES)
      {
        s_bits_counters[i] = (uint8_t)AddMs(s_bits_counters[i], UINT8_MAX);
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
    s_all_black_cycles = AddMs(s_all_black_cycles, UINT16_MAX);
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
  s_resume_search_active = false;
  s_resume_search_ms = 0U;
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
  bool gentle_center;
  float gentle_diff_max = LINE_FOLLOW_POST_CORNER_DIFF_MAX;

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
  gentle_center = (s_post_corner_gentle_ms > 0U) &&
                  (result->control.track.black_bits ==
                   (SENSOR_LEFT_INNER_MASK | SENSOR_RIGHT_INNER_MASK));
  if (gentle_center) {
    /* Do not learn a permanent turn from the expected straight-exit pattern. */
    s_error_direction = 0;
    s_error_direction_cycles = 0U;
    s_hard_turn_active = 0U;
    s_hard_turn_idle_cycles = 0U;
    hard_turn = false;
    if ((LINE_FOLLOW_POST_CORNER_FADE_MS > 0U) &&
        (s_post_corner_gentle_ms < LINE_FOLLOW_POST_CORNER_FADE_MS)) {
      float progress = 1.0f - (float)s_post_corner_gentle_ms /
                             (float)LINE_FOLLOW_POST_CORNER_FADE_MS;
      gentle_diff_max += progress * (motor_config->max_forward_command -
                         motor_config->min_forward_command - gentle_diff_max);
    }
  } else {
    hard_turn = LineFollow_ShouldUseHardTurn(deviation_adjusted);
  }

  correction = result->control.pd.output;

#if LINE_FOLLOW_DAMP_ENABLE
  /* 阻尼项：先低通偏差，再取其每周期变化量。
   * 偏差严格为 0 时不施加 —— 保证"居中两轮同速"这条硬要求不被破坏。
   *
   * ★ 首拍必须"对齐"而不是"从 0 开始滤波"：
   *   复位后 s_dev_filtered = 0，而实际偏差可能是 -1~-2，
   *   第一拍滤波值一下跳到 0.15×dev，变化率就被算成一个【假】的突变，
   *   阻尼项凭空给出 up to ±80 的猛修正 —— 车猛地一甩再反打，这就是发车摆头。
   *   现在首拍直接把滤波值对齐当前偏差，变化率为 0，不再产生假尖峰。 */
  {
    float deviation = deviation_adjusted;

    if (s_dev_filtered_valid == 0U)
    {
      s_dev_filtered = deviation;
      s_dev_filtered_prev = deviation;
      s_dev_filtered_valid = 1U;
    }
    else
    {
      s_dev_filtered += TimeAlpha(LINE_FOLLOW_DAMP_ALPHA) * (deviation - s_dev_filtered);
      if (fabsf(deviation) > 0.01f)
      {
        float rate = (s_dev_filtered - s_dev_filtered_prev) / (float)s_elapsed_ms;
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
    s_corner_recover_cycles = SubMs(s_corner_recover_cycles);
  }

#if LINE_FOLLOW_YAW_DAMP_ENABLE
  /* ===== 偏航角速度阻尼（治"弧线内扣 / 车身斜穿"）=====
   *
   * 4 路探头只能测"横向偏移"，测不到"车头方向"。现场视频里车身与胶带
   * 斜交 30~40°，而探头仍报 dev=-1（目标值）→ 控制器认为没问题，
   * 于是带着航向误差走，等横向偏移变大才猛修 → 车头甩过头 → 左右摆。
   *
   * ★ 只阻尼【摆动分量】，不扣"稳态转弯"：
   *   第一版直接拿绝对角速度做阻尼，结果弧线上（约 48 度/秒）被扣掉
   *   0.8×48 ≈ 38 的差速（需要约 65），转向能力只剩 1/3 → 车在弧线
   *   上"转不过去"直接卡住。这是设计错误。
   *
   *   现在用"角速度 - 慢平均"：恒定转弯时两者相等，阻尼为 0（不影响转弯）；
   *   只有来回摆动（角速度快速反向）才会产生阻尼 → 既转弯又抑摆。
   *
   * 若上车后摆动反而变大（说明陀螺仪极性相反），把 SIGN 改成 -1。 */
  {
    float yaw_damp = LINE_FOLLOW_YAW_DAMP * (s_yaw_rate_dps - s_yaw_rate_slow);

    if (yaw_damp > LINE_FOLLOW_YAW_DAMP_LIMIT)
    {
      yaw_damp = LINE_FOLLOW_YAW_DAMP_LIMIT;
    }
    else if (yaw_damp < -LINE_FOLLOW_YAW_DAMP_LIMIT)
    {
      yaw_damp = -LINE_FOLLOW_YAW_DAMP_LIMIT;
    }
    correction += LINE_FOLLOW_YAW_DAMP_SIGN * yaw_damp;
  }
#endif

#if LINE_FOLLOW_FF_ENABLE
  /* ===== 自适应曲率前馈（治"弧线拖到偏差大了才猛修 → 转过头"）=====
   *
   * 症状（现场）：进弧线后车先直着走，等和线拉开一定偏差才突然大修正，
   * 于是转过头，来回反复，直到出弯。
   *
   * 原因：贴边目标 dev=-1，纯 P 控制只能靠"持续误差"产生转向。
   * 弧线上 dev=-1 时误差为 0 → 修正为 0 → 车走直线 → 必然把偏差拖大。
   * 也就是说：弯道上"误差=0"根本不是平衡点。
   *
   * 这里学一个【有界的弯道补偿量】：
   *   误差存在 → 按误差方向累积 s_ff（约 35 ms 学到 70）
   *   误差消失 → 始终泄放；仍在转向时慢泄放，直线时快泄放
   *   防止补偿量自身维持转向，导致释放条件永远不成立
   * 于是进弯后误差不用再攒起来，车一进弯就跟着线转。
   *
   * 影响范围：只在 HandleNormal 里加；直角走独立状态机；
   * 直线时 s_ff 会被泄放到 0，所以直线不受影响。 */
  if (gentle_center) {
    s_ff = 0.0f;
  }
  else if (fabsf(deviation_adjusted) >= LINE_FOLLOW_FF_ERR_MIN)
  {
    s_ff += LINE_FOLLOW_FF_GAIN * deviation_adjusted * (float)s_elapsed_ms;
  }
  else
  {
    /* Always release at zero error: own steering must not lock the bias. */
    float leak = fabsf(s_yaw_rate_dps) < LINE_FOLLOW_FF_LEAK_YAW_MAX ?
                 LINE_FOLLOW_FF_LEAK : LINE_FOLLOW_FF_TURN_LEAK;
    s_ff *= (s_elapsed_ms == 1U ? leak : powf(leak, (float)s_elapsed_ms));
  }

  if (s_ff > LINE_FOLLOW_FF_LIMIT)
  {
    s_ff = LINE_FOLLOW_FF_LIMIT;
  }
  else if (s_ff < -LINE_FOLLOW_FF_LIMIT)
  {
    s_ff = -LINE_FOLLOW_FF_LIMIT;
  }

  correction += s_ff;
#endif

  /* 只有单个外探头压线（|偏差| >= 2.5）时：把两轮差速限制在上限内。
   * 否则 偏差3 → 误差最高 4 × Kp60 = 240，再叠加强修正 ×2.5 与内轮 450 下限，
   * 两轮差能到 480，对"只是偏了一点"来说过猛，车会猛地甩一下。
   * 这里用【限幅】而不是比例缩放：贴边目标 -1 时左外误差 -2、右外误差 +4，
   * 比例缩放会让左右不对称，限幅则两侧一致。 */
  {
    float abs_dev = result->control.track.deviation;

    if (abs_dev < 0.0f)
    {
      abs_dev = -abs_dev;
    }
    if (abs_dev >= LINE_FOLLOW_OUTER_ONLY_DEVIATION)
    {
      if (correction > LINE_FOLLOW_OUTER_ONLY_MAX_CORRECTION)
      {
        correction = LINE_FOLLOW_OUTER_ONLY_MAX_CORRECTION;
      }
      else if (correction < -LINE_FOLLOW_OUTER_ONLY_MAX_CORRECTION)
      {
        correction = -LINE_FOLLOW_OUTER_ONLY_MAX_CORRECTION;
      }
      hard_turn = false;
    }
  }

  /* Bound the combined P, damping and FF request after all additions.
   * Account for trim, which is applied inside MotorCommand_Calculate. */
  if (gentle_center) {
    float total = correction + motor_config->straight_trim;
    float half_diff = 0.5f * gentle_diff_max;
    if (total > half_diff) total = half_diff;
    if (total < -half_diff) total = -half_diff;
    correction = total - motor_config->straight_trim;
    hard_turn = false;
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
        (target_base > s_previous_base_speed + LINE_FOLLOW_BASE_STEP * (float)s_elapsed_ms))
    {
      target_base = s_previous_base_speed + LINE_FOLLOW_BASE_STEP * (float)s_elapsed_ms;
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
      s_reacquire_cycles = AddMs(s_reacquire_cycles, UINT16_MAX);
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

void LineFollow_Resume(void)
{
  uint16_t remaining = s_post_corner_gentle_ms;
  uint8_t had_line = s_has_valid_history;
  float last_deviation = s_last_valid_deviation;
  int8_t last_direction = s_last_valid_direction;
  LineFollow_ResetState(LINE_FOLLOW_TRACKING);
  s_post_corner_gentle_ms = remaining;
  /* A motor gate pauses motion, not the geometric evidence of the last line.
   * Integral/derivative, hard-turn and corner signatures stay reset. */
  s_has_valid_history = had_line;
  s_last_valid_deviation = last_deviation;
  s_last_valid_direction = last_direction;
  s_resume_search_active = (had_line != 0U);
  s_resume_search_ms = LINE_FOLLOW_RESUME_SEARCH_MS;
}


/** 陀螺仪 yaw 角与有效性（live=0 时直角自动退回按时间的方式）。 */
void LineFollow_SetYaw(float yaw_deg, uint8_t live)
{
  s_yaw_deg = yaw_deg;
  s_yaw_live = (live != 0U) ? 1U : 0U;
}

/* 由 yaw 算出【带符号】的角速度（度/秒），供循迹阻尼用。
 * 与直角的版本独立：那里只要"转了多少"的绝对值，这里要方向。
 * 只在 yaw 变化 >= LINE_FOLLOW_YAW_RATE_MIN_DEG 时更新，除以经过拍数，
 * 再轻度低通，避免 MPU 的 20 ms 帧率造成锯齿。 */
static void LineFollow_UpdateYawRateSigned(void)
{
  float delta;

  if (s_yaw_live == 0U)
  {
    s_yaw_rate_dps = 0.0f;
    s_yaw_rate_slow = 0.0f;
    s_yaw_rate_seeded = 0U;
    return;
  }

  s_yaw_rate_tick += s_elapsed_ms;

  if (s_yaw_rate_seeded == 0U)
  {
    s_yaw_rate_sample = s_yaw_deg;
    s_yaw_rate_tick = 0U;
    s_yaw_rate_seeded = 1U;
    s_yaw_rate_dps = 0.0f;
    s_yaw_rate_slow = 0.0f;
    return;
  }

  delta = s_yaw_deg - s_yaw_rate_sample;
  if (delta > 180.0f)
  {
    delta -= 360.0f;
  }
  else if (delta < -180.0f)
  {
    delta += 360.0f;
  }

  if (fabsf(delta) >= LINE_FOLLOW_YAW_RATE_MIN_DEG)
  {
    uint32_t span = s_yaw_rate_tick;

    if (span > 0U)
    {
      float inst = delta * 1000.0f / (float)span;

      s_yaw_rate_dps += 0.4f * (inst - s_yaw_rate_dps);
    }
    s_yaw_rate_sample = s_yaw_deg;
    s_yaw_rate_tick = 0U;
  }
  else if (s_yaw_rate_tick > 150U)
  {
    s_yaw_rate_dps *= (s_elapsed_ms == 1U ? 0.9f : powf(0.9f, (float)s_elapsed_ms));   /* 长时间不动 → 慢慢归零 */
  }

  /* 慢平均：跟随"稳态转弯"的角速度（时间常数约 1/alpha 毫秒 ≈ 50 ms）。
   * 阻尼只用【角速度 - 慢平均】，也就是只抑制"摆动分量"，
   * 恒定转弯（弧线上 48 度/秒那种）不会被扣掉。 */
  s_yaw_rate_slow += TimeAlpha(LINE_FOLLOW_YAW_RATE_SLOW_ALPHA) *
                     (s_yaw_rate_dps - s_yaw_rate_slow);

  /* 给黑匣子用的只读镜像 */
  g_lf_yaw_rate_dps = s_yaw_rate_dps;
  g_lf_yaw_rate_slow = s_yaw_rate_slow;
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
/* 更新角速度估计（只读，不改输出）。
 * 每拍 ++tick；yaw 有明显变化时用 delta/经过拍数 换算成 度/秒。 */
static void LineFollow_UpdateYawRate(void)
{
  float delta = s_yaw_deg - s_corner_yaw_sample;
  uint32_t span;

  s_corner_rate_tick += s_elapsed_ms;

  if (delta > 180.0f) delta -= 360.0f;
  else if (delta < -180.0f) delta += 360.0f;
  delta = fabsf(delta);

  if (delta >= LINE_FOLLOW_CORNER_RATE_MIN_DEG)
  {
    span = s_corner_rate_tick - s_corner_yaw_tick;
    if (span > 0U)
    {
      s_corner_rate_dps = delta * 1000.0f / (float)span;
    }
    s_corner_yaw_sample = s_yaw_deg;
    s_corner_yaw_tick = s_corner_rate_tick;
  }
  else if ((uint32_t)(s_corner_rate_tick - s_corner_yaw_tick) >
           LINE_FOLLOW_CORNER_RATE_TIMEOUT_CYCLES)
  {
    s_corner_rate_dps = 0.0f;   /* 一直没变化 → 不转了 */
  }
}


/* 结束直角：交给循迹，并设冷却与出弯软启动。 */
static void LineFollow_FinishCorner(LineFollowResult_t *result)
{
  s_corner_stage = 0U;
  s_corner_timer = 0U;
  s_post_corner_gentle_ms = LINE_FOLLOW_POST_CORNER_GENTLE_MS;
  s_corner_cooldown = LINE_FOLLOW_CORNER_COOLDOWN_CYCLES;
  s_corner_recover_cycles = LINE_FOLLOW_CORNER_RECOVER_CYCLES;
  s_lost_cycles = 0U;
  s_reacquire_cycles = 0U;
  s_corner_sig_dir = 0;
  s_corner_sig_cycles = LINE_FOLLOW_CORNER_SIG_WINDOW_CYCLES + 1U;
  s_state = LINE_FOLLOW_TRACKING;
  result->base_speed = 0.0f;
  result->left_command = 0.0f;
  result->right_command = 0.0f;
}

/* 放弃直角：退回前进搜索（不判故障、不倒车）。 */
static void LineFollow_AbortCorner(LineFollowResult_t *result)
{
  s_corner_stage = 0U;
  s_corner_timer = 0U;
  s_corner_cooldown = LINE_FOLLOW_CORNER_COOLDOWN_CYCLES;
  s_reacquire_cycles = 0U;
  s_corner_sig_dir = 0;
  s_corner_sig_cycles = LINE_FOLLOW_CORNER_SIG_WINDOW_CYCLES + 1U;
  s_state = LINE_FOLLOW_SEARCHING;
  result->base_speed = 0.0f;
  result->left_command = 0.0f;
  result->right_command = 0.0f;
}

/* 直角执行：停稳 → 原地转向（到角即止，全程不看线）。
 *
 * 为什么转向期间不看线：
 *   探头在驱动轴前方，原地转时它在半径为"轴到探头距离"的圆上扫过新线，
 *   真正压到线的角度窗口只有几十毫秒。若拿"看到线"当完成条件，
 *   必然在扫到的那一瞬间就退出 —— 表现为"只转了一点点就接上"。
 *
 * 方向约定：dev > 0 表示线在车右侧 → 右转 → 左轮正转、右轮反转。 */
static void LineFollow_HandleCorner(const MotorCommandConfig_t *motor_config,
                                    const LineFollowConfig_t *follow_config,
                                    LineFollowResult_t *result)
{
  (void)motor_config;
  (void)follow_config;

  /* ---- 阶段 1：先主动刹车，再静止保持（卸掉冲进拐角的惯性） ---- */
  if (s_corner_stage == 1U)
  {
    const uint32_t hold_end = (uint32_t)LINE_FOLLOW_CORNER_STOP_CYCLES +
                              (uint32_t)LINE_FOLLOW_CORNER_SETTLE_CYCLES;

    if (s_corner_timer < LINE_FOLLOW_CORNER_STOP_CYCLES)
    {
      /* ① 反向刹车：把前进惯性刹住（不是倒车导航，只是制动） */
      s_corner_timer = AddMs(s_corner_timer, UINT16_MAX);
      result->base_speed = 0.0f;
      result->left_command = -LINE_FOLLOW_CORNER_STOP_DUTY;
      result->right_command = -LINE_FOLLOW_CORNER_STOP_DUTY;
      return;
    }
    if (s_corner_timer < hold_end)
    {
      /* ② 静止保持：两轮 0，等车身真正停下来 */
      s_corner_timer = AddMs(s_corner_timer, UINT16_MAX);
      result->base_speed = 0.0f;
      result->left_command = 0.0f;
      result->right_command = 0.0f;
      return;
    }
    s_corner_stage = 2U;
    s_corner_timer = 0U;
    s_corner_yaw_start = s_yaw_deg;
    s_corner_yaw_valid = s_yaw_live;
    s_corner_yaw_sample = s_yaw_deg;
    s_corner_yaw_tick = s_corner_rate_tick = 0U;
    s_corner_rate_dps = 0.0f;
  }

  if (s_yaw_live != 0U) { LineFollow_UpdateYawRate(); }

  /* ---- 绝对上限：不管陀螺仪说什么，转太久必须收工 ----
   * 防"陀螺仪中途停止更新、角度冻结"时两个角度条件都不满足 → 一直转。 */
  if (s_corner_timer >= LINE_FOLLOW_CORNER_MAX_CYCLES)
  {
    /* 诊断：绝对上限放弃 */
    s_corner_last_deg = LineFollow_YawRotated(s_yaw_deg, s_corner_yaw_start);
    s_corner_last_reason = 2U;
    LineFollow_AbortCorner(result);
    return;
  }

  if (s_yaw_live == 0U) { s_corner_yaw_valid = 0U; }

  /* ---- 阶段 3：闭环刹车（按实测角速度决定何时停）----
   * 原来固定刹 120 ms：转得快刹不住（过转），转得慢刹过头（欠转）。
   * 现在每个周期算角速度，降到 BRAKE_RATE_STOP 以下就立即结束。
   * 陀螺仪无效时退回固定 120 ms。 */
  if (s_corner_stage == 3U)
  {
    uint8_t keep_braking;


    if (s_corner_yaw_valid == 0U)
    {
      /* 没有陀螺仪：只能按固定时长 */
      keep_braking = (uint8_t)((s_corner_timer < LINE_FOLLOW_CORNER_BRAKE_CYCLES) ? 1U : 0U);
    }
    else
    {
      uint8_t still_turning =
          (uint8_t)((s_corner_rate_dps > LINE_FOLLOW_CORNER_BRAKE_RATE_STOP) ? 1U : 0U);
      uint8_t min_done =
          (uint8_t)((s_corner_timer >= LINE_FOLLOW_CORNER_BRAKE_MIN_CYCLES) ? 1U : 0U);
      uint8_t max_hit =
          (uint8_t)((s_corner_timer >= LINE_FOLLOW_CORNER_BRAKE_MAX_CYCLES) ? 1U : 0U);

      keep_braking = (uint8_t)((min_done == 0U) ||
                               ((still_turning != 0U) && (max_hit == 0U)));
    }

    if (keep_braking != 0U)
    {
      s_corner_timer = AddMs(s_corner_timer, UINT16_MAX);
      result->base_speed = 0.0f;
      result->left_command = -(float)s_corner_dir * LINE_FOLLOW_CORNER_BRAKE_DUTY;
      result->right_command = (float)s_corner_dir * LINE_FOLLOW_CORNER_BRAKE_DUTY;
      return;
    }

    /* 刹车结束 → 进入角度收尾（陀螺仪有效时）。没有陀螺仪就直接交出去。 */
    if (s_corner_yaw_valid != 0U)
    {
      s_corner_stage = 4U;
      s_corner_timer = 0U;
      /* 本拍先按收尾逻辑输出，避免多一次停顿 */
    }
    else
    {
      LineFollow_FinishCorner(result);
      return;
    }
  }

  /* ---- 阶段 4：角度收尾（把最终角度拉到 90°±容差）----
   *
   * 刹车是开环的，只能对某一种进弯速度刚好；
   * 这里按实测最终角度做最后一次慢速修正，保证结果落在 90°。 */
  if (s_corner_stage == 4U)
  {
    float rotated = LineFollow_YawRotated(s_yaw_deg, s_corner_yaw_start);
    float error = rotated - LINE_FOLLOW_CORNER_TARGET_DEG;   /* >0 = 过转 */
    float abs_error = (error < 0.0f) ? -error : error;
    uint8_t give_up = (uint8_t)(((s_yaw_live == 0U) ||
                                 (s_corner_timer >= LINE_FOLLOW_CORNER_FIX_MAX_CYCLES))
                                ? 1U : 0U);

    if ((abs_error <= LINE_FOLLOW_CORNER_FIX_TOLERANCE_DEG) || (give_up != 0U))
    {
      s_corner_last_reason = (uint8_t)((give_up != 0U) ? 3U : s_corner_last_reason);
      LineFollow_FinishCorner(result);
      return;
    }

    {
      /* 过转 → 反向慢转回来；欠转 → 同向慢转补足 */
      float fix_dir = (error > 0.0f) ? -(float)s_corner_dir : (float)s_corner_dir;

      s_corner_timer = AddMs(s_corner_timer, UINT16_MAX);
      result->base_speed = 0.0f;
      result->left_command = fix_dir * LINE_FOLLOW_CORNER_FIX_DUTY;
      result->right_command = -fix_dir * LINE_FOLLOW_CORNER_FIX_DUTY;
      return;
    }
  }

  /* ---- 阶段 2：原地转向 ---- */
  if (s_corner_timer < UINT16_MAX)
  {
    s_corner_timer = AddMs(s_corner_timer, UINT16_MAX);
  }

  if ((s_corner_yaw_valid != 0U) && (s_yaw_live != 0U))
  {
    /* 每拍检查 s_yaw_live：陀螺仪中途掉线就立刻退回时间方式，
     * 而不是抱着冻结的角度一直转。 */
    {
      float rotated = LineFollow_YawRotated(s_yaw_deg, s_corner_yaw_start);
      /* 提前量：按实测角速度预测"还会再转多少度"，提前进入刹车。
       * 只按"到 90° 才刹"必然冲过去一点（刹车的生效本身有延迟）。 */
      float lead = s_corner_rate_dps *
                   ((float)LINE_FOLLOW_CORNER_BRAKE_LEAD_CYCLES / 1000.0f);

      if ((rotated + lead) >= LINE_FOLLOW_CORNER_TARGET_DEG)
      {
        /* 诊断：记下"开始刹车"这一刻的真实转角与原因 */
        s_corner_last_deg = rotated;
        s_corner_last_reason = 0U;
        s_corner_stage = 3U;      /* 进入刹车 */
        s_corner_timer = 0U;
        result->base_speed = 0.0f;
        result->left_command = -(float)s_corner_dir * LINE_FOLLOW_CORNER_BRAKE_DUTY;
        result->right_command = (float)s_corner_dir * LINE_FOLLOW_CORNER_BRAKE_DUTY;
        return;
      }
    }
  }
  else
  {
    s_corner_yaw_valid = 0U;      /* 陀螺仪掉了 → 后续按时间走 */
    if (s_corner_timer >= LINE_FOLLOW_CORNER_TIME_PIVOT_CYCLES)
    {
      /* 诊断：定时兜底结束。角度取"冻结前的最后一次读数"，仅用于观察 */
      s_corner_last_deg = LineFollow_YawRotated(s_yaw_deg, s_corner_yaw_start);
      s_corner_last_reason = 1U;
      s_corner_stage = 3U;
      s_corner_timer = 0U;
      result->base_speed = 0.0f;
      result->left_command = -(float)s_corner_dir * LINE_FOLLOW_CORNER_BRAKE_DUTY;
      result->right_command = (float)s_corner_dir * LINE_FOLLOW_CORNER_BRAKE_DUTY;
      return;
    }
  }

  result->base_speed = 0.0f;
  result->left_command = (float)s_corner_dir * LINE_FOLLOW_CORNER_PIVOT_DUTY;
  result->right_command = -(float)s_corner_dir * LINE_FOLLOW_CORNER_PIVOT_DUTY;
}
#endif

/* 直角只读诊断：不改变任何控制量，只把观察值提供给显示层。 */
void LineFollow_GetCornerDiag(float *last_deg, uint8_t *reason,
                              uint16_t *episodes)
{
#if LINE_FOLLOW_CORNER_ENABLE
  if (last_deg != NULL)
  {
    *last_deg = s_corner_last_deg;
  }
  if (reason != NULL)
  {
    *reason = s_corner_last_reason;
  }
  if (episodes != NULL)
  {
    *episodes = s_corner_episodes;
  }
#else
  if (last_deg != NULL)
  {
    *last_deg = 0.0f;
  }
  if (reason != NULL)
  {
    *reason = 0U;
  }
  if (episodes != NULL)
  {
    *episodes = 0U;
  }
#endif
}

/** 执行一次循迹状态机，不访问 PWM、方向 GPIO 或电机驱动。 */
LineFollowResult_t LineFollow_Update(const PDControllerConfig_t *pd_config,
                                     const MotorCommandConfig_t *motor_config,
                                     const LineFollowConfig_t *follow_config)
{
  LineFollowResult_t result;

  result.control = LineControl_Sample(pd_config);
  if (result.control.track.status != LINE_TRACK_STATUS_ALL_BLACK) {
    s_all_black_cycles = 0U;
  }

  result.base_speed = 0.0f;
  result.left_command = 0.0f;
  result.right_command = 0.0f;

#if LINE_FOLLOW_YAW_DAMP_ENABLE
  /* 每拍更新带符号角速度（循迹阻尼用），与直角那套互相独立 */
  LineFollow_UpdateYawRateSigned();
#endif

#if LINE_FOLLOW_CORNER_ENABLE
  /* 三灯特征计时：看到特征时被清零，之后每拍 +1 */
  if (s_corner_sig_cycles < UINT16_MAX)
  {
    s_corner_sig_cycles = AddMs(s_corner_sig_cycles, UINT16_MAX);
  }
  if (s_corner_cooldown > 0U)
  {
    s_corner_cooldown = SubMs(s_corner_cooldown);
  }
#endif

  if ((s_state != LINE_FOLLOW_STOPPED) &&
      ((LineFollow_ConfigIsValid(motor_config, follow_config) == 0U) ||
       (result.control.control_status == LINE_CONTROL_STATUS_INVALID_INPUT_STOP))) {
    LineFollow_SetFault(LINE_FOLLOW_FAULT_INVALID_INPUT, &result);
  }
  else if (s_state == LINE_FOLLOW_STOPPED)
  {
    /* 保持采样供调试观察，但停止状态绝不生成轮命令。 */
  }
  else if (s_state == LINE_FOLLOW_FAULT)
  {
    /* 故障锁存，必须重新 Start 或先 Stop 后再 Start。
     *
     * ★ 例外：NO_VALID_HISTORY（上电/复位后第一帧就没看到线）不锁存。
     *   发车那一刻探头可能还没压在线、或传感器刚上电没稳定，
     *   如果永久锁存，表现就是"车彻底不走"，只能断电重开。
     *   这里只要看到线就自动恢复循迹；其它故障仍然锁存。 */
    if ((s_fault == LINE_FOLLOW_FAULT_NO_VALID_HISTORY) &&
        (result.control.track.status == LINE_TRACK_STATUS_NORMAL))
    {
      s_fault = LINE_FOLLOW_FAULT_NONE;
      s_has_valid_history = 1U;
      s_lost_cycles = 0U;
      s_reacquire_cycles = 0U;
      s_state = LINE_FOLLOW_TRACKING;
      LineFollow_HandleNormal(motor_config, follow_config, &result);
    }
  }
#if LINE_FOLLOW_CORNER_ENABLE
  else if (s_state == LINE_FOLLOW_CORNER)
  {
    /* 直角处理优先级高于其他分支：转向途中即使遇到全黑（胶带重叠）
     * 或重新看到线，都由它自己按"停稳→转向→确认"的流程处理。 */
    LineFollow_HandleCorner(motor_config, follow_config, &result);
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

  s_post_corner_gentle_ms = SubMs(s_post_corner_gentle_ms);
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
