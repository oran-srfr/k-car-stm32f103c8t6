#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

#include "line_control.h"
#include "motor_command.h"

typedef enum
{
  LINE_FOLLOW_STOPPED = 0U,
  LINE_FOLLOW_TRACKING,
  LINE_FOLLOW_LINE_LOST,
  LINE_FOLLOW_SEARCHING,
  LINE_FOLLOW_REACQUIRE,
  LINE_FOLLOW_FAULT,
  LINE_FOLLOW_TURNING
} LineFollowState_t;

typedef enum
{
  LINE_FOLLOW_FAULT_NONE = 0U,
  LINE_FOLLOW_FAULT_LOST_TIMEOUT,
  LINE_FOLLOW_FAULT_ALL_BLACK,
  LINE_FOLLOW_FAULT_NO_VALID_HISTORY,
  LINE_FOLLOW_FAULT_INVALID_INPUT,
  LINE_FOLLOW_FAULT_TURN_TIMEOUT
} LineFollowFault_t;

typedef enum
{
  LINE_FOLLOW_TURN_NONE = 0U,
  LINE_FOLLOW_TURN_SEEK,
  LINE_FOLLOW_TURN_ALIGN
} LineFollowTurnPhase_t;

typedef struct
{
  float curve_speed;                 /* 普通弯基础命令。 */
  float sharp_curve_speed;           /* 急弯基础命令。 */
  float search_speed;                /* 丢线找线基础命令。 */
  float straight_max_deviation;      /* 小于等于该绝对偏差时使用直线速度。 */
  float curve_max_deviation;         /* 小于等于该绝对偏差时使用普通弯速度。 */
  float search_turn_correction;      /* 找线时按最后方向施加的差速修正量。 */
  uint16_t max_lost_cycles;          /* 丢线预算；连续稳定找回后才清零。 */
  uint16_t reacquire_cycles;         /* 连续找回确认周期数，同时平滑恢复基础速度。 */
} LineFollowConfig_t;

typedef struct
{
  LineControlResult_t control;
  LineFollowState_t state;
  LineFollowFault_t fault;
  float last_valid_deviation;
  int8_t last_valid_direction;       /* -1 左，0 居中，+1 右。 */
  uint16_t lost_cycles;
  uint16_t reacquire_cycles;
  float base_speed;
  float left_command;
  float right_command;
  LineFollowTurnPhase_t turn_phase;
  uint16_t turn_cycles;              /* Last/current turn duration, for diagnosis. */
} LineFollowResult_t;

/** 初始化循迹模块；上电默认进入 STOPPED，所有命令为 0。 */
void LineFollow_Init(void);

/** 清除故障和历史，从下一控制周期开始循迹。 */
void LineFollow_Start(void);

/** 立即进入停止状态，软件左右目标命令保持为 0。 */
void LineFollow_Stop(void);

/** 执行一次状态机更新；只生成软件目标命令，不访问电机硬件。 */
LineFollowResult_t LineFollow_Update(const PDControllerConfig_t *pd_config,
                                     const MotorCommandConfig_t *motor_config,
                                     const LineFollowConfig_t *follow_config);

/** 返回最近一次周期更新结果；不会重新读取传感器或执行计算。 */
LineFollowResult_t LineFollow_GetOutput(void);

#endif /* LINE_FOLLOW_H */
