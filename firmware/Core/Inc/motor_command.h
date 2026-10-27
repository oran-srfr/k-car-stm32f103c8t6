#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include "line_track.h"

typedef struct
{
  float base_speed;          /* 抽象前进命令，单位为软件命令单位，不是物理速度。 */
  float max_forward_command; /* 单侧最大前进命令；输出范围为 0 到该值。 */
} MotorCommandConfig_t;

typedef struct
{
  LineTrackStatus_t status;
  float left_command;  /* 抽象左轮前进命令，不代表电机方向或 PWM。 */
  float right_command; /* 抽象右轮前进命令，不代表电机方向或 PWM。 */
} MotorCommandResult_t;

/** 按循迹状态混合基础命令与 PD 修正并限幅；无效状态或参数返回 0。 */
MotorCommandResult_t MotorCommand_Calculate(LineTrackStatus_t status,
                                             float pd_correction,
                                             const MotorCommandConfig_t *config);

#endif /* MOTOR_COMMAND_H */
