#include "motor_command.h"

#include <math.h>
#include <stddef.h>

static float MotorCommand_ClampForward(float command, float maximum)
{
  if (command < 0.0f)
  {
    return 0.0f;
  }
  if (command > maximum)
  {
    return maximum;
  }
  return command;
}

/** 仅对正常且参数有效的输入计算前进命令；其他情况返回停止命令。 */
MotorCommandResult_t MotorCommand_Calculate(LineTrackStatus_t status,
                                             float pd_correction,
                                             const MotorCommandConfig_t *config)
{
  MotorCommandResult_t result;

  result.status = status;
  result.left_command = 0.0f;
  result.right_command = 0.0f;

  if ((status != LINE_TRACK_STATUS_NORMAL) || (config == NULL) ||
      !isfinite(pd_correction) || !isfinite(config->base_speed) ||
      !isfinite(config->max_forward_command) ||
      (config->max_forward_command < 0.0f))
  {
    return result;
  }

  /* 正修正提高左侧、降低右侧命令；命令保持非负，不表达倒车方向。 */
  result.left_command = MotorCommand_ClampForward(config->base_speed + pd_correction,
                                                   config->max_forward_command);
  result.right_command = MotorCommand_ClampForward(config->base_speed - pd_correction,
                                                    config->max_forward_command);
  return result;
}
