#include "motor_command.h"

#include <math.h>
#include <stddef.h>

static float MotorCommand_Clamp(float command, float minimum, float maximum)
{
  if (command < minimum)
  {
    return minimum;
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
  float base;
  float correction;
  float headroom;

  result.status = status;
  result.left_command = 0.0f;
  result.right_command = 0.0f;

  if ((status != LINE_TRACK_STATUS_NORMAL) || (config == NULL) ||
      !isfinite(pd_correction) || !isfinite(config->base_speed) ||
      !isfinite(config->max_forward_command) ||
      !isfinite(config->min_forward_command) ||
      !isfinite(config->straight_trim) ||
      (config->min_forward_command < 0.0f) ||
      (config->max_forward_command < config->min_forward_command) ||
      (config->base_speed <= 0.0f))
  {
    return result;
  }

  /* Positive deviation means the line is to the RIGHT: left wheel faster.
   * Limit differential before mixing, then shift the common speed to retain
   * steering authority without stalling the inner wheel or reversing it. */
  headroom = (config->max_forward_command - config->min_forward_command) * 0.5f;
  correction = MotorCommand_Clamp(pd_correction + config->straight_trim,
                                   -headroom, headroom);
  base = MotorCommand_Clamp(config->base_speed,
                             config->min_forward_command + fabsf(correction),
                             config->max_forward_command - fabsf(correction));
  result.left_command = base + correction;
  result.right_command = base - correction;
  return result;
}
