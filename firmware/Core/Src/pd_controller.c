#include "pd_controller.h"

#include <math.h>
#include <stddef.h>

static float s_previous_deviation = 0.0f;
static uint8_t s_has_previous_deviation = 0U;

/** 清除微分历史，供停止或重新启动循迹时使用。 */
void PDController_Reset(void)
{
  s_previous_deviation = 0.0f;
  s_has_previous_deviation = 0U;
}

/** 根据状态计算 PD 控制量；非正常状态不保留可被误用的转向输出。 */
PDControllerResult_t PDController_Calculate(LineTrackStatus_t status,
                                             float deviation,
                                             const PDControllerConfig_t *config)
{
  PDControllerResult_t result;
  float limit;

  result.status = status;
  result.output = 0.0f;
  result.derivative = 0.0f;

  if (status != LINE_TRACK_STATUS_NORMAL)
  {
    PDController_Reset();
    return result;
  }

  if ((config == NULL) || !isfinite(deviation) ||
      !isfinite(config->kp) || !isfinite(config->kd) ||
      !isfinite(config->output_limit))
  {
    PDController_Reset();
    return result;
  }

  if (s_has_previous_deviation == 0U)
  {
    result.derivative = 0.0f;
    s_has_previous_deviation = 1U;
  }
  else
  {
    result.derivative = (deviation - s_previous_deviation) / CONTROL_PERIOD_SECONDS;
  }

  s_previous_deviation = deviation;
  result.output = (config->kp * deviation) + (config->kd * result.derivative);

  limit = (config->output_limit >= 0.0f) ? config->output_limit : -config->output_limit;
  if (result.output > limit)
  {
    result.output = limit;
  }
  else if (result.output < -limit)
  {
    result.output = -limit;
  }

  return result;
}
