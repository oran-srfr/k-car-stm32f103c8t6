#include "pd_controller.h"

#include <math.h>
#include <stddef.h>

static float s_previous_deviation = 0.0f;
static uint8_t s_has_previous_deviation = 0U;
static float s_filtered_derivative = 0.0f;

/** 清除微分历史，供停止或重新启动循迹时使用。 */
void PDController_Reset(void)
{
  s_previous_deviation = 0.0f;
  s_has_previous_deviation = 0U;
  s_filtered_derivative = 0.0f;
}

/** 根据状态计算 PD 控制量；非正常状态不保留可被误用的转向输出。 */
PDControllerResult_t PDController_Calculate(LineTrackStatus_t status,
                                             float deviation,
                                             const PDControllerConfig_t *config)
{
  PDControllerResult_t result;
  float limit;
  float d_term;

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
    float derivative = (deviation - s_previous_deviation) / CONTROL_PERIOD_SECONDS;
    s_filtered_derivative += CONTROL_PD_DERIVATIVE_ALPHA *
                             (derivative - s_filtered_derivative);
    result.derivative = s_filtered_derivative;
  }

  s_previous_deviation = deviation;
  /* Digital sensors change in steps. Bound and filter D so one transition
   * cannot turn a small correction into a full steering reversal. */
  d_term = config->kd * result.derivative;
  if (d_term > CONTROL_PD_D_TERM_LIMIT) d_term = CONTROL_PD_D_TERM_LIMIT;
  if (d_term < -CONTROL_PD_D_TERM_LIMIT) d_term = -CONTROL_PD_D_TERM_LIMIT;
  result.output = (config->kp * deviation) + d_term;

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
