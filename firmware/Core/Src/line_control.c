#include "line_control.h"

#include <math.h>
#include <stddef.h>

/** 完成一次采样、循迹和 PD 计算，供更高层状态机复用。 */
LineControlResult_t LineControl_Sample(const PDControllerConfig_t *pd_config)
{
  LineControlResult_t result;

  result.raw = LineSensor_ReadRaw();
  result.raw_bits = (uint8_t)(result.raw.x1 | (result.raw.x2 << 1U) |
                              (result.raw.x3 << 2U) | (result.raw.x4 << 3U));
  result.track = LineTrack_Calculate(result.raw_bits);
  /* 贴边循迹：PD 的目标不是偏差 0，而是 LINE_FOLLOW_TARGET_DEVIATION。
   * 误差 = 实际偏差 - 设定值，于是车在设定偏移处得到一个稳定平台。 */
  result.pd = PDController_Calculate(result.track.status,
                                     result.track.deviation -
                                         LINE_FOLLOW_TARGET_DEVIATION,
                                     pd_config);

  if (result.track.status == LINE_TRACK_STATUS_LOST)
  {
    result.control_status = LINE_CONTROL_STATUS_LOST_STOP;
  }
  else if (result.track.status == LINE_TRACK_STATUS_ALL_BLACK)
  {
    result.control_status = LINE_CONTROL_STATUS_ALL_BLACK_STOP;
  }
  else if (result.track.status != LINE_TRACK_STATUS_NORMAL)
  {
    result.control_status = LINE_CONTROL_STATUS_INVALID_INPUT_STOP;
  }
  else if ((pd_config == NULL) ||
           !isfinite(result.track.deviation) ||
           !isfinite(pd_config->kp) || !isfinite(pd_config->kd) ||
           !isfinite(pd_config->output_limit) ||
           !isfinite(result.pd.output))
  {
    result.control_status = LINE_CONTROL_STATUS_INVALID_INPUT_STOP;
  }
  else
  {
    result.control_status = LINE_CONTROL_STATUS_ACTIVE;
  }

  return result;
}
