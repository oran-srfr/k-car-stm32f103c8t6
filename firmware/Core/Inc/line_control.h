#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include "line_sensor.h"
#include "line_track.h"
#include "pd_controller.h"

typedef enum
{
  LINE_CONTROL_STATUS_ACTIVE = 0U,
  LINE_CONTROL_STATUS_LOST_STOP,
  LINE_CONTROL_STATUS_ALL_BLACK_STOP,
  LINE_CONTROL_STATUS_DISABLED_STOP,
  LINE_CONTROL_STATUS_INVALID_INPUT_STOP
} LineControlStatus_t;

typedef struct
{
  LineSensorRaw_t raw;
  uint8_t raw_bits;
  LineTrackResult_t track;
  PDControllerResult_t pd;
  LineControlStatus_t control_status;
} LineControlResult_t;

/** 只进行一次传感器采样、循迹计算和 PD 计算，不生成轮命令。 */
LineControlResult_t LineControl_Sample(const PDControllerConfig_t *pd_config);

#endif /* LINE_CONTROL_H */
