#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include "line_track.h"
#include "control_config.h"

typedef struct
{
  float kp;           /* 比例增益：控制量单位/偏差单位。 */
  float kd;           /* 微分增益：控制量单位·秒/偏差单位。 */
  float output_limit; /* 输出绝对限幅；单位与 PD 输出相同。 */
} PDControllerConfig_t;

typedef struct
{
  LineTrackStatus_t status;
  float output;     /* 仅 NORMAL 时有效；是抽象控制量，不是电机指令。 */
  float derivative; /* 低通滤波后的偏差变化率，单位为偏差单位/秒。 */
} PDControllerResult_t;

/* 保留原宏名，实际数值由 control_config.h 集中管理。 */
#define PD_SAMPLE_PERIOD_MS      CONTROL_PERIOD_MS
#define PD_SAMPLE_PERIOD_SECONDS CONTROL_PERIOD_SECONDS

/** 正常循迹时按固定周期计算 PD；丢线、全黑或输入无效时返回 0 并重置历史。 */
PDControllerResult_t PDController_Calculate(LineTrackStatus_t status,
                                             float deviation,
                                             const PDControllerConfig_t *config);

/** 清除上一次偏差，保证下一次正常计算从零微分开始。 */
void PDController_Reset(void);

#endif /* PD_CONTROLLER_H */
