#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"

typedef struct
{
  uint8_t x1;
  uint8_t x2;
  uint8_t x3;
  uint8_t x4;
} LineSensorRaw_t;

/** 配置本项目四路循迹输入为带上拉的 GPIO 输入。 */
void LineSensor_Init(void);

/** 读取 X1 到 X4 的原始引脚电平；1 表示高电平，不解释黑线或左右顺序。 */
LineSensorRaw_t LineSensor_ReadRaw(void);

#endif /* LINE_SENSOR_H */
