#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H
#include "line_follow.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct {
  bool calibrated, active;
  float left_hz, right_hz, reference_hz;
  float left_correction, right_correction;
} WheelSpeedStatus;
extern volatile bool g_wheel_speed_enabled;
extern volatile WheelSpeedStatus g_wheel_speed_status;
void WheelSpeed_Init(void);
void WheelSpeed_Sample(uint32_t left, uint32_t right, uint32_t now_ms, bool moving);
void WheelSpeed_Apply(const LineFollowResult_t *follow, const MotorCommandConfig_t *config,
                      float *left, float *right);
#endif
