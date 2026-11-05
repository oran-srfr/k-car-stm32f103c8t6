#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "line_follow.h"

typedef struct
{
  LineFollowResult_t line_follow;
  bool tracking_started;
  bool motor_output_enabled;
  bool line_output_valid;
  float left_target_command;
  float right_target_command;
  int16_t left_applied_command;
  int16_t right_applied_command;
} VehicleControlOutput_t;

/* Watch变量：算法目标和安全门控后的实际命令分开保存。 */
extern volatile bool g_tracking_started;
extern volatile bool g_motor_output_enabled;
extern volatile float g_left_target_command;
extern volatile float g_right_target_command;
extern volatile int16_t g_left_applied_command;
extern volatile int16_t g_right_applied_command;
extern volatile LineFollowState_t g_vehicle_control_state;
extern volatile uint32_t g_control_call_count;

/** 初始化总控；保存配置地址并保持循迹停止、真实电机禁用。 */
void VehicleControl_Init(const PDControllerConfig_t *pd_config,
                         const MotorCommandConfig_t *motor_config,
                         const LineFollowConfig_t *follow_config);

/** 只启动循迹算法，不打开真实电机输出门控。 */
void VehicleControl_Start(void);

/** 覆盖所有状态，立即停止循迹并禁用真实电机。 */
void VehicleControl_Stop(void);

/** 每个固定周期调用一次，完成一次循迹更新和安全门控。 */
void VehicleControl_Update(void);
void VehicleControl_UpdateElapsed(uint32_t elapsed_ms);

/** 设置真实电机输出门控；打开门控不会立即启动驱动。 */
void VehicleControl_SetMotorOutputEnabled(bool enabled);

/* ===== 完赛停车 =====
 * 比赛要求：跑满两圈后、在 A 点过完直角时停车。
 * RequestFinishStop(ms) 会在 ms 毫秒内把两轮命令线性压到 0，之后保持 0。
 * 只作用于输出级，不改循迹算法；ClearFinishStop() 用于重新开始。 */
void VehicleControl_RequestFinishStop(uint16_t ramp_ms);
bool VehicleControl_IsFinishStopping(void);
void VehicleControl_ClearFinishStop(void);

/** 返回最近一次Update保存的结果，不重复执行控制计算。 */
VehicleControlOutput_t VehicleControl_GetOutput(void);

#endif /* VEHICLE_CONTROL_H */
