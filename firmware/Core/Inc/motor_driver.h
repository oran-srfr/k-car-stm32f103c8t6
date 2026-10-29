#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/* 抽象命令范围；正数表示逻辑前进，负数表示逻辑反转。 */
#define MOTOR_DRIVER_COMMAND_MAX       (1000)

/* 实车方向相反时只修改对应配置，不需要改方向控制逻辑。 */
#ifndef MOTOR_DRIVER_LEFT_REVERSED
#define MOTOR_DRIVER_LEFT_REVERSED     (0U)
#endif
#ifndef MOTOR_DRIVER_RIGHT_REVERSED
#define MOTOR_DRIVER_RIGHT_REVERSED    (0U)
#endif

/* Watch 调试变量：请求命令即接口输入，实际命令只在驱动使能时非零。 */
extern volatile uint8_t g_motor_driver_initialized;
extern volatile uint8_t g_motor_driver_enabled;
extern volatile int16_t g_motor_driver_left_request;
extern volatile int16_t g_motor_driver_right_request;
extern volatile int16_t g_motor_driver_left_actual;
extern volatile int16_t g_motor_driver_right_actual;
extern volatile uint32_t g_motor_driver_left_pwm_compare;
extern volatile uint32_t g_motor_driver_right_pwm_compare;
extern volatile int8_t g_motor_driver_left_direction;
extern volatile int8_t g_motor_driver_right_direction;

/** 初始化PWM和安全输出；完成后保持禁用。 */
void MotorDriver_Init(void);

/** 显式使能TB6612；使能后仍保持零输出，直到再次设置命令。 */
void MotorDriver_Enable(void);

/** 立即清零PWM和方向脚；STBY 硬接高电平，不受软件控制。 */
void MotorDriver_Disable(void);

/** 设置左轮命令，范围会限制到-1000～+1000。 */
void MotorDriver_SetLeft(int16_t command);

/** 设置右轮命令，范围会限制到-1000～+1000。 */
void MotorDriver_SetRight(int16_t command);

/** 同时设置左右轮命令。 */
void MotorDriver_SetBoth(int16_t left, int16_t right);

/** 两轮停止但不改变当前使能状态。 */
void MotorDriver_Stop(void);

#endif /* MOTOR_DRIVER_H */
