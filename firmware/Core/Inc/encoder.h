#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * 编码器测速（只计数，不做闭环）。
 *
 * 接线（见 board_pins.h / 引脚占用表）：
 *   左电机 A 相 → PA8   （EXTI8，与超声波 PA5 共用 EXTI9_5 向量）
 *   右电机 A 相 → PB12  （EXTI12 → EXTI15_10 向量）
 *   V → 3.3V，G → GND，B 相不接
 *
 * 用途：用数字判断"同样命令下左右轮实际转速是否一致"。
 * 之前靠肉眼看轮子分不出差别，摆头到底是控制问题还是电机个体差异一直无法定论，
 * 有了这个模块就能直接读数：
 *   居中（两个内探头压线）时左右 delta 应当接近相等；
 *   若长期相差 >10%，就是电机/减速箱的开环不对称，用 straight_trim 补偿。
 *
 * 限制（务必注意）：
 *   1) 只有 A 相 → 数不出方向。前进与倒车都会让计数增加，
 *      所以原地转向（一轮正转一轮反转）时两轮计数没有可比性。
 *   2) 分辨率有限：GM37-520 约 11 线 × 减速比，单相单边沿约 300~400 脉冲/圈。
 *      100 ms 窗口在低速时可能只有几个脉冲，量化误差大；想看细节就把
 *      ENCODER_WINDOW_MS 改大（例如 200 ms）。
 *   3) 编码器没供电（V/G 没接）时计数恒为 0 —— 这本身也是一个有用的诊断结论。
 */

typedef struct
{
    uint32_t left_total;   /* 上电以来的总脉冲数 */
    uint32_t right_total;
    uint16_t left_delta;   /* 最近一个窗口内的脉冲数（速度代理量） */
    uint16_t right_delta;
} EncoderStats_t;

/** 配置 PA8 / PB12 为上升沿外部中断；须在 App_Init 的模块初始化阶段调用。 */
void Encoder_Init(void);

/** 每 100 ms 结算一次窗口计数；在主循环里（或 Modules_Poll）反复调用即可。 */
void Encoder_Poll(uint32_t now_ms);

void Encoder_GetStats(EncoderStats_t *out);
uint16_t Encoder_GetLeftDelta(void);
uint16_t Encoder_GetRightDelta(void);

/**
 * 闭环配平：把"当前左右轮的软件命令"告诉本模块，用于判断何时该配平。
 * 只有左右命令相等（直线）时才积分脉冲差；转弯时冻结并缓慢泄放
 * （转弯本来就需要差速，不能强行拉平）。
 * 每 1 ms 调一次即可，内部只存两个 float。
 */
void Encoder_SetCommandContext(float left_cmd, float right_cmd);

/** 返回应叠加到左命令、并从右命令扣除的补偿量（已含符号与限幅）。 */
float Encoder_GetBalance(void);

/* 中断服务程序调用的计数入口；不要在普通代码里调用。 */
void Encoder_OnLeftEdge(void);
void Encoder_OnRightEdge(void);

#endif /* ENCODER_H */
