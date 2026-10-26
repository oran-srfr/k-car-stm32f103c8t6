/* ============================================================================
 * MpuLink.h —— MPU6050 串口姿态模块数据解析
 *
 * 模块协议（高精度 6 轴姿态模块，串口输出，115200，100Hz）：
 *
 *     角度帧长度 11 字节：
 *     [0] 0x55      帧头
 *     [1] 0x53      角度包类型
 *     [2] RollL      [3] RollH      (int16, 无量纲)
 *     [4] PitchL     [5] PitchH
 *     [6] YawL       [7] YawH       ← 判圈只用这个
 *     [8] VL         [9] VH         (版本号)
 *     [10] SUM                      = 前 10 字节累加和的低 8 位
 *
 *     Yaw(度) = (int16)(YawH<<8 | YawL) / 32768 * 180      范围 ±180°
 *
 * 设计要点：
 *   1. 中断只做「把字节塞进环形缓冲」，解析放主循环。
 *      Cortex-M3 没有 FPU，浮点运算压在中断里会拖慢编码器脉冲采集。
 *   2. 流式状态机 + 校验和，任意位置插入垃圾字节都能重新同步。
 *   3. 环形缓冲溢出会计数，方便发现主循环被拖慢。
 *
 * 用法：
 *     MpuLink_Init();
 *     MpuLink_AttachUart(&huart2);        // 内部会启动中断接收
 *
 *     // HAL 回调里转发
 *     void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
 *         if (h->Instance == USART2) MpuLink_RxCpltCallback();
 *     }
 *
 *     // 主循环
 *     MpuLink_Poll(HAL_GetTick());
 *     float yaw; uint32_t t;
 *     while (MpuLink_TakeYaw(&yaw, &t)) {
 *         Lap_FeedYaw(yaw, t);
 *     }
 * ========================================================================== */

#ifndef MPULINK_H
#define MPULINK_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 参数
 * ========================================================================== */

/* 环形缓冲大小，必须是 2 的幂。
 * 100Hz × 11 字节 = 1100 B/s，主循环哪怕卡 50ms 也只积压 55 字节，128 够用。
 * 太小会在主循环被刷屏拖慢时溢出。 */
#define MPU_RING_SIZE       128u

#define MPU_FRAME_LEN       11u
#define MPU_HEAD            0x55u
#define MPU_TYPE_ANGLE      0x53u

/* 超过这个时间（ms）没收到有效帧，认为链路断了 */
#define MPU_LIVE_TIMEOUT_MS 500u

/* Yaw 待取队列深度（必须是 2 的幂）。
 * 为什么需要队列而不是只留最新值：
 *   Lap_FeedYaw 需要每一帧都喂进去算增量。如果一次 Poll 解析出多帧却只交付
 *   最后一帧，中间的转角就永久丢失，累积角会偏小、判圈偏晚。
 * 8 帧 = 80ms 的积压容忍度，够主循环刷一次屏。 */
#define MPU_YAW_QUEUE       8u

/* ============================================================================
 * 统计信息
 * ========================================================================== */
typedef struct {
    uint32_t bytes_total;    /* 收到的总字节数 */
    uint32_t frames_ok;      /* 校验通过的帧数 */
    uint32_t frames_bad;     /* 校验失败的帧数 */
    uint32_t ring_overflow;  /* 环形缓冲溢出字节数（>0 说明主循环太慢）*/
    uint32_t frames_dropped; /* Yaw 队列满而丢弃的帧数（应该在 0 或极小）*/
    bool     live;           /* 最近 MPU_LIVE_TIMEOUT_MS 内收到过有效帧 */
} MpuLinkStats;

/* ============================================================================
 * 接口
 * ========================================================================== */

/* 上电初始化（清空所有状态）*/
void MpuLink_Init(void);

/* 绑定串口句柄（实际类型是 UART_HandleTypeDef*）并启动中断接收。
 * 这里用 void* 是为了让本头文件不依赖 HAL 头，方便在电脑上做单元测试。 */
void MpuLink_AttachUart(void *huart);

/* 在 HAL_UART_RxCpltCallback 里转发调用 */
void MpuLink_RxCpltCallback(void);

/* 在 HAL_UART_ErrorCallback 里转发调用（自动重启接收）*/
void MpuLink_ErrorCallback(void);

/* 主循环里调：从环形缓冲取字节、解析帧。
 * now_ms 传 HAL_GetTick() */
void MpuLink_Poll(uint32_t now_ms);

/* 取最新 Yaw。有新帧则返回 true 并消费掉；没有则返回 false。
 * 可以在 while 循环里反复取，一次取出所有新帧（判圈需要每一帧都喂） */
bool MpuLink_TakeYaw(float *yaw_deg, uint32_t *t_ms);

/* 取统计信息 */
void MpuLink_GetStats(MpuLinkStats *out);

/* 直接喂一个字节（仅供离线测试用，正常运行时由中断调用）*/
void MpuLink_FeedByte(uint8_t b);

/* 把 16 位原始值换算成角度（度）。单独暴露出来方便测试 */
float MpuLink_RawToDeg(int16_t raw);

#endif /* MPULINK_H */
