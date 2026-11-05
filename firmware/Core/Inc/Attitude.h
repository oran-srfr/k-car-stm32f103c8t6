#ifndef K_CAR_ATTITUDE_H
#define K_CAR_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#include "LapTimer.h"
#include "MpuLink.h"

typedef struct {
    bool ready;
    bool live;
    float yaw_deg;
    uint32_t yaw_ms;
    MpuLinkStats link;
    LapStatus lap;
} AttitudeStatus;

/* ===== 陀螺仪健康判断（只读，不改变任何控制量）= =====
 *
 * 只看两件事就能判断"陀螺仪到底有没有在工作"：
 *   ① 收到过帧吗（frames_ok）—— 一帧都没有 = 接线/波特率/模块供电问题
 *   ② 最近多久没新帧（age_ms）—— 超过超时 = 中途掉了
 *
 * moving 只是辅助观察：车在转时 yaw 应变；车停着 yaw 不变是正常的，
 * 所以【不要】用 moving == false 判断故障。 */
typedef enum {
    ATTITUDE_GYRO_NO_DATA = 0,  /* 一帧都没收到过 */
    ATTITUDE_GYRO_STALE   = 1,  /* 收到过，但现在已超时没有新帧 */
    ATTITUDE_GYRO_OK      = 2   /* 正在持续出数据 */
} AttitudeGyroCode;

typedef struct {
    AttitudeGyroCode code;
    uint32_t age_ms;      /* 距最后一帧的毫秒数（从未收到时为 999） */
    uint32_t frames_ok;   /* 累计有效帧数 */
    uint32_t frames_bad;  /* 累计校验失败帧数 */
    float yaw_deg;        /* 当前 yaw */
    bool moving;          /* 最近窗口内 yaw 是否有变化 */
} AttitudeGyroHealth;

bool Attitude_Init(uint32_t baud_rate);
void Attitude_Poll(uint32_t now_ms);
void Attitude_StartLap(uint32_t now_ms);
void Attitude_StopLap(void);
void Attitude_GetStatus(AttitudeStatus *out);

/** 判断陀螺仪当前是否在正常工作；now_ms 传 HAL_GetTick()。 */
void Attitude_GetGyroHealth(uint32_t now_ms, AttitudeGyroHealth *out);

/** 便捷判断：true = 陀螺仪正在持续出数据。 */
bool Attitude_IsGyroWorking(uint32_t now_ms);

#endif
