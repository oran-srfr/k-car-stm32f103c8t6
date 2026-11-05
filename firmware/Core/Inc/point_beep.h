#ifndef K_CAR_POINT_BEEP_H
#define K_CAR_POINT_BEEP_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * point_beep —— B / C 点鸣笛（场地：A→B 直道 85cm，B→C 为 R60 半圆）
 *
 * B、C 是半圆的两个端点。本模块只回答"现在是不是到了 B / C"，
 * 具体响什么由调用者决定（firmware 里接 Modules_Sound(APP_SOUND_B/C)）。
 *
 * 判定思路（方案 1：陀螺仪曲率 + 累计转角）
 *   B = 检测到"开始持续转向"（车驶入半圆）
 *   C = 从 B 起累计转过 ARC_EXIT_TOTAL_DEG 度，且转向已经停止（车驶出半圆）
 *
 * 为什么这样判：
 *   B→C 是固定 180° 的半圆，所以"累计转过多少度"是最硬的判据，
 *   不需要标定"每厘米多少脉冲"，也不受电量、地面、速度影响。
 *
 * 只读：不参与任何控制，也不改变循迹/直角的判断。
 * 每个点每圈只触发一次，PointBeep_Reset() 清除。
 *
 * 用法（1 ms 调一次）：
 *   uint8_t ev = PointBeep_Update(yaw_deg, gyro_live, now_ms);
 *   if (ev == POINT_BEEP_B) { 响 B }
 *   else if (ev == POINT_BEEP_C) { 响 C }
 * ========================================================================== */

typedef enum {
    POINT_BEEP_NONE = 0,
    POINT_BEEP_B = 1,
    POINT_BEEP_C = 2
} PointBeepEvent;

typedef struct {
    uint8_t  b_done;      /* 本圈 B 已响过 */
    uint8_t  c_done;      /* 本圈 C 已响过 */
    uint8_t  in_arc;      /* 1 = 已判定在弯道里（B 之后、C 之前） */
    uint16_t arc_deg;     /* 从 B 起累计转过的角度（整数度） */
} PointBeepStatus;

void PointBeep_Init(void);

/* 每圈开始（或每次发车）清一次状态 */
void PointBeep_Reset(void);

/* 1 ms 调一次；返回本拍要响的事件（0 = 不响）。 */
PointBeepEvent PointBeep_Update(float yaw_deg, uint8_t live, uint32_t now_ms);

void PointBeep_GetStatus(PointBeepStatus *out);

#endif /* K_CAR_POINT_BEEP_H */
