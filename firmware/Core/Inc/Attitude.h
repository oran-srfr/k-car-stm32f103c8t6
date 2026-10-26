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

bool Attitude_Init(uint32_t baud_rate);
void Attitude_Poll(uint32_t now_ms);
void Attitude_StartLap(uint32_t now_ms);
void Attitude_StopLap(void);
void Attitude_GetStatus(AttitudeStatus *out);

#endif
