#ifndef K_CAR_BUZZER_H
#define K_CAR_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUZZER_POINT_B, BUZZER_POINT_C, BUZZER_OBSTACLE, BUZZER_LEAVE_LINE,
    BUZZER_EVENT_COUNT
} BuzzerEvent;

/* Foreground-only API, not callable from an interrupt. No automatic boot beep. */
bool Buzzer_Init(void);
bool Buzzer_Play(BuzzerEvent event, uint32_t now_ms);
void Buzzer_Task(uint32_t now_ms);
void Buzzer_Stop(void); /* Silence immediately and discard queued events. */
bool Buzzer_IsReady(void);
bool Buzzer_IsBusy(void);

#endif
