#ifndef K_CAR_ULTRASONIC_H
#define K_CAR_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ULR_NOT_READY, ULR_OK, ULR_TOO_CLOSE, ULR_TOO_FAR,
    ULR_NO_ECHO, ULR_ECHO_STUCK, ULR_HW_ERROR
} UlrStatus;

typedef struct {
    UlrStatus status;
    uint16_t distance_mm;
    uint16_t pulse_us;
    uint32_t sample_ms;
} UlrReading;

/* Foreground API. Init owns TIM4 and PA4/PA5; no motors are configured. */
bool Ulr_Init(uint32_t now_ms);
void Ulr_Task(uint32_t now_ms);
UlrReading Ulr_GetReading(void);

/* ISR entry points: timestamps must be captured at the edge, not in the loop. */
void Ulr_OnTriggerEnd(void);
void Ulr_OnEchoEdge(bool high, uint16_t counter_us, uint32_t now_ms);

#endif
