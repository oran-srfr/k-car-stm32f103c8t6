#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H
#include <stdbool.h>
#include <stdint.h>
/* Init only after sensors, follower and motor driver are ready. */
void ControlLoop_Init(void);
/* SysTick context: bounded cached-yaw/GPIO/control/PWM work, no serial or LCD. */
void ControlLoop_Tick(uint32_t now_ms);
/* Foreground publishes an atomic mailbox; only Tick mutates follower yaw. */
void ControlLoop_SetYaw(float yaw, uint32_t sample_ms, bool valid);
extern volatile uint32_t g_control_max_gap_ms;
extern volatile uint32_t g_control_max_cycles;
extern volatile uint32_t g_control_overrun_count;
#endif
