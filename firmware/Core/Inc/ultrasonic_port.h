#ifndef K_CAR_ULTRASONIC_PORT_H
#define K_CAR_ULTRASONIC_PORT_H

#include <stdbool.h>
#include <stdint.h>

/* Hardware boundary, replaced by a mock in host tests. */
bool UlrHw_Init(void);
uint16_t UlrHw_Counter(void);
bool UlrHw_EchoHigh(void);
void UlrHw_Trigger(void); /* Raise Trig, schedule interrupt to lower it after >=10 us. */
void UlrHw_CancelTrigger(void);
uint32_t UlrHw_Lock(void); /* Save PRIMASK and disable interrupts, nesting-safe. */
void UlrHw_Unlock(uint32_t key);

#endif
