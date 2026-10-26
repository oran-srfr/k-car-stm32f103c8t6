#ifndef K_CAR_BUZZER_PORT_H
#define K_CAR_BUZZER_PORT_H

#include <stdbool.h>
#include <stdint.h>

bool BuzzerHw_Init(void);
/* 0 Hz means silent/PA0 low. Nonzero frequencies use approximately 50% PWM. */
void BuzzerHw_SetFrequency(uint16_t frequency_hz);

#endif
