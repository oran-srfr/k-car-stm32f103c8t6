#ifndef K_CAR_BUZZER_PORT_H
#define K_CAR_BUZZER_PORT_H

#include <stdbool.h>
#include <stdint.h>

/* The installed buzzer module sounds when PA0 is low. */
#define BUZZER_ACTIVE_LOW 1U

bool BuzzerHw_Init(void);
/* 0 Hz means silent. Nonzero frequencies use approximately 50% PWM. */
void BuzzerHw_SetFrequency(uint16_t frequency_hz);

#endif
