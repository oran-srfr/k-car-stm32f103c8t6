#ifndef CONTROL_ATOMIC_H
#define CONTROL_ATOMIC_H
#include <stdint.h>
#if defined(__arm__) || defined(__thumb__)
#include "stm32f1xx_hal.h"
#endif
static inline uint32_t ControlAtomic_Enter(void)
{
#if defined(__arm__) || defined(__thumb__)
  uint32_t key = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return key;
#else
  return 0U;
#endif
}
static inline void ControlAtomic_Exit(uint32_t key)
{
#if defined(__arm__) || defined(__thumb__)
  __DMB();
  __set_PRIMASK(key);
#else
  (void)key;
#endif
}
#endif
