/* 仅用于电脑上跑 MpuLink 单元测试的最小 HAL 桩，不参与固件编译 */
#ifndef STM32F1XX_HAL_H_TEST_STUB
#define STM32F1XX_HAL_H_TEST_STUB

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef struct {
    void *Instance;
} UART_HandleTypeDef;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t size);

#endif /* STM32F1XX_HAL_H_TEST_STUB */
