#include "main.h"

/* Watch these variables in the debugger. No board-specific GPIO is driven. */
volatile uint32_t heartbeat_count = 0U;
volatile uint32_t uptime_ms = 0U;
volatile uint32_t system_clock_hz = 0U;
volatile uint32_t error_code = 0U;

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    /* Use the internal 8 MHz oscillator until the board crystal is confirmed. */
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        error_code = 1U;
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_0) != HAL_OK) {
        error_code = 2U;
        Error_Handler();
    }
}

int main(void)
{
    if (HAL_Init() != HAL_OK) {
        error_code = 3U;
        Error_Handler();
    }
    SystemClock_Config();
    system_clock_hz = HAL_RCC_GetSysClockFreq();

    while (1) {
        HAL_Delay(1000U);
        uptime_ms = HAL_GetTick();
        heartbeat_count++;
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Inspect error_code after pausing the debugger. */
    }
}
