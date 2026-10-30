#include "main.h"
#include "app.h"

/* Watch these variables in the debugger. Motor outputs remain unconfigured. */
volatile uint32_t heartbeat_count = 0U;
volatile uint32_t uptime_ms = 0U;
volatile uint32_t system_clock_hz = 0U;
volatile uint32_t error_code = 0U;

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    /* HSI 8MHz / 2 = 4MHz，PLL × 16 = 64MHz。不依赖外部晶振，任何板子上都能跑。
     *
     * 为什么必须开 PLL：
     *   原来跑 8MHz，比 64MHz 慢 8 倍。软件 SPI 刷一次 16 字符的屏幕要 ~1.2M 周期，
     *   在 8MHz 下就是 ~175ms，而显示每 100ms 才触发一次 —— 控制循环被切碎到
     *   几 Hz，循迹必然「反应慢」。
     *
     * APB1 分频 = 2（32MHz，RM0008 规定 APB1 上限 36MHz）。
     * 注意：APB1 分频不为 1 时，TIM2/TIM3/TIM4 的时钟是 PCLK1 的 2 倍（=64MHz）。
     *      buzzer_stm32.c / ultrasonic_stm32.c 已经处理了这条规则，所以它们
     *      算出来的 1MHz 定时器节拍仍然正确。 */
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState  = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    oscillator.PLL.PLLMUL    = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        error_code = 1U;
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    /* 64MHz 必须 2 个 Flash 等待周期 */
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK) {
        error_code = 2U;
        Error_Handler();
    }

    /* 预取缓冲：64MHz 下对性能影响明显 */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

int main(void)
{
    if (HAL_Init() != HAL_OK) {
        error_code = 3U;
        Error_Handler();
    }
    SystemClock_Config();
    system_clock_hz = HAL_RCC_GetSysClockFreq();
    App_Init(HAL_GetTick());
    uint32_t heartbeat_ms = HAL_GetTick();

    while (1) {
        uptime_ms = HAL_GetTick();
        App_Task(uptime_ms);
        if ((uint32_t)(uptime_ms - heartbeat_ms) >= 1000U) {
            heartbeat_ms = uptime_ms;
            heartbeat_count++;
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Inspect error_code after pausing the debugger. */
    }
}
