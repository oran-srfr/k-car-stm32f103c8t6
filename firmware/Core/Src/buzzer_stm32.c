#include "buzzer_port.h"
#include "board_pins.h"

bool BuzzerHw_Init(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) { clock_hz *= 2U; }

    /* Require a 1 MHz timer tick and the default TIM2_CH1=PA0 routing.
     * Do not modify AFIO MAPR here: preserve SWD/JTAG and other peripheral remaps. */
    bool supported = clock_hz >= 1000000U && clock_hz % 1000000U == 0U &&
                     clock_hz / 1000000U <= 65536U &&
                     (AFIO->MAPR & AFIO_MAPR_TIM2_REMAP) == 0U;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();

    /* Hold the transistor input low even if this clock/routing is unsupported. */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BUZZER_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    if (!supported) { return false; }

    TIM2->PSC = clock_hz / 1000000U - 1U;
    TIM2->ARR = 999U;
    TIM2->CCR1 = 0U;
    TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM2->CCER = TIM_CCER_CC1E; /* PWM1, active high, only CH1 enabled. */
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    gpio.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    return true;
}

void BuzzerHw_SetFrequency(uint16_t frequency_hz)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    if (frequency_hz == 0U) {
        /* Force OC1REF inactive as well as clearing CCR: never freeze PA0 high. */
        TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;
        TIM2->CCR1 = 0U;
        TIM2->EGR = TIM_EGR_UG;
    } else {
        uint32_t period = (1000000U + frequency_hz / 2U) / frequency_hz;
        if (period < 2U) { period = 2U; }
        if (period > 65536U) { period = 65536U; }
        TIM2->ARR = period - 1U;
        TIM2->CCR1 = period / 2U;
        TIM2->CNT = 0U;
        TIM2->EGR = TIM_EGR_UG;
        TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M) |
                      TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
        TIM2->CR1 |= TIM_CR1_CEN;
    }
    TIM2->SR = 0U;
}
