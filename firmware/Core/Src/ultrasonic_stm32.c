#include "ultrasonic.h"
#include "ultrasonic_port.h"
#include "board_pins.h"

uint32_t UlrHw_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}

void UlrHw_Unlock(uint32_t key)
{
    __DMB();
    __set_PRIMASK(key);
}

bool UlrHw_Init(void)
{
    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
    /* APB1 timer clock doubles when APB1 is divided (RM0008). */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) { clock_hz *= 2U; }
    if (clock_hz < 1000000U || clock_hz % 1000000U != 0U ||
        clock_hz / 1000000U > 65536U) { return false; }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM4_FORCE_RESET();
    __HAL_RCC_TIM4_RELEASE_RESET();

    HAL_GPIO_WritePin(SONAR_PORT, SONAR_TRIG_PIN, GPIO_PIN_RESET);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SONAR_TRIG_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SONAR_PORT, &gpio);
    gpio.Pin = SONAR_ECHO_PIN;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SONAR_PORT, &gpio);

    /* Dedicated free-running TIM4: CH1 compare is internal, no PB6 pin mapping.
     * Direct timer registers avoid enabling unrelated HAL TIM driver modules. */
    TIM4->PSC = clock_hz / 1000000U - 1U;
    TIM4->ARR = 0xFFFFU;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
    TIM4->CR1 = TIM_CR1_CEN;
    __HAL_GPIO_EXTI_CLEAR_IT(SONAR_ECHO_PIN);
    HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    /* HAL_Init() uses priority group 4. Keep encoder priority 0 available. */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 0U);
    HAL_NVIC_SetPriority(TIM4_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    return true;
}

uint16_t UlrHw_Counter(void)
{
    return (uint16_t)TIM4->CNT;
}

bool UlrHw_EchoHigh(void)
{
    return HAL_GPIO_ReadPin(SONAR_PORT, SONAR_ECHO_PIN) == GPIO_PIN_SET;
}

void UlrHw_Trigger(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(SONAR_ECHO_PIN);
    HAL_GPIO_WritePin(SONAR_PORT, SONAR_TRIG_PIN, GPIO_PIN_SET);
    TIM4->CCR1 = (uint16_t)(UlrHw_Counter() + 10U);
    TIM4->SR = (uint16_t)~TIM_SR_CC1IF;
    TIM4->DIER |= TIM_DIER_CC1IE;
}

void UlrHw_CancelTrigger(void)
{
    TIM4->DIER &= ~TIM_DIER_CC1IE;
    TIM4->SR = (uint16_t)~TIM_SR_CC1IF;
    HAL_GPIO_WritePin(SONAR_PORT, SONAR_TRIG_PIN, GPIO_PIN_RESET);
}

void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & TIM_SR_CC1IF) != 0U && (TIM4->DIER & TIM_DIER_CC1IE) != 0U) {
        UlrHw_CancelTrigger();
        Ulr_OnTriggerEnd();
    }
}

void EXTI9_5_IRQHandler(void)
{
    /* EXTI5 is the only enabled line in this group today. Add dispatch if
     * future modules enable EXTI6..9; never define a second group handler. */
    if (__HAL_GPIO_EXTI_GET_IT(SONAR_ECHO_PIN) != 0U) {
        uint16_t counter = UlrHw_Counter();
        bool high = UlrHw_EchoHigh();
        uint32_t key = UlrHw_Lock();
        __HAL_GPIO_EXTI_CLEAR_IT(SONAR_ECHO_PIN);
        Ulr_OnEchoEdge(high, counter, HAL_GetTick());
        UlrHw_Unlock(key);
    }
}
