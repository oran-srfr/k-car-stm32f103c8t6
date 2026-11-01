#include "encoder.h"
#include "control_atomic.h"
#include "board_pins.h"

volatile uint32_t g_encoder_left_edges, g_encoder_right_edges;
void Encoder_LeftEdge(void) { ++g_encoder_left_edges; }
void Encoder_RightEdge(void) { ++g_encoder_right_edges; }
EncoderCounts Encoder_Read(void)
{
  uint32_t key = ControlAtomic_Enter();
  EncoderCounts counts = {g_encoder_left_edges, g_encoder_right_edges};
  ControlAtomic_Exit(key);
  return counts;
}
void Encoder_Init(void)
{
  g_encoder_left_edges = g_encoder_right_edges = 0U;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLUP;
  gpio.Pin = ENCODER_LEFT_PIN;
  HAL_GPIO_Init(ENCODER_LEFT_PORT, &gpio);
  gpio.Pin = ENCODER_RIGHT_PIN;
  HAL_GPIO_Init(ENCODER_RIGHT_PORT, &gpio);
  __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_LEFT_PIN | ENCODER_RIGHT_PIN);
  /* EXTI9_5 is shared with sonar EXTI5: dispatch in ultrasonic_stm32.c. */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 0U);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_SetPriority(SysTick_IRQn, 3U, 0U);
}
void EXTI15_10_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(ENCODER_RIGHT_PIN) != 0U) {
    __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_RIGHT_PIN);
    Encoder_RightEdge();
  }
}
