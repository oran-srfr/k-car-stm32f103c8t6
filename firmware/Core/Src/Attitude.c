#include "Attitude.h"

#include "board_pins.h"
#include "stm32f1xx_hal.h"

#include <stddef.h>

static UART_HandleTypeDef s_uart;
static bool s_ready;
static float s_yaw_deg;
static uint32_t s_yaw_ms;

bool Attitude_Init(uint32_t baud_rate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    gpio.Pin = ATTITUDE_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ATTITUDE_PORT, &gpio);

    gpio.Pin = ATTITUDE_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ATTITUDE_PORT, &gpio);

    s_uart.Instance = USART2;
    s_uart.Init.BaudRate = baud_rate;
    s_uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits = UART_STOPBITS_1;
    s_uart.Init.Parity = UART_PARITY_NONE;
    s_uart.Init.Mode = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    s_ready = HAL_UART_Init(&s_uart) == HAL_OK;
    if (!s_ready) {
        return false;
    }

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    MpuLink_Init();
    Lap_Init();
    s_yaw_deg = 0.0f;
    s_yaw_ms = 0U;

    MpuLink_AttachUart(&s_uart);
    return true;
}

void Attitude_Poll(uint32_t now_ms)
{
    float yaw;
    uint32_t yaw_ms;

    MpuLink_Poll(now_ms);
    while (MpuLink_TakeYaw(&yaw, &yaw_ms)) {
        s_yaw_deg = yaw;
        s_yaw_ms = yaw_ms;
        Lap_FeedYaw(yaw, yaw_ms);
    }
    Lap_Tick(now_ms);
}

void Attitude_StartLap(uint32_t now_ms)
{
    Lap_Start(now_ms);
}

void Attitude_StopLap(void)
{
    Lap_Stop();
}

void Attitude_GetStatus(AttitudeStatus *out)
{
    if (out == NULL) {
        return;
    }

    out->ready = s_ready;
    out->yaw_deg = s_yaw_deg;
    out->yaw_ms = s_yaw_ms;
    MpuLink_GetStats(&out->link);
    out->live = out->link.live;
    Lap_GetStatus(&out->lap);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_uart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART2) {
        MpuLink_RxCpltCallback();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART2) {
        MpuLink_ErrorCallback();
    }
}
