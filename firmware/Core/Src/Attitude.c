#include "Attitude.h"

#include "board_pins.h"
#include "stm32f1xx_hal.h"

#include <stddef.h>

static UART_HandleTypeDef s_uart;
static bool s_ready;
static float s_yaw_deg;
static uint32_t s_yaw_ms;

/* 健康判断用的辅助状态：记录"上一次 yaw 明显变化"的时刻 */
static float s_yaw_ref;
static uint32_t s_yaw_ref_ms;
static bool s_yaw_moving;

/* 判断"最近多久没有新帧"时用的超时；与 MpuLink 的 live 超时保持同量级 */
#define ATTITUDE_GYRO_AGE_LIMIT_MS   (300U)
/* yaw 变化超过这个角度才算"在转"（噪声级别约 ±1 度） */
#define ATTITUDE_GYRO_MOVE_DEG       (2.0f)
/* 多久没有明显变化就把 moving 归零 */
#define ATTITUDE_GYRO_MOVE_HOLD_MS   (500U)

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
    s_yaw_ref = 0.0f;
    s_yaw_ref_ms = 0U;
    s_yaw_moving = false;

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

        /* 辅助状态：yaw 有明显的角度变化就更新"在动"标记 */
        {
            float delta = yaw - s_yaw_ref;

            while (delta > 180.0f) {
                delta -= 360.0f;
            }
            while (delta < -180.0f) {
                delta += 360.0f;
            }
            if ((delta >= ATTITUDE_GYRO_MOVE_DEG) ||
                (delta <= -ATTITUDE_GYRO_MOVE_DEG)) {
                s_yaw_ref = yaw;
                s_yaw_ref_ms = yaw_ms;
                s_yaw_moving = true;
            }
        }
    }

    if (s_yaw_moving &&
        ((uint32_t)(now_ms - s_yaw_ref_ms) > ATTITUDE_GYRO_MOVE_HOLD_MS)) {
        s_yaw_moving = false;
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

/* ===== 陀螺仪健康判断 =====
 *
 * 结论只有三种，判断依据非常直接：
 *   NO_DATA : frames_ok == 0             —— 从来没收到过（接线/波特率/供电）
 *   STALE   : 收到过，但 age > 300 ms     —— 中途掉了（丢帧/干扰/掉线）
 *   OK      : 正在持续出数据
 *
 * 注意：不要用 moving 判断故障。车停着不动时 yaw 不变化是正常的。
 */
void Attitude_GetGyroHealth(uint32_t now_ms, AttitudeGyroHealth *out)
{
    MpuLinkStats stats;
    uint32_t age;

    if (out == NULL) {
        return;
    }

    MpuLink_GetStats(&stats);

    out->frames_ok = stats.frames_ok;
    out->frames_bad = stats.frames_bad;
    out->yaw_deg = s_yaw_deg;
    out->moving = s_yaw_moving;

    if (stats.frames_ok == 0U) {
        out->code = ATTITUDE_GYRO_NO_DATA;
        out->age_ms = 999U;
        return;
    }

    age = (uint32_t)(now_ms - s_yaw_ms);
    out->age_ms = age;

    if ((stats.live == false) || (age > ATTITUDE_GYRO_AGE_LIMIT_MS)) {
        out->code = ATTITUDE_GYRO_STALE;
    } else {
        out->code = ATTITUDE_GYRO_OK;
    }
}

bool Attitude_IsGyroWorking(uint32_t now_ms)
{
    AttitudeGyroHealth health;

    Attitude_GetGyroHealth(now_ms, &health);
    return health.code == ATTITUDE_GYRO_OK;
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_uart);
}void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
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
