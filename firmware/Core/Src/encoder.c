#include "encoder.h"

#include <math.h>

#include "control_config.h"
#include "stm32f1xx_hal.h"

/* 编码器 A 相引脚：左 PA8、右 PB12（与引脚占用表一致） */
#define ENC_LEFT_PORT    GPIOA
#define ENC_LEFT_PIN     GPIO_PIN_8
#define ENC_RIGHT_PORT   GPIOB
#define ENC_RIGHT_PIN    GPIO_PIN_12

#ifndef ENCODER_WINDOW_MS
#define ENCODER_WINDOW_MS   (100U)
#endif

/* volatile：在中断里自增，主循环里读取 */
static volatile uint32_t s_left_total = 0U;
static volatile uint32_t s_right_total = 0U;

static uint32_t s_left_snapshot = 0U;
static uint32_t s_right_snapshot = 0U;
static uint16_t s_left_delta = 0U;
static uint16_t s_right_delta = 0U;
static uint32_t s_window_start_ms = 0U;

/* ---- 闭环配平状态 ---- */
static float s_cmd_left = 0.0f;    /* VehicleControl 每周期喂进来的软件命令 */
static float s_cmd_right = 0.0f;
#if ENCODER_BALANCE_ENABLE
static float s_balance = 0.0f;     /* 积分得到的补偿量（未乘符号） */
#endif

/* 调试器可直接观察这四个变量，不必看屏幕 */
volatile uint32_t g_enc_left_total = 0U;
volatile uint32_t g_enc_right_total = 0U;
volatile uint16_t g_enc_left_delta = 0U;
volatile uint16_t g_enc_right_delta = 0U;

void Encoder_OnLeftEdge(void)
{
    ++s_left_total;
}

void Encoder_OnRightEdge(void)
{
    ++s_right_total;
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* 上拉 + 上升沿：模块输出为开集/推挽均可，上拉保证悬空时不乱计数 */
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = ENC_LEFT_PIN;
    HAL_GPIO_Init(ENC_LEFT_PORT, &gpio);
    gpio.Pin = ENC_RIGHT_PIN;
    HAL_GPIO_Init(ENC_RIGHT_PORT, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(ENC_LEFT_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(ENC_RIGHT_PIN);

    /* EXTI9_5 与超声波共用：编码器优先级取 0（超声波把它设成 1，
     * 引脚占用表里也注明"保留 encoder priority 0"）。
     * 计一个数只要几条指令，不会拖慢回波边沿的采样。 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0U, 0U);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    s_left_total = 0U;
    s_right_total = 0U;
    s_left_snapshot = 0U;
    s_right_snapshot = 0U;
    s_left_delta = 0U;
    s_right_delta = 0U;
    s_window_start_ms = HAL_GetTick();
    g_enc_left_total = 0U;
    g_enc_right_total = 0U;
    g_enc_left_delta = 0U;
    g_enc_right_delta = 0U;
}

void Encoder_Poll(uint32_t now_ms)
{
    uint32_t left;
    uint32_t right;
    uint32_t left_delta;
    uint32_t right_delta;

    if ((uint32_t)(now_ms - s_window_start_ms) < ENCODER_WINDOW_MS)
    {
        return;
    }
    s_window_start_ms = now_ms;

    left = s_left_total;
    right = s_right_total;
    left_delta = left - s_left_snapshot;
    right_delta = right - s_right_snapshot;
    s_left_snapshot = left;
    s_right_snapshot = right;

    /* 窗口内不可能超过 65535 个脉冲（100 ms @ 655 kHz），截断只是防御 */
    s_left_delta = (left_delta > 0xFFFFU) ? 0xFFFFU : (uint16_t)left_delta;
    s_right_delta = (right_delta > 0xFFFFU) ? 0xFFFFU : (uint16_t)right_delta;

    g_enc_left_total = left;
    g_enc_right_total = right;
    g_enc_left_delta = s_left_delta;
    g_enc_right_delta = s_right_delta;

#if ENCODER_BALANCE_ENABLE
    /* 只在"左右命令相等"的直线段做配平：转弯时两轮本来就该有差速，
     * 强行拉平会破坏循迹。速度太低（起步/原地）时也不做，脉冲太少没有统计意义。 */
    {
        bool straight = (fabsf(s_cmd_left - s_cmd_right) < 1.0f) &&
                        (s_cmd_left > 100.0f) && (s_cmd_right > 100.0f);

        if (straight)
        {
            float err = (float)s_left_delta - (float)s_right_delta;

            s_balance += ENCODER_BALANCE_KI * err;
            if (s_balance > ENCODER_BALANCE_LIMIT)
            {
                s_balance = ENCODER_BALANCE_LIMIT;
            }
            if (s_balance < -ENCODER_BALANCE_LIMIT)
            {
                s_balance = -ENCODER_BALANCE_LIMIT;
            }
        }
        else
        {
            /* 转弯中缓慢泄放，避免出弯后残留把车带偏 */
            s_balance *= ENCODER_BALANCE_DECAY;
        }
    }
#endif
}

void Encoder_GetStats(EncoderStats_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->left_total = g_enc_left_total;
    out->right_total = g_enc_right_total;
    out->left_delta = g_enc_left_delta;
    out->right_delta = g_enc_right_delta;
}

uint16_t Encoder_GetLeftDelta(void)
{
    return g_enc_left_delta;
}

uint16_t Encoder_GetRightDelta(void)
{
    return g_enc_right_delta;
}

void Encoder_SetCommandContext(float left_cmd, float right_cmd)
{
    s_cmd_left = left_cmd;
    s_cmd_right = right_cmd;
}

float Encoder_GetBalance(void)
{
#if ENCODER_BALANCE_ENABLE
    return s_balance * ENCODER_BALANCE_SIGN;
#else
    return 0.0f;
#endif
}

/* PB12 = EXTI12，属于 EXTI15_10 向量，本文件独占。 */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(ENC_RIGHT_PIN) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENC_RIGHT_PIN);
        Encoder_OnRightEdge();
    }
}
