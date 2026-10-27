#include "motor_driver.h"

#include "board_pins.h"

#define MOTOR_LEFT_IN1_PORT  MOTOR_B_PORT
#define MOTOR_LEFT_IN1_PIN   MOTOR_BIN1_PIN
#define MOTOR_LEFT_IN2_PIN   MOTOR_BIN2_PIN
#define MOTOR_RIGHT_IN1_PORT MOTOR_A_PORT
#define MOTOR_RIGHT_IN1_PIN  MOTOR_AIN1_PIN
#define MOTOR_RIGHT_IN2_PIN  MOTOR_AIN2_PIN

#define MOTOR_PWM_AUTORELOAD 3599U

volatile uint8_t g_motor_driver_initialized = 0U;
volatile uint8_t g_motor_driver_enabled = 0U;
volatile int16_t g_motor_driver_left_request = 0;
volatile int16_t g_motor_driver_right_request = 0;
volatile int16_t g_motor_driver_left_actual = 0;
volatile int16_t g_motor_driver_right_actual = 0;
volatile uint32_t g_motor_driver_left_pwm_compare = 0U;
volatile uint32_t g_motor_driver_right_pwm_compare = 0U;
volatile int8_t g_motor_driver_left_direction = 0;
volatile int8_t g_motor_driver_right_direction = 0;

static int16_t MotorDriver_LimitCommand(int16_t command)
{
  if (command > MOTOR_DRIVER_COMMAND_MAX) {
    return MOTOR_DRIVER_COMMAND_MAX;
  }
  if (command < -MOTOR_DRIVER_COMMAND_MAX) {
    return -MOTOR_DRIVER_COMMAND_MAX;
  }
  return command;
}

static uint32_t MotorDriver_CommandToCompare(int16_t command)
{
  uint32_t magnitude;

  magnitude = (command < 0) ? (uint32_t)(-(int32_t)command)
                            : (uint32_t)command;
  return (magnitude * (uint32_t)TIM3->ARR) / MOTOR_DRIVER_COMMAND_MAX;
}

static void MotorDriver_SetLeftDirection(int16_t command)
{
  uint8_t forward = (command > 0) ? 1U : 0U;

  if (command == 0) {
    HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_RESET);
    g_motor_driver_left_direction = 0;
    return;
  }

  if (MOTOR_DRIVER_LEFT_REVERSED != 0U) {
    forward = (forward == 0U) ? 1U : 0U;
  }
  HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN,
                    (forward != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN2_PIN,
                    (forward != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  g_motor_driver_left_direction = (command > 0) ? 1 : -1;
}

static void MotorDriver_SetRightDirection(int16_t command)
{
  uint8_t forward = (command > 0) ? 1U : 0U;

  if (command == 0) {
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
    g_motor_driver_right_direction = 0;
    return;
  }

  if (MOTOR_DRIVER_RIGHT_REVERSED != 0U) {
    forward = (forward == 0U) ? 1U : 0U;
  }
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN,
                    (forward != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN2_PIN,
                    (forward != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  g_motor_driver_right_direction = (command > 0) ? 1 : -1;
}

static void MotorDriver_ForceSafePins(void)
{
  TIM3->CCR1 = 0U;
  TIM3->CCR2 = 0U;
  HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);

  g_motor_driver_left_actual = 0;
  g_motor_driver_right_actual = 0;
  g_motor_driver_left_pwm_compare = 0U;
  g_motor_driver_right_pwm_compare = 0U;
  g_motor_driver_left_direction = 0;
  g_motor_driver_right_direction = 0;
}

void MotorDriver_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  g_motor_driver_initialized = 0U;
  g_motor_driver_enabled = 0U;
  g_motor_driver_left_request = 0;
  g_motor_driver_right_request = 0;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();

  /* This project uses the default TIM3 CH1/CH2 routing on PA6/PA7. */
  if ((AFIO->MAPR & AFIO_MAPR_TIM3_REMAP) != 0U) {
    return;
  }

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN;
  HAL_GPIO_Init(MOTOR_LEFT_IN1_PORT, &gpio);
  gpio.Pin = MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN;
  HAL_GPIO_Init(MOTOR_RIGHT_IN1_PORT, &gpio);

  gpio.Pin = MOTOR_LEFT_PWM_PIN | MOTOR_RIGHT_PWM_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOTOR_PWM_PORT, &gpio);

  TIM3->PSC = 0U;
  TIM3->ARR = MOTOR_PWM_AUTORELOAD;
  TIM3->CCR1 = 0U;
  TIM3->CCR2 = 0U;
  TIM3->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM3->CR1 = TIM_CR1_ARPE;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0U;
  TIM3->CR1 |= TIM_CR1_CEN;

  MotorDriver_ForceSafePins();
  g_motor_driver_initialized = 1U;
}

void MotorDriver_Enable(void)
{
  if (g_motor_driver_initialized == 0U) {
    return;
  }
  MotorDriver_Stop();
  g_motor_driver_enabled = 1U;
}

void MotorDriver_Disable(void)
{
  g_motor_driver_enabled = 0U;
  if (g_motor_driver_initialized != 0U) {
    MotorDriver_ForceSafePins();
  }
}

void MotorDriver_SetLeft(int16_t command)
{
  uint32_t compare;

  command = MotorDriver_LimitCommand(command);
  g_motor_driver_left_request = command;
  TIM3->CCR1 = 0U;
  g_motor_driver_left_pwm_compare = 0U;

  if ((g_motor_driver_initialized == 0U) ||
      (g_motor_driver_enabled == 0U)) {
    MotorDriver_SetLeftDirection(0);
    g_motor_driver_left_actual = 0;
    return;
  }

  MotorDriver_SetLeftDirection(command);
  compare = MotorDriver_CommandToCompare(command);
  TIM3->CCR1 = compare;
  g_motor_driver_left_pwm_compare = compare;
  g_motor_driver_left_actual = command;
}

void MotorDriver_SetRight(int16_t command)
{
  uint32_t compare;

  command = MotorDriver_LimitCommand(command);
  g_motor_driver_right_request = command;
  TIM3->CCR2 = 0U;
  g_motor_driver_right_pwm_compare = 0U;

  if ((g_motor_driver_initialized == 0U) ||
      (g_motor_driver_enabled == 0U)) {
    MotorDriver_SetRightDirection(0);
    g_motor_driver_right_actual = 0;
    return;
  }

  MotorDriver_SetRightDirection(command);
  compare = MotorDriver_CommandToCompare(command);
  TIM3->CCR2 = compare;
  g_motor_driver_right_pwm_compare = compare;
  g_motor_driver_right_actual = command;
}

void MotorDriver_SetBoth(int16_t left, int16_t right)
{
  MotorDriver_SetLeft(left);
  MotorDriver_SetRight(right);
}

void MotorDriver_Stop(void)
{
  g_motor_driver_left_request = 0;
  g_motor_driver_right_request = 0;

  if (g_motor_driver_initialized != 0U) {
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
  }
  MotorDriver_SetLeftDirection(0);
  MotorDriver_SetRightDirection(0);

  g_motor_driver_left_actual = 0;
  g_motor_driver_right_actual = 0;
  g_motor_driver_left_pwm_compare = 0U;
  g_motor_driver_right_pwm_compare = 0U;
}
