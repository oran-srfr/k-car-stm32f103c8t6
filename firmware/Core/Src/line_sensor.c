#include "line_sensor.h"
#include "board_pins.h"

/* X1..X4 follow the project's confirmed PB0, PB1, PB10, PB11 wiring. */
#define LINE_SENSOR_GPIO_PORT TRACK_PORT
#define LINE_SENSOR_X1_PIN    TRACK_1_PIN
#define LINE_SENSOR_X2_PIN    TRACK_2_PIN
#define LINE_SENSOR_X3_PIN    TRACK_3_PIN
#define LINE_SENSOR_X4_PIN    TRACK_4_PIN

void LineSensor_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = LINE_SENSOR_X1_PIN | LINE_SENSOR_X2_PIN |
             LINE_SENSOR_X3_PIN | LINE_SENSOR_X4_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LINE_SENSOR_GPIO_PORT, &gpio);
}

/** 只采集四个 GPIO 的高低电平，返回 X1..X4 原始读数。 */
LineSensorRaw_t LineSensor_ReadRaw(void)
{
  LineSensorRaw_t raw;

  uint32_t pins = LINE_SENSOR_GPIO_PORT->IDR;
  raw.x1 = ((pins & LINE_SENSOR_X1_PIN) != 0U) ? 1U : 0U;
  raw.x2 = ((pins & LINE_SENSOR_X2_PIN) != 0U) ? 1U : 0U;
  raw.x3 = ((pins & LINE_SENSOR_X3_PIN) != 0U) ? 1U : 0U;
  raw.x4 = ((pins & LINE_SENSOR_X4_PIN) != 0U) ? 1U : 0U;

  return raw;
}
