#ifndef K_CAR_BOARD_PINS_H
#define K_CAR_BOARD_PINS_H

#include "stm32f1xx_hal.h"

/* Confirmed wiring only; this header does not configure or drive any pin. */
#define MOTOR_A_PORT GPIOB
#define MOTOR_AIN1_PIN GPIO_PIN_6
#define MOTOR_AIN2_PIN GPIO_PIN_5
#define MOTOR_B_PORT GPIOB
#define MOTOR_BIN1_PIN GPIO_PIN_7
#define MOTOR_BIN2_PIN GPIO_PIN_8
#define MOTOR_PWM_PORT GPIOA
#define MOTOR_LEFT_PWM_PIN GPIO_PIN_6 /* TIM3_CH1, PWMB */
#define MOTOR_RIGHT_PWM_PIN GPIO_PIN_7 /* TIM3_CH2, PWMA */
#define SONAR_PORT GPIOA
#define SONAR_TRIG_PIN GPIO_PIN_4
#define SONAR_ECHO_PIN GPIO_PIN_5
#define BUZZER_PORT GPIOA
#define BUZZER_PIN GPIO_PIN_0 /* Passive buzzer, TIM2_CH1 via transistor */
#define TRACK_PORT GPIOB
#define TRACK_1_PIN GPIO_PIN_0
#define TRACK_2_PIN GPIO_PIN_1
#define TRACK_3_PIN GPIO_PIN_10
#define TRACK_4_PIN GPIO_PIN_11
/* Raw order X1..X4 = PB0, PB1, PB10, PB11; default black level is low.
 * Physical probes left-to-right: SW2, SW1, SW3, SW4.
 * User-confirmed electrical mapping: SW1->X1, SW2->X2, SW3->X3, SW4->X4.
 * Connector labels left-to-right: X4, X3, X2, X1 (not the probe order). */
#define TFT_PORT GPIOB
#define TFT_CLK_PIN GPIO_PIN_14
#define TFT_MOSI_PIN GPIO_PIN_15
#define TFT_MISO_PIN GPIO_PIN_13 /* Font ROM; reserved even before jumper is fitted */
#define TFT_CONTROL_PORT GPIOA
#define TFT_CS_PIN GPIO_PIN_1
#define TFT_DC_PIN GPIO_PIN_12
#define ATTITUDE_PORT GPIOA
#define ATTITUDE_TX_PIN GPIO_PIN_2
#define ATTITUDE_RX_PIN GPIO_PIN_3
#define ATTITUDE_BAUD_RATE 115200U
/* PA9/PA10: USART1 K230; PA2/PA3: USART2 attitude module.
 * PA8/PB12: encoder A; PA13/PA14: SWD, keep reserved.
 * TB6612 STBY is wired to 3.3 V, not an MCU pin.
 * HSI remains 8 MHz: derive timer prescalers from actual clock, not PSC=71.
 */

#endif
