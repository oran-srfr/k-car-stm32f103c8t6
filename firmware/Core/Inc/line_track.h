#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdint.h>

/* Set to 0 when the tracking module drives a high level over a black line. */
#ifndef LINE_SENSOR_BLACK_ACTIVE_LOW
#define LINE_SENSOR_BLACK_ACTIVE_LOW 1U
#endif

/* Confirmed SWn->Xn; probes left-to-right SW2, SW1, SW3, SW4.
 * Raw bit numbers X1=0, X2=1, X3=2, X4=3. Change only after measuring
 * the physical order with the car facing forward. Never invert PD to hide it. */
#ifndef LINE_SENSOR_LEFT_OUTER_BIT
#define LINE_SENSOR_LEFT_OUTER_BIT  1U
#define LINE_SENSOR_LEFT_INNER_BIT  0U
#define LINE_SENSOR_RIGHT_INNER_BIT 2U
#define LINE_SENSOR_RIGHT_OUTER_BIT 3U
#endif
#if (LINE_SENSOR_LEFT_OUTER_BIT > 3U) || (LINE_SENSOR_LEFT_INNER_BIT > 3U) || \
    (LINE_SENSOR_RIGHT_INNER_BIT > 3U) || (LINE_SENSOR_RIGHT_OUTER_BIT > 3U) || \
    (((1U << LINE_SENSOR_LEFT_OUTER_BIT) | (1U << LINE_SENSOR_LEFT_INNER_BIT) | \
      (1U << LINE_SENSOR_RIGHT_INNER_BIT) | (1U << LINE_SENSOR_RIGHT_OUTER_BIT)) != 0x0FU)
#error "Line sensor mapping must be a permutation of raw bits 0..3"
#endif

typedef enum
{
  LINE_TRACK_STATUS_NORMAL = 0U,
  LINE_TRACK_STATUS_LOST,
  LINE_TRACK_STATUS_ALL_BLACK
} LineTrackStatus_t;

typedef struct
{
  uint8_t black_bits;
  LineTrackStatus_t status;
  float deviation;
} LineTrackResult_t;

/** 将 X1..X4 原始电平转换为左到右黑线位图并计算偏差；不访问 GPIO。 */
LineTrackResult_t LineTrack_Calculate(uint8_t sensor_raw_bits);

#endif /* LINE_TRACK_H */
