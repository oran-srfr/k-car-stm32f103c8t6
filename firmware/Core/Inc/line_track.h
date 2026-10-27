#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdint.h>

/* Set to 0 when the tracking module drives a high level over a black line. */
#define LINE_SENSOR_BLACK_ACTIVE_LOW 1U

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
