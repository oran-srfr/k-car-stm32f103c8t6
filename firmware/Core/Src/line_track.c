#include "line_track.h"

/* 原始位序为 X1..X4；黑线低电平，空间顺序为 默认 X2、X1、X3、X4。 */
#define BLACK_BITS_ALL   0x0FU

/** 纯数据计算：重排并反相原始电平，区分丢线、正常和全黑状态。 */
LineTrackResult_t LineTrack_Calculate(uint8_t sensor_raw_bits)
{
  static const int8_t weights[4] = {-3, -1, 1, 3};
  LineTrackResult_t result;
  int16_t weighted_sum = 0;
  uint8_t count = 0U;
  uint8_t bit;

  result.black_bits = 0U;
  /* 非正常状态使用确定值 0，是否有效只看 status。 */
  result.deviation = 0.0f;

  sensor_raw_bits &= 0x0FU;
#if LINE_SENSOR_BLACK_ACTIVE_LOW == 0U
  sensor_raw_bits ^= 0x0FU;
#endif

  /* bit0 左外；bit1 左内；bit2 右内；bit3 右外（映射见 line_track.h）。 */
  if ((sensor_raw_bits & (1U << LINE_SENSOR_LEFT_OUTER_BIT)) == 0U) result.black_bits |= (1U << 0U);
  if ((sensor_raw_bits & (1U << LINE_SENSOR_LEFT_INNER_BIT)) == 0U) result.black_bits |= (1U << 1U);
  if ((sensor_raw_bits & (1U << LINE_SENSOR_RIGHT_INNER_BIT)) == 0U) result.black_bits |= (1U << 2U);
  if ((sensor_raw_bits & (1U << LINE_SENSOR_RIGHT_OUTER_BIT)) == 0U) result.black_bits |= (1U << 3U);

  if (result.black_bits == 0U)
  {
    result.status = LINE_TRACK_STATUS_LOST;
    return result;
  }

  if (result.black_bits == BLACK_BITS_ALL)
  {
    result.status = LINE_TRACK_STATUS_ALL_BLACK;
    return result;
  }

  for (bit = 0U; bit < 4U; ++bit)
  {
    if ((result.black_bits & (1U << bit)) != 0U)
    {
      weighted_sum += weights[bit];
      ++count;
    }
  }

  result.status = LINE_TRACK_STATUS_NORMAL;
  result.deviation = (float)weighted_sum / (float)count;
  return result;
}
