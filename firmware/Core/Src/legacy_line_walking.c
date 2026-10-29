#include "legacy_line_walking.h"

#define LEFT_OUTER_BIT   (1U << 0U)
#define LEFT_INNER_BIT   (1U << 1U)
#define RIGHT_INNER_BIT  (1U << 2U)
#define RIGHT_OUTER_BIT  (1U << 3U)

LegacyLineWalkingPattern_t LegacyLineWalking_Classify(uint8_t black_bits)
{
  uint8_t left_outer = (black_bits & LEFT_OUTER_BIT) != 0U;
  uint8_t left_inner = (black_bits & LEFT_INNER_BIT) != 0U;
  uint8_t right_inner = (black_bits & RIGHT_INNER_BIT) != 0U;
  uint8_t right_outer = (black_bits & RIGHT_OUTER_BIT) != 0U;

  /* Preserve the imported controller's outermost-sensor priority. */
  if (left_outer && right_outer) {
    return LEGACY_LINE_BOTH_OUTER;
  }
  if (left_outer) {
    return LEGACY_LINE_LEFT_OUTER;
  }
  if (right_outer) {
    return LEGACY_LINE_RIGHT_OUTER;
  }
  if (left_inner && right_inner) {
    return LEGACY_LINE_STRAIGHT;
  }
  if (left_inner) {
    return LEGACY_LINE_LEFT_INNER;
  }
  if (right_inner) {
    return LEGACY_LINE_RIGHT_INNER;
  }
  return LEGACY_LINE_ALL_OFF;
}
