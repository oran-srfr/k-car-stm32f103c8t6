#ifndef K_CAR_LEGACY_LINE_WALKING_H
#define K_CAR_LEGACY_LINE_WALKING_H

#include <stdint.h>

/*
 * Four-sensor adaptation of the decision table in the imported Keil project.
 * The source uses seven sensors; this board exposes left/right inner and
 * outer sensors, so adjacent three-sensor cases collapse to the outer case.
 */
typedef enum
{
  LEGACY_LINE_STRAIGHT = 0U,
  LEGACY_LINE_LEFT_INNER,
  LEGACY_LINE_LEFT_OUTER,
  LEGACY_LINE_RIGHT_INNER,
  LEGACY_LINE_RIGHT_OUTER,
  LEGACY_LINE_BOTH_OUTER,
  LEGACY_LINE_ALL_OFF
} LegacyLineWalkingPattern_t;

/**
 * Classify the project's physical left-to-right black bitmap:
 * bit0=left outer, bit1=left inner, bit2=right inner, bit3=right outer.
 */
LegacyLineWalkingPattern_t LegacyLineWalking_Classify(uint8_t black_bits);

#endif /* K_CAR_LEGACY_LINE_WALKING_H */
