#ifndef MEGA_BALL_SEARCH_PATTERN_H
#define MEGA_BALL_SEARCH_PATTERN_H
#include <stdint.h>
#include "DriveLogic.h"

namespace search {
// A bounded, forward-moving raster around the last known ball side.
// Sideways legs balance out; there is no blind reverse or commanded turn.
static const uint16_t SIDE_MS = 350;
static const uint16_t FORWARD_MS = 250;
static const uint16_t TOTAL_MS = 6 * SIDE_MS + 3 * FORWARD_MS;

struct Step { drive::Motion motion; uint8_t leg; bool finished; };

inline Step at(uint32_t elapsed, int8_t side, float pwm) {
  float firstSide = side < 0 ? -pwm : pwm;
  if (elapsed < SIDE_MS) return {{0, firstSide, 0}, 0, false};
  elapsed -= SIDE_MS;
  if (elapsed < FORWARD_MS) return {{pwm, 0, 0}, 1, false};
  elapsed -= FORWARD_MS;
  if (elapsed < 2UL * SIDE_MS) return {{0, -firstSide, 0}, 2, false};
  elapsed -= 2UL * SIDE_MS;
  if (elapsed < FORWARD_MS) return {{pwm, 0, 0}, 3, false};
  elapsed -= FORWARD_MS;
  if (elapsed < 2UL * SIDE_MS) return {{0, firstSide, 0}, 4, false};
  elapsed -= 2UL * SIDE_MS;
  if (elapsed < FORWARD_MS) return {{pwm, 0, 0}, 5, false};
  elapsed -= FORWARD_MS;
  if (elapsed < SIDE_MS) return {{0, -firstSide, 0}, 6, false};
  return {{0, 0, 0}, 7, true};
}
}
#endif
