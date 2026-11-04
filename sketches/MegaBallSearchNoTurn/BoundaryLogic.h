#ifndef SOCCER_BOUNDARY_LOGIC_H
#define SOCCER_BOUNDARY_LOGIC_H
#include <stdint.h>
#ifdef BOUNDARY_CONSTEXPR_TEST
#define LINE_FN constexpr
#else
#define LINE_FN inline
#endif
namespace line {
enum Side : uint8_t {
  FRONT = 1, RIGHT = 2, REAR = 4, LEFT = 8,
  FRONT_RIGHT = FRONT | RIGHT, REAR_RIGHT = REAR | RIGHT,
  REAR_LEFT = REAR | LEFT, FRONT_LEFT = FRONT | LEFT
};
struct Threshold { uint16_t on, off; bool valid; };
LINE_FN Threshold thresholds(float floor, float white, float floorNoise, float whiteNoise) {
  float gap = white - floor;
  float noiseMargin = 6 * (floorNoise + whiteNoise);
  if (!(floor >= 0) || !(white <= 65535) || !(floorNoise >= 0) || !(whiteNoise >= 0) ||
      !(gap >= 8) || !(gap >= noiseMargin) || !(white >= floor * 1.15f))
    return {0, 0, false};
  uint16_t on = uint16_t(floor + gap * 0.45f + 0.5f);
  uint16_t off = uint16_t(floor + gap * 0.25f + 0.5f);
  return {on, off, on > off};
}
LINE_FN bool whiteNow(bool previous, uint16_t clear, Threshold threshold) {
  return threshold.valid && (previous ? clear > threshold.off : clear >= threshold.on);
}
LINE_FN bool validSide(uint8_t side) {
  return side == FRONT || side == RIGHT || side == REAR || side == LEFT ||
         side == FRONT_RIGHT || side == REAR_RIGHT || side == REAR_LEFT || side == FRONT_LEFT;
}
LINE_FN bool validLayout(const uint8_t *channels, const uint8_t *sides, uint8_t count) {
  if (!count || count > 8) return false;
  for (uint8_t i = 0; i < count; ++i) {
    if (channels[i] > 7 || !validSide(sides[i])) return false;
    for (uint8_t j = 0; j < i; ++j) if (channels[i] == channels[j]) return false;
  }
  return true;
}
LINE_FN uint8_t sideMask(uint8_t hits, const uint8_t *sides, uint8_t count) {
  uint8_t result = 0;
  for (uint8_t i = 0; i < count; ++i) if (hits & (1U << i)) result |= sides[i];
  return result;
}
LINE_FN bool mayMove(bool enabled, bool calibrated, bool fresh, bool gyro,
                     bool capturing, bool saving) {
  return enabled && calibrated && fresh && gyro && !capturing && !saving;
}
LINE_FN uint16_t crc16(const uint8_t *data, uint8_t count) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < count; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
struct EscapeResult { int8_t forward, strafe; bool ownsDrive, fault, cleared; };
class Escape {
 public:
  LINE_FN Escape() : active_(false), failed_(false), clearing_(false),
      started_(0), clearSince_(0), seen_(0) {}
  LINE_FN void reset() { active_ = failed_ = clearing_ = false; seen_ = 0; }
  LINE_FN bool failed() const { return failed_; }
  LINE_FN EscapeResult update(uint8_t sides, uint32_t now, uint16_t timeoutMs, uint16_t clearMs) {
    if (failed_) return {0, 0, true, true, false};
    if (!active_) {
      if (!sides) return {0, 0, false, false, false};
      active_ = true; clearing_ = false; started_ = now; seen_ = 0;
    }
    seen_ |= sides;
    if (((seen_ & FRONT) && (seen_ & REAR)) || ((seen_ & LEFT) && (seen_ & RIGHT)) ||
        uint32_t(now - started_) >= timeoutMs) {
      failed_ = true;
      return {0, 0, true, true, false};
    }
    if (!sides) {
      if (!clearing_) { clearing_ = true; clearSince_ = now; }
      if (uint32_t(now - clearSince_) >= clearMs) {
        active_ = clearing_ = false;
        return {0, 0, true, false, true};
      }
    } else clearing_ = false;
    int8_t forward = (seen_ & REAR) ? 1 : ((seen_ & FRONT) ? -1 : 0);
    int8_t strafe = (seen_ & LEFT) ? 1 : ((seen_ & RIGHT) ? -1 : 0);
    return {forward, strafe, true, false, false};
  }
 private:
  bool active_, failed_, clearing_;
  uint32_t started_, clearSince_;
  uint8_t seen_;
};
}
#undef LINE_FN
#endif
