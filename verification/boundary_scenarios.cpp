#define BOUNDARY_CONSTEXPR_TEST
#include "../sketches/MegaBallSearchNoTurn/BoundaryLogic.h"

constexpr uint8_t channels[] = {0, 1, 2, 3};
constexpr uint8_t sides[] = {line::FRONT, line::RIGHT, line::REAR, line::LEFT};
constexpr uint8_t duplicateChannels[] = {0, 1, 1, 3};
constexpr uint8_t invalidChannels[] = {0, 1, 2, 8};
constexpr uint8_t cornerSides[] = {line::FRONT_LEFT, line::FRONT_RIGHT, line::REAR_RIGHT, line::REAR_LEFT};
constexpr uint8_t invalidSides[] = {line::FRONT, 0, line::REAR, line::LEFT};
static_assert(line::validLayout(channels, sides, 4), "Four unique channels and cardinal positions are valid");
static_assert(line::validLayout(channels, cornerSides, 4), "Corner mounting is supported");
static_assert(!line::validLayout(duplicateChannels, sides, 4), "Duplicate I2C channels must not pass");
static_assert(!line::validLayout(invalidChannels, sides, 4), "TCA channel 8 does not exist");
static_assert(!line::validLayout(channels, invalidSides, 4), "Unset position must not pass");
static_assert(line::sideMask(1, sides, 4) == line::FRONT, "Sensor 1 maps to configured front");
static_assert(line::sideMask(2, sides, 4) == line::RIGHT, "Sensor 2 maps to configured right");
static_assert(line::sideMask(3, cornerSides, 4) == (line::FRONT | line::LEFT | line::RIGHT),
              "Corner masks preserve both physical edges");

constexpr auto threshold = line::thresholds(100, 1000, 1, 1);
static_assert(threshold.valid && threshold.on == 505 && threshold.off == 325,
              "Measured field/white contrast gives separate trip and release thresholds");
static_assert(!line::thresholds(100, 100, 0, 0).valid, "Identical field and white must block driving");
static_assert(!line::thresholds(200, 100, 0, 0).valid, "Inverted contrast must not be mistaken for white");
static_assert(!line::thresholds(100, 110, 0, 0).valid, "Small relative contrast must be rejected");
static_assert(!line::thresholds(0, 7, 0, 0).valid, "Tiny absolute contrast must be rejected");
static_assert(!line::thresholds(100, 200, 10, 10).valid, "Contrast buried in noise must be rejected");
static_assert(!line::whiteNow(false, 504, threshold), "Field below trip does not assert white");
static_assert(line::whiteNow(false, 505, threshold), "White asserts at trip threshold");
static_assert(line::whiteNow(true, 400, threshold), "Hysteresis retains white between thresholds");
static_assert(!line::whiteNow(true, 325, threshold), "White clears at the lower release threshold");
static_assert(!line::whiteNow(true, 65535, {0,0,false}), "Uncalibrated values are never treated as a valid white classification");

static_assert(line::mayMove(true,true,true,true,false,false), "Ready system may move");
static_assert(!line::mayMove(false,true,true,true,false,false), "Manual stop overrides following");
static_assert(!line::mayMove(true,false,true,true,false,false), "Missing calibration blocks movement");
static_assert(!line::mayMove(true,true,false,true,false,false), "Stale line data blocks movement");
static_assert(!line::mayMove(true,true,true,false,false,false), "Gyro fault blocks movement");
static_assert(!line::mayMove(true,true,true,true,true,false), "Sampling calibration blocks movement");
static_assert(!line::mayMove(true,true,true,true,false,true), "Saving calibration blocks movement");

constexpr bool everyEdgeMovesAway() {
  for (uint8_t mask = 0; mask < 16; ++mask) {
    line::Escape escape;
    auto result = escape.update(mask, 100, 650, 120);
    if (!mask) { if (result.ownsDrive) return false; continue; }
    if (!result.ownsDrive) return false;
    bool opposed = ((mask & line::FRONT) && (mask & line::REAR)) ||
                   ((mask & line::LEFT) && (mask & line::RIGHT));
    if (opposed) {
      if (!result.paused || result.forward || result.strafe) return false;
    } else {
      if (result.paused) return false;
      if ((mask & line::FRONT) && result.forward >= 0) return false;
      if ((mask & line::REAR) && result.forward <= 0) return false;
      if ((mask & line::RIGHT) && result.strafe >= 0) return false;
      if ((mask & line::LEFT) && result.strafe <= 0) return false;
    }
  }
  return true;
}
static_assert(everyEdgeMovesAway(), "All 16 side combinations must retreat away or stop on ambiguity");

constexpr bool clearRequiresTime() {
  line::Escape escape;
  escape.update(line::FRONT, 0, 650, 120);
  if (escape.update(0, 100, 650, 120).cleared) return false;
  if (escape.update(0, 219, 650, 120).cleared) return false;
  auto clear = escape.update(0, 220, 650, 120);
  return clear.cleared && clear.ownsDrive && !clear.forward && !clear.strafe &&
         !escape.update(0, 221, 650, 120).ownsDrive;
}
static_assert(clearRequiresTime(), "Escape must own drive until stable clear then release control to play");

constexpr bool reappearingLineRestartsClear() {
  line::Escape escape;
  escape.update(line::FRONT, 0, 650, 120);
  escape.update(0, 100, 650, 120);
  escape.update(line::FRONT, 200, 650, 120);
  escape.update(0, 300, 650, 120);
  return !escape.update(0, 419, 650, 120).cleared && escape.update(0, 420, 650, 120).cleared;
}
static_assert(reappearingLineRestartsClear(), "A new white reading restarts the clear interval");

constexpr bool timeoutRetries() {
  line::Escape escape;
  escape.update(line::RIGHT, 1000, 650, 120);
  if (escape.update(line::RIGHT, 1649, 650, 120).paused) return false;
  auto result = escape.update(line::RIGHT, 1650, 650, 120);
  if (!result.paused || result.forward || result.strafe) return false;
  if (!escape.update(line::RIGHT, 1769, 650, 120).paused) return false;
  auto retry = escape.update(line::RIGHT, 1770, 650, 120);
  return !retry.paused && retry.strafe == -1 && retry.ownsDrive;
}
static_assert(timeoutRetries(), "Long escape brakes briefly then retries automatically from fresh readings");

constexpr bool evolvingCorner() {
  line::Escape escape;
  escape.update(line::FRONT, 0, 650, 120);
  auto corner = escape.update(line::FRONT | line::RIGHT, 20, 650, 120);
  if (corner.forward != -1 || corner.strafe != -1) return false;
  auto opposed = escape.update(line::FRONT | line::REAR, 40, 650, 120);
  if (!opposed.paused || opposed.forward || opposed.strafe) return false;
  auto recovered = escape.update(line::REAR, 60, 650, 120);
  return !recovered.paused && recovered.forward == 1 && recovered.strafe == 0;
}
static_assert(evolvingCorner(), "Current adjacent hits combine; ambiguity holds then recovers without manual reset");

constexpr bool rollover() {
  line::Escape escape;
  escape.update(line::FRONT, 0xFFFFFFF0UL, 650, 120);
  if (escape.update(line::FRONT, 0x10, 650, 120).paused) return false;
  escape.update(0, 0x20, 650, 120);
  return escape.update(0, 0x98, 650, 120).cleared;
}
static_assert(rollover(), "Retreat and clear timing survive millis rollover");
constexpr uint8_t crcText[] = {'1','2','3','4','5','6','7','8','9'};
static_assert(line::crc16(crcText, 9) == 0x29B1, "EEPROM CRC matches the standard CRC-16/CCITT-FALSE check vector");
