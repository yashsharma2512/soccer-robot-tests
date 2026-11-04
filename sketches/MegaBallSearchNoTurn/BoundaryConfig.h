#ifndef SOCCER_BOUNDARY_CONFIG_H
#define SOCCER_BOUNDARY_CONFIG_H
#include "BoundaryLogic.h"
namespace boundarycfg {
static const uint8_t TCA_ADDRESS = 0x70;
static const uint8_t SENSOR_COUNT = 4;
static const uint8_t CHANNELS[SENSOR_COUNT] = {0, 1, 2, 3};

// EXAMPLE POSITIONS ONLY: edit these to match your actual mounting.
// Available: line::FRONT, RIGHT, REAR, LEFT, FRONT_LEFT, FRONT_RIGHT,
// REAR_LEFT, REAR_RIGHT. Order corresponds to CHANNELS above.
static const uint8_t SIDES[SENSOR_COUNT] = {
  line::FRONT, line::RIGHT, line::REAR, line::LEFT
};

static const uint8_t ATIME = 250;       // About 16.7 ms, faster than the bench test.
static const uint8_t GAIN = 1;          // 4x. Recalibrate after changing gain/time.
static const uint16_t SLOT_MS = 5;      // One channel per slot; no integration delay().
static const uint16_t MAX_AGE_MS = 100; // Missing/stale readings block movement.
static const uint16_t CAL_SAMPLES = 32;
static const uint16_t CAL_TIMEOUT_MS = 8000;
static const uint16_t CLEAR_MS = 120;
static const uint16_t ESCAPE_TIMEOUT_MS = 650;
static const float ESCAPE_PWM = 70;
static const int16_t ESCAPE_WHEEL_CAP = 100;
static const int16_t FOLLOW_WHEEL_CAP = 90;
static const int EEPROM_BASE = 0;       // Bytes 0..29 reserved for line calibration.
}
#endif
