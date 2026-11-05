// Linux-named standalone Mega 2560 sketch with APDS boundary, saved gyro, and bounded ball search.
// Edit boundarycfg::SIDES in this file to match the physical sensor positions.
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// Inlined RobotConfig.h
#include <stdint.h>
namespace cfg {
static const uint16_t GYRO_POLL_MS = 5;
static const uint16_t GYRO_STALE_MS = 50;
static const uint16_t GYRO_CAL_SAMPLES = 500;
static const uint32_t GYRO_CAL_TIMEOUT_MS = 7000;
static const int GYRO_EEPROM_BASE = 64; // Bytes 64..79; APDS uses 0..29.
static const float GYRO_Z_SIGN = 1.0f; // Positive yaw should mean counterclockwise.
// Set an individual sign to -1 if that wheel spins backward for positive PWM.
static const int8_t FL_SIGN = 1;
static const int8_t FR_SIGN = 1;
static const int8_t BL_SIGN = -1;
static const int8_t BR_SIGN = 1;
static const float YAW_CORRECTION_SIGN = 1.0f; // Same convention as FastRoboCup.
static const float HOLD_KP = 2.5f;
static const float HOLD_KD = 0.0f;
}

// Inlined DriveLogic.h
#include <stdint.h>
#ifdef NAV_CONSTEXPR_TEST
#define NAV_FN constexpr
#else
#define NAV_FN inline
#endif

namespace drive {
struct Motion { float forward, strafe, rotation; };
struct Wheels { int16_t fl, fr, bl, br; };

NAV_FN float absolute(float value) { return value < 0 ? -value : value; }
NAV_FN float limit(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}
NAV_FN float wrap180(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}
NAV_FN int16_t rounded(float value) {
  return int16_t(value >= 0 ? value + 0.5f : value - 0.5f);
}
NAV_FN float yawCorrection(float current, float target, float rate,
                           float kp, float kd, float sign, float maximum) {
  return limit(sign * (kp * wrap180(current - target) + kd * rate), -maximum, maximum);
}
NAV_FN Wheels mix(Motion motion, int16_t ceiling) {
  // Original FastRoboCup drivetrain equations.
  float fl = motion.forward + motion.strafe + motion.rotation;
  float fr = motion.forward - motion.strafe - motion.rotation;
  float bl = motion.forward - motion.strafe + motion.rotation;
  float br = motion.forward + motion.strafe - motion.rotation;
  float biggest = absolute(fl);
  if (absolute(fr) > biggest) biggest = absolute(fr);
  if (absolute(bl) > biggest) biggest = absolute(bl);
  if (absolute(br) > biggest) biggest = absolute(br);
  if (biggest > ceiling) {
    float scale = ceiling / biggest;
    fl *= scale; fr *= scale; bl *= scale; br *= scale;
  }
  return {rounded(fl), rounded(fr), rounded(bl), rounded(br)};
}

}
#undef NAV_FN

// Inlined BoundaryLogic.h
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
LINE_FN Threshold thresholdFromField(float floor, float noise, uint16_t saturationLimit) {
  // A bright boundary is at least 25% above green, 8 noise deviations above it,
  // and 20 clear counts above it. No white reference is required.
  if (!(floor >= 0) || !(noise >= 0) || !(floor < saturationLimit * 0.80f))
    return {0, 0, false};
  float margin = floor * 0.25f;
  if (margin < 8.0f * noise) margin = 8.0f * noise;
  if (margin < 20.0f) margin = 20.0f;
  float trip = floor + margin;
  float release = floor + margin * 0.5f;
  if (!(trip < saturationLimit) || !(release < trip)) return {0, 0, false};
  return {uint16_t(trip + 0.5f), uint16_t(release + 0.5f), true};
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
struct EscapeResult { int8_t forward, strafe; bool ownsDrive, paused, cleared; };
class Escape {
 public:
  LINE_FN Escape() : active_(false), clearing_(false), paused_(false),
      started_(0), clearSince_(0), pauseStart_(0), lastSides_(0),
      forward_(0), strafe_(0) {}
  LINE_FN void reset() {
    active_ = clearing_ = paused_ = false;
    lastSides_ = 0; forward_ = strafe_ = 0;
  }
  LINE_FN EscapeResult update(uint8_t sides, uint32_t now, uint16_t timeoutMs, uint16_t clearMs) {
    if (!active_) {
      if (!sides) return {0, 0, false, false, false};
      active_ = true; clearing_ = paused_ = false; started_ = now;
      lastSides_ = 0; forward_ = strafe_ = 0;
    }
    if (!sides) {
      if (!clearing_) { clearing_ = true; clearSince_ = now; }
      if (uint32_t(now - clearSince_) >= clearMs) {
        reset();
        return {0, 0, true, false, true};
      }
      return {0, 0, true, true, false}; // Wait for a stable clear reading.
    }
    clearing_ = false;
    uint8_t newlySeen = sides & uint8_t(~lastSides_);
    forward_ = axisDirection(sides, newlySeen, FRONT, REAR, forward_);
    strafe_ = axisDirection(sides, newlySeen, RIGHT, LEFT, strafe_);
    lastSides_ = sides;
    if (paused_) {
      if (uint32_t(now - pauseStart_) < clearMs) return {0, 0, true, true, false};
      paused_ = false;
      started_ = now;
    }
    if (uint32_t(now - started_) >= timeoutMs) {
      paused_ = true; pauseStart_ = now;
      return {0, 0, true, true, false}; // Brief brake, then retry automatically.
    }
    if (!forward_ && !strafe_) return {0, 0, true, true, false};
    return {forward_, strafe_, true, false, false};
  }
 private:
  LINE_FN static int8_t axisDirection(uint8_t sides, uint8_t newlySeen,
                                      uint8_t negativeSide, uint8_t positiveSide,
                                      int8_t previous) {
    bool negative = sides & negativeSide, positive = sides & positiveSide;
    if (negative && !positive) return -1;
    if (positive && !negative) return 1;
    if (!negative && !positive) return 0;
    if ((newlySeen & negativeSide) && !(newlySeen & positiveSide)) return -1;
    if ((newlySeen & positiveSide) && !(newlySeen & negativeSide)) return 1;
    return previous; // Both edges: retain the last unambiguous escape direction.
  }
  bool active_, clearing_, paused_;
  uint32_t started_, clearSince_, pauseStart_;
  uint8_t lastSides_;
  int8_t forward_, strafe_;
};
}
#undef LINE_FN

// Inlined BoundaryConfig.h
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
static const int EEPROM_BASE = 0;       // Bytes 0..29 reserved for green-only calibration.
}

// Inlined MpuYaw.h
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>

class MpuYaw {
 public:
  enum Error { OK, NOT_FOUND, WRONG_ID, READ_FAILED, CONFIG_CHANGED,
               NOT_STILL, NO_NEW_DATA, SAMPLE_GAP, CAL_TIMEOUT, SAVE_FAILED, CAL_CANCELLED };
  MpuYaw() : address_(0x68), identity_(0), online_(false), calibrated_(false), calibrating_(false),
      error_(NOT_FOUND), count_(0), mean_(0), m2_(0), minimum_(0), maximum_(0),
      yaw_(0), rate_(0), lastPoll_(0), lastSampleMs_(0), lastSampleUs_(0),
      lastConfigCheck_(0), calibrationStart_(0), saving_(false), loaded_(false),
      firstSample_(true), saveStep_(0), recoveredReads_(0),
      readFailure_{0, 0, 0, 0, false, false}, hasReadFailure_(false) {}

  // Startup/rescan may reuse EEPROM; f and c always request a fresh calibration.
  bool startCalibration(bool useSaved = false) {
    online_ = calibrated_ = calibrating_ = false;
    saving_ = loaded_ = false;
    firstSample_ = true;
    recoveredReads_ = 0;
    hasReadFailure_ = false;
    if (!useSaved) EEPROM.update(cfg::GYRO_EEPROM_BASE, 0);
    error_ = NOT_FOUND;
    identity_ = 0;
    bool responded = false;
    bool unstableRead = false;
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      address_ = attempt == 0 ? 0x68 : 0x69;
      uint8_t id = 0;
      if (!read(0x75, &id, 1)) continue;
      responded = true;
      identity_ = id;
      uint8_t second = 0, third = 0;
      Serial.print(F("MPU address=0x")); Serial.print(address_, HEX);
      Serial.print(F(" WHO_AM_I=0x")); Serial.print(id, HEX);
      if (!read(0x75, &second, 1) || !read(0x75, &third, 1)) {
        Serial.println(F(" confirmation read failed"));
        unstableRead = true;
        continue;
      }
      Serial.print(F("/0x")); Serial.print(second, HEX);
      Serial.print(F("/0x")); Serial.print(third, HEX);
      if (id != second || second != third) {
        Serial.println(F(" UNSTABLE"));
        unstableRead = true;
        continue;
      }
      Serial.println(F(" stable"));
      // Match the standalone MPU-only test: 6050, 6500, and 6515.
      if (id != 0x68 && id != 0x70 && id != 0x74) {
        Serial.println(F("Unsupported MPU identity; motion remains blocked."));
        continue;
      }
      if (!write(0x6B, 0x01) || !write(0x6C, 0x00) || !write(0x1A, 0x03) ||
          !write(0x19, 0x04) || !write(0x1B, 0x00) || !write(0x1C, 0x00) ||
          !write(0x37, 0x00) || !write(0x38, 0x01) ||
          (identity_ != 0x68 && !write(0x1D, 0x03))) {
        fail(READ_FAILED);
        return false;
      }
      online_ = calibrating_ = true;
      error_ = OK;
      count_ = 0;
      mean_ = m2_ = yaw_ = rate_ = 0;
      calibrationStart_ = lastSampleMs_ = lastConfigCheck_ = millis();
      lastPoll_ = calibrationStart_;
      lastSampleUs_ = micros();
      if (useSaved && loadCalibration()) {
        loaded_ = true;
        calibrating_ = false;
        Serial.println(F("MPU calibration loaded from EEPROM; waiting for fresh data."));
      } else {
        EEPROM.update(cfg::GYRO_EEPROM_BASE, 0);
        Serial.println(F("MPU: keep still; calibrating then saving gyro offset."));
      }
      return true;
    }
    error_ = unstableRead ? READ_FAILED : (responded ? WRONG_ID : NOT_FOUND);
    return false;
  }

  void service() {
    if (!online_) return;
    if (saving_) {
      saveOneByte();
      if (!online_) return;
    }
    uint32_t now = millis();
    if (calibrating_ && uint32_t(now - calibrationStart_) > cfg::GYRO_CAL_TIMEOUT_MS) {
      fail(CAL_TIMEOUT); return;
    }
    if ((firstSample_ || calibrating_) && uint32_t(now - calibrationStart_) < 200) return;
    if (uint32_t(now - lastPoll_) < cfg::GYRO_POLL_MS) return;
    lastPoll_ = now;

    uint8_t packet[15];
    if (!read(0x3A, packet, 15)) { fail(READ_FAILED); return; }
    now = millis();
    if (!(packet[0] & 0x01)) {
      if (uint32_t(now - lastSampleMs_) > cfg::GYRO_STALE_MS) fail(NO_NEW_DATA);
      return;
    }
    int16_t raw = int16_t((uint16_t(packet[13]) << 8) | packet[14]);
    uint32_t sampleUs = micros();

    // A loaded offset is not healthy until a real sample arrives. Seed the
    // integration clock here so startup settling cannot cause SAMPLE_GAP.
    if (firstSample_ && !calibrating_) {
      firstSample_ = false;
      calibrated_ = true;
      lastSampleMs_ = now;
      lastSampleUs_ = sampleUs;
      return;
    }
    firstSample_ = false;

    if (calibrating_) {
      float ax = int16_t((uint16_t(packet[1]) << 8) | packet[2]);
      float ay = int16_t((uint16_t(packet[3]) << 8) | packet[4]);
      float az = int16_t((uint16_t(packet[5]) << 8) | packet[6]);
      float accelerationG = sqrtf(ax * ax + ay * ay + az * az) / 16384.0f;
      if (accelerationG < 0.7f || accelerationG > 1.3f) { fail(NOT_STILL); return; }
      if (count_ == 0) minimum_ = maximum_ = raw;
      if (raw < minimum_) minimum_ = raw;
      if (raw > maximum_) maximum_ = raw;
      ++count_;
      float difference = raw - mean_;
      mean_ += difference / count_;
      m2_ += difference * (raw - mean_);
      lastSampleMs_ = now;
      lastSampleUs_ = sampleUs;
      if (count_ >= cfg::GYRO_CAL_SAMPLES) {
        float standardDeviation = sqrtf(m2_ / (count_ - 1)) / 131.0f;
        float span = (float(maximum_) - minimum_) / 131.0f;
        if (standardDeviation > 1.5f || span > 10.0f || fabsf(mean_) / 131.0f > 10.0f) {
          fail(NOT_STILL); return;
        }
        calibrating_ = false;
        calibrated_ = true;
        yaw_ = rate_ = 0;
        startSave();
      }
      return;
    }

    uint32_t elapsedUs = sampleUs - lastSampleUs_;
    if (elapsedUs > uint32_t(cfg::GYRO_STALE_MS) * 1000UL) { fail(SAMPLE_GAP); return; }
    lastSampleUs_ = sampleUs;
    lastSampleMs_ = now;
    rate_ = ((raw - mean_) / 131.0f) * cfg::GYRO_Z_SIGN;
    yaw_ = drive::wrap180(yaw_ + rate_ * (elapsedUs / 1000000.0f));

    if (uint32_t(now - lastConfigCheck_) >= 250) {
      lastConfigCheck_ = now;
      uint8_t config[5] = {}, power[2], interruptConfig[2];
      uint8_t configLength = identity_ == 0x68 ? 4 : 5;
      if (!read(0x19, config, configLength) || !read(0x6B, power, 2) || !read(0x37, interruptConfig, 2)) {
        fail(READ_FAILED); return;
      }
      if (config[0] != 4 || config[1] != 3 || config[2] != 0 || config[3] != 0 ||
          power[0] != 1 || power[1] != 0 || interruptConfig[0] != 0 || interruptConfig[1] != 1 ||
          (identity_ != 0x68 && config[4] != 3))
        fail(CONFIG_CHANGED);
    }
  }

  bool healthy(uint32_t now) const {
    return online_ && calibrated_ && !saving_ && uint32_t(now - lastSampleMs_) <= cfg::GYRO_STALE_MS;
  }
  bool calibrating() const { return calibrating_; }
  bool calibrated() const { return calibrated_; }
  bool saving() const { return saving_; }
  bool loaded() const { return loaded_; }
  void cancelCalibration() {
    if (calibrating_ || saving_) {
      if (saving_) EEPROM.update(cfg::GYRO_EEPROM_BASE, 0);
      fail(CAL_CANCELLED);
    }
  }
  uint16_t progress() const { return count_; }
  uint8_t address() const { return address_; }
  uint8_t identity() const { return identity_; }
  Error error() const { return error_; }
  uint16_t recoveredReads() const { return recoveredReads_; }
  void printLastReadFailure() const {
    if (!hasReadFailure_) return;
    Serial.print(F("MPU I2C address=0x")); Serial.print(address_, HEX);
    Serial.print(F(" register=0x")); Serial.print(readFailure_.reg, HEX);
    Serial.print(F(" step="));
    Serial.print(readFailure_.selecting ? F("register-select") : F("data-read"));
    Serial.print(F(" wire_status=")); Serial.print(readFailure_.txStatus);
    Serial.print(F(" bytes=")); Serial.print(readFailure_.received);
    Serial.print('/'); Serial.print(readFailure_.expected);
    Serial.print(F(" timeout=")); Serial.println(readFailure_.timeout ? F("yes") : F("no"));
  }
  static const __FlashStringHelper *errorName(Error error) {
    switch (error) {
      case OK: return F("OK");
      case NOT_FOUND: return F("MPU_NOT_FOUND");
      case WRONG_ID: return F("MPU_WRONG_ID");
      case READ_FAILED: return F("MPU_READ_FAILED");
      case CONFIG_CHANGED: return F("MPU_CONFIG_CHANGED");
      case NOT_STILL: return F("MPU_CAL_NOT_STILL");
      case NO_NEW_DATA: return F("MPU_NO_NEW_DATA");
      case SAMPLE_GAP: return F("MPU_SAMPLE_GAP");
      case CAL_TIMEOUT: return F("MPU_CAL_TIMEOUT");
      case SAVE_FAILED: return F("MPU_EEPROM_FAILED");
      case CAL_CANCELLED: return F("MPU_CAL_CANCELLED");
    }
    return F("MPU_UNKNOWN_ERROR");
  }
  float yaw() const { return yaw_; }
  float rate() const { return rate_; }
  float biasRaw() const { return mean_; }
  void zero() { yaw_ = 0; lastSampleUs_ = micros(); }

 private:
  static_assert(sizeof(float) == 4, "Gyro EEPROM record requires a 32-bit float");
  static_assert(cfg::GYRO_EEPROM_BASE >= 0 && cfg::GYRO_EEPROM_BASE + 16 <= 4096,
                "Gyro record must fit the Mega EEPROM");
  bool loadCalibration() {
    if (cfg::GYRO_EEPROM_BASE + 16 > EEPROM.length()) return false;
    for (uint8_t i = 0; i < 16; ++i) record_[i] = EEPROM.read(cfg::GYRO_EEPROM_BASE + i);
    uint16_t crc = uint16_t(record_[14]) | (uint16_t(record_[15]) << 8);
    if (record_[0] != 'G' || record_[1] != 'Y' || record_[2] != 'R' || record_[3] != 'O' ||
        record_[4] != 1 || record_[5] != address_ || record_[6] != identity_ ||
        record_[7] != 0 || record_[8] != 4 || record_[9] != 3 ||
        line::crc16(record_, 14) != crc) return false;
    float bias;
    memcpy(&bias, record_ + 10, sizeof(bias));
    if (!isfinite(bias) || fabsf(bias) > 1310.0f) return false;
    mean_ = bias;
    return true;
  }
  void startSave() {
    const uint8_t header[10] = {'G', 'Y', 'R', 'O', 1, address_, identity_, 0, 4, 3};
    memcpy(record_, header, sizeof(header));
    memcpy(record_ + 10, &mean_, sizeof(mean_));
    uint16_t crc = line::crc16(record_, 14);
    record_[14] = uint8_t(crc); record_[15] = uint8_t(crc >> 8);
    saving_ = true; saveStep_ = 0;
    Serial.println(F("MPU: saving gyro calibration; motors remain stopped."));
  }
  void saveOneByte() {
    if (saveStep_ == 0) EEPROM.update(cfg::GYRO_EEPROM_BASE, 0);
    else if (saveStep_ < 16) EEPROM.update(cfg::GYRO_EEPROM_BASE + saveStep_, record_[saveStep_]);
    else {
      EEPROM.update(cfg::GYRO_EEPROM_BASE, record_[0]);
      saving_ = false;
      float expected = mean_;
      if (!loadCalibration() || mean_ != expected) {
        EEPROM.update(cfg::GYRO_EEPROM_BASE, 0);
        fail(SAVE_FAILED); return;
      }
      Serial.println(F("MPU calibration saved to EEPROM."));
    }
    ++saveStep_;
  }
  bool write(uint8_t reg, uint8_t value) {
    hasReadFailure_ = false;
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(address_);
    Wire.write(reg); Wire.write(value);
    uint8_t result = Wire.endTransmission();
    return result == 0 && !Wire.getWireTimeoutFlag();
  }
  bool read(uint8_t reg, uint8_t *buffer, uint8_t length) {
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      Wire.clearWireTimeoutFlag();
      Wire.beginTransmission(address_);
      Wire.write(reg);
      uint8_t txStatus = Wire.endTransmission(false);
      bool timedOut = Wire.getWireTimeoutFlag();
      if (txStatus != 0 || timedOut) {
        readFailure_ = {reg, txStatus, 0, length, true, timedOut};
        hasReadFailure_ = true;
        continue;
      }
      uint8_t received = Wire.requestFrom(address_, length);
      timedOut = Wire.getWireTimeoutFlag();
      if (timedOut || received != length) {
        while (Wire.available()) Wire.read();
        readFailure_ = {reg, txStatus, received, length, false, timedOut};
        hasReadFailure_ = true;
        continue;
      }
      for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
      if (attempt) ++recoveredReads_;
      hasReadFailure_ = false;
      return true;
    }
    return false;
  }
  void fail(Error error) {
    error_ = error;
    online_ = calibrated_ = calibrating_ = false;
    saving_ = false;
  }
  uint8_t address_, identity_;
  bool online_, calibrated_, calibrating_;
  Error error_;
  uint16_t count_;
  float mean_, m2_;
  int16_t minimum_, maximum_;
  float yaw_, rate_;
  uint32_t lastPoll_, lastSampleMs_, lastSampleUs_, lastConfigCheck_, calibrationStart_;
  bool saving_, loaded_, firstSample_;
  uint8_t saveStep_, record_[16];
  uint16_t recoveredReads_;
  struct ReadFailure {
    uint8_t reg, txStatus, received, expected;
    bool selecting, timeout;
  } readFailure_;
  bool hasReadFailure_;
};

// Inlined BoundarySensors.h
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

class BoundarySensors {
 public:
  BoundarySensors() : isolated_(false), running_(false), saving_(false),
      next_(0), saveStep_(0), capture_(-1), pendingCapture_(-1), preparing_(false),
      lastSlot_(0), captureStart_(0), prepareStart_(0) {}

  bool begin() {
    running_ = false; capture_ = pendingCapture_ = -1;
    preparing_ = saving_ = false; next_ = 0;
    for (uint8_t i = 0; i < count; ++i) sensors_[i] = Sensor{};
    if (!select(0)) { Serial.println(F("LINE: mux unavailable")); return false; }
    if (!line::validLayout(boundarycfg::CHANNELS, boundarycfg::SIDES, count)) {
      Serial.println(F("LINE: invalid channel/side mapping")); return false;
    }
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(0x39);
    uint8_t upstream = Wire.endTransmission();
    if (Wire.getWireTimeoutFlag() || upstream != 2) {
      Serial.println(F("LINE: upstream 0x39 conflict or bus error")); return false;
    }
    uint8_t found = 0;
    for (uint8_t i = 0; i < count; ++i) {
      if (!select(uint8_t(1U << boundarycfg::CHANNELS[i]))) return false;
      uint8_t id = 0;
      bool ok = read(0x92, &id, 1) && (id == 0xAB || id == 0x9C || id == 0x9E);
      if (ok) {
        ok = write(0x80, 0) && write(0xAB, 0) && write(0x81, boundarycfg::ATIME) &&
             write(0x8F, boundarycfg::GAIN) && write(0x8D, 0x60) &&
             write(0x90, 0x41) && write(0x9F, 0) && write(0x80, 1);
        delay(10); // Only during stopped initialization; sampling never delays.
        ok = ok && write(0x80, 3) && configurationOK();
      }
      if (!select(0)) return false;
      sensors_[i].configured = ok;
      sensors_[i].fault = ok ? WAITING : MISSING;
      found += ok;
      Serial.print(F("LINE sensor ")); Serial.print(i + 1);
      Serial.print(F(" CH")); Serial.print(boundarycfg::CHANNELS[i]);
      Serial.print(F(" ID=0x")); Serial.print(id, HEX);
      Serial.println(ok ? F(" ready") : F(" MISSING/CONFIG ERROR"));
    }
    running_ = true; lastSlot_ = millis();
    bool loaded = loadCalibration();
    Serial.println(loaded ? F("LINE green calibration loaded; auto-run when sensors are ready.") :
        F("LINE calibration required: c=MPU, g=GREEN; no white capture."));
    return found == count;
  }

  void service() {
    if (saving_) saveOneByte(); // At most one EEPROM byte per loop; motors are stopped.
    if (!isolated_) { select(0); return; }
    if (!running_) return;
    uint32_t now = millis();
    if (preparing_ && uint32_t(now - prepareStart_) >= CAL_PREPARE_MS) {
      preparing_ = false;
       if (!healthy(now)) {
        pendingCapture_ = -1;
         Serial.println(F("LINE preparation failed: sensor fault or stale data; retry g."));
        return;
      }
      capture_ = pendingCapture_;
      pendingCapture_ = -1;
      captureStart_ = now;
       Serial.println(F("LINE: collecting GREEN now; hold all sensors on field."));
    }
    if (capture_ != -1 && uint32_t(now - captureStart_) > boundarycfg::CAL_TIMEOUT_MS) {
      capture_ = -1;
      Serial.println(F("LINE calibration timeout; check sensors and repeat."));
    }
    if (uint32_t(now - lastSlot_) < boundarycfg::SLOT_MS) return;
    lastSlot_ = now;
    uint8_t i = next_;
    next_ = uint8_t((next_ + 1) % count);
    Sensor &sensor = sensors_[i];
    if (!sensor.configured) return;
    if (!select(uint8_t(1U << boundarycfg::CHANNELS[i]))) return;

    uint8_t status = 0, data[8] = {};
    bool ok = read(0x93, &status, 1);
    bool newData = ok && (status & 1);
    if (newData) ok = read(0x94, data, 8);
    uint16_t clear = decode(data);
    bool saturated = status & 0x80;
    // Datasheet ADC ceiling is 1025 counts per integration cycle.
    uint32_t limit = uint32_t(256U - boundarycfg::ATIME) * 1025UL;
    if (limit > 65535) limit = 65535;
    if (newData) for (uint8_t n = 0; n < 4; ++n)
      saturated |= decode(data + n * 2) >= limit;
    if (status & 0x80) {
      Wire.clearWireTimeoutFlag(); Wire.beginTransmission(0x39); Wire.write(uint8_t(0xE6));
      ok = (Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag()) && ok;
    }
    if (uint32_t(now - sensor.configChecked) >= 250) {
      sensor.configChecked = now;
      if (!configurationOK()) { sensor.configured = false; ok = false; sensor.fault = CONFIG_ERROR; }
    }
    if (!select(0)) return; // Always close the mux before the shared MPU/IR bus is used.
    now = millis();
    bool brightSaturation = saturated && ok && newData && capture_ == -1 &&
                            !preparing_ && sensor.threshold.valid;
    if (!ok || (saturated && !brightSaturation)) {
      sensor.good = false;
      if (sensor.configured) sensor.fault = saturated ? SATURATED : READ_ERROR;
      if (newData && ok) sensor.clear = clear;
      return;
    }
    if (!newData) return; // Older valid samples expire via healthy().
    sensor.clear = clear; sensor.fresh = now; sensor.good = true; sensor.fault = NONE;
    sensor.brightSaturated = brightSaturation;
    sensor.white = brightSaturation || line::whiteNow(sensor.white, clear, sensor.threshold);

    if (capture_ == -2) {
      Stats &stats = stats_[i];
      if (stats.n < boundarycfg::CAL_SAMPLES) stats.add(clear);
      bool complete = true;
       for (uint8_t n = 0; n < count; ++n)
         if (stats_[n].n < boundarycfg::CAL_SAMPLES) complete = false;
      if (complete) finishCapture();
    }
  }

  bool startField() {
    if (!canCapture()) return false;
    for (uint8_t i = 0; i < count; ++i) {
      stats_[i] = Stats{};
      sensors_[i].threshold.valid = sensors_[i].white = false;
      sensors_[i].brightSaturated = false;
    }
    EEPROM.update(boundarycfg::EEPROM_BASE, 0);
    capture_ = -1;
    pendingCapture_ = -2;
    preparing_ = true;
    prepareStart_ = millis();
    Serial.println(F("LINE: 3 seconds to put all four sensors over GREEN; motors braked."));
    return true;
  }
  void cancelCapture() { capture_ = pendingCapture_ = -1; preparing_ = false; }
  bool capturing() const { return capture_ != -1 || preparing_; }
  bool preparing() const { return preparing_; }
  bool saving() const { return saving_; }
  bool busIsolated() const { return isolated_; }
  bool calibrated() const {
    for (uint8_t i = 0; i < count; ++i) if (!sensors_[i].threshold.valid) return false;
    return true;
  }
  bool healthy(uint32_t now) const {
    if (!running_ || !isolated_) return false;
    for (uint8_t i = 0; i < count; ++i)
      if (!sensors_[i].configured || !sensors_[i].good ||
          uint32_t(now - sensors_[i].fresh) > boundarycfg::MAX_AGE_MS) return false;
    return true;
  }
  uint8_t hitMask() const {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < count; ++i) if (sensors_[i].white) mask |= uint8_t(1U << i);
    return mask;
  }
  void printReadings() const {
    Serial.print(F(" line=0x")); Serial.print(hitMask(), HEX);
    Serial.print(F(" C="));
    for (uint8_t i = 0; i < count; ++i) {
      if (i) Serial.print('/');
       if (sensors_[i].brightSaturated && sensors_[i].good) Serial.print(F("SAT"));
       else Serial.print(sensors_[i].clear);
      if (!sensors_[i].good || uint32_t(millis() - sensors_[i].fresh) > boundarycfg::MAX_AGE_MS)
        Serial.print('!');
    }
    for (uint8_t i = 0; i < count; ++i) {
      const Sensor &sensor = sensors_[i];
      if (sensor.good && uint32_t(millis() - sensor.fresh) <= boundarycfg::MAX_AGE_MS) continue;
      Serial.print(F(" L")); Serial.print(i + 1); Serial.print('=');
      if (sensor.fault == MISSING) Serial.print(F("MISSING"));
      else if (sensor.fault == READ_ERROR) Serial.print(F("READ_ERROR"));
      else if (sensor.fault == SATURATED) Serial.print(F("SATURATED"));
      else if (sensor.fault == CONFIG_ERROR) Serial.print(F("CONFIG_ERROR"));
      else Serial.print(F("STALE/WAIT"));
    }
  }

 private:
  static const uint8_t count = boundarycfg::SENSOR_COUNT;
  static const uint16_t CAL_PREPARE_MS = 3000;
  static_assert(count == 4, "This sketch supports four boundary sensors");
  static_assert(boundarycfg::CAL_SAMPLES >= 2, "Calibration needs multiple fresh samples");
  static_assert(boundarycfg::EEPROM_BASE >= 0 && boundarycfg::EEPROM_BASE + 30 <= 4096,
                "Boundary EEPROM storage must fit the Mega's EEPROM");
  enum Fault : uint8_t { NONE, WAITING, MISSING, READ_ERROR, SATURATED, CONFIG_ERROR };
  struct Sensor {
    bool configured = false, good = false, white = false, brightSaturated = false;
    uint16_t clear = 0;
    uint32_t fresh = 0, configChecked = 0;
    line::Threshold threshold = {0, 0, false};
    Fault fault = WAITING;
  };
  struct Stats {
    uint16_t n = 0;
    float mean = 0, m2 = 0;
    void add(uint16_t value) {
      ++n; float delta = value - mean;
      mean += delta / n; m2 += delta * (value - mean);
    }
    float noise() const { return n > 1 ? sqrtf(m2 / (n - 1)) : 0; }
  };
  bool muxFailure() {
    isolated_ = false;
    for (uint8_t i = 0; i < count; ++i) sensors_[i].good = false;
    return false;
  }
  bool select(uint8_t mask) {
    Wire.clearWireTimeoutFlag(); Wire.beginTransmission(boundarycfg::TCA_ADDRESS);
    Wire.write(mask);
    if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) return muxFailure();
    uint8_t received = Wire.requestFrom(boundarycfg::TCA_ADDRESS, uint8_t(1));
    if (received != 1 || Wire.getWireTimeoutFlag()) {
      while (Wire.available()) Wire.read();
      return muxFailure();
    }
    if (uint8_t(Wire.read()) != mask) return muxFailure();
    isolated_ = mask == 0;
    return true;
  }
  bool read(uint8_t reg, uint8_t *data, uint8_t length) {
    Wire.clearWireTimeoutFlag(); Wire.beginTransmission(0x39); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t received = Wire.requestFrom(uint8_t(0x39), length);
    if (received != length || Wire.getWireTimeoutFlag()) {
      while (Wire.available()) Wire.read();
      return false;
    }
    for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
    return true;
  }
  bool write(uint8_t reg, uint8_t value) {
    Wire.clearWireTimeoutFlag(); Wire.beginTransmission(0x39);
    Wire.write(reg); Wire.write(value);
    return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
  }
  bool configurationOK() {
    uint8_t enable = 0, atime = 0, gain = 0;
    return read(0x80, &enable, 1) && enable == 3 &&
           read(0x81, &atime, 1) && atime == boundarycfg::ATIME &&
           read(0x8F, &gain, 1) && gain == boundarycfg::GAIN;
  }
  static uint16_t decode(const uint8_t *data) {
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
  }
  bool canCapture() const {
    if (!saving_ && !capturing() && healthy(millis())) return true;
    Serial.println(F("LINE not ready: wait for fresh sensors, or b=rescan."));
    return false;
  }
  void finishCapture() {
    capture_ = -1;
    uint32_t integrationMax = uint32_t(256U - boundarycfg::ATIME) * 1025UL;
    uint16_t saturationLimit = integrationMax > 65535UL ? 65535U : uint16_t(integrationMax);
    bool allValid = true;
    for (uint8_t i = 0; i < count; ++i) {
      Sensor &sensor = sensors_[i];
      sensor.threshold = line::thresholdFromField(stats_[i].mean, stats_[i].noise(), saturationLimit);
      Serial.print(F("GREEN sensor ")); Serial.print(i + 1);
      Serial.print(F(" mean=")); Serial.print(stats_[i].mean, 1);
      Serial.print(F(" noise=")); Serial.print(stats_[i].noise(), 1);
      if (!sensor.threshold.valid) {
        allValid = false;
        Serial.println(F(" TOO BRIGHT/NOISY: lower gain or improve green placement; retry g."));
      } else {
        Serial.print(F(" bright=")); Serial.print(sensor.threshold.on);
        Serial.print(F(" release=")); Serial.println(sensor.threshold.off);
      }
    }
    if (!allValid) {
      for (uint8_t i = 0; i < count; ++i) sensors_[i].threshold.valid = false;
      return;
    }
    startSave();
  }
  bool loadCalibration() {
    if (boundarycfg::EEPROM_BASE < 0 || boundarycfg::EEPROM_BASE + 30 > EEPROM.length()) return false;
    for (uint8_t i = 0; i < 30; ++i) record_[i] = EEPROM.read(boundarycfg::EEPROM_BASE + i);
    if (record_[0] != 'L' || record_[1] != 'I' || record_[2] != 'N' || record_[3] != 'E' ||
        record_[4] != 2 || record_[5] != boundarycfg::ATIME || record_[6] != boundarycfg::GAIN ||
        record_[7] != count || line::crc16(record_, 28) != decode(record_ + 28)) return false;
    for (uint8_t i = 0; i < count; ++i) {
      if (record_[8 + i] != boundarycfg::CHANNELS[i] ||
          decode(record_ + 12 + i * 4) <= decode(record_ + 14 + i * 4)) return false;
    }
    for (uint8_t i = 0; i < count; ++i)
      sensors_[i].threshold = {decode(record_ + 12 + i * 4), decode(record_ + 14 + i * 4), true};
    return true;
  }
  void startSave() {
    record_[0] = 'L'; record_[1] = 'I'; record_[2] = 'N'; record_[3] = 'E';
    record_[4] = 2; record_[5] = boundarycfg::ATIME;
    record_[6] = boundarycfg::GAIN; record_[7] = count;
    for (uint8_t i = 0; i < count; ++i) {
      record_[8 + i] = boundarycfg::CHANNELS[i];
      uint16_t on = sensors_[i].threshold.on, off = sensors_[i].threshold.off;
      record_[12 + i * 4] = uint8_t(on); record_[13 + i * 4] = uint8_t(on >> 8);
      record_[14 + i * 4] = uint8_t(off); record_[15 + i * 4] = uint8_t(off >> 8);
    }
    uint16_t crc = line::crc16(record_, 28);
    record_[28] = uint8_t(crc); record_[29] = uint8_t(crc >> 8);
    saving_ = true; saveStep_ = 0;
    Serial.println(F("LINE: saving calibration; motors remain stopped."));
  }
  void saveOneByte() {
    // Invalidate first; commit the magic byte last after the checksummed payload.
    if (saveStep_ == 0) EEPROM.update(boundarycfg::EEPROM_BASE, 0);
    else if (saveStep_ < 30)
      EEPROM.update(boundarycfg::EEPROM_BASE + saveStep_, record_[saveStep_]);
    else {
      EEPROM.update(boundarycfg::EEPROM_BASE, record_[0]); saving_ = false;
      Serial.println(F("LINE calibration saved. Auto-run resumes when sensors are healthy."));
    }
    ++saveStep_;
  }
  Sensor sensors_[count];
  Stats stats_[count];
  uint8_t record_[30] = {};
  bool isolated_, running_, saving_;
  uint8_t next_, saveStep_;
  int8_t capture_, pendingCapture_;
  bool preparing_;
  uint32_t lastSlot_, captureStart_, prepareStart_;
};

// Mega 2560: HiTechnic IR Seeker V2 + MPU heading-held ball follower.
// Ball visible: follow with heading correction. Ball lost: bounded no-turn search.

static_assert(cfg::GYRO_EEPROM_BASE >= boundarycfg::EEPROM_BASE + 30 ||
              cfg::GYRO_EEPROM_BASE + 16 <= boundarycfg::EEPROM_BASE,
              "Gyro and line EEPROM records must not overlap");

const uint8_t IR_ADDRESS = 0x08;
const uint8_t FL1 = 8, FL2 = 6;
const uint8_t FR1 = 9, FR2 = 7;
const uint8_t BL1 = 2, BL2 = 3;
const uint8_t BR1 = 10, BR2 = 11;
const uint8_t MOTOR_PINS[8] = {FL1, FL2, FR1, FR2, BL1, BL2, BR1, BR2};

const uint16_t IR_POLL_MS = 50;
const uint16_t REPORT_MS = 200;
const uint8_t MIN_STRENGTH = 1;
const float CHASE_PWM = 75;
const float SEARCH_PWM = 60;
const uint16_t BALL_LOST_GRACE_MS = 200;
const uint16_t SEARCH_SIDE_MS = 250;
const uint16_t SEARCH_FORWARD_MS = 150;
const uint16_t SEARCH_TOTAL_MS = 6 * SEARCH_SIDE_MS + 3 * SEARCH_FORWARD_MS;
const float MAX_HEADING_PWM = 30;
const int16_t MAX_WHEEL_PWM = boundarycfg::FOLLOW_WHEEL_CAP;

MpuYaw imu;
BoundarySensors boundary;
line::Escape lineEscape;
drive::Wheels wheels = {0, 0, 0, 0};
bool enabled = false; // Setup enables automatic running after sensor safety checks.
bool headingLocked = false;
bool searchStarted = false;
bool searchFinished = false;
int8_t preferredSide = 1;
float targetYaw = 0;
uint32_t lastBallSeenMs = 0;
uint32_t searchStartMs = 0;
uint32_t lastIrPollMs = 0;
uint32_t lastReportMs = 0;
uint8_t lastDirection = 0;
uint16_t lastStrength = 0;
const __FlashStringHelper *state = nullptr;
MpuYaw::Error reportedMpuError = MpuYaw::OK;

void motorPair(uint8_t pin1, uint8_t pin2, int16_t value) {
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
  if (value > 0) analogWrite(pin1, uint8_t(value));
  else if (value < 0) analogWrite(pin2, uint8_t(-value));
}

void applyWheels(drive::Wheels next) {
  wheels = next;
  motorPair(FL1, FL2, next.fl * cfg::FL_SIGN);
  motorPair(FR1, FR2, next.fr * cfg::FR_SIGN);
  motorPair(BL1, BL2, next.bl * cfg::BL_SIGN);
  motorPair(BR1, BR2, next.br * cfg::BR_SIGN);
}

void stopMotors() { applyWheels({0, 0, 0, 0}); } // MDD3A LOW/LOW = brake.

// Preserve the user's tested AC read: register 0x49, direction plus 5 levels.
bool readIR(uint8_t &direction, uint16_t &strength) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(uint8_t(0x49));
  uint8_t error = Wire.endTransmission();
  if (error != 0 || Wire.getWireTimeoutFlag()) return false;

  uint8_t received = Wire.requestFrom(IR_ADDRESS, uint8_t(6));
  if (Wire.getWireTimeoutFlag() || received != 6) {
    while (Wire.available()) Wire.read();
    return false;
  }
  uint8_t data[6];
  for (uint8_t i = 0; i < 6; ++i) data[i] = Wire.read();
  direction = data[0];
  strength = 0;
  if (direction >= 1 && direction <= 9) {
    uint8_t index = direction / 2 + 1;
    if (direction & 1) strength = data[index];
    else strength = (uint16_t(data[index - 1]) + data[index]) / 2;
  }
  return true;
}

drive::Motion motionForBall(uint8_t direction) {
  // No deliberate turning, including for sectors 1 and 9.
  switch (direction) {
    case 1: case 2: return {0, -CHASE_PWM, 0};
    case 3: return {50, -60, 0};
    case 4: return {CHASE_PWM, -35, 0};
    case 5: return {CHASE_PWM, 0, 0};
    case 6: return {CHASE_PWM, 35, 0};
    case 7: return {50, 60, 0};
    case 8: case 9: return {0, CHASE_PWM, 0};
    default: return {0, 0, 0};
  }
}

void driveWithHeading(drive::Motion motion) {
  motion.rotation = drive::yawCorrection(imu.yaw(), targetYaw, imu.rate(),
      cfg::HOLD_KP, cfg::HOLD_KD, cfg::YAW_CORRECTION_SIGN, MAX_HEADING_PWM);
  applyWheels(drive::mix(motion, MAX_WHEEL_PWM));
}

// Short sideways and forward raster; no deliberate turn or blind reverse.
drive::Motion searchMotion(uint32_t elapsed) {
  float side = preferredSide < 0 ? -SEARCH_PWM : SEARCH_PWM;
  if (elapsed < SEARCH_SIDE_MS) return {0, side, 0};
  elapsed -= SEARCH_SIDE_MS;
  if (elapsed < SEARCH_FORWARD_MS) return {SEARCH_PWM, 0, 0};
  elapsed -= SEARCH_FORWARD_MS;
  if (elapsed < 2UL * SEARCH_SIDE_MS) return {0, -side, 0};
  elapsed -= 2UL * SEARCH_SIDE_MS;
  if (elapsed < SEARCH_FORWARD_MS) return {SEARCH_PWM, 0, 0};
  elapsed -= SEARCH_FORWARD_MS;
  if (elapsed < 2UL * SEARCH_SIDE_MS) return {0, side, 0};
  elapsed -= 2UL * SEARCH_SIDE_MS;
  if (elapsed < SEARCH_FORWARD_MS) return {SEARCH_PWM, 0, 0};
  elapsed -= SEARCH_FORWARD_MS;
  if (elapsed < SEARCH_SIDE_MS) return {0, -side, 0};
  return {0, 0, 0};
}

void beginCalibration(bool useSaved = false) {
  stopMotors();
  enabled = false;
  lineEscape.reset();
  headingLocked = false;
  searchStarted = false;
  searchFinished = false;
  lastBallSeenMs = millis();
  state = F("CALIBRATING");
  if (!imu.startCalibration(useSaved)) {
    state = F("MPU_FAULT");
    Serial.print(F("MPU start failed, code "));
    Serial.print(uint8_t(imu.error()));
    Serial.print(F(" ")); Serial.println(MpuYaw::errorName(imu.error()));
  } else {
    reportedMpuError = MpuYaw::OK;
    if (!imu.loaded()) Serial.println(F("Keep robot still for MPU calibration."));
  }
}

void executeCommand(char command) {
  if (command == 'r') {
    if (!line::mayMove(true, boundary.calibrated(), boundary.healthy(millis()),
                       imu.healthy(millis()), boundary.capturing(), boundary.saving())) {
      enabled = false;
      stopMotors();
      Serial.println(F("Not ready: check MPU and line data; c=gyro, g=green, b=rescan."));
      return;
    }
    lineEscape.reset();
    enabled = true;
    searchStarted = false;
    searchFinished = false;
    lastBallSeenMs = millis();
    targetYaw = imu.yaw();
    Serial.println(F("Following/search enabled; boundary takes priority."));
  } else if (command == 'c') {
    boundary.cancelCapture();
    beginCalibration();
    enabled = true; // Resume automatically when MPU calibration and save finish.
  } else if (command == 'g') {
    enabled = false;
    searchStarted = false;
    searchFinished = false;
    stopMotors();
    lineEscape.reset();
    if (imu.calibrating() || imu.saving()) {
      Serial.println(F("Wait for MPU calibration/save before pressing g again."));
      return;
    }
    enabled = boundary.startField(); // Resume automatically after green capture/save.
  } else if (command == 'b') {
    enabled = false;
    searchStarted = false;
    searchFinished = true;
    stopMotors();
    boundary.begin();
    beginCalibration(true);
    enabled = true;
  } else {
    enabled = false;
    searchStarted = false;
    searchFinished = true;
    stopMotors();
    state = F("STOPPED");
    imu.cancelCalibration();
    boundary.cancelCapture();
    lineEscape.reset();
    Serial.println(F("Stopped. c=gyro g=green r=run b=rescan."));
  }
}

void handleSerial() {
  // Movement commands require a complete, single-character line.
  // Stop is immediate and discards the rest of its line.
  static char pending = 0;
  static bool extraCharacters = false, discardLine = false;
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    char command = char(Serial.read());
    if (command >= 'A' && command <= 'Z') command += 'a' - 'A';
    if (command == '\r' || command == '\n') {
      if (!discardLine && pending)
        executeCommand(extraCharacters ? '?' : pending);
      pending = 0;
      extraCharacters = discardLine = false;
    } else if (command == 's' || command == 'x' || command == '0' || command == ' ') {
      enabled = false;
      searchStarted = false;
      searchFinished = true;
      stopMotors();
      boundary.cancelCapture();
      imu.cancelCalibration();
      lineEscape.reset();
      state = F("STOPPED");
      if (!discardLine) Serial.println(F("Stopped. r=resume"));
      pending = 0;
      discardLine = true;
    } else if (!discardLine) {
      if (pending) extraCharacters = true;
      else pending = command;
    }
  }
}

void report(uint32_t now) {
  if (uint32_t(now - lastReportMs) < REPORT_MS) return;
  lastReportMs = now;
  Serial.print(state);
  Serial.print(F(" dir=")); Serial.print(lastDirection);
  Serial.print(F(" strength=")); Serial.print(lastStrength);
  Serial.print(F(" yaw=")); Serial.print(imu.yaw(), 1);
  Serial.print(F(" mpu=")); Serial.print(MpuYaw::errorName(imu.error()));
  Serial.print(F(" id=0x")); Serial.print(imu.identity(), HEX);
  Serial.print(F(" mpu_retries=")); Serial.print(imu.recoveredReads());
  boundary.printReadings();
  Serial.print(F(" PWM="));
  Serial.print(wheels.fl); Serial.print('/');
  Serial.print(wheels.fr); Serial.print('/');
  Serial.print(wheels.bl); Serial.print('/');
  Serial.println(wheels.br);
}

void setup() {
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  Serial.println(F("FOLLOW + BOUNDED SEARCH + APDS BOUNDARY. Auto-runs when ready; edit boundarycfg::SIDES."));
  Serial.println(F("c=calibrate MPU; g=calibrate GREEN; s=stop; r=resume; b=rescan/load"));
  boundary.begin();
  beginCalibration(true);
  enabled = true; // No run command needed; faults/missing calibration still block motors.
}

bool handleBoundary(uint32_t now) {
  if (boundary.capturing()) {
    stopMotors(); state = boundary.preparing() ? F("LINE_PREPARE") : F("LINE_CALIBRATING"); return true;
  }
  if (boundary.saving()) {
    stopMotors(); state = F("LINE_SAVING"); return true;
  }
  if (!boundary.healthy(now)) {
    stopMotors(); state = F("LINE_SENSOR_FAULT"); return true;
  }
  if (!boundary.calibrated()) {
    stopMotors(); state = F("LINE_CAL_REQUIRED"); return true;
  }
  if (!enabled) {
    stopMotors(); state = F("STOPPED"); return true;
  }
  uint8_t sides = line::sideMask(boundary.hitMask(), boundarycfg::SIDES, boundarycfg::SENSOR_COUNT);
  line::EscapeResult escape = lineEscape.update(sides, now,
      boundarycfg::ESCAPE_TIMEOUT_MS, boundarycfg::CLEAR_MS);
  if (!escape.ownsDrive) return false;
  if (escape.paused) {
    stopMotors(); state = F("LINE_HOLD");
  } else if (escape.cleared) {
    stopMotors(); state = F("LINE_CLEARED");
  } else {
    state = F("LINE_ESCAPE");
    float rotation = drive::yawCorrection(imu.yaw(), targetYaw, imu.rate(),
        cfg::HOLD_KP, cfg::HOLD_KD, cfg::YAW_CORRECTION_SIGN, MAX_HEADING_PWM);
    applyWheels(drive::mix({escape.forward * boundarycfg::ESCAPE_PWM,
                           escape.strafe * boundarycfg::ESCAPE_PWM, rotation},
                          boundarycfg::ESCAPE_WHEEL_CAP));
  }
  return true;
}

void loop() {
  boundary.service();
  if (boundary.busIsolated()) imu.service();
  handleSerial();
  uint32_t now = millis();

  if (imu.error() != MpuYaw::OK && imu.error() != reportedMpuError) {
    stopMotors();
    reportedMpuError = imu.error();
    Serial.print(F("MPU fault, code "));
    Serial.print(uint8_t(reportedMpuError));
    Serial.print(F(" ")); Serial.println(MpuYaw::errorName(reportedMpuError));
    if (reportedMpuError == MpuYaw::READ_FAILED) imu.printLastReadFailure();
  }
  if (!boundary.busIsolated()) {
    searchStarted = false; searchFinished = true;
    stopMotors(); state = F("LINE_MUX_FAULT"); report(now); return;
  }
  if (!imu.healthy(now)) {
    stopMotors();
    state = imu.saving() ? F("MPU_SAVING") : imu.calibrating() ? F("CALIBRATING") :
        imu.error() == MpuYaw::OK ? F("MPU_WAIT_DATA") : F("MPU_FAULT");
    report(now);
    return;
  }
  if (!headingLocked) {
    targetYaw = imu.yaw();
    headingLocked = true;
    lastBallSeenMs = now;
    Serial.println(F("MPU ready; holding heading."));
  }
  if (handleBoundary(now)) {
    searchStarted = false; searchFinished = false; lastBallSeenMs = now;
    report(now);
    return;
  }
  if (uint32_t(now - lastIrPollMs) < IR_POLL_MS) { report(now); return; }
  lastIrPollMs = now;

  uint8_t direction = 0;
  uint16_t strength = 0;
  if (!readIR(direction, strength) || direction > 9) {
    stopMotors();
    lastDirection = 0;
    lastStrength = 0;
    state = F("IR_FAULT");
    searchStarted = false; searchFinished = true;
    Serial.println(F("IR read failed; stopped. Retrying automatically."));
    report(millis());
    return;
  }
  lastDirection = direction;
  lastStrength = strength;
  boundary.service();
  if (!boundary.busIsolated()) {
    stopMotors(); state = F("LINE_MUX_FAULT"); report(millis()); return;
  }
  imu.service();
  now = millis();
  if (!imu.healthy(now)) {
    stopMotors();
    state = F("MPU_FAULT");
    report(now);
    return;
  }
  if (handleBoundary(now)) {
    searchStarted = false; searchFinished = false; lastBallSeenMs = now;
    report(now); return;
  }

  if (direction >= 1 && strength >= MIN_STRENGTH) {
    lastBallSeenMs = now;
    searchStarted = false;
    searchFinished = false;
    if (direction < 5) preferredSide = -1;
    else if (direction > 5) preferredSide = 1;
    state = F("FOLLOW");
    driveWithHeading(motionForBall(direction));
  } else if (uint32_t(now - lastBallSeenMs) < BALL_LOST_GRACE_MS) {
    stopMotors();
    state = F("WAIT_BALL");
  } else if (searchFinished) {
    stopMotors();
    state = F("SEARCH_DONE");
  } else {
    if (!searchStarted) {
      searchStartMs = now;
      searchStarted = true;
    }
    uint32_t elapsed = uint32_t(now - searchStartMs);
    if (elapsed >= SEARCH_TOTAL_MS) {
      stopMotors();
      searchStarted = false;
      searchFinished = true;
      state = F("SEARCH_DONE");
      Serial.println(F("Bounded search finished; ball unseen. g=retry search."));
    } else {
      state = F("SEARCH");
      driveWithHeading(searchMotion(elapsed));
    }
  }
  report(now);
}
