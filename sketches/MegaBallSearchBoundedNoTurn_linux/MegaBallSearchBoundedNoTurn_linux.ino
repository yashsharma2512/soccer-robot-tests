// Standalone Mega 2560 bounded ball search, based on soccer-robot-tests main 67d42e6.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Inlined RobotConfig.h
#include <stdint.h>
namespace cfg {
static const uint16_t GYRO_POLL_MS = 5;
static const uint16_t GYRO_STALE_MS = 50;
static const uint16_t GYRO_CAL_SAMPLES = 500;
static const uint32_t GYRO_CAL_TIMEOUT_MS = 7000;
static const float GYRO_Z_SIGN = 1.0f; // Positive yaw should mean counterclockwise.
// Set an individual sign to -1 if that wheel spins backward for positive PWM.
static const int8_t FL_SIGN = 1;
static const int8_t FR_SIGN = 1;
static const int8_t BL_SIGN = 1;
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

// Inlined MpuYaw.h
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MpuYaw {
 public:
  enum Error { OK, NOT_FOUND, WRONG_ID, READ_FAILED, CONFIG_CHANGED,
               NOT_STILL, NO_NEW_DATA, SAMPLE_GAP, CAL_TIMEOUT };
  MpuYaw() : address_(0x68), identity_(0), online_(false), calibrated_(false), calibrating_(false),
      error_(NOT_FOUND), count_(0), mean_(0), m2_(0), minimum_(0), maximum_(0),
      yaw_(0), rate_(0), lastPoll_(0), lastSampleMs_(0), lastSampleUs_(0),
      lastConfigCheck_(0), calibrationStart_(0) {}

  bool startCalibration() {
    online_ = calibrated_ = calibrating_ = false;
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
      return true;
    }
    error_ = unstableRead ? READ_FAILED : (responded ? WRONG_ID : NOT_FOUND);
    return false;
  }

  void service() {
    if (!online_) return;
    uint32_t now = millis();
    if (calibrating_ && uint32_t(now - calibrationStart_) > cfg::GYRO_CAL_TIMEOUT_MS) {
      fail(CAL_TIMEOUT); return;
    }
    if (calibrating_ && uint32_t(now - calibrationStart_) < 200) return;
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
    return online_ && calibrated_ && uint32_t(now - lastSampleMs_) <= cfg::GYRO_STALE_MS;
  }
  bool calibrating() const { return calibrating_; }
  bool calibrated() const { return calibrated_; }
  uint16_t progress() const { return count_; }
  uint8_t address() const { return address_; }
  uint8_t identity() const { return identity_; }
  Error error() const { return error_; }
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
    }
    return F("MPU_UNKNOWN_ERROR");
  }
  float yaw() const { return yaw_; }
  float rate() const { return rate_; }
  float biasRaw() const { return mean_; }
  void zero() { yaw_ = 0; lastSampleUs_ = micros(); }

 private:
  bool write(uint8_t reg, uint8_t value) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(address_);
    Wire.write(reg); Wire.write(value);
    uint8_t result = Wire.endTransmission();
    return result == 0 && !Wire.getWireTimeoutFlag();
  }
  bool read(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t received = Wire.requestFrom(address_, length);
    if (Wire.getWireTimeoutFlag() || received != length) {
      while (Wire.available()) Wire.read();
      return false;
    }
    for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
    return true;
  }
  void fail(Error error) {
    error_ = error;
    online_ = calibrated_ = calibrating_ = false;
  }
  uint8_t address_, identity_;
  bool online_, calibrated_, calibrating_;
  Error error_;
  uint16_t count_;
  float mean_, m2_;
  int16_t minimum_, maximum_;
  float yaw_, rate_;
  uint32_t lastPoll_, lastSampleMs_, lastSampleUs_, lastConfigCheck_, calibrationStart_;
};

// Mega 2560: HiTechnic IR Seeker V2 + MPU heading-held ball follower.
// Ball visible: follow with heading correction. Ball lost: bounded no-turn search.

const uint8_t IR_ADDRESS = 0x08;
const uint8_t FL1 = 8, FL2 = 6;
const uint8_t FR1 = 9, FR2 = 7;
const uint8_t BL1 = 4, BL2 = 2;
const uint8_t BR1 = 3, BR2 = 5;
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
const int16_t MAX_WHEEL_PWM = 110;

MpuYaw imu;
drive::Wheels wheels = {0, 0, 0, 0};
bool enabled = true;
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

void stopMotors() { applyWheels({0, 0, 0, 0}); }

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

// Timed translations preserve heading and never command a backward leg.
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

void beginCalibration() {
  stopMotors();
  headingLocked = false;
  searchStarted = false;
  searchFinished = false;
  lastBallSeenMs = millis();
  state = F("CALIBRATING");
  if (!imu.startCalibration()) {
    state = F("MPU_FAULT");
    Serial.print(F("MPU start failed, code "));
    Serial.print(uint8_t(imu.error()));
    Serial.print(F(" ")); Serial.println(MpuYaw::errorName(imu.error()));
  } else {
    reportedMpuError = MpuYaw::OK;
    Serial.println(F("Keep robot still for MPU calibration."));
  }
}

void executeCommand(char command) {
  if (command == 'g') {
    enabled = true;
    searchStarted = false;
    searchFinished = false;
    lastBallSeenMs = millis();
    if (imu.healthy(millis())) targetYaw = imu.yaw();
    Serial.println(F("Following/search enabled; IR reads continue."));
  } else if (command == 'c') {
    beginCalibration();
  } else {
    enabled = false;
    searchStarted = false;
    stopMotors();
    state = F("STOPPED");
    Serial.println(F("Stopped. Commands: g=follow/search, s=stop, c=calibrate."));
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
      stopMotors();
      state = F("STOPPED");
      if (!discardLine) Serial.println(F("Stopped. g=resume"));
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
  Serial.println(F("BOUNDED SEARCH: follow ball; if lost, sweep sideways and forward without turns."));
  Serial.println(F("Pins FL 8/6 FR 9/7 BL 4/2 BR 3/5; s=stop g=follow c=recalibrate; use Newline"));
  beginCalibration();
}

void loop() {
  imu.service();
  handleSerial();
  uint32_t now = millis();

  if (imu.error() != MpuYaw::OK && imu.error() != reportedMpuError) {
    reportedMpuError = imu.error();
    Serial.print(F("MPU fault, code "));
    Serial.print(uint8_t(reportedMpuError));
    Serial.print(F(" ")); Serial.println(MpuYaw::errorName(reportedMpuError));
  }
  if (!imu.healthy(now)) {
    stopMotors();
    if (!imu.calibrating()) state = F("MPU_FAULT");
    report(now);
    return;
  }
  if (!headingLocked) {
    targetYaw = imu.yaw();
    headingLocked = true;
    lastBallSeenMs = now;
    Serial.println(F("MPU ready; holding heading."));
  }
  if (!enabled) {
    stopMotors();
    state = F("STOPPED");
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
    searchStarted = false;
    searchFinished = false;
    lastBallSeenMs = now;
    Serial.println(F("IR read failed; stopped. Retrying automatically."));
    report(millis());
    return;
  }
  lastDirection = direction;
  lastStrength = strength;
  imu.service();
  now = millis();
  if (!imu.healthy(now)) {
    stopMotors();
    state = F("MPU_FAULT");
    report(now);
    return;
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
