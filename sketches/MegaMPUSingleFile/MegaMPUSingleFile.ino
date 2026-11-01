// COMPLETE REPLACEMENT SKETCH: Mega 2560, two MDD3A, MPU6050 or MPU6500.
// Paste this entire file over your motor test. No extra .h tabs are required.
// SDA=20, SCL=21. Serial Monitor: 115200 baud, Newline.
// IR is not used in this stage. Motors stay stopped until a valid command.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#ifndef MEGA_MPU_NAV_CONFIG_H
#define MEGA_MPU_NAV_CONFIG_H
#include <stdint.h>
namespace cfg {
enum Mode { IDLE, HEADING_HOLD, TRANSLATE, TURN, NAVIGATION, MOTOR_PULSE };
static const uint16_t GYRO_POLL_MS = 5;
static const uint16_t GYRO_STALE_MS = 50;
static const uint16_t GYRO_CAL_SAMPLES = 500;
static const uint32_t GYRO_CAL_TIMEOUT_MS = 7000;
static const float GYRO_Z_SIGN = 1.0f; // Positive yaw should mean counterclockwise.
static const float YAW_CORRECTION_SIGN = 1.0f; // Same convention as FastRoboCup.
static const float HOLD_KP = 2.5f;
static const float HOLD_KD = 0.0f;
static const float MAX_HOLD_PWM = 40.0f;
static const float TURN_KP = 2.5f;
static const float TURN_KD = 0.8f;
static const float MIN_TURN_PWM = 30.0f;
static const float MAX_TURN_PWM = 60.0f;
static const uint16_t CONTROL_MS = 20;
static const uint16_t REPORT_MS = 200;
static const uint16_t MOVE_MS = 1500;
static const uint16_t PAUSE_MS = 500;
static const uint16_t HOLD_MS = 3000;
static const uint16_t TURN_TIMEOUT_MS = 6000;
static const uint8_t BASE_PWM = 80;
static const uint8_t MAX_PWM = 120;
}
#endif

#ifndef MEGA_MPU_NAV_LOGIC_H
#define MEGA_MPU_NAV_LOGIC_H
#include <stdint.h>
#ifdef NAV_CONSTEXPR_TEST
#define NAV_FN constexpr
#else
#define NAV_FN inline
#endif

namespace drive {
struct Motion { float forward, strafe, rotation; };
struct Wheels { int16_t fl, fr, bl, br; };
struct NavigationLeg { Motion motion; uint8_t number; bool paused, finished; };
NAV_FN float absolute(float value);
NAV_FN float limit(float value, float low, float high);
NAV_FN float wrap180(float angle);
NAV_FN int16_t rounded(float value);
NAV_FN float yawCorrection(float current, float target, float rate,
                           float kp, float kd, float sign, float maximum);
NAV_FN Wheels mix(Motion motion, int16_t ceiling);
NAV_FN float turnCorrection(float current, float target, float rate,
                            float kp, float kd, float sign, float minimum, float maximum);
NAV_FN NavigationLeg navigationLeg(uint32_t elapsed, uint32_t moveMs,
                                   uint32_t pauseMs, float speed);

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

NAV_FN float turnCorrection(float current, float target, float rate,
                            float kp, float kd, float sign, float minimum, float maximum) {
  float error = wrap180(current - target);
  float correction = yawCorrection(current, target, rate, kp, kd, sign, maximum);
  // Assist low-speed stiction only when moving slowly and correcting toward
  // the target. Do not override a damping command that is braking the turn.
  if (absolute(error) > 3 && absolute(rate) < 15 &&
      absolute(correction) < minimum && correction * (error * sign) > 0)
    correction = correction > 0 ? minimum : -minimum;
  return correction;
}

class TurnSettler {
 public:
  NAV_FN TurnSettler() : active_(false), since_(0) {}
  NAV_FN void reset() { active_ = false; since_ = 0; }
  NAV_FN bool update(float error, float rate, uint32_t now) {
    if (absolute(error) > 3 || absolute(rate) > 8) { active_ = false; return false; }
    if (!active_) { active_ = true; since_ = now; }
    return uint32_t(now - since_) >= 200;
  }
 private:
  bool active_;
  uint32_t since_;
};

NAV_FN NavigationLeg navigationLeg(uint32_t elapsed, uint32_t moveMs,
                                   uint32_t pauseMs, float speed) {
  uint32_t period = moveMs + pauseMs;
  if (period == 0 || elapsed / period >= 4) return {{0,0,0},4,false,true};
  uint8_t number = elapsed / period;
  bool paused = elapsed % period >= moveMs;
  if (paused) return {{0,0,0},number,true,false};
  if (number == 0) return {{speed,0,0},number,false,false};
  if (number == 1) return {{0,speed,0},number,false,false};
  if (number == 2) return {{-speed,0,0},number,false,false};
  return {{0,-speed,0},number,false,false};
}
}
#undef NAV_FN
#endif

#ifndef MAGNUM_MPU_YAW_H
#define MAGNUM_MPU_YAW_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MpuYaw {
 public:
  enum Error { OK, NOT_FOUND, WRONG_ID, READ_FAILED, CONFIG_CHANGED,
               NOT_STILL, NO_NEW_DATA, SAMPLE_GAP, CAL_TIMEOUT };
  MpuYaw() : address_(0x68), id_(0), online_(false), calibrated_(false), calibrating_(false),
      error_(NOT_FOUND), count_(0), mean_(0), m2_(0), minimum_(0), maximum_(0),
      yaw_(0), rate_(0), lastPoll_(0), lastSampleMs_(0), lastSampleUs_(0),
      lastConfigCheck_(0), calibrationStart_(0) {}

  bool startCalibration() {
    online_ = calibrated_ = calibrating_ = false;
    error_ = NOT_FOUND;
    id_ = 0;
    bool responded = false;
    bool unstable = false;
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      address_ = attempt == 0 ? 0x68 : 0x69;
      uint8_t id = 0;
      if (!read(0x75, &id, 1)) {
        Serial.print(F("MPU addr=0x")); Serial.print(address_, HEX);
        Serial.println(F(": no identity response"));
        continue;
      }
      responded = true;
      uint8_t confirm1 = 0, confirm2 = 0;
      if (!read(0x75, &confirm1, 1) || !read(0x75, &confirm2, 1)) {
        Serial.print(F("MPU addr=0x")); Serial.print(address_, HEX);
        Serial.println(F(": identity confirmation read failed"));
        unstable = true;
        continue;
      }
      Serial.print(F("MPU addr=0x")); Serial.print(address_, HEX);
      Serial.print(F(" WHO_AM_I="));
      printIdentityByte(id); Serial.print(',');
      printIdentityByte(confirm1); Serial.print(',');
      printIdentityByte(confirm2);
      bool stable = id == confirm1 && id == confirm2;
      Serial.println(stable ? F(" (stable)") : F(" (UNSTABLE)"));
      if (!stable) { unstable = true; continue; }
      id_ = id;
      if (!isSupportedId(id)) continue;
      if (!write(0x6B, 0x01) || !write(0x6C, 0x00) || !write(0x1A, 0x03) ||
          !write(0x19, 0x04) || !write(0x1B, 0x00) || !write(0x1C, 0x00) ||
          !write(0x37, 0x00) || !write(0x38, 0x01)) {
        fail(READ_FAILED);
        return false;
      }
      // MPU6500 has an additional accelerometer filter register.
      if (id_ == 0x70 && !write(0x1D, 0x03)) {
        fail(READ_FAILED); return false;
      }
      Serial.println(id_ == 0x70 ? F("Detected MPU6500") : F("Detected MPU6050"));
      online_ = calibrating_ = true;
      error_ = OK;
      count_ = 0;
      mean_ = m2_ = yaw_ = rate_ = 0;
      calibrationStart_ = lastSampleMs_ = lastConfigCheck_ = millis();
      lastPoll_ = calibrationStart_;
      lastSampleUs_ = micros();
      return true;
    }
    error_ = unstable ? READ_FAILED : (responded ? WRONG_ID : NOT_FOUND);
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
      uint8_t config[5], power[2], interruptConfig[2];
      if (!read(0x19, config, 5) || !read(0x6B, power, 2) || !read(0x37, interruptConfig, 2)) {
        fail(READ_FAILED); return;
      }
      if (config[0] != 4 || config[1] != 3 || config[2] != 0 || config[3] != 0 ||
          power[0] != 1 || power[1] != 0 || interruptConfig[0] != 0 || interruptConfig[1] != 1 ||
          (id_ == 0x70 && config[4] != 3))
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
  uint8_t identity() const { return id_; }
  static constexpr bool isSupportedId(uint8_t id) { return id == 0x68 || id == 0x70; }
  Error error() const { return error_; }
  float yaw() const { return yaw_; }
  float rate() const { return rate_; }
  float biasRaw() const { return mean_; }
  void zero() { yaw_ = 0; lastSampleUs_ = micros(); }

 private:
  void printIdentityByte(uint8_t id) {
    Serial.print(F("0x"));
    if (id < 16) Serial.print('0');
    Serial.print(id, HEX);
  }
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
  uint8_t address_, id_;
  bool online_, calibrated_, calibrating_;
  Error error_;
  uint16_t count_;
  float mean_, m2_;
  int16_t minimum_, maximum_;
  float yaw_, rate_;
  uint32_t lastPoll_, lastSampleMs_, lastSampleUs_, lastConfigCheck_, calibrationStart_;
};
#endif

// Main program declarations
void motor(uint8_t input1, uint8_t input2, int requested);
void setMotors(int fl, int fr, int bl, int br);
void stopWheels();
void stopNavigation();
void driveRobot(drive::Motion motion);
void printGyroError();
void printHelp();
bool prepareNavigation(cfg::Mode requestedMode);
void executeCommand(char command);
void handleSerial();
void updateNavigation(uint32_t now);
void printReport(uint32_t now);
void setup();
void loop();

// Mega 2560 + MPU6050 + two MDD3A + four omni motors.
// MPU movement/navigation tests first. IR is not read by this sketch.
// Serial Monitor: 115200 baud, Newline. Motors stay stopped after calibration.
#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega or Mega 2560 in Tools > Board.
#endif
#include <Wire.h>

const uint8_t FL1 = 10, FL2 = 11;
const uint8_t FR1 = 4, FR2 = 5;
const uint8_t BL1 = 8, BL2 = 9;
const uint8_t BR1 = 3, BR2 = 2;
const int8_t FL_SIGN = 1, FR_SIGN = 1, BL_SIGN = 1, BR_SIGN = 1;
const uint8_t MOTOR_PINS[8] = {FL1, FL2, FR1, FR2, BL1, BL2, BR1, BR2};

MpuYaw gyro;
drive::TurnSettler turnSettler;
cfg::Mode mode = cfg::IDLE;
drive::Motion translation = {0,0,0};
drive::Wheels actualPWM = {0,0,0,0};
uint32_t started = 0, lastControl = 0, lastReport = 0;
float targetYaw = 0;
uint8_t pulseWheel = 0;
int8_t pulseSign = 1;
bool gyroReadyReported = false;
MpuYaw::Error previousError = MpuYaw::OK;
char commandLine[8] = {};
uint8_t commandLength = 0;
bool discardLine = false;

void motor(uint8_t input1, uint8_t input2, int requested) {
  int pwm = constrain(requested, -int(cfg::MAX_PWM), int(cfg::MAX_PWM));
  if (abs(pwm) < 6) pwm = 0;
  analogWrite(input1, 0); analogWrite(input2, 0);
  if (pwm > 0) analogWrite(input1, pwm);
  else if (pwm < 0) analogWrite(input2, -pwm);
}

void setMotors(int fl, int fr, int bl, int br) {
  motor(FL1, FL2, fl * FL_SIGN);
  motor(FR1, FR2, fr * FR_SIGN);
  motor(BL1, BL2, bl * BL_SIGN);
  motor(BR1, BR2, br * BR_SIGN);
  actualPWM = {int16_t(fl), int16_t(fr), int16_t(bl), int16_t(br)};
}

void stopWheels() { setMotors(0,0,0,0); }

void stopNavigation() {
  mode = cfg::IDLE;
  turnSettler.reset();
  stopWheels();
}

void driveRobot(drive::Motion motion) {
  drive::Wheels wheels = drive::mix(motion, cfg::MAX_PWM);
  setMotors(wheels.fl, wheels.fr, wheels.bl, wheels.br);
}

void printGyroError() {
  switch (gyro.error()) {
    case MpuYaw::OK: Serial.print(F("OK")); break;
    case MpuYaw::NOT_FOUND: Serial.print(F("MPU_NOT_FOUND")); break;
    case MpuYaw::WRONG_ID: Serial.print(F("MPU_WRONG_ID")); break;
    case MpuYaw::READ_FAILED: Serial.print(F("MPU_READ_FAILED")); break;
    case MpuYaw::CONFIG_CHANGED: Serial.print(F("MPU_CONFIG_CHANGED")); break;
    case MpuYaw::NOT_STILL: Serial.print(F("MPU_CAL_NOT_STILL")); break;
    case MpuYaw::NO_NEW_DATA: Serial.print(F("MPU_NO_NEW_DATA")); break;
    case MpuYaw::SAMPLE_GAP: Serial.print(F("MPU_SAMPLE_GAP")); break;
    case MpuYaw::CAL_TIMEOUT: Serial.print(F("MPU_CAL_TIMEOUT")); break;
  }
}

void printHelp() {
  Serial.println(F("1..4 FL/FR/BL/BR positive; 5..8 reverse; 800ms each"));
  Serial.println(F("w/b: forward/back + gyro 1.5s; a/d: left/right + gyro 1.5s"));
  Serial.println(F("g: heading hold 3s; q/e: turn left/right 90deg, max 6s"));
  Serial.println(F("n: forward/right/back/left, 1.5s each + 0.5s pauses"));
  Serial.println(F("c: calibrate STILL; z: zero yaw; s/x/0/space: stop; ?: help"));
}

bool prepareNavigation(cfg::Mode requestedMode) {
  stopNavigation();
  if (!gyro.healthy(millis())) {
    Serial.print(F("Movement refused: "));
    if (gyro.calibrating()) Serial.println(F("calibration running."));
    else { printGyroError(); Serial.println(F(". Send c with robot stationary.")); }
    return false;
  }
  targetYaw = gyro.yaw();
  started = millis();
  mode = requestedMode;
  return true;
}

void executeCommand(char command) {
  stopNavigation();
  if (command == '?') { printHelp(); return; }
  if (command == 'c') {
    gyroReadyReported = false;
    Serial.println(F("Keep robot completely STILL for gyro calibration."));
    if (!gyro.startCalibration()) { printGyroError(); Serial.println(); }
    return;
  }
  if (command == 'z') {
    gyro.zero(); targetYaw = 0;
    Serial.println(F("Yaw zeroed; motors stopped.")); return;
  }
  if (command >= '1' && command <= '8') {
    if (!prepareNavigation(cfg::MOTOR_PULSE)) return;
    uint8_t number = command - '1';
    pulseWheel = number % 4;
    pulseSign = number < 4 ? 1 : -1;
  } else if (command == 'w' || command == 'b' || command == 'a' || command == 'd') {
    if (!prepareNavigation(cfg::TRANSLATE)) return;
    translation = {0,0,0};
    if (command == 'w') translation.forward = cfg::BASE_PWM;
    if (command == 'b') translation.forward = -int(cfg::BASE_PWM);
    if (command == 'a') translation.strafe = -int(cfg::BASE_PWM);
    if (command == 'd') translation.strafe = cfg::BASE_PWM;
  } else if (command == 'g') {
    if (!prepareNavigation(cfg::HEADING_HOLD)) return;
  } else if (command == 'q' || command == 'e') {
    if (!prepareNavigation(cfg::TURN)) return;
    targetYaw = drive::wrap180(targetYaw + (command == 'q' ? 90.0f : -90.0f));
  } else if (command == 'n') {
    if (!prepareNavigation(cfg::NAVIGATION)) return;
  } else { Serial.println(F("Unknown command; motors stopped.")); return; }
  Serial.print(F("Started. Target yaw=")); Serial.println(targetYaw,1);
}

void handleSerial() {
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    if (c == '\r' || c == '\n') {
      if (!discardLine) {
        if (commandLength == 1) executeCommand(commandLine[0]);
        else if (commandLength > 1) {
          stopNavigation(); Serial.println(F("Send one command per line."));
        }
      }
      commandLength = 0; discardLine = false;
      continue;
    }
    if (c == 's' || c == 'x' || c == '0' || c == ' ') {
      stopNavigation();
      if (!discardLine) Serial.println(F("STOPPED"));
      commandLength = 0; discardLine = true;
      continue;
    }
    if (discardLine) continue;
    if (commandLength < sizeof(commandLine) - 1) commandLine[commandLength++] = c;
    else {
      stopNavigation(); commandLength = 0; discardLine = true;
      Serial.println(F("Command too long; motors stopped."));
    }
  }
}

void updateNavigation(uint32_t now) {
  if (mode == cfg::IDLE) return;
  if (!gyro.healthy(now)) {
    stopNavigation(); Serial.print(F("Stopped: ")); printGyroError(); Serial.println(); return;
  }
  uint32_t elapsed = now - started;
  if ((mode == cfg::HEADING_HOLD && elapsed >= cfg::HOLD_MS) ||
      (mode == cfg::TRANSLATE && elapsed >= cfg::MOVE_MS) ||
      (mode == cfg::MOTOR_PULSE && elapsed >= 800)) {
    stopNavigation(); Serial.println(F("Test complete; stopped.")); return;
  }
  if (mode == cfg::TURN && elapsed >= cfg::TURN_TIMEOUT_MS) {
    stopNavigation(); Serial.println(F("Turn timeout; stopped. Check direction/gains/friction.")); return;
  }
  if (uint32_t(now - lastControl) < cfg::CONTROL_MS) return;
  lastControl = now;

  if (mode == cfg::MOTOR_PULSE) {
    int pwm = pulseSign * cfg::BASE_PWM;
    setMotors(pulseWheel == 0 ? pwm : 0, pulseWheel == 1 ? pwm : 0,
              pulseWheel == 2 ? pwm : 0, pulseWheel == 3 ? pwm : 0);
    return;
  }
  float error = drive::wrap180(gyro.yaw() - targetYaw);
  if (mode == cfg::TURN) {
    if (turnSettler.update(error, gyro.rate(), now)) {
      stopNavigation(); Serial.println(F("Turn reached target and settled; stopped.")); return;
    }
    if (drive::absolute(error) <= 3) { stopWheels(); return; }
    float correction = drive::turnCorrection(gyro.yaw(), targetYaw, gyro.rate(),
        cfg::TURN_KP, cfg::TURN_KD, cfg::YAW_CORRECTION_SIGN,
        cfg::MIN_TURN_PWM, cfg::MAX_TURN_PWM);
    driveRobot({0,0,correction});
    return;
  }

  float correction = drive::yawCorrection(gyro.yaw(), targetYaw, gyro.rate(),
      cfg::HOLD_KP, cfg::HOLD_KD, cfg::YAW_CORRECTION_SIGN, cfg::MAX_HOLD_PWM);
  drive::Motion command = {0,0,correction};
  if (mode == cfg::TRANSLATE) {
    command.forward = translation.forward;
    command.strafe = translation.strafe;
  } else if (mode == cfg::NAVIGATION) {
    drive::NavigationLeg leg = drive::navigationLeg(elapsed, cfg::MOVE_MS, cfg::PAUSE_MS, cfg::BASE_PWM);
    if (leg.finished) { stopNavigation(); Serial.println(F("Navigation sequence complete; stopped.")); return; }
    if (leg.paused) { stopWheels(); return; }
    command.forward = leg.motion.forward;
    command.strafe = leg.motion.strafe;
  }
  driveRobot(command);
}

void printReport(uint32_t now) {
  if (uint32_t(now - lastReport) < cfg::REPORT_MS) return;
  lastReport = now;
  Serial.print(F("mode=")); Serial.print(int(mode));
  Serial.print(F(" gyro="));
  if (gyro.calibrating()) {
    Serial.print(F("CAL ")); Serial.print(gyro.progress()); Serial.print('/'); Serial.print(cfg::GYRO_CAL_SAMPLES);
  } else printGyroError();
  Serial.print(F(" id=0x")); Serial.print(gyro.identity(), HEX);
  Serial.print(F(" yaw=")); Serial.print(gyro.yaw(),1);
  Serial.print(F(" target=")); Serial.print(targetYaw,1);
  Serial.print(F(" rate=")); Serial.print(gyro.rate(),1);
  Serial.print(F(" PWM FL/FR/BL/BR="));
  Serial.print(actualPWM.fl); Serial.print('/'); Serial.print(actualPWM.fr);
  Serial.print('/'); Serial.print(actualPWM.bl); Serial.print('/'); Serial.println(actualPWM.br);
}

void setup() {
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(MOTOR_PINS[i], LOW); pinMode(MOTOR_PINS[i], OUTPUT);
  }
  stopNavigation();
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(25000,true);
  Serial.println(F("MEGA MPU SINGLE FILE: motors start stopped. 115200 baud, Newline."));
  printHelp();
  Serial.println(F("Keep robot STILL for approximately 3 seconds."));
  if (!gyro.startCalibration()) { printGyroError(); Serial.println(F("; c retries.")); }
}

void loop() {
  handleSerial();
  gyro.service();
  uint32_t now = millis();
  if (gyro.calibrated() && !gyroReadyReported) {
    gyroReadyReported = true;
    Serial.println(F("MPU ready. Motors remain stopped. Choose a test."));
  }
  if (gyro.error() != previousError) {
    previousError = gyro.error();
    if (previousError != MpuYaw::OK) {
      stopNavigation(); Serial.print(F("MPU fault: ")); printGyroError(); Serial.println();
    }
  }
  updateNavigation(now);
  printReport(now);
}
