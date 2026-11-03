// Mega 2560: HiTechnic IR Seeker V2 + MPU heading-held ball follower.
// Ball visible: follow with heading correction. Ball lost: stop and keep reading.
#include <Arduino.h>
#include <Wire.h>
#include "DriveLogic.h"
#include "RobotConfig.h"
#include "MpuYaw.h"

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
const float MAX_HEADING_PWM = 30;
const int16_t MAX_WHEEL_PWM = 110;

MpuYaw imu;
drive::Wheels wheels = {0, 0, 0, 0};
bool enabled = true;
bool headingLocked = false;
float targetYaw = 0;
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

void beginCalibration() {
  stopMotors();
  headingLocked = false;
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
    if (imu.healthy(millis())) targetYaw = imu.yaw();
    Serial.println(F("Following enabled. No ball = stopped; IR reads continue."));
  } else if (command == 'c') {
    beginCalibration();
  } else {
    enabled = false;
    stopMotors();
    state = F("STOPPED");
    Serial.println(F("Stopped. Commands: g=follow, s=stop, c=calibrate."));
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
  Serial.println(F("SIMPLE FOLLOW: ball seen = move; no ball = stop and keep checking."));
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
    state = F("FOLLOW");
    driveWithHeading(motionForBall(direction));
  } else {
    stopMotors();
    state = F("WAIT_BALL");
  }
  report(now);
}
