#pragma once
#include "PlaySensors.h"
#include "PlayMotor.h"

const uint8_t IR_ADDRESS = 0x08;
const uint8_t (&MOTOR_PINS)[8]=PlayMotor::pins;

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

PlayMotor motors;
uint8_t ir[6]={},irStatus=0,irBytes=0;
bool irOK=false;
uint32_t irFresh=0,irGood=0,irFailures=0,lastIR=0;
void pollIR() {
  if(uint32_t(millis()-lastIR)<30) return;
  lastIR=millis();uint8_t next[6];
  irOK=bus::readDevice(IR_ADDRESS,0x49,next,6,irStatus,irBytes,false) && next[0]<=9;
  if(irOK) { memcpy(ir,next,6);irFresh=millis();++irGood; } else ++irFailures;
}
bool freshIR() { return irOK && uint32_t(millis()-irFresh)<=100; }
MpuYaw imu;
BoundarySensors boundary;
line::Escape lineEscape;
drive::Wheels wheels = {0, 0, 0, 0};
bool enabled = false; // Setup enables automatic running after sensor safety checks.
bool headingLocked = false;
bool searchStarted = false;
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

bool pwmReady=false, busInitialized=false;
void emergencyMotorOff() { motors.halt();pwmReady=false;wheels={0,0,0,0}; }
bool initializeMotorPwm() { pwmReady=motors.begin();return pwmReady; }
void applyWheels(drive::Wheels next) {
  auto motor=motors.state();
  if(!motor.ready) { pwmReady=false;enabled=false;wheels={0,0,0,0};return; }
  if(!motors.request(next.fl,next.fr,next.bl,next.br)) { wheels={0,0,0,0};return; }
  wheels=next;
}
void stopMotors() { applyWheels({0,0,0,0}); }

// Original HiTechnic AC layout: direction plus five strengths at register 0x49.
bool readIR(uint8_t &direction,uint16_t &strength) {
  if(!freshIR()) return false;
  const uint8_t *data=ir;
  direction=data[0]; strength=0;
  if (direction>9) return false;
  if (direction) {
    uint8_t index=direction/2+1;
    strength=(direction&1) ? data[index] : (uint16_t(data[index-1])+data[index])/2;
  }
  return true;
}

uint32_t seenStorageGeneration=0;
void serviceImu() {
  if (seenStorageGeneration!=calibstore::generation) {
    imu.resyncAfterStorage(); seenStorageGeneration=calibstore::generation;
  }
  if (boundary.busIsolated()) imu.service();
  if (seenStorageGeneration!=calibstore::generation) {
    imu.resyncAfterStorage(); seenStorageGeneration=calibstore::generation;
  }
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

bool readyToResume() {
  return pwmReady && busInitialized && calibstore::ready && freshIR() &&
    line::mayMove(true,boundary.calibrated(),boundary.healthy(millis()),imu.healthy(millis()),
                  boundary.capturing(),boundary.saving());
}
bool permitPlay(); // Request generation is checked atomically before enabling PWM.
void executeCommand(char command) {
  if (command == 'r') {
    if (!readyToResume() || !motors.idle() || !permitPlay()) {
      enabled = false;
      stopMotors();
      Serial.println(F("Not ready: check MPU and line data; c=gyro, g=green, b=rescan."));
      return;
    }
    lineEscape.reset();
    enabled = true;
    searchStarted = false;
    lastBallSeenMs = millis();
    targetYaw = imu.yaw();
    Serial.println(F("Continuous search/follow enabled; boundary takes priority."));
  } else if (command == 'c') {
    boundary.cancelCapture();
    beginCalibration();
    enabled = false; // AP/Serial calibration stays stopped until explicit Resume.
  } else if (command == 'g') {
    enabled = false;
    searchStarted = false;
    stopMotors();
    lineEscape.reset();
    if (imu.calibrating() || imu.saving()) {
      Serial.println(F("Wait for MPU calibration/save before pressing g again."));
      return;
    }
    boundary.startField(); // Keep stopped after green capture/save.
  } else if (command == 'b') {
    enabled = false;
    searchStarted = false;
    stopMotors();
    boundary.begin();
    beginCalibration(true);
    enabled = false;
  } else {
    enabled = false;
    searchStarted = false;
    stopMotors();
    state = F("STOPPED");
    motors.halt();
    imu.cancelCalibration();
    boundary.cancelCapture();
    lineEscape.reset();
    Serial.println(F("Stopped. c=gyro g=green r=run b=rescan."));
  }
}

void report(uint32_t) {} // Live status is cached by the control task for AP/Serial.

void setupPlay() {
  for (uint8_t pin:MOTOR_PINS) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
  Serial.begin(115200);
  if (!initializeMotorPwm()) { state=F("PWM_FAULT"); Serial.println(state); return; }
  busInitialized=Wire.begin(espconfig::SDA_PIN,espconfig::SCL_PIN,100000);
  if (!busInitialized) {
    state=F("I2C_INIT_FAULT"); Serial.println(state); return;
  }
  Wire.setTimeOut(espconfig::I2C_TIMEOUT_MS);
  if (!calibstore::begin()) { state=F("NVS_FAULT"); Serial.println(state); return; }
  Serial.println(F("ESP32 AP FULL PLAY: continuous search/follow + line escape."));
  Serial.println(F("c=gyro; g=GREEN; s=stop; r=resume; b=rescan/load. Use Newline."));
  Serial.println(F("SDA21 SCL22: IR+MPU on main rails; APDS on TCA0..3. Edit boundarycfg::SIDES."));
  boundary.begin(); beginCalibration(true);
  seenStorageGeneration=calibstore::generation;
  enabled=false;
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

void servicePlay() {
  if (!pwmReady || !busInitialized || !calibstore::ready) {
    stopMotors(); enabled=false;
    state=!pwmReady ? F("PWM_FAULT") : !busInitialized ? F("I2C_INIT_FAULT") : F("NVS_FAULT");
    report(millis()); return;
  }
  boundary.service();
  serviceImu();
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
    searchStarted = false;
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
    searchStarted = false; lastBallSeenMs = now;
    report(now);
    return;
  }
  if (!freshIR()) {
    stopMotors();lastDirection=0;lastStrength=0;searchStarted=false;
    state=F("IR_FAULT");report(now);return;
  }
  if (uint32_t(now - lastIrPollMs) < IR_POLL_MS) {
    applyWheels(wheels); // Refresh watchdog only after live sensor/line checks passed.
    report(now);return;
  }
  lastIrPollMs = now;

  uint8_t direction = 0;
  uint16_t strength = 0;
  if (!readIR(direction, strength) || direction > 9) {
    stopMotors();
    lastDirection = 0;
    lastStrength = 0;
    state = F("IR_FAULT");
    searchStarted = false;
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
  serviceImu();
  now = millis();
  if (!imu.healthy(now)) {
    stopMotors();
    state = F("MPU_FAULT");
    report(now);
    return;
  }
  if (handleBoundary(now)) {
    searchStarted = false; lastBallSeenMs = now;
    report(now); return;
  }

  if (direction >= 1 && strength >= MIN_STRENGTH) {
    lastBallSeenMs = now;
    searchStarted = false;
    if (direction < 5) preferredSide = -1;
    else if (direction > 5) preferredSide = 1;
    state = F("FOLLOW");
    driveWithHeading(motionForBall(direction));
  } else if (uint32_t(now - lastBallSeenMs) < BALL_LOST_GRACE_MS) {
    stopMotors();
    state = F("WAIT_BALL");
  } else {
    if (!searchStarted) {
      searchStartMs = now;
      searchStarted = true;
    }
    uint32_t elapsed = uint32_t(now - searchStartMs);
    if (elapsed >= SEARCH_TOTAL_MS) {
      stopMotors();
      searchStarted = false;
      lastBallSeenMs = now;
      state = F("SEARCH_PAUSE");
    } else {
      state = F("SEARCH");
      driveWithHeading(searchMotion(elapsed));
    }
  }
  report(now);
}
