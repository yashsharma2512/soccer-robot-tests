/*
  TCS34725 soccer robot for Arduino Mega 2560, based on MegaOGTest.
  Same ball following, sideways/forward search, gyro heading hold, and
  TCS34725 white-line escape through a TCA9548A multiplexer.
  The robot starts in AUTO after startup calibration and fresh color readings.
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>

#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega or Mega 2560 with the ATmega2560 processor.
#endif

struct TcsColor {
  uint16_t clear;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
};

// Mega hardware I2C pins: SDA D20, SCL D21.
#define I2C_SDA 20
#define I2C_SCL 21

void stopAll();
void driveWithGyro(int vy, int vx);
extern int debugVX;
extern int debugVY;

// ============================================================
// MOTOR PINS
//
// Current Arduino Mega motor pins (digital pins, all PWM-capable).
// Keep the motor ORDER the same as the stable code.
//
// FL = Front Left
// FR = Front Right
// BL = Back Left
// BR = Back Right
// ============================================================

#define FL_PWM_A 9
#define FL_PWM_B 7

#define FR_PWM_A 5
#define FR_PWM_B 6

#define BL_PWM_A 11
#define BL_PWM_B 13

#define BR_PWM_A 10
#define BR_PWM_B 12

// ============================================================
// MOTOR STRUCTURE
// ============================================================

struct Motor {
  uint8_t pinA;
  uint8_t pinB;
  int dirSign;
};

Motor motorFL = {FL_PWM_A, FL_PWM_B, 1};
Motor motorFR = {FR_PWM_A, FR_PWM_B, 1};
Motor motorBL = {BL_PWM_A, BL_PWM_B, -1};
Motor motorBR = {BR_PWM_A, BR_PWM_B, 1};

// Bit 0 = FL, 1 = FR, 2 = BL, 3 = BR. The original BL polarity is reversed.
const uint8_t DEFAULT_MOTOR_REVERSE_MASK = 0x04;
uint8_t motorReverseMask = DEFAULT_MOTOR_REVERSE_MASK;

void applyMotorDirections(uint8_t reverseMask) {
  motorFL.dirSign = (reverseMask & 0x01) ? -1 : 1;
  motorFR.dirSign = (reverseMask & 0x02) ? -1 : 1;
  motorBL.dirSign = (reverseMask & 0x04) ? -1 : 1;
  motorBR.dirSign = (reverseMask & 0x08) ? -1 : 1;
  motorReverseMask = reverseMask & 0x0F;
}

// EEPROM bytes 0 and 1 hold a marker and the four motor direction bits.
void loadMotorDirections() {
  uint8_t mask = DEFAULT_MOTOR_REVERSE_MASK;
  if (EEPROM.read(0) == 0xA5) {
    uint8_t saved = EEPROM.read(1);
    if (saved <= 0x0F) mask = saved;
  }
  applyMotorDirections(mask);
}

bool saveMotorDirections(uint8_t reverseMask) {
  reverseMask &= 0x0F;
  EEPROM.update(1, reverseMask);
  EEPROM.update(0, 0xA5);
  applyMotorDirections(reverseMask);
  return true;
}


// TCS34725 x4 on a TCA9548A. The ground strip is on Mega D15.
constexpr uint8_t TCA_ADDR = 0x70;
constexpr uint8_t TCS_ADDR = 0x29;
constexpr uint8_t TCS_COMMAND = 0x80;
constexpr uint8_t TCS_AUTO_INCREMENT = 0x20;
constexpr uint8_t TCS_ENABLE_REG = 0x00;
constexpr uint8_t TCS_ATIME_REG = 0x01;
constexpr uint8_t TCS_CONTROL_REG = 0x0F;
constexpr uint8_t TCS_ID_REG = 0x12;
constexpr uint8_t TCS_STATUS_REG = 0x13;
constexpr uint8_t TCS_AVALID = 0x01;
constexpr uint8_t TCS_PON = 0x01;
constexpr uint8_t TCS_AEN = 0x02;

constexpr uint8_t TCS_ATIME = 0xFC;  // Four 2.4 ms integration steps.
constexpr uint8_t TCS_GAIN = 0x00;   // 1x. Use 0x01 for 4x if readings are too dim.
constexpr uint8_t TCS_LIGHT_BRIGHTNESS = 255;
constexpr uint8_t TCS_CAL_SAMPLES = 4;
constexpr unsigned long TCS_CAL_SPACING_MS = ((256UL - TCS_ATIME) * 24 + 9) / 10 + 5;
constexpr uint32_t TCS_FULL_SCALE = (256UL - TCS_ATIME) * 1024UL;
constexpr uint16_t TCS_SATURATION = TCS_FULL_SCALE < 65535UL ? TCS_FULL_SCALE : 65535UL;
constexpr unsigned long TCS_POLL_INTERVAL_US = 2500;
constexpr unsigned long TCS_MAX_AGE_MS = 60;
// Smaller distance catches tape whose colour is closer to the green reference.
constexpr float TCS_GREEN_TOLERANCE = 0.05f;
constexpr float TCS_MIN_BRIGHTNESS_RATIO = 0.30f;
// Bright tape is a line even if its normalized RGB looks similar to green.
constexpr float TCS_WHITE_BRIGHTNESS_RATIO = 1.50f;
constexpr int LINE_ESCAPE_SPEED = 165;

const uint8_t tcsChannels[] = {0, 1, 3, 2};
const char *const tcsNames[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
constexpr uint8_t tcsCount = sizeof(tcsChannels);
Adafruit_NeoPixel tcsPixels(16, 15, NEO_GRB + NEO_KHZ800);

struct GreenReference {
  float red;
  float green;
  float blue;
  float clear;
};

struct TcsReading {
  TcsColor color;
  unsigned long lastUpdate;
  bool available;
  bool white;
};

GreenReference tcsGreen[tcsCount];
TcsReading tcsLive[tcsCount];
bool tcsHardwareReady = false;
bool tcsCalibrationComplete = false;
uint8_t tcsRoundRobinIndex = 0;
unsigned long lastTcsPollUs = 0;

bool tcsSetMux(uint8_t mask) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

bool tcsWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(TCS_COMMAND | reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool tcsReadRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  uint8_t command = TCS_COMMAND | reg;
  if (count > 1) command |= TCS_AUTO_INCREMENT;
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(command);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TCS_ADDR, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = uint8_t(Wire.read());
  return true;
}

bool tcsStartContinuous() {
  if (!tcsWriteRegister(TCS_ENABLE_REG, 0) ||
      !tcsWriteRegister(TCS_ATIME_REG, TCS_ATIME) ||
      !tcsWriteRegister(TCS_CONTROL_REG, TCS_GAIN) ||
      !tcsWriteRegister(TCS_ENABLE_REG, TCS_PON)) {
    return false;
  }
  delay(3);  // Oscillator startup. This only runs during calibration.
  return tcsWriteRegister(TCS_ENABLE_REG, TCS_PON | TCS_AEN);
}

bool initializeTcsSensors() {
  if (!tcsSetMux(0)) {
    Serial.println(F("TCA9548A unavailable at 0x70."));
    return false;
  }
  bool ready = true;
  for (uint8_t i = 0; i < tcsCount; ++i) {
    tcsLive[i].available = false;
    Serial.print(tcsNames[i]);
    Serial.print(F(" CH"));
    Serial.print(tcsChannels[i]);
    uint8_t id = 0;
    const bool selected = tcsSetMux(uint8_t(1U << tcsChannels[i]));
    const bool readId = selected && tcsReadRegisters(TCS_ID_REG, &id, 1);
    // Include the compatible IDs accepted by the Adafruit TCS34725 driver.
    const bool supported = id == 0x44 || id == 0x10 || id == 0x4D;
    const bool started = readId && supported && tcsStartContinuous();
    if (readId) {
      Serial.print(F(" ID=0x"));
      Serial.print(id, HEX);
    }
    Serial.println(started ? F(" ready") : F(" TCS initialization failed"));
    if (!started) ready = false;
  }
  return tcsSetMux(0) && ready;
}

uint16_t tcsWord(const uint8_t *bytes) {
  return uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8);
}

bool tcsReadColor(uint8_t sensor, TcsColor &color) {
  const bool selected = tcsSetMux(uint8_t(1U << tcsChannels[sensor]));
  // A single burst reads STATUS followed by the four little-endian channels.
  uint8_t bytes[9];
  const bool read = selected && tcsReadRegisters(TCS_STATUS_REG, bytes, sizeof(bytes));
  const bool deselected = tcsSetMux(0);
  if (!read || !deselected || !(bytes[0] & TCS_AVALID)) return false;

  color.clear = tcsWord(bytes + 1);
  color.red = tcsWord(bytes + 3);
  color.green = tcsWord(bytes + 5);
  color.blue = tcsWord(bytes + 7);
  return true;
}

bool tcsSaturated(const TcsColor &color) {
  return color.clear >= TCS_SATURATION || color.red >= TCS_SATURATION ||
         color.green >= TCS_SATURATION || color.blue >= TCS_SATURATION;
}

float tcsGreenDistanceSquared(uint8_t sensor, const TcsColor &color) {
  const float total = float(color.red) + color.green + color.blue;
  if (total == 0) return 1.0f;
  const GreenReference &reference = tcsGreen[sensor];
  const float redDifference = color.red / total - reference.red;
  const float greenDifference = color.green / total - reference.green;
  const float blueDifference = color.blue / total - reference.blue;
  return redDifference * redDifference + greenDifference * greenDifference +
         blueDifference * blueDifference;
}

bool tcsMatchesGreen(uint8_t sensor, const TcsColor &color) {
  if (color.clear == 0 || tcsSaturated(color)) return false;
  const float brightnessRatio = color.clear / tcsGreen[sensor].clear;
  if (brightnessRatio < TCS_MIN_BRIGHTNESS_RATIO ||
      brightnessRatio >= TCS_WHITE_BRIGHTNESS_RATIO) return false;
  return tcsGreenDistanceSquared(sensor, color) <=
         TCS_GREEN_TOLERANCE * TCS_GREEN_TOLERANCE;
}

void serviceTCS() {
  const unsigned long now = micros();
  if (!tcsHardwareReady || now - lastTcsPollUs < TCS_POLL_INTERVAL_US) return;
  lastTcsPollUs = now;
  const uint8_t sensor = tcsRoundRobinIndex;
  tcsRoundRobinIndex = (sensor + 1) % tcsCount;
  TcsReading &reading = tcsLive[sensor];
  TcsColor color;
  if (tcsReadColor(sensor, color)) {
    reading.color = color;
    reading.available = true;
    reading.white = tcsCalibrationComplete && !tcsMatchesGreen(sensor, reading.color);
    reading.lastUpdate = millis();
  } else if (millis() - reading.lastUpdate > TCS_MAX_AGE_MS) {
    reading.available = false;
  }
}

bool tcsReadingsReady() {
  if (!tcsCalibrationComplete) return false;
  const unsigned long now = millis();
  for (uint8_t i = 0; i < tcsCount; ++i) {
    if (!tcsLive[i].available || now - tcsLive[i].lastUpdate > TCS_MAX_AGE_MS) return false;
  }
  return true;
}

void calibrateTcsGreen() {
  stopAll();
  tcsCalibrationComplete = false;
  tcsHardwareReady = initializeTcsSensors();
  if (!tcsHardwareReady) {
    Serial.println(F("GREEN calibration unavailable. Check all four TCS sensors and send g."));
    return;
  }

  Serial.println(F("Calibrating GREEN. Keep all four sensors over the green field."));
  bool complete = true;
  for (uint8_t sensor = 0; sensor < tcsCount; ++sensor) {
    GreenReference sum = {};
    uint8_t accepted = 0;
    for (uint8_t sample = 0; sample < TCS_CAL_SAMPLES; ++sample) {
      delay(TCS_CAL_SPACING_MS);
      TcsColor color;
      if (!tcsReadColor(sensor, color) || color.clear == 0 || tcsSaturated(color)) continue;
      const float total = float(color.red) + color.green + color.blue;
      if (total == 0) continue;
      sum.red += color.red / total;
      sum.green += color.green / total;
      sum.blue += color.blue / total;
      sum.clear += color.clear;
      ++accepted;
    }
    Serial.print(tcsNames[sensor]);
    if (accepted < 2) {
      Serial.println(F(" calibration failed. Check light level and sensor connection."));
      complete = false;
      continue;
    }
    tcsGreen[sensor] = {sum.red / accepted, sum.green / accepted,
                        sum.blue / accepted, sum.clear / accepted};
    Serial.print(F(" green clear reference="));
    Serial.println(tcsGreen[sensor].clear, 1);
  }
  tcsCalibrationComplete = complete;
  Serial.println(complete ? F("GREEN calibration complete.") : F("GREEN calibration failed. AUTO held."));
}

void printTcsReadings() {
  const unsigned long now = millis();
  for (uint8_t i = 0; i < tcsCount; ++i) {
    Serial.print(tcsNames[i]);
    Serial.print(F(" CH"));
    Serial.print(tcsChannels[i]);
    const TcsReading &reading = tcsLive[i];
    if (!reading.available || now - reading.lastUpdate > TCS_MAX_AGE_MS) {
      Serial.println(F(" NO DATA"));
      continue;
    }
    Serial.print(F(" R="));
    Serial.print(reading.color.red);
    Serial.print(F(" G="));
    Serial.print(reading.color.green);
    Serial.print(F(" B="));
    Serial.print(reading.color.blue);
    Serial.print(F(" C="));
    Serial.print(reading.color.clear);
    if (!tcsCalibrationComplete) {
      Serial.println(F(" UNCALIBRATED"));
    } else {
      Serial.print(F(" C/green="));
      Serial.print(reading.color.clear / tcsGreen[i].clear, 2);
      Serial.print(F(" distance="));
      Serial.print(sqrtf(tcsGreenDistanceSquared(i, reading.color)), 3);
      if (tcsSaturated(reading.color)) Serial.print(F(" SATURATED"));
      Serial.println(reading.white ? F(" LINE") : F(" GREEN"));
    }
  }
}

// ------------------------------------------------------------
// White-line escape
// ------------------------------------------------------------

bool anyWhiteLine() {

  for (uint8_t i = 0; i < 4; i++) {

    if (tcsLive[i].available &&
        tcsLive[i].white &&
        (millis() - tcsLive[i].lastUpdate <= TCS_MAX_AGE_MS)) {

      return true;
    }
  }

  return false;
}

bool handleWhiteLineEscape() {

  if (!tcsHardwareReady) return false;
  if (!anyWhiteLine()) return false;

  int vy = 0;
  int vx = 0;

  // FRONT white -> go BACKWARD.
  if (tcsLive[0].available &&
      tcsLive[0].white) {
    vy -= LINE_ESCAPE_SPEED;
  }

  // BACK white -> go FORWARD.
  if (tcsLive[2].available &&
      tcsLive[2].white) {
    vy += LINE_ESCAPE_SPEED;
  }

  // LEFT white -> go RIGHT.
  if (tcsLive[1].available &&
      tcsLive[1].white) {
    vx += LINE_ESCAPE_SPEED;
  }

  // RIGHT white -> go LEFT.
  if (tcsLive[3].available &&
      tcsLive[3].white) {
    vx -= LINE_ESCAPE_SPEED;
  }

  vy = constrain(
    vy,
    -LINE_ESCAPE_SPEED,
    LINE_ESCAPE_SPEED
  );

  vx = constrain(
    vx,
    -LINE_ESCAPE_SPEED,
    LINE_ESCAPE_SPEED
  );

  // If opposite sensors cancel, use backward escape.
  if (vy == 0 && vx == 0) {
    vy = -LINE_ESCAPE_SPEED;
  }

  debugVX = vx;
  debugVY = vy;

  // Translation ONLY. No rotation.
  driveWithGyro(vy, vx);

  return true;
}


// ============================================================
// HITECHNIC IR SEEKER V2 BALL SENSOR
// AC direction + five detector strengths begin at register 0x49.
// Mega D20 = SDA, D21 = SCL; this sensor is on the main I2C bus.
// ============================================================

const uint8_t IR_ADDRESS = 0x08;
const uint8_t IR_AC_DIRECTION_REGISTER = 0x49;
uint8_t irLastTxStatus = 0xFF;
uint8_t irLastReceived = 0;
uint8_t irLastDirection = 0;
uint16_t irLastStrength = 0;
bool irLastTimeout = false;
uint16_t lineEscapeEvents = 0;
uint16_t tcsWaitEvents = 0;

// ============================================================
// SENSOR SMOOTHING
// ============================================================

float smooth[6] = {
  0, 0, 0, 0, 0, 0
};

float alpha = 0.1;

// Direction and five raw detector strengths for the Wi-Fi debug page.
int rawSensor[6] = {0, 0, 0, 0, 0, 0};

// Latest control values for the debug page.
int debugBallPosition = 2500;
int debugVX = 0;
int debugVY = 0;
bool debugBallDetected = false;

// ============================================================
// BALL DETECTION
// ============================================================

const int BALL_THRESHOLD = 1;  // Minimum derived HiTechnic AC strength.
const unsigned long IR_READ_INTERVAL_MS = 20;
const uint8_t BALL_CONFIRM_READS = 2;
const unsigned long BALL_LOST_GRACE_MS = 120;
const unsigned long BALL_FILTER_RESET_MS = 200;
const uint8_t IR_FAULT_READS = 5;

// ============================================================
// BALL POSITION
// ============================================================

int lastPosition = 2500;
unsigned long lastIRReadMs = 0;
unsigned long lastBallSeenMs = 0;
uint8_t validBallReads = 0;
uint8_t irFailedReads = 0;
bool sampledBallDetected = false;
int lastFollowVx = 0;

// ============================================================
// BALL FOLLOW SPEEDS
// ============================================================

const int BALL_FORWARD_SPEED = 170;
const int BALL_STRAFE_SPEED = 170;
const int BALL_STRAFE_MIN = 55;
const int CENTER_DEADZONE_ENTER = 350;
const int CENTER_DEADZONE_EXIT = 180;

// ============================================================
// SEARCH SETTINGS
// ============================================================

const int SEARCH_BACK_SPEED = 100;
const int SEARCH_BACK_STEER = 60;

// ============================================================
// SEARCH STATE
// ============================================================

unsigned long ballLostTime = 0;
bool ballWasVisible = false;

// ============================================================
// MPU6050
// ============================================================

#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_GYRO_ZOUT_H 0x47

const float GYRO_SENS = 131.0;

// ============================================================
// GYRO
// ============================================================

float gyroZBiasRaw = 0.0;

float currentYaw = 0.0;
float targetYaw = 0.0;

unsigned long lastYawUpdateMs = 0;

// ============================================================
// HEADING CONTROL
// ============================================================

float YAW_KP = 2.5;

const int MAX_YAW_CORRECTION = 80;

// ============================================================
// MPU WRITE
// ============================================================

void mpuWriteReg(
  uint8_t reg,
  uint8_t value
) {
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}

// ============================================================
// READ GYRO Z
// ============================================================

int16_t readGyroZRaw() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 2);

  if (Wire.available() >= 2) {

    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();

    return (int16_t)((highByte << 8) | lowByte);
  }

  return 0;
}

// ============================================================
// MPU INIT
// ============================================================

void mpuInit() {

  Wire.begin();
  Wire.setClock(100000);

  delay(100);

  mpuWriteReg(
    REG_PWR_MGMT_1,
    0x00
  );

  delay(50);

  mpuWriteReg(
    REG_GYRO_CONFIG,
    0x00
  );

  delay(50);
}

// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro() {

  Serial.println("================================");
  Serial.println("GYRO CALIBRATION");
  Serial.println("KEEP ROBOT COMPLETELY STILL");
  Serial.println("================================");

  delay(1000);

  const int samples = 800;

  long sum = 0;

  for (
    int i = 0;
    i < samples;
    i++
  ) {

    sum += readGyroZRaw();

    delay(3);
  }

  gyroZBiasRaw =
    (float)sum / samples;

  Serial.print("Gyro Z bias = ");
  Serial.println(gyroZBiasRaw);

  Serial.println("Calibration complete.");
  Serial.println();
}

// ============================================================
// UPDATE YAW
// ============================================================

void updateYaw() {

  unsigned long now = millis();

  float dt =
    (now - lastYawUpdateMs) / 1000.0;

  lastYawUpdateMs = now;

  if (
    dt <= 0 ||
    dt > 0.5
  ) {
    return;
  }

  int16_t raw =
    readGyroZRaw();

  float gyroZ =
    (raw - gyroZBiasRaw) / GYRO_SENS;

  currentYaw +=
    gyroZ * dt;
}

// ============================================================
// YAW ERROR
// ============================================================

float getYawError() {

  return currentYaw - targetYaw;
}

// ============================================================
// YAW CORRECTION
// ============================================================

int getYawCorrection() {

  float error =
    getYawError();

  float correction =
    YAW_KP * error;

  correction =
    constrain(
      correction,
      -MAX_YAW_CORRECTION,
      MAX_YAW_CORRECTION
    );

  return (int)correction;
}

// ============================================================
// MOTOR CONTROL
//
// Mega PWM pins use analogWrite(pin, duty) in the 0-255 range.
// ============================================================

void setMotorSpeed(
  Motor &m,
  int speed
) {

  speed *= m.dirSign;

  speed =
    constrain(
      speed,
      -255,
      255
    );

  if (speed > 0) {

    analogWrite(
      m.pinA,
      speed
    );

    analogWrite(
      m.pinB,
      0
    );
  }

  else if (speed < 0) {

    analogWrite(
      m.pinA,
      0
    );

    analogWrite(
      m.pinB,
      -speed
    );
  }

  else {

    analogWrite(
      m.pinA,
      0
    );

    analogWrite(
      m.pinB,
      0
    );
  }
}

// ============================================================
// STOP
// ============================================================

void stopAll() {

  setMotorSpeed(motorFL, 0);
  setMotorSpeed(motorFR, 0);
  setMotorSpeed(motorBL, 0);
  setMotorSpeed(motorBR, 0);
}

void setSelectedMotor(int8_t index, int speed) {
  if (index == 0) setMotorSpeed(motorFL, speed);
  else if (index == 1) setMotorSpeed(motorFR, speed);
  else if (index == 2) setMotorSpeed(motorBL, speed);
  else if (index == 3) setMotorSpeed(motorBR, speed);
}

// ============================================================
// OMNI DRIVE
// ============================================================

void drive(
  int vy,
  int vx,
  int w
) {

  int flSpeed =
    vy + vx + w;

  int frSpeed =
    vy - vx - w;

  int blSpeed =
    vy - vx + w;

  int brSpeed =
    vy + vx - w;

  int maxMag =
    max(
      max(
        abs(flSpeed),
        abs(frSpeed)
      ),
      max(
        abs(blSpeed),
        abs(brSpeed)
      )
    );

  if (maxMag > 255) {

    flSpeed =
      flSpeed * 255L / maxMag;

    frSpeed =
      frSpeed * 255L / maxMag;

    blSpeed =
      blSpeed * 255L / maxMag;

    brSpeed =
      brSpeed * 255L / maxMag;
  }

  setMotorSpeed(motorFL, flSpeed);
  setMotorSpeed(motorFR, frSpeed);
  setMotorSpeed(motorBL, blSpeed);
  setMotorSpeed(motorBR, brSpeed);
}

// ============================================================
// DRIVE WITH GYRO
// ============================================================

void driveWithGyro(
  int vy,
  int vx
) {

  updateYaw();

  int correction =
    getYawCorrection();

  drive(
    vy,
    vx,
    correction
  );
}

// ============================================================
// READ BALL
// ============================================================

bool readBall(
  int &position
) {
  const unsigned long now = millis();
  if (now - lastIRReadMs < IR_READ_INTERVAL_MS) {
    position = lastPosition;
    return sampledBallDetected;
  }
  lastIRReadMs = now;

  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(IR_AC_DIRECTION_REGISTER);
  irLastTxStatus = Wire.endTransmission();
  irLastReceived = 0;
  if (irLastTxStatus == 0) {
    irLastReceived = Wire.requestFrom(IR_ADDRESS, uint8_t(6));
  }
  irLastTimeout = Wire.getWireTimeoutFlag();
  if (irLastReceived == 6 && !irLastTimeout) {
    for (int i = 0; i < 6; i++) data[i] = Wire.read();
    irLastDirection = data[0];
  } else {
    irLastDirection = 0;
    while (Wire.available()) Wire.read();
  }
  const bool readOK = irLastTxStatus == 0 && irLastReceived == 6 &&
                      !irLastTimeout && data[0] <= 9;
  if (readOK) irFailedReads = 0;
  else if (irFailedReads < IR_FAULT_READS) ++irFailedReads;

  // Channel 0 is direction; channels 1-5 are the AC detectors.
  for (int i = 0; i < 6; i++) {
    rawSensor[i] = readOK ? data[i] : 0;
    smooth[i] = alpha * rawSensor[i] + (1.0 - alpha) * smooth[i];
  }

  uint8_t direction = readOK ? data[0] : 0;
  uint16_t strength = 0;
  if (direction >= 1 && direction <= 9) {
    uint8_t index = direction / 2 + 1;
    if (direction & 1)
      strength = data[index];
    else
      strength = (uint16_t(data[index - 1]) + data[index]) / 2;
  }
  irLastStrength = strength;

  if (!readOK || direction == 0 || strength < BALL_THRESHOLD) {
    validBallReads = 0;
    sampledBallDetected = false;
    debugBallDetected = false;
    position = lastPosition;
    debugBallPosition = position;
    return false;
  }

  // Direction 1 is left, 5 is front, 9 is right. Smooth at the fixed
  // 20 ms sample rate, then reset quickly when the ball reappears elsewhere.
  const int rawPosition = (9 - direction) * 625;
  position = (now - lastBallSeenMs > BALL_FILTER_RESET_MS)
               ? rawPosition
               : (lastPosition * 2 + rawPosition) / 3;
  lastPosition = position;
  lastBallSeenMs = now;
  if (validBallReads < BALL_CONFIRM_READS) ++validBallReads;
  sampledBallDetected = validBallReads >= BALL_CONFIRM_READS;
  debugBallDetected = sampledBallDetected;
  debugBallPosition = position;
  return sampledBallDetected;
}

void reportIrState(const __FlashStringHelper *state) {
  static unsigned long lastReportMs = 0;
  if (millis() - lastReportMs < 200) return;
  lastReportMs = millis();

  Serial.print(F("IR state="));
  Serial.print(state);
  Serial.print(F(" dir="));
  Serial.print(irLastDirection);
  Serial.print(F(" strength="));
  Serial.print(irLastStrength);
  Serial.print(F(" tx="));
  Serial.print(irLastTxStatus);
  Serial.print(F(" rx="));
  Serial.print(irLastReceived);
  Serial.print(F("/6 timeout="));
  Serial.print(irLastTimeout ? F("yes") : F("no"));
  Serial.print(F(" pos="));
  Serial.print(debugBallPosition);
  Serial.print(F(" vx="));
  Serial.print(debugVX);
  Serial.print(F(" vy="));
  Serial.print(debugVY);
  Serial.print(F(" lineEvents="));
  Serial.print(lineEscapeEvents);
  Serial.print(F(" tcsWaitEvents="));
  Serial.println(tcsWaitEvents);
  lineEscapeEvents = 0;
  tcsWaitEvents = 0;
}

// ============================================================
// BALL STRAFE
// ============================================================

int calculateBallStrafe(
  int position
) {
  static bool strafing = false;
  const int error = 2500 - position;
  const int magnitude = abs(error);
  if (magnitude <= CENTER_DEADZONE_EXIT) strafing = false;
  else if (magnitude >= CENTER_DEADZONE_ENTER) strafing = true;
  if (!strafing) return 0;

  const int speed = constrain(
    map(magnitude, CENTER_DEADZONE_EXIT, 2500,
        BALL_STRAFE_MIN, BALL_STRAFE_SPEED),
    BALL_STRAFE_MIN, BALL_STRAFE_SPEED);
  return error < 0 ? -speed : speed;
}

// ============================================================
// SEARCH
// ============================================================

void searchForBall() {
  // Keep reversing diagonally until IR sees the ball again. TCS line escape
  // takes priority in followBall(), and the gyro holds heading.
  const int awaySide = (lastPosition > 2500) ? 1 : -1;
  debugVX = awaySide * SEARCH_BACK_STEER;
  debugVY = -SEARCH_BACK_SPEED;
  driveWithGyro(debugVY, debugVX);
}

// ============================================================
// FOLLOW BALL
// ============================================================

void followBall() {
  const bool tcsReady = tcsReadingsReady();
  if (!tcsReady) stopAll();

  // Boundary protection comes first.
  // GREEN is learned automatically; anything else is WHITE.
  // Escape uses translation only (w = 0).

  const bool lineEscape = tcsReady && handleWhiteLineEscape();

  int position = 2500;

  bool ballDetected =
    readBall(position);

  debugBallDetected = ballDetected;
  debugBallPosition = position;

  if (!tcsReady) {
    if (tcsWaitEvents < 65535) ++tcsWaitEvents;
    reportIrState(F("TCS_WAIT"));
    return;
  }

  if (lineEscape) {
    if (lineEscapeEvents < 65535) ++lineEscapeEvents;
    reportIrState(F("LINE_ESCAPE"));
    return;
  }

  if (irFailedReads >= IR_FAULT_READS) {
    stopAll();
    debugVX = 0;
    debugVY = 0;
    reportIrState(F("IR_FAULT"));
    return;
  }

  if (!ballDetected) {

    if (ballWasVisible) {

      ballLostTime =
        millis();

      ballWasVisible =
        false;
    }

    if (millis() - ballLostTime < BALL_LOST_GRACE_MS) {
      debugVX = lastFollowVx / 2;
      debugVY = 0;
      driveWithGyro(debugVY, debugVX);
      reportIrState(F("GRACE"));
      return;
    }

    searchForBall();
    reportIrState(F("BACKOFF"));

    return;
  }

  ballWasVisible =
    true;

  const int vx = calculateBallStrafe(position);
  const int sideError = abs(2500 - position);
  // Mega int is 16-bit: promote before multiplying (170 * 2500 = 425000).
  const int vy = (int32_t)BALL_FORWARD_SPEED * (2500 - sideError) / 2500;
  lastFollowVx = vx;

  debugBallPosition = position;
  debugVX = vx;
  debugVY = vy;

  driveWithGyro(
    vy,
    vx
  );
  reportIrState(F("FOLLOW"));
}

// ============================================================
// OPTIONAL SERIAL CONTROLS (115200 baud, Newline)
// ============================================================

bool manualMode = false;
int8_t motorTestIndex = -1;
unsigned long motorTestStartedMs = 0;
const unsigned long MOTOR_TEST_MS = 700;
const int MOTOR_TEST_PWM = 100;
const int FORWARD_TEST_PWM = 170;
bool forwardTestActive = false;
unsigned long forwardTestStartedMs = 0;
char commandLine[12] = {0};
uint8_t commandLength = 0;

void printMotorDirections() {
  Serial.print(F("Directions FL/FR/BL/BR: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print((motorReverseMask & (1U << i)) ? F("reversed") : F("normal"));
    if (i < 3) Serial.print('/');
  }
  Serial.println();
}

void printCommands() {
  Serial.println(F("Commands: a=AUTO, s=STOP, c=gyro recalibrate, g=green recalibrate"));
  Serial.println(F("l=show TCS readings, p=show directions, d1..d4=flip and save FL/FR/BL/BR"));
  Serial.println(F("f=test all four motors forward for 700 ms; t1f..t4f or t1r..t4r=test one motor"));
}

void processSerialCommand(const char* command) {
  // Any new command ends an in-progress forward test immediately.
  if (forwardTestActive) {
    forwardTestActive = false;
    stopAll();
  }
  if (strcmp(command, "a") == 0) {
    stopAll();
    motorTestIndex = -1;
    ballLostTime = millis();
    ballWasVisible = false;
    manualMode = false;
    Serial.println(tcsReadingsReady() ? F("AUTO") : F("AUTO waiting for four calibrated TCS readings."));
  } else if (strcmp(command, "s") == 0) {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    Serial.println(F("STOPPED (MANUAL)"));
  } else if (strcmp(command, "p") == 0) {
    printMotorDirections();
  } else if (strcmp(command, "l") == 0) {
    printTcsReadings();
  } else if (strcmp(command, "f") == 0) {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    drive(FORWARD_TEST_PWM, 0, 0);
    forwardTestStartedMs = millis();
    forwardTestActive = true;
    Serial.println(F("Pure forward test: vy=170 vx=0 yaw correction=0, auto-stop in 700 ms."));
    printMotorDirections();
  } else if (strcmp(command, "g") == 0) {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    calibrateTcsGreen();
    Serial.println(F("Send a to resume AUTO."));
  } else if (strcmp(command, "c") == 0) {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    calibrateGyro();
    currentYaw = 0.0f;
    targetYaw = 0.0f;
    lastYawUpdateMs = millis();
    Serial.println(F("Send a to resume AUTO."));
  } else if (command[0] == 'd' && command[1] >= '1' &&
             command[1] <= '4' && command[2] == '\0') {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    uint8_t mask = motorReverseMask ^ (1U << (command[1] - '1'));
    saveMotorDirections(mask);
    printMotorDirections();
    Serial.println(F("Saved in EEPROM. Send a to resume AUTO."));
  } else if (command[0] == 't' && command[1] >= '1' &&
             command[1] <= '4' &&
             (command[2] == 'f' || command[2] == 'r') &&
             command[3] == '\0') {
    manualMode = true;
    stopAll();
    motorTestIndex = command[1] - '1';
    int speed = (command[2] == 'f') ? MOTOR_TEST_PWM : -MOTOR_TEST_PWM;
    setSelectedMotor(motorTestIndex, speed);
    motorTestStartedMs = millis();
    Serial.println(F("Testing one motor; auto-stop in 700 ms."));
  } else {
    printCommands();
  }
}

void serviceSerialCommands() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      commandLine[commandLength] = '\0';
      if (commandLength > 0) processSerialCommand(commandLine);
      commandLength = 0;
    } else if (commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = ch;
    } else {
      commandLength = 0;
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  // ----------------------------------------------------------
  // MOTOR PINS
  // ----------------------------------------------------------

  const Motor *const motors[] = {&motorFL, &motorFR, &motorBL, &motorBR};
  for (const Motor *motor : motors) {
    digitalWrite(motor->pinA, LOW);
    digitalWrite(motor->pinB, LOW);
    pinMode(motor->pinA, OUTPUT);
    pinMode(motor->pinB, OUTPUT);
  }
  stopAll();

  Serial.begin(115200);
  delay(500);
  loadMotorDirections();

  // ----------------------------------------------------------
  // I2C / TCS34725 / MPU
  // ----------------------------------------------------------

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);

  // ----------------------------------------------------------
  // 16-LED GROUND ILLUMINATION
  // ----------------------------------------------------------

  tcsPixels.begin();
  tcsPixels.setBrightness(TCS_LIGHT_BRIGHTNESS);
  tcsPixels.fill(tcsPixels.Color(255, 255, 255));
  tcsPixels.show();
  delay(120);

  // Place all four sensors over green before powering the robot on.
  calibrateTcsGreen();

  // ----------------------------------------------------------
  // MPU
  // ----------------------------------------------------------

  mpuInit();

  calibrateGyro();

  // ----------------------------------------------------------
  // HEADING
  // ----------------------------------------------------------

  currentYaw = 0.0;
  targetYaw = 0.0;

  lastYawUpdateMs =
    millis();

  lastPosition =
    2500;
  debugBallPosition = 2500;
  debugBallDetected = false;
  debugVX = 0;
  debugVY = 0;

  ballLostTime =
    millis();

  ballWasVisible =
    false;

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "HITECHNIC IR OMNI SOCCER ROBOT"
  );

  Serial.println(
    "ARDUINO MEGA 2560 PORT"
  );

  Serial.println(
    "TCS34725 LINE SENSORS"
  );

  Serial.println(
    "======================================"
  );

  Serial.println("HiTechnic IR Seeker V2 = 0x08 (AC mode)");
  Serial.println("Direction 1 = left, 5 = forward, 9 = right");

  Serial.println(
    "I2C SDA = D20"
  );

  Serial.println(
    "I2C SCL = D21"
  );

  Serial.println(
    "TCA9548A = 0x70"
  );

  Serial.println(
    "TCS34725 = 0x29 / expected ID 0x44"
  );

  Serial.println(
    "TCS CH0 FRONT / CH1 LEFT / CH3 BACK / CH2 RIGHT"
  );

  Serial.println(
    "NeoPixel = D15 / 16 LEDs / BRIGHT WHITE"
  );

  Serial.println(
    "TCS integration = 9.6 ms / l shows line readings"
  );

  Serial.println(
    "Lost ball = continuous diagonal reverse until IR sees ball"
  );

  Serial.println(
    "Starting..."
  );

  ballLostTime =
    millis();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  serviceSerialCommands();
  serviceTCS();

  if (manualMode) {
    if (forwardTestActive && millis() - forwardTestStartedMs >= MOTOR_TEST_MS) {
      stopAll();
      forwardTestActive = false;
      Serial.println(F("Forward test stopped; still MANUAL. Send a for AUTO."));
    }
    if (motorTestIndex >= 0 && millis() - motorTestStartedMs >= MOTOR_TEST_MS) {
      stopAll();
      motorTestIndex = -1;
      Serial.println(F("Motor test stopped; still MANUAL."));
    }
  } else {
    followBall();
  }
}
