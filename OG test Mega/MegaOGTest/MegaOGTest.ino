/*
  OG test soccer robot — Arduino Mega 2560 port.
  Same ball following, sideways/forward search, gyro heading hold, and
  APDS white-line escape as the ESP32 sketch. No Wi-Fi, web page, or OTA.
  The robot starts in AUTO after startup calibration; Serial is optional.
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>

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

#define FL_PWM_A 6
#define FL_PWM_B 7

#define FR_PWM_A 4
#define FR_PWM_B 5

#define BL_PWM_A 11
#define BL_PWM_B 10

#define BR_PWM_A 2
#define BR_PWM_B 3

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


// Forward declarations for globals/functions used by the APDS subsystem.
void stopAll();
void driveWithGyro(int vy, int vx);

// ============================================================
// APDS9960 x4 + TCA9548A + 16 NEOPIXEL
//
// GREEN-ONLY CALIBRATION:
//   GREEN = matches calibrated normalized RGB signature
//   WHITE = anything that does NOT match GREEN
//
// Your APDS modules report ID 0x9E, so direct APDS registers are
// used instead of the Adafruit APDS9960 library's strict ID check.
//
// TCA channels:
//   CH3 = FRONT
//   CH2 = LEFT
//   CH1 = BACK
//   CH0 = RIGHT
//
// NeoPixels:
//   Mega D15, 16 LEDs, bright WHITE for ground illumination.
// ============================================================

#define TCA_ADDR           0x70
#define APDS_ADDR          0x39

#define APDS_CH_FRONT      3
#define APDS_CH_LEFT       2
#define APDS_CH_BACK       1
#define APDS_CH_RIGHT      0

#define APDS_ID_REG        0x92
#define APDS_ENABLE_REG    0x80
#define APDS_ATIME_REG     0x81
#define APDS_CONTROL_REG   0x8F
#define APDS_STATUS_REG    0x93
#define APDS_CDATAL_REG    0x94
#define APDS_RDATAL_REG    0x96
#define APDS_GDATAL_REG    0x98
#define APDS_BDATAL_REG    0x9A

#define APDS_STATUS_AVALID 0x01
#define APDS_ENABLE_PON    0x01
#define APDS_ENABLE_AEN    0x02
#define APDS_GAIN_4X       0x01

// Fast runtime/calibration integration, approximately 11 ms.
#define APDS_ATIME         0xFC

// Very fast startup calibration.
#define APDS_CAL_SAMPLES   4

// Tested starting value from the isolated system.
float apdsGreenTolerance = 0.08f;

// Bright white ground illumination.
uint8_t apdsNeoBrightness = 255;

Adafruit_NeoPixel apdsPixels(
  16,
  15,
  NEO_GRB + NEO_KHZ800
);

const uint8_t apdsChannel[4] = {
  APDS_CH_FRONT,
  APDS_CH_LEFT,
  APDS_CH_BACK,
  APDS_CH_RIGHT
};

const char* apdsName[4] = {
  "FRONT",
  "LEFT",
  "BACK",
  "RIGHT"
};

struct APDSGreenCal {
  float rNorm;
  float gNorm;
  float bNorm;
  float clear;
  bool valid;
};

struct APDSLive {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
  float greenDistance;
  float brightnessRatio;
  bool green;
  bool white;
  bool available;
  unsigned long lastUpdate;
};

APDSGreenCal apdsGreen[4];
APDSLive apdsLive[4];

bool apdsHardwareReady = false;
bool apdsCalibrationRunning = false;
bool apdsCalibrationComplete = false;
String apdsCalibrationMessage = "Starting automatic GREEN calibration...";

uint8_t apdsRoundRobinIndex = 0;
unsigned long lastAPDSPollUs = 0;

// Non-blocking runtime polling.
const unsigned long APDS_POLL_INTERVAL_US = 2500;

// Broad brightness sanity range ONLY.
// This is not white calibration.
const float APDS_MIN_BRIGHTNESS_RATIO = 0.30f;
const float APDS_MAX_BRIGHTNESS_RATIO = 3.00f;

// Translation-only white-line escape speed.
const int LINE_ESCAPE_SPEED = 165;

// ------------------------------------------------------------
// APDS ground illumination
// ------------------------------------------------------------

void apdsGroundLightOn() {
  apdsPixels.setBrightness(apdsNeoBrightness);

  for (uint8_t i = 0; i < 16; i++) {
    apdsPixels.setPixelColor(i, 255, 255, 255);
  }

  apdsPixels.show();
}

// ------------------------------------------------------------
// TCA9548A
// ------------------------------------------------------------

bool apdsSelectTCA(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);

  bool ok = (Wire.endTransmission() == 0);

  delayMicroseconds(250);
  return ok;
}

void apdsDisableTCA() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ------------------------------------------------------------
// Direct APDS I2C
// ------------------------------------------------------------

bool apdsWrite8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool apdsRead8(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(uint8_t(APDS_ADDR), uint8_t(1)) != 1) return false;
  if (!Wire.available()) return false;

  value = Wire.read();
  return true;
}

bool apdsRead16(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(uint8_t(APDS_ADDR), uint8_t(2)) != 2) return false;

  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();

  value = ((uint16_t)hi << 8) | lo;
  return true;
}

// ------------------------------------------------------------
// APDS initialization
// ------------------------------------------------------------

bool apdsInitOne() {
  if (!apdsWrite8(APDS_ENABLE_REG, 0x00)) return false;
  if (!apdsWrite8(APDS_ATIME_REG, APDS_ATIME)) return false;
  if (!apdsWrite8(APDS_CONTROL_REG, APDS_GAIN_4X)) return false;
  if (!apdsWrite8(APDS_ENABLE_REG, APDS_ENABLE_PON | APDS_ENABLE_AEN)) return false;

  delay(15);
  return true;
}

// ------------------------------------------------------------
// Blocking color read - calibration only
// ------------------------------------------------------------

bool apdsReadColorFast(
  uint16_t &c,
  uint16_t &r,
  uint16_t &g,
  uint16_t &b
) {
  uint8_t status = 0;
  unsigned long start = micros();

  while ((micros() - start) < 18000UL) {
    if (!apdsRead8(APDS_STATUS_REG, status)) return false;

    if (status & APDS_STATUS_AVALID) break;

    delayMicroseconds(250);
  }

  if (!(status & APDS_STATUS_AVALID)) return false;

  if (!apdsRead16(APDS_CDATAL_REG, c)) return false;
  if (!apdsRead16(APDS_RDATAL_REG, r)) return false;
  if (!apdsRead16(APDS_GDATAL_REG, g)) return false;
  if (!apdsRead16(APDS_BDATAL_REG, b)) return false;

  return true;
}

// ------------------------------------------------------------
// Non-blocking color read - runtime
// ------------------------------------------------------------

bool apdsReadColorIfReady(
  uint16_t &c,
  uint16_t &r,
  uint16_t &g,
  uint16_t &b
) {
  uint8_t status = 0;

  if (!apdsRead8(APDS_STATUS_REG, status)) return false;
  if (!(status & APDS_STATUS_AVALID)) return false;

  if (!apdsRead16(APDS_CDATAL_REG, c)) return false;
  if (!apdsRead16(APDS_RDATAL_REG, r)) return false;
  if (!apdsRead16(APDS_GDATAL_REG, g)) return false;
  if (!apdsRead16(APDS_BDATAL_REG, b)) return false;

  return true;
}

// ------------------------------------------------------------
// Green classifier
// ------------------------------------------------------------

float apdsGreenDistance(
  uint8_t sensor,
  uint16_t r,
  uint16_t g,
  uint16_t b
) {
  if (!apdsGreen[sensor].valid) return 999.0f;

  float total = (float)r + (float)g + (float)b;
  if (total <= 0.0f) return 999.0f;

  float rn = (float)r / total;
  float gn = (float)g / total;
  float bn = (float)b / total;

  float dr = rn - apdsGreen[sensor].rNorm;
  float dg = gn - apdsGreen[sensor].gNorm;
  float db = bn - apdsGreen[sensor].bNorm;

  return sqrtf(dr * dr + dg * dg + db * db);
}

bool apdsIsGreen(
  uint8_t sensor,
  uint16_t c,
  uint16_t r,
  uint16_t g,
  uint16_t b
) {
  if (!apdsGreen[sensor].valid) return false;
  if (c == 0 || apdsGreen[sensor].clear <= 0.0f) return false;

  float brightnessRatio =
    (float)c / apdsGreen[sensor].clear;

  // Only prevents wildly different brightness from matching.
  // It is NOT a white threshold.
  if (brightnessRatio < APDS_MIN_BRIGHTNESS_RATIO ||
      brightnessRatio > APDS_MAX_BRIGHTNESS_RATIO) {
    return false;
  }

  return apdsGreenDistance(sensor, r, g, b)
         <= apdsGreenTolerance;
}

// ------------------------------------------------------------
// Runtime APDS update
// ------------------------------------------------------------

void apdsUpdateOne(uint8_t sensor) {
  if (!apdsHardwareReady) return;

  if (!apdsSelectTCA(apdsChannel[sensor])) {
    apdsLive[sensor].available = false;
    return;
  }

  uint16_t c, r, g, b;

  if (!apdsReadColorIfReady(c, r, g, b)) {
    if (millis() - apdsLive[sensor].lastUpdate > 150) {
      apdsLive[sensor].available = false;
    }
    return;
  }

  apdsLive[sensor].available = true;

  apdsLive[sensor].c = c;
  apdsLive[sensor].r = r;
  apdsLive[sensor].g = g;
  apdsLive[sensor].b = b;

  apdsLive[sensor].greenDistance =
    apdsGreenDistance(sensor, r, g, b);

  if (apdsGreen[sensor].valid &&
      apdsGreen[sensor].clear > 0.0f) {

    apdsLive[sensor].brightnessRatio =
      (float)c / apdsGreen[sensor].clear;

  } else {
    apdsLive[sensor].brightnessRatio = 0;
  }

  apdsLive[sensor].green =
    apdsIsGreen(sensor, c, r, g, b);

  // Requested behavior:
  // anything that is not green is treated as WHITE.
  apdsLive[sensor].white =
    !apdsLive[sensor].green;

  apdsLive[sensor].lastUpdate = millis();
}

void serviceAPDS() {
  if (!apdsHardwareReady) return;

  unsigned long now = micros();

  if (now - lastAPDSPollUs < APDS_POLL_INTERVAL_US) return;

  lastAPDSPollUs = now;

  apdsUpdateOne(apdsRoundRobinIndex);

  apdsRoundRobinIndex++;

  if (apdsRoundRobinIndex >= 4) {
    apdsRoundRobinIndex = 0;
  }

  // Leave the selected TCA channel active.
  // The MPU6050 remains on the main I2C bus and is still accessible.
}

// ------------------------------------------------------------
// FAST GREEN-ONLY CALIBRATION
// ------------------------------------------------------------

void calibrateAPDSGreen() {
  if (!apdsHardwareReady || apdsCalibrationRunning) return;

  apdsCalibrationRunning = true;
  apdsCalibrationComplete = false;

  apdsCalibrationMessage =
    "Calibrating GREEN... keep all four sensors over GREEN.";

  Serial.println();
  Serial.println("======================================");
  Serial.println("FAST APDS GREEN CALIBRATION");
  Serial.println("======================================");
  Serial.println("GREEN ONLY - EVERYTHING ELSE = WHITE");

  // Safe calibration: robot must not move.
  stopAll();

  apdsGroundLightOn();

  // Small, fixed stabilization delay.
  delay(120);

  unsigned long start = millis();

  for (uint8_t sensor = 0; sensor < 4; sensor++) {

    apdsGreen[sensor].valid = false;

    if (!apdsSelectTCA(apdsChannel[sensor])) {
      Serial.print(apdsName[sensor]);
      Serial.println(" TCA SELECT FAILED");
      continue;
    }

    float sumRN = 0.0f;
    float sumGN = 0.0f;
    float sumBN = 0.0f;
    float sumC  = 0.0f;

    uint8_t good = 0;

    for (uint8_t sample = 0;
         sample < APDS_CAL_SAMPLES;
         sample++) {

      uint16_t c, r, g, b;

      if (!apdsReadColorFast(c, r, g, b)) continue;

      float total =
        (float)r + (float)g + (float)b;

      if (total <= 0.0f) continue;

      sumRN += (float)r / total;
      sumGN += (float)g / total;
      sumBN += (float)b / total;
      sumC  += (float)c;

      good++;
    }

    if (good >= 2) {

      apdsGreen[sensor].rNorm = sumRN / good;
      apdsGreen[sensor].gNorm = sumGN / good;
      apdsGreen[sensor].bNorm = sumBN / good;
      apdsGreen[sensor].clear  = sumC / good;
      apdsGreen[sensor].valid  = true;

      Serial.print(apdsName[sensor]);
      Serial.print(": R=");
      Serial.print(apdsGreen[sensor].rNorm, 4);
      Serial.print(" G=");
      Serial.print(apdsGreen[sensor].gNorm, 4);
      Serial.print(" B=");
      Serial.print(apdsGreen[sensor].bNorm, 4);
      Serial.print(" C=");
      Serial.println(apdsGreen[sensor].clear, 1);

    } else {

      Serial.print(apdsName[sensor]);
      Serial.println(": CALIBRATION FAILED");
    }
  }

  apdsDisableTCA();
  apdsGroundLightOn();

  apdsCalibrationComplete = true;

  for (uint8_t i = 0; i < 4; i++) {
    if (!apdsGreen[i].valid) apdsCalibrationComplete = false;
  }

  apdsCalibrationRunning = false;

  unsigned long elapsed =
    millis() - start;

  if (apdsCalibrationComplete) {
    apdsCalibrationMessage =
      "GREEN calibration complete.";
  } else {
    apdsCalibrationMessage =
      "GREEN calibration completed with sensor errors.";
  }

  Serial.print("APDS calibration time = ");
  Serial.print(elapsed);
  Serial.println(" ms");

  Serial.println(apdsCalibrationMessage);
  Serial.println("======================================");
}

// ------------------------------------------------------------
// White-line escape
// ------------------------------------------------------------

bool anyWhiteLine() {

  for (uint8_t i = 0; i < 4; i++) {

    if (apdsLive[i].available &&
        apdsLive[i].white &&
        (millis() - apdsLive[i].lastUpdate <= 150)) {

      return true;
    }
  }

  return false;
}

bool handleWhiteLineEscape() {

  if (!apdsHardwareReady) return false;
  if (!anyWhiteLine()) return false;

  int vy = 0;
  int vx = 0;

  // FRONT white -> go BACKWARD.
  if (apdsLive[0].available &&
      apdsLive[0].white) {
    vy -= LINE_ESCAPE_SPEED;
  }

  // BACK white -> go FORWARD.
  if (apdsLive[2].available &&
      apdsLive[2].white) {
    vy += LINE_ESCAPE_SPEED;
  }

  // LEFT white -> go RIGHT.
  if (apdsLive[1].available &&
      apdsLive[1].white) {
    vx += LINE_ESCAPE_SPEED;
  }

  // RIGHT white -> go LEFT.
  if (apdsLive[3].available &&
      apdsLive[3].white) {
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

// ============================================================
// BALL POSITION
// ============================================================

int lastPosition = 2500;

// ============================================================
// BALL FOLLOW SPEEDS
// ============================================================

const int BALL_FORWARD_SPEED = 150;
const int BALL_STRAFE_SPEED = 130;
const int CENTER_DEADZONE = 250;

// ============================================================
// SEARCH SETTINGS
// ============================================================

const int SEARCH_SIDE_SPEED = 125;
const int SEARCH_FORWARD_SPEED = 110;

// Side, forward, double-side, forward, side: one sideways sweep.
const unsigned long SIDE_SEARCH_TIME = 650;
const unsigned long FORWARD_SEARCH_TIME = 600;
const unsigned long SEARCH_CYCLE_TIME =
  4UL * SIDE_SEARCH_TIME + 2UL * FORWARD_SEARCH_TIME;

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
      flSpeed * 255 / maxMag;

    frSpeed =
      frSpeed * 255 / maxMag;

    blSpeed =
      blSpeed * 255 / maxMag;

    brSpeed =
      brSpeed * 255 / maxMag;
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
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  bool readOK = false;

  // Use the same AC-mode transaction as the working HiTechnic IR test.
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(IR_AC_DIRECTION_REGISTER);
  if (Wire.endTransmission() == 0 &&
      Wire.requestFrom(IR_ADDRESS, uint8_t(6)) == 6) {
    for (int i = 0; i < 6; i++) data[i] = Wire.read();
    readOK = (data[0] <= 9);
  } else {
    while (Wire.available()) Wire.read();
  }

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

  if (direction == 0 || strength < BALL_THRESHOLD) {
    debugBallDetected = false;
    position = (lastPosition > 2500) ? 5000 : 0;
    debugBallPosition = position;
    return false;
  }

  // Preserve the existing 0..5000 ball-position and strafe logic.
  // HiTechnic 1 is left and 9 is right; the old position scale is reversed.
  position = (9 - direction) * 625;

  // ORIGINAL 50/50 SMOOTHING
  position = (lastPosition * 5 + position * 5) / 10;
  lastPosition = position;
  debugBallDetected = true;
  debugBallPosition = position;
  return true;
}

// ============================================================
// BALL STRAFE
// ============================================================

int calculateBallStrafe(
  int position
) {

  int error =
    2500 - position;

  if (
    abs(error) <
    CENTER_DEADZONE
  ) {

    return 0;
  }

  int vx =
    map(
      error,
      -2500,
      2500,
      -BALL_STRAFE_SPEED,
      BALL_STRAFE_SPEED
    );

  vx =
    constrain(
      vx,
      -BALL_STRAFE_SPEED,
      BALL_STRAFE_SPEED
    );

  return vx;
}

// ============================================================
// SEARCH
// ============================================================

void searchForBall() {
  // followBall() already checked the IR sensor this loop. Search keeps
  // translating and checks for the ball again on the next loop.
  unsigned long elapsed =
    (millis() - ballLostTime) % SEARCH_CYCLE_TIME;

  // The old position scale is reversed: >2500 means the ball was left.
  int side = (lastPosition > 2500) ? -1 : 1;
  int vy = 0;
  int vx = 0;

  if (elapsed < SIDE_SEARCH_TIME) {
    // First pass toward the ball's last known side.
    vx = side * SEARCH_SIDE_SPEED;
  } else if (elapsed < SIDE_SEARCH_TIME + FORWARD_SEARCH_TIME) {
    vy = SEARCH_FORWARD_SPEED;
  } else if (elapsed < 3UL * SIDE_SEARCH_TIME + FORWARD_SEARCH_TIME) {
    // Cross the full width in the other direction.
    vx = -side * SEARCH_SIDE_SPEED;
  } else if (elapsed < 3UL * SIDE_SEARCH_TIME + 2UL * FORWARD_SEARCH_TIME) {
    vy = SEARCH_FORWARD_SPEED;
  } else {
    // Return sideways to the starting lane before the next sweep.
    vx = side * SEARCH_SIDE_SPEED;
  }

  debugVX = vx;
  debugVY = vy;
  driveWithGyro(vy, vx);
}

// ============================================================
// FOLLOW BALL
// ============================================================

void followBall() {

  // Boundary protection comes first.
  // GREEN is learned automatically; anything else is WHITE.
  // Escape uses translation only (w = 0).

  if (handleWhiteLineEscape()) {
    return;
  }

  int position = 2500;

  bool ballDetected =
    readBall(position);

  debugBallDetected = ballDetected;
  debugBallPosition = position;

  if (!ballDetected) {

    if (ballWasVisible) {

      ballLostTime =
        millis();

      ballWasVisible =
        false;
    }

    searchForBall();

    return;
  }

  ballWasVisible =
    true;

  int vx =
    calculateBallStrafe(
      position
    );

  int vy =
    BALL_FORWARD_SPEED;

  debugBallPosition = position;
  debugVX = vx;
  debugVY = vy;

  driveWithGyro(
    vy,
    vx
  );

  // DEBUG (Mega UART is slower than the ESP32 USB serial path.)
  static unsigned long lastBallReportMs = 0;
  if (millis() - lastBallReportMs < 200) return;
  lastBallReportMs = millis();

  Serial.print("BALL | ");

  for (
    int i = 0;
    i < 6;
    i++
  ) {

    Serial.print(i == 0 ? "Dir" : "S");
    if (i != 0) Serial.print(i);
    Serial.print(":");
    Serial.print((int)smooth[i]);

    if (i < 5)
      Serial.print(" ");
  }

  Serial.print(" | POS=");
  Serial.print(position);

  Serial.print(" | VX=");
  Serial.print(vx);

  Serial.print(" | VY=");
  Serial.print(vy);

  Serial.print(" | YAW=");
  Serial.println(currentYaw);
}

// ============================================================
// OPTIONAL SERIAL CONTROLS (115200 baud, Newline)
// ============================================================

bool manualMode = false;
int8_t motorTestIndex = -1;
unsigned long motorTestStartedMs = 0;
const unsigned long MOTOR_TEST_MS = 700;
const int MOTOR_TEST_PWM = 100;
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
  Serial.println(F("p=show directions, d1..d4=flip and save FL/FR/BL/BR"));
  Serial.println(F("t1f..t4f or t1r..t4r=test one motor for 700 ms"));
}

void processSerialCommand(const char* command) {
  if (strcmp(command, "a") == 0) {
    stopAll();
    motorTestIndex = -1;
    ballLostTime = millis();
    ballWasVisible = false;
    manualMode = false;
    Serial.println(F("AUTO"));
  } else if (strcmp(command, "s") == 0) {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    Serial.println(F("STOPPED (MANUAL)"));
  } else if (strcmp(command, "p") == 0) {
    printMotorDirections();
  } else if (strcmp(command, "g") == 0) {
    manualMode = true;
    motorTestIndex = -1;
    stopAll();
    calibrateAPDSGreen();
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

  Serial.begin(115200);

  delay(500);

  // ----------------------------------------------------------
  // MOTOR PINS
  // ----------------------------------------------------------

  pinMode(motorFL.pinA, OUTPUT);
  pinMode(motorFL.pinB, OUTPUT);

  pinMode(motorFR.pinA, OUTPUT);
  pinMode(motorFR.pinB, OUTPUT);

  pinMode(motorBL.pinA, OUTPUT);
  pinMode(motorBL.pinB, OUTPUT);

  pinMode(motorBR.pinA, OUTPUT);
  pinMode(motorBR.pinB, OUTPUT);

  stopAll();
  loadMotorDirections();

  // ----------------------------------------------------------
  // I2C / APDS / MPU
  // ----------------------------------------------------------

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);

  // ----------------------------------------------------------
  // 16-LED GROUND ILLUMINATION
  // ----------------------------------------------------------

  apdsPixels.begin();
  apdsPixels.clear();
  apdsPixels.show();
  apdsNeoBrightness = 255;
  apdsGroundLightOn();

  // ----------------------------------------------------------
  // APDS / TCA INITIALIZATION
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("======================================");
  Serial.println("APDS / TCA INITIALIZATION");
  Serial.println("======================================");

  Wire.beginTransmission(TCA_ADDR);

  if (Wire.endTransmission() != 0) {

    Serial.println("TCA9548A NOT FOUND at 0x70.");
    apdsHardwareReady = false;

  } else {

    apdsHardwareReady = true;

    for (uint8_t i = 0; i < 4; i++) {

      Serial.print("Initializing ");
      Serial.print(apdsName[i]);
      Serial.print(" CH");
      Serial.print(apdsChannel[i]);
      Serial.print(" ... ");

      if (!apdsSelectTCA(apdsChannel[i])) {
        Serial.println("TCA ERROR");
        apdsHardwareReady = false;
        continue;
      }

      uint8_t id = 0;

      if (!apdsRead8(APDS_ID_REG, id)) {
        Serial.println("ID ERROR");
        apdsHardwareReady = false;
        continue;
      }

      if (!apdsInitOne()) {
        Serial.println("COLOR ENGINE ERROR");
        apdsHardwareReady = false;
        continue;
      }

      apdsGreen[i].valid = false;
      apdsLive[i].available = false;

      Serial.print("OK ID=0x");
      if (id < 0x10) Serial.print("0");
      Serial.println(id, HEX);
    }

    apdsDisableTCA();
  }

  // ----------------------------------------------------------
  // Initial HiTechnic readings are zero until the first I2C read.
  for (int i = 0; i < 6; i++) {
    rawSensor[i] = 0;
    smooth[i] = 0;
  }

  // ----------------------------------------------------------
  // FAST AUTOMATIC APDS GREEN CALIBRATION
  //
  // Place the robot over GREEN before powering it on.
  // No countdown: this is intentionally very fast.
  // ----------------------------------------------------------

  if (apdsHardwareReady) {

    Serial.println();
    Serial.println("======================================");
    Serial.println("AUTOMATIC GREEN CALIBRATION");
    Serial.println("KEEP ROBOT OVER GREEN");
    Serial.println("======================================");

    calibrateAPDSGreen();

  } else {

    Serial.println(
      "APDS unavailable - white-line detection disabled."
    );
  }

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
    "STABLE VERSION"
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
    "APDS = 0x39 / ID 0x9E"
  );

  Serial.println(
    "APDS CH3 FRONT / CH2 LEFT / CH1 BACK / CH0 RIGHT"
  );

  Serial.println(
    "NeoPixel = D15 / 16 LEDs / BRIGHT WHITE"
  );

  Serial.println(
    "Green tolerance = 0.08"
  );

  Serial.println(
    "Search = sideways sweep with forward steps"
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
  serviceAPDS();

  if (manualMode) {
    if (motorTestIndex >= 0 && millis() - motorTestStartedMs >= MOTOR_TEST_MS) {
      stopAll();
      motorTestIndex = -1;
      Serial.println(F("Motor test stopped; still MANUAL."));
    }
  } else {
    followBall();
  }

  delay(2);
}
