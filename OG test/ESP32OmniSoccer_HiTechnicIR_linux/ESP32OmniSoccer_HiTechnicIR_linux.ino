/*
  ============================================================
  OMNI SOCCER ROBOT - ESP32 PORT
  Based on the stable Arduino Mega 2560 version

  BOARD:
    ESP32-WROOM 30-pin / ESP32-CP2102-MicroUSB-30Pins

  MOTOR DRIVER:
    L298N
    2 PWM pins per motor

  BALL SENSOR:
    HiTechnic IR Seeker V2, AC mode, I2C address 0x08.
    Direction 1 = left, 5 = ahead, 9 = right, 0 = no ball.

  SEARCH:
    - Sideways toward the last seen ball direction
    - Sweep the other side, stepping forward between passes
    - Repeat without turning or reversing toward the own goal

  I2C:
    SDA = GPIO21
    SCL = GPIO22

  MPU6050:
    Address = 0x68
*/

// ============================================================
// I2C
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ============================================================
// WIFI DEBUGGING
// ============================================================
// Enter the Wi-Fi network that the robot should join.
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Backup Wi-Fi access point.
// Use this at competitions or whenever the normal router is unavailable.
const char* AP_SSID = "SoccerMagnumOpus";
const char* AP_PASSWORD = "soccer123";

WebServer server(80);

bool wifiConnected = false;
bool wifiServicesStarted = false;
bool apStarted = false;
unsigned long lastWifiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 10000;

// ------------------------------------------------------------
// REMOTE CONTROL / SAFETY
// ------------------------------------------------------------
bool manualMode = false;
int manualVy = 0;
int manualVx = 0;
int manualW = 0;
int manualSpeed = 150;
unsigned long lastManualCommand = 0;
const unsigned long MANUAL_TIMEOUT = 800;
int8_t motorTestIndex = -1;  // -1 = normal manual drive, 0..3 = FL/FR/BL/BR.
int motorTestPwm = 0;

// ------------------------------------------------------------
// OTA
// ------------------------------------------------------------
const char* OTA_HOSTNAME = "SoccerMagnumOpus";
const char* OTA_PASSWORD = "soccer123";


#define I2C_SDA 21
#define I2C_SCL 22

extern int debugVX;
extern int debugVY;
void stopAll();
void driveWithGyro(int vy, int vx);

// ============================================================
// MOTOR PINS
//
// ESP32 replacement for the Mega motor pins.
// These are GPIO numbers on the ESP32.
// Keep the motor ORDER the same as the stable code.
//
// FL = Front Left
// FR = Front Right
// BL = Back Left
// BR = Back Right
// ============================================================

#define FL_PWM_A 25
#define FL_PWM_B 26

#define FR_PWM_A 16
#define FR_PWM_B 17

#define BL_PWM_A 32
#define BL_PWM_B 33

#define BR_PWM_A 18
#define BR_PWM_B 19

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
//   CH0 = FRONT
//   CH1 = LEFT
//   CH2 = BACK
//   CH3 = RIGHT
//
// NeoPixels:
//   GPIO15, 16 LEDs, bright WHITE for ground illumination.
// ============================================================

#define TCA_ADDR           0x70
#define APDS_ADDR          0x39

#define APDS_CH_FRONT      0
#define APDS_CH_LEFT       1
#define APDS_CH_BACK       2
#define APDS_CH_RIGHT      3

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
  if (Wire.requestFrom(APDS_ADDR, (uint8_t)1) != 1) return false;
  if (!Wire.available()) return false;

  value = Wire.read();
  return true;
}

bool apdsRead16(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(APDS_ADDR, (uint8_t)2) != 2) return false;

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
// GPIO21 = SDA, GPIO22 = SCL; this sensor is on the main I2C bus.
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

  Wire.begin(I2C_SDA, I2C_SCL);
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
// ESP32 Arduino Core 3.x supports analogWrite(pin, duty)
// with the normal 0-255 Arduino PWM range.
//
// No old ledcSetup()/ledcAttachPin() API is used.
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

  // DEBUG

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
// WIFI DEBUG WEB SERVER
// ============================================================

// ============================================================
// WIFI WEB DEBUG + REMOTE CONTROL + BROWSER OTA
// ============================================================

void handleRoot();
void handleDebugData();
void handleControl();
void handleMotorTest();
void handleMode();
void handleHeadingReset();
void handleOTAUpload();
void handleOTAResult();
void handleNotFound();
void handleAPDSCalibrate();
void handleAPDSTolerance();
void handleAPDSBrightness();

String makeDebugPage() {
  String html;
  html.reserve(15000);

  html += F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Soccer Magnum Opus</title>"
            "<style>"
            "body{font-family:Arial,sans-serif;margin:0;padding:12px;background:#101317;color:#eee}"
            "h1{font-size:23px;margin:4px 0 12px}.card{background:#1c2229;border-radius:12px;padding:13px;margin:9px 0}"
            ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}.sensor{background:#272e36;padding:10px;border-radius:9px;text-align:center}"
            ".name{color:#aaa;font-size:13px}.raw{font-size:23px;font-weight:bold}.smooth{font-size:15px;color:#bbb}.bar{height:11px;background:#444;border-radius:6px;margin-top:7px;overflow:hidden}.fill{height:100%;background:#52c878;width:0%}"
            ".balltrack{position:relative;height:42px;background:#303740;border-radius:21px;margin:13px 4px}.ball{position:absolute;top:7px;width:28px;height:28px;border-radius:50%;background:#ff9f1c;transform:translateX(-50%);transition:left .08s}"
            ".labels{display:flex;justify-content:space-between;color:#aaa;font-size:12px}.big{font-size:20px}.ok{color:#55dd77}.warn{color:#ffcc55}"
            ".controls{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;max-width:420px;margin:auto}.controls button,.row button{font-size:20px;padding:13px;border:0;border-radius:9px;background:#39434d;color:white}.stop{background:#a83232!important}.auto{background:#286b45!important}.modeManual{background:#a36b20!important}"
            ".row{display:flex;gap:7px;justify-content:center;margin-top:8px}.row button{font-size:15px;padding:10px 13px}.wide{width:100%;box-sizing:border-box}"
            ".motorrow{display:grid;grid-template-columns:40px 1fr 1fr;gap:7px;align-items:center;margin-top:7px}.motorrow button{font-size:16px;padding:12px;border:0;border-radius:9px;background:#39434d;color:white;touch-action:none;user-select:none}"
            "input[type=range]{width:100%}.mode{font-size:18px;font-weight:bold}.mono{font-family:monospace}"
            "@media(max-width:600px){.grid{grid-template-columns:repeat(2,1fr)}}"
            "</style></head><body>");

  html += F("<h1>Soccer Magnum Opus</h1>");
  html += F("<div class='card'><div>Normal Wi-Fi: <span id='wifi'>");
  html += wifiConnected ? WiFi.localIP().toString() : "not connected";
  html += F("</span></div><div>Robot AP: <span>");
  html += apStarted ? WiFi.softAPIP().toString() : "not started";
  html += F("</span></div><div>Mode: <span class='mode' id='mode'>AUTO</span></div>");
  html += F("<div>Threshold: <span id='th'>"); html += String(BALL_THRESHOLD); html += F("</span></div></div>");

  html += F("<div class='card'><b>HITECHNIC IR SEEKER V2 (AC)</b><div class='grid'>");
  for (int i=0;i<6;i++) {
    html += F("<div class='sensor'><div class='name'>"); html += (i == 0 ? "Direction" : "S" + String(i));
    html += F("</div><div class='raw' id='r"); html += String(i); html += F("'>0</div>");
    html += F("<div class='smooth'>Smooth: <span id='s"); html += String(i); html += F("'>0</span></div>");
    html += F("<div class='bar'><div class='fill' id='b"); html += String(i); html += F("'></div></div></div>");
  }
  html += F("</div></div>");

  html += F("<div class='card'><b>BALL POSITION</b><div class='labels'><span>DIR 1 LEFT</span><span>DIR 5 CENTER</span><span>DIR 9 RIGHT</span></div>");
  html += F("<div class='balltrack'><div class='ball' id='ballDot' style='left:50%'></div></div>");
  html += F("<div class='big'>Detected: <span id='ball'>NO</span> &nbsp; Position: <span id='pos'>2500</span></div></div>");

  html += F("<div class='card'><b>GYRO HEADING</b><div class='big'>Yaw: <span id='yaw'>0.00</span>&deg;</div><div class='big'>Target: <span id='target'>0.00</span>&deg;</div>");
  html += F("<div class='row'><button onclick=fetch('/heading/reset')>Reset Heading to 0&deg;</button></div></div>");

  html += F("<div class='card'><b>4 APDS WHITE-LINE SENSORS</b>");
  html += F("<div class='grid'>");
  for (int i=0;i<4;i++) {
    html += F("<div class='sensor' id='apbox"); html += String(i); html += F("'>");
    html += F("<div class='name'>"); html += apdsName[i]; html += F(" / CH"); html += String(i); html += F("</div>");
    html += F("<div class='raw' id='apst"); html += String(i); html += F("'>---</div>");
    html += F("<div class='smooth'>R:<span id='apr"); html += String(i); html += F("'>0</span> ");
    html += F("G:<span id='apg"); html += String(i); html += F("'>0</span> ");
    html += F("B:<span id='apb"); html += String(i); html += F("'>0</span></div>");
    html += F("<div class='smooth'>C:<span id='apc"); html += String(i); html += F("'>0</span></div>");
    html += F("<div class='smooth'>Green distance: <span id='apd"); html += String(i); html += F("'>---</span></div>");
    html += F("<div class='smooth'>Brightness ratio: <span id='apbr"); html += String(i); html += F("'>---</span></div>");
    html += F("<div class='big' id='aps"); html += String(i); html += F("'>---</div>");
    html += F("</div>");
  }
  html += F("</div>");
  html += F("<div class='row'><button onclick=\"fetch('/apds/calibrate',{method:'POST'})\">CALIBRATE GREEN</button></div>");
  html += F("<div class='big'>Green tolerance: <span id='aptol'>0.08</span></div>");
  html += F("<input id='aptolSlider' type='range' min='0.03' max='0.40' step='0.01' value='0.08' oninput='setAPDSTol(this.value)'>");
  html += F("<div class='big'>NeoPixel brightness: <span id='apbright'>255</span></div>");
  html += F("<input id='apbrightSlider' type='range' min='10' max='255' value='255' oninput='setAPDSBright(this.value)'>");
  html += F("<div class='small' id='apmsg'>GREEN auto-calibrates at boot. Anything else = WHITE.</div></div>");

  html += F("<div class='card'><b>MANUAL MOTOR CONTROL</b><div class='big'>Speed: <span id='spd'>150</span></div>");
  html += F("<input id='speed' type='range' min='60' max='255' value='150' oninput='spd.textContent=this.value'>");
  html += F("<div class='controls'>");
  html += F("<div></div><button onmousedown=go('F') ontouchstart=go('F')>&uarr;</button><div></div>");
  html += F("<button onmousedown=go('L') ontouchstart=go('L')>&larr;</button><button class='stop' onclick=go('S')>■</button><button onmousedown=go('R') ontouchstart=go('R')>&rarr;</button>");
  html += F("<div></div><button onmousedown=go('B') ontouchstart=go('B')>&darr;</button><div></div>");
  html += F("</div><div class='row'><button onmousedown=go('RL') ontouchstart=go('RL')>&#8634; Rotate</button><button onmousedown=go('RR') ontouchstart=go('RR')>Rotate &#8635;</button></div>");
  html += F("<div class='row'><button id='modeBtn' class='auto wide' onclick=toggleMode()>SWITCH TO MANUAL</button></div></div>");

  html += F("<div class='card'><b>INDIVIDUAL MOTOR TEST</b><div>Hold a button to run only that wheel. Release to stop.</div>");
  html += F("<div>Test PWM: <span id='motorSpd'>100</span></div>");
  html += F("<input id='motorSpeed' type='range' min='40' max='160' value='100' oninput='motorSpd.textContent=this.value'>");
  const char* motorNames[4] = {"FL", "FR", "BL", "BR"};
  for (int i = 0; i < 4; i++) {
    html += F("<div class='motorrow'><b>"); html += motorNames[i]; html += F("</b>");
    html += F("<button onpointerdown=\"startMotorTest(event,"); html += String(i);
    html += F(",1)\" onpointerleave=\"stopMotorTest(event)\">Forward</button>");
    html += F("<button onpointerdown=\"startMotorTest(event,"); html += String(i);
    html += F(",-1)\" onpointerleave=\"stopMotorTest(event)\">Reverse</button></div>");
  }
  html += F("<div class='row'><button class='stop' onclick='forceStopMotorTest()'>STOP MOTOR</button></div>");
  html += F("<div>Active: <span id='motorStatus'>None</span></div></div>");

  html += F("<div class='card'><b>OTA FIRMWARE</b><form method='POST' action='/update' enctype='multipart/form-data'>");
  html += F("<input class='wide' type='file' name='firmware' accept='.bin'><div class='row'><button type='submit'>Upload Firmware</button></div></form>");
  html += F("<small>Arduino IDE: use the Network Port named SoccerMagnumOpus after the first USB upload.</small></div>");

  html += F("<script>"
             "const speed=document.getElementById('speed'),spd=document.getElementById('spd');"
             "const modeBtn=document.getElementById('modeBtn');"
             "let motorTimer=0,motorHolding=false,motorQueue=Promise.resolve();"
             "function sendMotor(i,v){motorQueue=motorQueue.catch(()=>{}).then(()=>fetch('/motor/test?motor='+i+'&speed='+v,{method:'POST',cache:'no-store'}));}"
             "function startMotorTest(e,i,sign){e.preventDefault();if(motorHolding)stopMotorTest();motorHolding=true;const v=sign*Number(document.getElementById('motorSpeed').value);sendMotor(i,v);motorTimer=setInterval(()=>{if(motorHolding)sendMotor(i,v)},250);}"
             "function stopMotorTest(e){if(e)e.preventDefault();if(!motorHolding)return;motorHolding=false;clearInterval(motorTimer);sendMotor(-1,0);}"
             "function forceStopMotorTest(){stopMotorTest();sendMotor(-1,0);}"
             "window.addEventListener('pointerup',stopMotorTest);window.addEventListener('pointercancel',stopMotorTest);window.addEventListener('blur',stopMotorTest);"
             "document.addEventListener('visibilitychange',()=>{if(document.hidden)stopMotorTest()});"
             "async function setAPDSTol(v){document.getElementById('aptol').textContent=Number(v).toFixed(2);fetch('/apds/tolerance?v='+v);}"
             "async function setAPDSBright(v){document.getElementById('apbright').textContent=v;fetch('/apds/brightness?v='+v);}"
             "async function updateAPDS(){try{let r=await fetch('/apds',{cache:'no-store'});let j=await r.json();document.getElementById('aptol').textContent=Number(j.tolerance).toFixed(2);document.getElementById('aptolSlider').value=j.tolerance;document.getElementById('apbright').textContent=j.brightness;document.getElementById('apbrightSlider').value=j.brightness;document.getElementById('apmsg').textContent=j.message;for(let i=0;i<4;i++){let a=j.sensors[i];let box=document.getElementById('apbox'+i);let st=document.getElementById('aps'+i);document.getElementById('apr'+i).textContent=a.r;document.getElementById('apg'+i).textContent=a.g;document.getElementById('apb'+i).textContent=a.b;document.getElementById('apc'+i).textContent=a.c;document.getElementById('apd'+i).textContent=Number(a.distance).toFixed(3);document.getElementById('apbr'+i).textContent=Number(a.brightness).toFixed(2);st.textContent=!a.available?'NO DATA':(a.green?'GREEN':'WHITE');box.className='sensor '+(a.green?'ok':'warn');}}catch(e){}}"
             "async function go(c){"

             " if(c!=='S')await fetch('/mode?set=manual');"
             " let v=(c==='S')?0:+speed.value;"
             " await fetch('/control?cmd='+c+'&speed='+v);"
             "}"
             "async function toggleMode(){"
             " let manual=modeBtn.dataset.manual==='1';"
             " await fetch('/mode?set='+(manual?'auto':'manual'));"
             " await update();"
             "}"
             "function setModeUI(manual){"
             " modeBtn.dataset.manual=manual?'1':'0';"
             " modeBtn.textContent=manual?'SWITCH TO AUTO':'SWITCH TO MANUAL';"
             "}"
             "async function update(){try{"
             " let r=await fetch('/data',{cache:'no-store'});"
             " let j=await r.json();"
             " for(let i=0;i<6;i++){"
             "  document.getElementById('r'+i).textContent=j.raw[i];"
             "  document.getElementById('s'+i).textContent=j.smooth[i];"
             "  document.getElementById('b'+i).style.width=Math.min(100,j.smooth[i]/(i===0?0.09:2.55))+'%';"
             " }"
             " document.getElementById('ball').textContent=j.ball?'DETECTED':'NO';"
             " document.getElementById('ball').className=j.ball?'ok':'warn';"
             " document.getElementById('pos').textContent=j.position;"
             " document.getElementById('yaw').textContent=j.yaw.toFixed(2);"
             " document.getElementById('target').textContent=j.target.toFixed(2);"
             " document.getElementById('mode').textContent=j.manual?'MANUAL':'AUTO';"
             " document.getElementById('mode').className=j.manual?'warn':'ok';"
             " document.getElementById('motorStatus').textContent=j.motorTest<0?'None':(['FL','FR','BL','BR'][j.motorTest]+' '+(j.motorPwm>0?'Forward ':'Reverse ')+Math.abs(j.motorPwm));"
             " setModeUI(j.manual);"
             " document.getElementById('ballDot').style.left=(100-(j.position/50))+'%';"
             "}catch(e){}}"
             "setInterval(update,100);setInterval(updateAPDS,150);update();updateAPDS();"
             "</script></body></html>");
  return html;
}

void handleRoot() { server.send(200,"text/html; charset=utf-8",makeDebugPage()); }

void handleDebugData() {
  String json; json.reserve(900);
  json += "{\"raw\":[";
  for(int i=0;i<6;i++){json+=String(rawSensor[i]);if(i<5)json+=",";}
  json += "],\"smooth\":[";
  for(int i=0;i<6;i++){json+=String((int)smooth[i]);if(i<5)json+=",";}
  json += "],\"ball\":"; json += debugBallDetected?"true":"false";
  json += ",\"position\":"+String(debugBallPosition);
  json += ",\"yaw\":"+String(currentYaw,2);
  json += ",\"target\":"+String(targetYaw,2);
  json += ",\"manual\":"; json += manualMode?"true":"false";
  json += ",\"motorTest\":"+String(motorTestIndex);
  json += ",\"motorPwm\":"+String(motorTestPwm);
  json += "}";
  server.send(200,"application/json",json);
}

void handleControl() {
  String cmd=server.arg("cmd");
  int spd=server.hasArg("speed") ? server.arg("speed").toInt() : manualSpeed;
  spd=constrain(spd,0,255); manualSpeed=spd;
  manualMode=true; lastManualCommand=millis();
  motorTestIndex=-1; motorTestPwm=0;
  manualVy=manualVx=manualW=0;
  if(cmd=="F") manualVy=spd;
  else if(cmd=="B") manualVy=-spd;
  else if(cmd=="L") manualVx=-spd;
  else if(cmd=="R") manualVx=spd;
  else if(cmd=="RL") manualW=-spd;
  else if(cmd=="RR") manualW=spd;
  else if(cmd=="S") { manualMode=true; manualVy=manualVx=manualW=0; }
  drive(manualVy,manualVx,manualW);
  server.send(200,"text/plain","OK");
}

void handleMotorTest() {
  if (!server.hasArg("motor") || !server.hasArg("speed")) {
    server.send(400, "text/plain", "motor and speed are required");
    return;
  }

  String motor = server.arg("motor");
  int speed = server.arg("speed").toInt();
  if (motor == "-1" && speed == 0) {
    manualMode = true;
    manualVy = manualVx = manualW = 0;
    motorTestIndex = -1;
    motorTestPwm = 0;
    lastManualCommand = millis();
    stopAll();
    server.send(200, "text/plain", "Stopped");
    return;
  }

  if ((motor != "0" && motor != "1" && motor != "2" && motor != "3") ||
      speed == 0) {
    server.send(400, "text/plain", "Choose motor 0..3 and a nonzero speed");
    return;
  }

  int8_t index = motor.toInt();
  speed = constrain(speed, -160, 160);
  manualMode = true;
  manualVy = manualVx = manualW = 0;
  lastManualCommand = millis();
  if (motorTestIndex != index || motorTestPwm != speed) {
    stopAll();
    motorTestIndex = index;
    motorTestPwm = speed;
    setSelectedMotor(motorTestIndex, motorTestPwm);
  }
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  String set=server.arg("set");
  if(set=="auto") { manualMode=false; manualVy=manualVx=manualW=0; motorTestIndex=-1; motorTestPwm=0; stopAll(); }
  else if(set=="manual") { manualMode=true; manualVy=manualVx=manualW=0; motorTestIndex=-1; motorTestPwm=0; lastManualCommand=millis(); stopAll(); }
  server.send(200,"text/plain",manualMode?"MANUAL":"AUTO");
}

void handleHeadingReset() {
  currentYaw=0.0; targetYaw=0.0; lastYawUpdateMs=millis();
  server.send(200,"text/plain","Heading reset");
}

void handleOTAResult() {
  bool ok=!Update.hasError();
  server.sendHeader("Connection","close");
  server.send(200,"text/plain",ok?"Update successful. Rebooting...":"Update failed.");
  delay(300);
  if(ok) ESP.restart();
}

void handleOTAUpload() {
  HTTPUpload& upload=server.upload();
  if(upload.status==UPLOAD_FILE_START) {
    Serial.printf("OTA upload: %s\n",upload.filename.c_str());
    stopAll();
    motorTestIndex=-1; motorTestPwm=0;
    manualMode=false;
    if(!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if(upload.status==UPLOAD_FILE_WRITE) {
    if(Update.write(upload.buf,upload.currentSize)!=upload.currentSize) Update.printError(Serial);
  } else if(upload.status==UPLOAD_FILE_END) {
    if(Update.end(true)) Serial.printf("OTA written: %u bytes\n",upload.totalSize);
    else Update.printError(Serial);
  } else if(upload.status==UPLOAD_FILE_ABORTED) {
    Update.abort(); Serial.println("OTA upload aborted");
  }
}


void handleAPDSCalibrate() {
  // Calibration is a deliberate intervention: stop motors and stay MANUAL.
  manualMode = true;
  manualVy = 0;
  manualVx = 0;
  manualW  = 0;
  motorTestIndex = -1;
  motorTestPwm = 0;
  stopAll();

  calibrateAPDSGreen();

  server.send(
    200,
    "text/plain",
    apdsCalibrationComplete
      ? "GREEN calibration complete"
      : "GREEN calibration error"
  );
}

void handleAPDSTolerance() {

  if (server.hasArg("v")) {

    float v =
      server.arg("v").toFloat();

    apdsGreenTolerance =
      constrain(v, 0.03f, 0.40f);
  }

  server.send(200, "text/plain", "OK");
}

void handleAPDSBrightness() {

  if (server.hasArg("v")) {

    int v =
      server.arg("v").toInt();

    apdsNeoBrightness =
      constrain(v, 10, 255);

    apdsGroundLightOn();
  }

  server.send(200, "text/plain", "OK");
}

void handleAPDSPageData() {

  String json;
  json.reserve(1200);

  json += "{\"tolerance\":";
  json += String(apdsGreenTolerance, 3);

  json += ",\"brightness\":";
  json += String(apdsNeoBrightness);

  json += ",\"message\":\"";

  String msg = apdsCalibrationMessage;
  msg.replace("\"", "'");

  json += msg;
  json += "\",\"sensors\":[";

  for (int i=0; i<4; i++) {

    if (i > 0) json += ",";

    json += "{\"r\":";
    json += String(apdsLive[i].r);

    json += ",\"g\":";
    json += String(apdsLive[i].g);

    json += ",\"b\":";
    json += String(apdsLive[i].b);

    json += ",\"c\":";
    json += String(apdsLive[i].c);

    json += ",\"distance\":";
    json += String(apdsLive[i].greenDistance, 4);

    json += ",\"brightness\":";
    json += String(apdsLive[i].brightnessRatio, 3);

    json += ",\"green\":";
    json += apdsLive[i].green ? "true" : "false";

    json += ",\"available\":";
    json += apdsLive[i].available ? "true" : "false";

    json += "}";
  }

  json += "]}";

  server.send(
    200,
    "application/json",
    json
  );
}

void handleNotFound() { server.send(404,"text/plain","Not found"); }

void setupArduinoOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){ stopAll(); Serial.println("Arduino OTA start"); });
  ArduinoOTA.onEnd([](){ Serial.println("Arduino OTA end"); });
  ArduinoOTA.onProgress([](unsigned int progress,unsigned int total){
    Serial.printf("OTA %u%%\r",(progress*100)/total);
  });
  ArduinoOTA.onError([](ota_error_t error){ Serial.printf("OTA error[%u]\n",error); });
  ArduinoOTA.begin();
  Serial.print("Arduino OTA hostname: "); Serial.println(OTA_HOSTNAME);
}

void startWiFiDebug() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("WIFI / AP+STA DEBUG");
  Serial.println("======================================");

  // Start the ESP32 AP first and do NOT depend on the router.
  // AP operation is the primary debug/control path.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  delay(200);

  // Fixed AP channel and visible SSID improve compatibility with phones.
  // The AP remains available even when the router is unavailable.
  bool apOK = WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    6,        // channel
    false,    // hidden = false
    4         // max connections
  );

  if (apOK) {
    apStarted = true;
    Serial.println("Robot AP started successfully.");
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    apStarted = false;
    Serial.println("ERROR: Robot AP failed to start.");
  }

  // Start normal Wi-Fi in the background only.
  // Never wait for it.
  if (String(WIFI_SSID) != "YOUR_WIFI_NAME") {
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("Normal Wi-Fi connection started in background.");
  } else {
    Serial.println("Normal Wi-Fi credentials not configured.");
  }

  Serial.println("Soccer control runs independently of Wi-Fi.");
}


void startWiFiServicesIfConnected() {
  if (wifiServicesStarted) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED && !apStarted) {
    return;
  }

  if (wifiServicesStarted) {
    return;
  }

  wifiServicesStarted = true;

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleDebugData);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/motor/test", HTTP_POST, handleMotorTest);
  server.on("/mode", HTTP_GET, handleMode);
  server.on("/heading/reset", HTTP_GET, handleHeadingReset);
  server.on("/apds", HTTP_GET, handleAPDSPageData);
  server.on("/apds/calibrate", HTTP_POST, handleAPDSCalibrate);
  server.on("/apds/tolerance", HTTP_GET, handleAPDSTolerance);
  server.on("/apds/brightness", HTTP_GET, handleAPDSBrightness);
  server.on("/update", HTTP_POST, handleOTAResult, handleOTAUpload);
  server.onNotFound(handleNotFound);

  server.begin();

  // Arduino IDE OTA uses the STA network when available.
  // It can still be discovered through the AP when the ESP32
  // is connected directly to the robot AP.
  setupArduinoOTA();

  Serial.println("Web debug server started.");

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print("STA IP: ");
    Serial.println(WiFi.localIP());
  }

  if (apStarted) {
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
}


void serviceWiFi() {
  // Wi-Fi is a background service only.
  // NOTHING here is allowed to stop autonomous soccer control.

  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;

      Serial.print("Normal Wi-Fi connected. STA IP: ");
      Serial.println(WiFi.localIP());
    }

    startWiFiServicesIfConnected();

    // Service dashboard and OTA only when the network stack is alive.
    server.handleClient();
    ArduinoOTA.handle();

    // Manual control timeout is independent safety logic.
    if (manualMode &&
        (motorTestIndex >= 0 || manualVy != 0 || manualVx != 0 || manualW != 0) &&
        millis() - lastManualCommand > MANUAL_TIMEOUT) {

      manualVy = 0;
      manualVx = 0;
      manualW  = 0;
      motorTestIndex = -1;
      motorTestPwm = 0;

      stopAll();

      // Stop the motors, but KEEP MANUAL MODE selected.
      // AUTO must only be selected explicitly by the user.
      Serial.println("Manual command timeout -> motors stopped (still MANUAL)");
    }

    return;
  }

  // Normal router is unavailable.
  // The robot AP can still keep the dashboard alive.
  wifiConnected = false;

  if (apStarted) {
    if (!wifiServicesStarted) {
      startWiFiServicesIfConnected();
    }

    server.handleClient();
    ArduinoOTA.handle();

    if (manualMode &&
        (motorTestIndex >= 0 || manualVy != 0 || manualVx != 0 || manualW != 0) &&
        millis() - lastManualCommand > MANUAL_TIMEOUT) {

      manualVy = 0;
      manualVx = 0;
      manualW  = 0;
      motorTestIndex = -1;
      motorTestPwm = 0;

      stopAll();

      // Stop the motors, but KEEP MANUAL MODE selected.
      // AUTO must only be selected explicitly by the user.
      Serial.println("Manual command timeout -> motors stopped (still MANUAL)");
    }
  }

  // Retry the normal router periodically.
  if (millis() - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL &&
      String(WIFI_SSID) != "YOUR_WIFI_NAME") {

    lastWifiReconnectAttempt = millis();
    WiFi.reconnect();
  }

  // IMPORTANT:
  // No motor stop here.
  // loop() continues directly into the soccer algorithm.
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);

  // ----------------------------------------------------------
  // WIFI DEBUG
  // ----------------------------------------------------------
  startWiFiDebug();

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

  // ----------------------------------------------------------
  // I2C / APDS / MPU
  // ----------------------------------------------------------

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  Wire.setClock(100000);

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
    "ESP32-WROOM PORT"
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
    "I2C SDA = GPIO21"
  );

  Serial.println(
    "I2C SCL = GPIO22"
  );

  Serial.println(
    "TCA9548A = 0x70"
  );

  Serial.println(
    "APDS = 0x39 / ID 0x9E"
  );

  Serial.println(
    "APDS CH0 FRONT / CH1 LEFT / CH2 BACK / CH3 RIGHT"
  );

  Serial.println(
    "NeoPixel = GPIO15 / 16 LEDs / BRIGHT WHITE"
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

  // Wi-Fi is a background/debug feature.
  // NEVER let Wi-Fi determine whether the soccer algorithm runs.
  serviceWiFi();

  // Poll only one APDS at a time so the soccer loop stays responsive.
  serviceAPDS();

  if (manualMode) {

    if (millis() - lastManualCommand <= MANUAL_TIMEOUT) {
      if (motorTestIndex >= 0)
        setSelectedMotor(motorTestIndex, motorTestPwm);
      else
        drive(manualVy, manualVx, manualW);
    }
    else {

      manualVy = 0;
      manualVx = 0;
      manualW  = 0;
      motorTestIndex = -1;
      motorTestPwm = 0;

      stopAll();

      // Stay MANUAL after timeout.
      // AUTO requires an explicit command from the user.
    }

  } else {

    followBall();
  }

  delay(2);
}
