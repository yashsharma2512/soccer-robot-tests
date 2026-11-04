// Mega MPU-only test. SDA=20, SCL=21. Serial=115200.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const uint8_t MOTOR_PINS[] = {8, 6, 9, 7, 2, 3, 10, 11};
uint8_t address = 0x68, chipId = 0;
bool online = false, calibrating = true;
uint16_t samples = 0;
float bias = 0, m2 = 0, yaw = 0;
uint32_t lastPoll = 0, lastFresh = 0, lastUs = 0;
uint32_t lastReport = 0, calibrationStart = 0;

bool readRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(address, count);
  if (Wire.getWireTimeoutFlag() || received != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
}

int16_t signedValue(const uint8_t *data) {
  return int16_t((uint16_t(data[0]) << 8) | data[1]);
}

void fault(const __FlashStringHelper *message) {
  online = false;
  Serial.println(message);
  Serial.println(F("Fix/check the sensor, then send c to retry."));
}

void startTest() {
  online = false;
  bool found = false;
  for (uint8_t candidate = 0x68; candidate <= 0x69; ++candidate) {
    address = candidate;
    uint8_t ids[3] = {};
    bool complete = true;
    Serial.print(F("Address 0x")); Serial.print(address, HEX);
    Serial.print(F(" WHO_AM_I: "));
    for (uint8_t i = 0; i < 3; ++i) {
      if (i) Serial.print('/');
      if (!readRegisters(0x75, &ids[i], 1)) {
        complete = false;
        Serial.print(F("ERR"));
      } else {
        Serial.print(F("0x")); Serial.print(ids[i], HEX);
      }
    }
    Serial.println();
    if (!complete || ids[0] != ids[1] || ids[1] != ids[2]) continue;
    chipId = ids[0];
    if (chipId == 0x68 || chipId == 0x70 || chipId == 0x74) {
      found = true;
      break;
    }
  }
  if (!found) {
    fault(F("No supported, stable MPU identity found."));
    return;
  }

  if (!writeRegister(0x6B, 1) || !writeRegister(0x6C, 0) ||
      !writeRegister(0x1A, 3) || !writeRegister(0x19, 4) ||
      !writeRegister(0x1B, 0) || !writeRegister(0x1C, 0) ||
      !writeRegister(0x37, 0) || !writeRegister(0x38, 1) ||
      (chipId != 0x68 && !writeRegister(0x1D, 3))) {
    fault(F("MPU configuration write failed."));
    return;
  }
  delay(200);
  samples = 0;
  bias = m2 = yaw = 0;
  calibrating = true;
  online = true;
  lastPoll = lastFresh = calibrationStart = millis();
  lastUs = micros();
  Serial.println(F("Keep STILL for about 3 seconds. c=recalibrate, z=zero yaw"));
}

void setup() {
  for (uint8_t i = 0; i < sizeof(MOTOR_PINS); ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  Serial.println(F("MEGA MPU ONLY. Motor outputs LOW. SDA=20 SCL=21."));
  startTest();
}

void loop() {
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') startTest();
    else if (c == 'z' || c == 'Z') yaw = 0;
  }
  if (!online) return;
  uint32_t now = millis();
  if (calibrating && uint32_t(now - calibrationStart) > 7000) {
    fault(F("Calibration timed out.")); return;
  }
  if (uint32_t(now - lastPoll) < 5) return;
  lastPoll = now;

  uint8_t data[15]; // Status, accel XYZ, temperature, gyro XYZ.
  if (!readRegisters(0x3A, data, 15)) {
    fault(F("MPU I2C read failed.")); return;
  }
  now = millis();
  if (!(data[0] & 1)) {
    if (uint32_t(now - lastFresh) > 50)
      fault(F("No fresh MPU samples."));
    return;
  }

  uint32_t sampleUs = micros();
  uint32_t elapsedUs = sampleUs - lastUs;
  lastUs = sampleUs;
  lastFresh = now;
  float ax = signedValue(data + 1) / 16384.0f;
  float ay = signedValue(data + 3) / 16384.0f;
  float az = signedValue(data + 5) / 16384.0f;
  float rawZ = signedValue(data + 13);

  if (calibrating) {
    float gravity = sqrtf(ax * ax + ay * ay + az * az);
    if (gravity < 0.7f || gravity > 1.3f) {
      fault(F("Calibration: unexpected acceleration. Keep STILL.")); return;
    }
    ++samples;
    float difference = rawZ - bias;
    bias += difference / samples;
    m2 += difference * (rawZ - bias);
    if (samples == 500) {
      if (sqrtf(m2 / 499.0f) / 131.0f > 1.5f || fabsf(bias) / 131.0f > 10) {
        fault(F("Calibration unstable. Keep STILL and send c.")); return;
      }
      calibrating = false;
      Serial.println(F("READY: Ax(g) Ay(g) Az(g) Gz(deg/s) Yaw(deg)"));
    }
    return;
  }
  if (elapsedUs > 50000UL) {
    fault(F("Sample gap too long; yaw invalid.")); return;
  }
  float rate = (rawZ - bias) / 131.0f;
  yaw += rate * (elapsedUs / 1000000.0f);
  while (yaw > 180) yaw -= 360;
  while (yaw < -180) yaw += 360;

  if (uint32_t(now - lastReport) >= 200) {
    lastReport = now;
    Serial.print(ax, 2); Serial.print('\t');
    Serial.print(ay, 2); Serial.print('\t');
    Serial.print(az, 2); Serial.print('\t');
    Serial.print(rate, 2); Serial.print('\t');
    Serial.println(yaw, 1);
  }
}
