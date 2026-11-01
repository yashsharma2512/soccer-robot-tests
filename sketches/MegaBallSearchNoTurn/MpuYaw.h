#ifndef MAGNUM_MPU_YAW_H
#define MAGNUM_MPU_YAW_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "RobotConfig.h"
#include "DriveLogic.h"

class MpuYaw {
 public:
  enum Error { OK, NOT_FOUND, WRONG_ID, READ_FAILED, CONFIG_CHANGED,
               NOT_STILL, NO_NEW_DATA, SAMPLE_GAP, CAL_TIMEOUT };
  MpuYaw() : address_(0x68), online_(false), calibrated_(false), calibrating_(false),
      error_(NOT_FOUND), count_(0), mean_(0), m2_(0), minimum_(0), maximum_(0),
      yaw_(0), rate_(0), lastPoll_(0), lastSampleMs_(0), lastSampleUs_(0),
      lastConfigCheck_(0), calibrationStart_(0) {}

  bool startCalibration() {
    online_ = calibrated_ = calibrating_ = false;
    error_ = NOT_FOUND;
    bool responded = false;
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      address_ = attempt == 0 ? 0x68 : 0x69;
      uint8_t id = 0;
      if (!read(0x75, &id, 1)) continue;
      responded = true;
      // 0x68 is the MPU-6050 ID. MPU-6515 parts report 0x74 at the same
      // I2C address and use the MPU-6500-compatible gyro register map.
      if ((id & 0x7E) != 0x68 && id != 0x74) continue;
      if (!write(0x6B, 0x01) || !write(0x6C, 0x00) || !write(0x1A, 0x03) ||
          !write(0x19, 0x04) || !write(0x1B, 0x00) || !write(0x1C, 0x00) ||
          !write(0x37, 0x00) || !write(0x38, 0x01)) {
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
    error_ = responded ? WRONG_ID : NOT_FOUND;
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
      uint8_t config[4], power[2], interruptConfig[2];
      if (!read(0x19, config, 4) || !read(0x6B, power, 2) || !read(0x37, interruptConfig, 2)) {
        fail(READ_FAILED); return;
      }
      if (config[0] != 4 || config[1] != 3 || config[2] != 0 || config[3] != 0 ||
          power[0] != 1 || power[1] != 0 || interruptConfig[0] != 0 || interruptConfig[1] != 1)
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
  Error error() const { return error_; }
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
  uint8_t address_;
  bool online_, calibrated_, calibrating_;
  Error error_;
  uint16_t count_;
  float mean_, m2_;
  int16_t minimum_, maximum_;
  float yaw_, rate_;
  uint32_t lastPoll_, lastSampleMs_, lastSampleUs_, lastConfigCheck_, calibrationStart_;
};
#endif
