#ifndef MAGNUM_MPU_YAW_H
#define MAGNUM_MPU_YAW_H
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include "RobotConfig.h"
#include "DriveLogic.h"
#include "BoundaryLogic.h"

class MpuYaw {
 public:
  enum Error { OK, NOT_FOUND, WRONG_ID, READ_FAILED, CONFIG_CHANGED,
               NOT_STILL, NO_NEW_DATA, SAMPLE_GAP, CAL_TIMEOUT, SAVE_FAILED, CAL_CANCELLED };
  MpuYaw() : address_(0x68), identity_(0), online_(false), calibrated_(false), calibrating_(false),
      error_(NOT_FOUND), count_(0), mean_(0), m2_(0), minimum_(0), maximum_(0),
      yaw_(0), rate_(0), lastPoll_(0), lastSampleMs_(0), lastSampleUs_(0),
      lastConfigCheck_(0), calibrationStart_(0), saving_(false), loaded_(false),
      firstSample_(true), saveStep_(0) {}

  // Startup/rescan may reuse EEPROM; f and c always request a fresh calibration.
  bool startCalibration(bool useSaved = false) {
    online_ = calibrated_ = calibrating_ = false;
    saving_ = loaded_ = false;
    firstSample_ = true;
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
};
#endif
