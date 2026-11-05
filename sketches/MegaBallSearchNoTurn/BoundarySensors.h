#ifndef SOCCER_BOUNDARY_SENSORS_H
#define SOCCER_BOUNDARY_SENSORS_H
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include "BoundaryConfig.h"

class BoundarySensors {
 public:
  BoundarySensors() : isolated_(false), running_(false), saving_(false),
      next_(0), saveStep_(0), capture_(-1), lastSlot_(0), captureStart_(0) {}

  bool begin() {
    running_ = false; capture_ = -1; saving_ = false; next_ = 0;
    for (uint8_t i = 0; i < count; ++i) sensors_[i] = Sensor{};
    if (!select(0)) { Serial.println(F("LINE: mux unavailable")); return false; }
    if (!line::validLayout(boundarycfg::CHANNELS, boundarycfg::SIDES, count)) {
      Serial.println(F("LINE: invalid channel/side mapping")); return false;
    }
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(0x39);
    uint8_t upstream = Wire.endTransmission();
    if (Wire.getWireTimeoutFlag() || upstream != 2) {
      Serial.println(F("LINE: upstream 0x39 conflict or bus error")); return false;
    }
    uint8_t found = 0;
    for (uint8_t i = 0; i < count; ++i) {
      if (!select(uint8_t(1U << boundarycfg::CHANNELS[i]))) return false;
      uint8_t id = 0;
      bool ok = read(0x92, &id, 1) && (id == 0xAB || id == 0x9C || id == 0x9E);
      if (ok) {
        ok = write(0x80, 0) && write(0xAB, 0) && write(0x81, boundarycfg::ATIME) &&
             write(0x8F, boundarycfg::GAIN) && write(0x8D, 0x60) &&
             write(0x90, 0x41) && write(0x9F, 0) && write(0x80, 1);
        delay(10); // Only during stopped initialization; sampling never delays.
        ok = ok && write(0x80, 3) && configurationOK();
      }
      if (!select(0)) return false;
      sensors_[i].configured = ok;
      sensors_[i].fault = ok ? WAITING : MISSING;
      found += ok;
      Serial.print(F("LINE sensor ")); Serial.print(i + 1);
      Serial.print(F(" CH")); Serial.print(boundarycfg::CHANNELS[i]);
      Serial.print(F(" ID=0x")); Serial.print(id, HEX);
      Serial.println(ok ? F(" ready") : F(" MISSING/CONFIG ERROR"));
    }
    running_ = true; lastSlot_ = millis();
    bool loaded = loadCalibration();
    Serial.println(loaded ? F("LINE calibration loaded; start with g when ready.") :
        F("LINE calibration required: f=field; 1/2/3/4=white per sensor."));
    return found == count;
  }

  void service() {
    if (saving_) saveOneByte(); // At most one EEPROM byte per loop; motors are stopped.
    if (!isolated_) { select(0); return; }
    if (!running_) return;
    uint32_t now = millis();
    if (capture_ != -1 && uint32_t(now - captureStart_) > boundarycfg::CAL_TIMEOUT_MS) {
      capture_ = -1;
      Serial.println(F("LINE calibration timeout; check sensors and repeat."));
    }
    if (uint32_t(now - lastSlot_) < boundarycfg::SLOT_MS) return;
    lastSlot_ = now;
    uint8_t i = next_;
    next_ = uint8_t((next_ + 1) % count);
    Sensor &sensor = sensors_[i];
    if (!sensor.configured) return;
    if (!select(uint8_t(1U << boundarycfg::CHANNELS[i]))) return;

    uint8_t status = 0, data[8] = {};
    bool ok = read(0x93, &status, 1);
    bool newData = ok && (status & 1);
    if (newData) ok = read(0x94, data, 8);
    uint16_t clear = decode(data);
    bool saturated = status & 0x80;
    uint32_t limit = uint32_t(256U - boundarycfg::ATIME) * 1000UL;
    if (limit > 65535) limit = 65535;
    if (newData) for (uint8_t n = 0; n < 4; ++n)
      saturated |= decode(data + n * 2) >= limit;
    if (status & 0x80) {
      Wire.clearWireTimeoutFlag(); Wire.beginTransmission(0x39); Wire.write(uint8_t(0xE6));
      ok = (Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag()) && ok;
    }
    if (uint32_t(now - sensor.configChecked) >= 250) {
      sensor.configChecked = now;
      if (!configurationOK()) { sensor.configured = false; ok = false; sensor.fault = CONFIG_ERROR; }
    }
    if (!select(0)) return; // Always close the mux before the shared MPU/IR bus is used.
    now = millis();
    bool brightSaturation = saturated && ok && newData && capture_ == -1 &&
                            sensor.threshold.valid;
    if (!ok || (saturated && !brightSaturation)) {
      sensor.good = false;
      if (sensor.configured) sensor.fault = saturated ? SATURATED : READ_ERROR;
      if (newData && ok) sensor.clear = clear;
      return;
    }
    if (!newData) return; // Older valid samples expire via healthy().
    sensor.clear = clear; sensor.fresh = now; sensor.good = true; sensor.fault = NONE;
    sensor.brightSaturated = brightSaturation;
    sensor.white = brightSaturation || line::whiteNow(sensor.white, clear, sensor.threshold);

    if (capture_ == -2 || capture_ == int8_t(i)) {
      Stats &stats = stats_[i];
      if (stats.n < boundarycfg::CAL_SAMPLES) stats.add(clear);
      bool complete = true;
      if (capture_ == -2) {
        for (uint8_t n = 0; n < count; ++n)
          if (stats_[n].n < boundarycfg::CAL_SAMPLES) complete = false;
      } else complete = stats.n >= boundarycfg::CAL_SAMPLES;
      if (complete) finishCapture();
    }
  }

  bool startField() {
    if (!canCapture()) return false;
    for (uint8_t i = 0; i < count; ++i) {
      stats_[i] = Stats{}; sensors_[i].floorReady = false;
      sensors_[i].threshold.valid = sensors_[i].white = false;
    }
    EEPROM.update(boundarycfg::EEPROM_BASE, 0);
    capture_ = -2; captureStart_ = millis();
    Serial.println(F("LINE: collecting FIELD on all four sensors; motors stopped."));
    return true;
  }
  bool startWhite(uint8_t i) {
    if (i >= count || !canCapture()) return false;
    if (!sensors_[i].floorReady) {
      Serial.println(F("LINE: capture field first with f.")); return false;
    }
    stats_[i] = Stats{};
    sensors_[i].threshold.valid = sensors_[i].white = false;
    EEPROM.update(boundarycfg::EEPROM_BASE, 0);
    capture_ = int8_t(i); captureStart_ = millis();
    Serial.print(F("LINE: collecting WHITE for sensor ")); Serial.println(i + 1);
    return true;
  }
  void cancelCapture() { capture_ = -1; }
  bool capturing() const { return capture_ != -1; }
  bool saving() const { return saving_; }
  bool busIsolated() const { return isolated_; }
  bool calibrated() const {
    for (uint8_t i = 0; i < count; ++i) if (!sensors_[i].threshold.valid) return false;
    return true;
  }
  bool healthy(uint32_t now) const {
    if (!running_ || !isolated_) return false;
    for (uint8_t i = 0; i < count; ++i)
      if (!sensors_[i].configured || !sensors_[i].good ||
          uint32_t(now - sensors_[i].fresh) > boundarycfg::MAX_AGE_MS) return false;
    return true;
  }
  uint8_t hitMask() const {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < count; ++i) if (sensors_[i].white) mask |= uint8_t(1U << i);
    return mask;
  }
  void printReadings() const {
    Serial.print(F(" line=0x")); Serial.print(hitMask(), HEX);
    Serial.print(F(" C="));
    for (uint8_t i = 0; i < count; ++i) {
      if (i) Serial.print('/');
      if (sensors_[i].brightSaturated && sensors_[i].good) Serial.print(F("SAT"));
      else Serial.print(sensors_[i].clear);
      if (!sensors_[i].good || uint32_t(millis() - sensors_[i].fresh) > boundarycfg::MAX_AGE_MS)
        Serial.print('!');
    }
    for (uint8_t i = 0; i < count; ++i) {
      const Sensor &sensor = sensors_[i];
      if (sensor.good && uint32_t(millis() - sensor.fresh) <= boundarycfg::MAX_AGE_MS) continue;
      Serial.print(F(" L")); Serial.print(i + 1); Serial.print('=');
      if (sensor.fault == MISSING) Serial.print(F("MISSING"));
      else if (sensor.fault == READ_ERROR) Serial.print(F("READ_ERROR"));
      else if (sensor.fault == SATURATED) Serial.print(F("SATURATED"));
      else if (sensor.fault == CONFIG_ERROR) Serial.print(F("CONFIG_ERROR"));
      else Serial.print(F("STALE/WAIT"));
    }
  }

 private:
  static const uint8_t count = boundarycfg::SENSOR_COUNT;
  static_assert(count == 4, "Calibration commands support four boundary sensors");
  static_assert(boundarycfg::CAL_SAMPLES >= 2, "Calibration needs multiple fresh samples");
  static_assert(boundarycfg::EEPROM_BASE >= 0 && boundarycfg::EEPROM_BASE + 30 <= 4096,
                "Boundary EEPROM storage must fit the Mega's EEPROM");
  enum Fault : uint8_t { NONE, WAITING, MISSING, READ_ERROR, SATURATED, CONFIG_ERROR };
  struct Sensor {
    bool configured = false, good = false, white = false, floorReady = false;
    bool brightSaturated = false;
    uint16_t clear = 0;
    uint32_t fresh = 0, configChecked = 0;
    float floorMean = 0, floorNoise = 0;
    line::Threshold threshold = {0, 0, false};
    Fault fault = WAITING;
  };
  struct Stats {
    uint16_t n = 0;
    float mean = 0, m2 = 0;
    void add(uint16_t value) {
      ++n; float delta = value - mean;
      mean += delta / n; m2 += delta * (value - mean);
    }
    float noise() const { return n > 1 ? sqrtf(m2 / (n - 1)) : 0; }
  };
  bool muxFailure() {
    isolated_ = false;
    for (uint8_t i = 0; i < count; ++i) sensors_[i].good = false;
    return false;
  }
  bool select(uint8_t mask) {
    Wire.clearWireTimeoutFlag(); Wire.beginTransmission(boundarycfg::TCA_ADDRESS);
    Wire.write(mask);
    if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) return muxFailure();
    uint8_t received = Wire.requestFrom(boundarycfg::TCA_ADDRESS, uint8_t(1));
    if (received != 1 || Wire.getWireTimeoutFlag()) {
      while (Wire.available()) Wire.read();
      return muxFailure();
    }
    if (uint8_t(Wire.read()) != mask) return muxFailure();
    isolated_ = mask == 0;
    return true;
  }
  bool read(uint8_t reg, uint8_t *data, uint8_t length) {
    Wire.clearWireTimeoutFlag(); Wire.beginTransmission(0x39); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t received = Wire.requestFrom(uint8_t(0x39), length);
    if (received != length || Wire.getWireTimeoutFlag()) {
      while (Wire.available()) Wire.read();
      return false;
    }
    for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
    return true;
  }
  bool write(uint8_t reg, uint8_t value) {
    Wire.clearWireTimeoutFlag(); Wire.beginTransmission(0x39);
    Wire.write(reg); Wire.write(value);
    return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
  }
  bool configurationOK() {
    uint8_t enable = 0, atime = 0, gain = 0;
    return read(0x80, &enable, 1) && enable == 3 &&
           read(0x81, &atime, 1) && atime == boundarycfg::ATIME &&
           read(0x8F, &gain, 1) && gain == boundarycfg::GAIN;
  }
  static uint16_t decode(const uint8_t *data) {
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
  }
  bool canCapture() const {
    if (!saving_ && !capturing() && healthy(millis())) return true;
    Serial.println(F("LINE not ready: wait for fresh sensors, or b=rescan."));
    return false;
  }
  void finishCapture() {
    int8_t mode = capture_; capture_ = -1;
    if (mode == -2) {
      for (uint8_t i = 0; i < count; ++i) {
        sensors_[i].floorMean = stats_[i].mean;
        sensors_[i].floorNoise = stats_[i].noise();
        sensors_[i].floorReady = true;
        Serial.print(F("FIELD sensor ")); Serial.print(i + 1);
        Serial.print(F(" mean=")); Serial.print(stats_[i].mean, 1);
        Serial.print(F(" noise=")); Serial.println(stats_[i].noise(), 1);
      }
      Serial.println(F("Now put each sensor over white and send its number 1..4."));
      return;
    }
    uint8_t i = uint8_t(mode);
    Sensor &sensor = sensors_[i];
    sensor.threshold = line::thresholds(sensor.floorMean, stats_[i].mean,
                                       sensor.floorNoise, stats_[i].noise());
    Serial.print(F("WHITE sensor ")); Serial.print(i + 1);
    Serial.print(F(" mean=")); Serial.print(stats_[i].mean, 1);
    if (!sensor.threshold.valid) {
      Serial.println(F(" LOW CONTRAST/NOISY: check light/height; repeat field/white."));
      return;
    }
    Serial.print(F(" trip=")); Serial.print(sensor.threshold.on);
    Serial.print(F(" release=")); Serial.println(sensor.threshold.off);
    if (calibrated()) startSave();
  }
  bool loadCalibration() {
    if (boundarycfg::EEPROM_BASE < 0 || boundarycfg::EEPROM_BASE + 30 > EEPROM.length()) return false;
    for (uint8_t i = 0; i < 30; ++i) record_[i] = EEPROM.read(boundarycfg::EEPROM_BASE + i);
    if (record_[0] != 'L' || record_[1] != 'I' || record_[2] != 'N' || record_[3] != 'E' ||
        record_[4] != 1 || record_[5] != boundarycfg::ATIME || record_[6] != boundarycfg::GAIN ||
        record_[7] != count || line::crc16(record_, 28) != decode(record_ + 28)) return false;
    for (uint8_t i = 0; i < count; ++i) {
      if (record_[8 + i] != boundarycfg::CHANNELS[i] ||
          decode(record_ + 12 + i * 4) <= decode(record_ + 14 + i * 4)) return false;
    }
    for (uint8_t i = 0; i < count; ++i)
      sensors_[i].threshold = {decode(record_ + 12 + i * 4), decode(record_ + 14 + i * 4), true};
    return true;
  }
  void startSave() {
    record_[0] = 'L'; record_[1] = 'I'; record_[2] = 'N'; record_[3] = 'E';
    record_[4] = 1; record_[5] = boundarycfg::ATIME;
    record_[6] = boundarycfg::GAIN; record_[7] = count;
    for (uint8_t i = 0; i < count; ++i) {
      record_[8 + i] = boundarycfg::CHANNELS[i];
      uint16_t on = sensors_[i].threshold.on, off = sensors_[i].threshold.off;
      record_[12 + i * 4] = uint8_t(on); record_[13 + i * 4] = uint8_t(on >> 8);
      record_[14 + i * 4] = uint8_t(off); record_[15 + i * 4] = uint8_t(off >> 8);
    }
    uint16_t crc = line::crc16(record_, 28);
    record_[28] = uint8_t(crc); record_[29] = uint8_t(crc >> 8);
    saving_ = true; saveStep_ = 0;
    Serial.println(F("LINE: saving calibration; motors remain stopped."));
  }
  void saveOneByte() {
    // Invalidate first; commit the magic byte last after the checksummed payload.
    if (saveStep_ == 0) EEPROM.update(boundarycfg::EEPROM_BASE, 0);
    else if (saveStep_ < 30)
      EEPROM.update(boundarycfg::EEPROM_BASE + saveStep_, record_[saveStep_]);
    else {
      EEPROM.update(boundarycfg::EEPROM_BASE, record_[0]); saving_ = false;
      Serial.println(F("LINE calibration saved. Reposition on field, then g=start."));
    }
    ++saveStep_;
  }
  Sensor sensors_[count];
  Stats stats_[count];
  uint8_t record_[30] = {};
  bool isolated_, running_, saving_;
  uint8_t next_, saveStep_;
  int8_t capture_;
  uint32_t lastSlot_, captureStart_;
};
#endif
