// Runs the actual main sketch with simulated registers, time, and EEPROM.
#include "fakes/Arduino.h"
#include "fakes/Wire.h"
#include "fakes/EEPROM.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
uint64_t fakeUs = 0;
int fakePwm[80] = {};
FakeSerial Serial;
FakeWire Wire;
FakeEEPROM EEPROM;
#include "../sketches/MegaBallSearchNoTurn/MegaBallSearchNoTurn.ino"

void require(bool condition, const char *message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n' << Serial.output; std::exit(1); }
}
void stopped() {
  require(!enabled, "calibration must keep following disabled");
  for (uint8_t pin : MOTOR_PINS) require(fakePwm[pin] == 0, "calibration must keep every motor input LOW");
}
void step() { fakeUs += 5000; loop(); }
template<class Predicate> void until(Predicate done, int limit = 1800) {
  while (!done() && limit-- > 0) step();
  require(done(), "timed out waiting for calibration state");
}
void reboot() {
  // Preserve EEPROM, replace volatile MCU and sensor state.
  fakeUs = 0; Wire = FakeWire(); Serial = FakeSerial();
  imu = MpuYaw(); boundary = BoundarySensors(); lineEscape = line::Escape();
  enabled = headingLocked = fieldAfterGyro = false;
  wheels = {0, 0, 0, 0}; targetYaw = 0;
  lastIrPollMs = lastReportMs = 0; lastDirection = 0; lastStrength = 0;
  reportedMpuError = MpuYaw::OK;
  setup();
}
void refreshGyroCRC() {
  uint16_t crc = line::crc16(EEPROM.bytes + cfg::GYRO_EEPROM_BASE, 14);
  EEPROM.bytes[cfg::GYRO_EEPROM_BASE + 14] = uint8_t(crc);
  EEPROM.bytes[cfg::GYRO_EEPROM_BASE + 15] = uint8_t(crc >> 8);
}
int main() {
  reboot();
  require(imu.calibrating() && !imu.loaded(), "empty EEPROM must trigger fresh gyro calibration");
  executeCommand('f');
  require(fieldAfterGyro && !boundary.capturing(), "f must calibrate gyro before FIELD");
  executeCommand('1');
  require(fieldAfterGyro && !boundary.capturing(), "early white command must not bypass gyro");
  executeCommand('g'); stopped();
  until([] { return imu.saving(); });
  require(!boundary.capturing() && !imu.healthy(millis()), "pending EEPROM save must block capture and drive");
  until([] { return boundary.capturing(); }); stopped();
  require(EEPROM.read(cfg::GYRO_EEPROM_BASE) == 'G', "gyro must be committed before FIELD starts");
  auto savedAt = Serial.output.find("MPU calibration saved to EEPROM.");
  auto fieldAt = Serial.output.find("LINE: collecting FIELD");
  require(savedAt != std::string::npos && savedAt < fieldAt, "save message must precede FIELD capture");
  until([] { return !boundary.capturing(); });
  for (int i = 0; i < 4; ++i) {
    Wire.clear(i, 1000);
    executeCommand(char('1' + i));
    require(boundary.capturing(), "white capture must start after FIELD completes");
    until([] { return !boundary.capturing(); });
    Wire.clear(i, 100);
  }
  until([] { return !boundary.saving(); });
  require(boundary.calibrated() && EEPROM.read(0) == 'L', "line thresholds must save separately");
  uint8_t good[4096]; std::copy(EEPROM.bytes, EEPROM.bytes + 4096, good);
  size_t writesBeforeLoad = EEPROM.writes.size();
  reboot();
  require(imu.loaded() && !imu.calibrating() && boundary.calibrated(), "both calibrations must load after power cycle");
  require(EEPROM.writes.size() == writesBeforeLoad, "loading valid records must not rewrite EEPROM");
  require(!imu.healthy(millis()), "loaded gyro needs fresh data before motion");
  until([] { return imu.healthy(millis()) && boundary.healthy(millis()); });
  require(std::fabs(imu.biasRaw() - 131) < 0.001f && std::fabs(imu.yaw()) < 0.001f,
          "saved bias must cancel stationary rotation without a startup yaw jump");
  require(imu.progress() == 0, "valid EEPROM must skip 500-sample gyro calibration");
  executeCommand('g'); require(enabled, "healthy saved calibrations must allow explicit g");
  executeCommand('f'); stopped();
  Serial.input.push_back('s'); Serial.input.push_back('\n'); step();
  require(!fieldAfterGyro && !boundary.capturing() && !imu.calibrating(), "stop must cancel pending FIELD sequence");
  for (int i = 0; i < 700; ++i) step();
  require(!boundary.capturing() && EEPROM.read(64) == 0, "cancelled fresh gyro must not restore an old record"); stopped();

  std::copy(good, good + 4096, EEPROM.bytes); reboot();
  executeCommand('f'); Wire.mpu[0x3F] = 0; // Not stationary: no 1 g acceleration.
  until([] { return imu.error() == MpuYaw::NOT_STILL; }); step();
  require(!fieldAfterGyro && !boundary.capturing(), "failed gyro must never start colour capture"); stopped();

  std::copy(good, good + 4096, EEPROM.bytes); reboot();
  Wire.mpu[0x3A] = 0;
  until([] { return imu.error() == MpuYaw::NO_NEW_DATA; });
  executeCommand('g'); stopped();
  require(!imu.healthy(millis()), "saved bias must not bypass missing fresh MPU data");

  std::copy(good, good + 4096, EEPROM.bytes); reboot();
  Wire.mpuPresent = false; executeCommand('f'); step();
  require(imu.error() == MpuYaw::NOT_FOUND && !fieldAfterGyro && !boundary.capturing(),
          "saved bias must not bypass missing MPU hardware"); stopped();

  std::copy(good, good + 4096, EEPROM.bytes); reboot();
  executeCommand('f'); until([] { return imu.saving(); });
  executeCommand('s');
  require(!imu.saving() && !fieldAfterGyro && EEPROM.read(64) == 0, "stop during save must invalidate and cancel it");
  for (int i = 0; i < 25; ++i) step();
  require(!boundary.capturing(), "cancelled save must never start FIELD"); stopped();

  std::copy(good, good + 4096, EEPROM.bytes); reboot();
  executeCommand('f'); until([] { return imu.saving(); });
  for (int i = 0; i < 4; ++i) step();
  reboot();
  require(!imu.loaded() && imu.calibrating(), "interrupted write must be rejected after reset");

  std::copy(good, good + 4096, EEPROM.bytes); EEPROM.bytes[74] ^= 1; reboot();
  require(!imu.loaded() && imu.calibrating(), "bad gyro CRC must trigger fresh calibration");
  require(boundary.calibrated(), "bad gyro record must not damage line record");
  std::copy(good, good + 4096, EEPROM.bytes); EEPROM.bytes[70] = 0x68; refreshGyroCRC(); reboot();
  require(!imu.loaded(), "a different MPU identity must reject the saved bias even with a valid CRC");
  std::copy(good, good + 4096, EEPROM.bytes); EEPROM.bytes[71] = 8; refreshGyroCRC(); reboot();
  require(!imu.loaded(), "a different gyro range must reject the saved bias");
  std::copy(good, good + 4096, EEPROM.bytes);
  float invalidBias = NAN; memcpy(EEPROM.bytes + 74, &invalidBias, 4); refreshGyroCRC(); reboot();
  require(!imu.loaded(), "NaN bias must be rejected even with a valid CRC");

  std::copy(good, good + 4096, EEPROM.bytes); reboot();
  executeCommand('c'); Wire.gyro(-262);
  until([] { return imu.healthy(millis()); });
  require(!boundary.capturing() && std::fabs(imu.biasRaw() + 262) < 0.001f, "c must save gyro only");
  require(std::equal(good, good + 30, EEPROM.bytes), "gyro save must preserve APDS EEPROM bytes");
  reboot(); until([] { return imu.healthy(millis()); });
  require(imu.loaded() && std::fabs(imu.biasRaw() + 262) < 0.001f, "negative saved offsets must survive reset");

  executeCommand('f'); until([] { return imu.saving(); });
  EEPROM.dropWrites = true;
  until([] { return imu.error() == MpuYaw::SAVE_FAILED; }); step();
  require(!fieldAfterGyro && !boundary.capturing(), "failed EEPROM verification must block FIELD"); stopped();
  EEPROM.dropWrites = false;
  for (int address : EEPROM.writes)
    require((address >= 0 && address < 30) || (address >= 64 && address < 80), "calibration writes must stay in reserved records");
  std::cout << "PASS: actual main sketch gyro-first workflow, save/reload, motion guards, stop/fault cancellation,\n"
               "      interrupted writes, CRC/identity/config/NaN rejection, negative bias, and EEPROM isolation.\n";
}
