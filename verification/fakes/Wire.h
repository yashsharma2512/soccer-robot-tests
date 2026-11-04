#pragma once
#include "Arduino.h"
#include <algorithm>
#include <vector>
struct FakeWire {
  uint8_t mpu[256] = {}, apds[4][256] = {}, ir[256] = {};
  uint8_t mux = 0, address = 0, reg = 0;
  bool mpuPresent = true;
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  FakeWire() {
    mpu[0x75] = 0x70;
    mpu[0x3A] = 1; mpu[0x3F] = 0x40; // Data ready; stationary 1 g on Z.
    gyro(131);
    for (int i = 0; i < 4; ++i) { apds[i][0x92] = 0x9E; apds[i][0x93] = 1; clear(i, 100); }
  }
  void gyro(int16_t value) { mpu[0x47] = uint8_t(uint16_t(value) >> 8); mpu[0x48] = uint8_t(value); }
  void clear(int channel, uint16_t value) {
    apds[channel][0x94] = uint8_t(value); apds[channel][0x95] = uint8_t(value >> 8);
  }
  uint8_t *device() {
    if (address == 0x68 && mpuPresent) return mpu;
    if (address == 0x08) return ir;
    if (address == 0x39) for (int i = 0; i < 4; ++i) if (mux & (1 << i)) return apds[i];
    return nullptr;
  }
  void begin() {}
  void setClock(int) {}
  void setWireTimeout(int, bool) {}
  void clearWireTimeoutFlag() {}
  bool getWireTimeoutFlag() { return false; }
  void beginTransmission(uint8_t target) { address = target; tx.clear(); }
  void write(uint8_t value) { tx.push_back(value); }
  uint8_t endTransmission(bool = true) {
    if (address == 0x70) { if (!tx.empty()) mux = tx[0]; return 0; }
    uint8_t *registers = device();
    if (!registers) return 2;
    if (!tx.empty()) {
      reg = tx[0];
      for (size_t i = 1; i < tx.size(); ++i) registers[uint8_t(reg + i - 1)] = tx[i];
    }
    return 0;
  }
  uint8_t requestFrom(uint8_t target, uint8_t count) {
    address = target; rx.clear();
    if (address == 0x70) { rx.push_back(mux); return 1; }
    uint8_t *registers = device();
    if (!registers) return 0;
    for (uint8_t i = 0; i < count; ++i) rx.push_back(registers[uint8_t(reg + i)]);
    return count;
  }
  int available() { return int(rx.size()); }
  int read() { uint8_t value = rx.front(); rx.pop_front(); return value; }
};
extern FakeWire Wire;
