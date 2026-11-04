#pragma once
#include "Arduino.h"
#include <algorithm>
#include <vector>
struct FakeEEPROM {
  uint8_t bytes[4096];
  bool dropWrites = false;
  std::vector<int> writes;
  FakeEEPROM() { std::fill(bytes, bytes + 4096, uint8_t(255)); }
  int length() { return 4096; }
  uint8_t read(int address) { return bytes[address]; }
  void update(int address, uint8_t value) {
    if (bytes[address] == value) return;
    writes.push_back(address);
    fakeUs += 3300; // Model EEPROM byte-write latency while the control loop runs.
    if (!dropWrites) bytes[address] = value;
  }
};
extern FakeEEPROM EEPROM;
