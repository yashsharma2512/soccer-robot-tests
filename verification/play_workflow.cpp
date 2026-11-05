// Exercise both maintained entry points with the same actual-sketch play tests.
#include "fakes/Arduino.h"
#include "fakes/Wire.h"
#include "fakes/EEPROM.h"
#include <cstdlib>
#include <iostream>
uint64_t fakeUs = 0;
int fakePwm[80] = {};
FakeSerial Serial;
FakeWire Wire;
FakeEEPROM EEPROM;
#ifdef TEST_LINUX_FULL
#include "../sketches/test code linux full/test code linux full.ino"
#else
#include "../sketches/MegaBallSearchNoTurn/MegaBallSearchNoTurn.ino"
#endif
void check(bool value, const char *message) {
  if (!value) { std::cerr << "FAIL: " << message << '\n' << Serial.output; std::exit(1); }
}
void step() { fakeUs += 5000; loop(); }
bool isState(const char *name) {
  return std::string(reinterpret_cast<const char *>(state)) == name;
}
template<class Predicate> void until(Predicate done, int remaining = 1800) {
  while (!done() && remaining-- > 0) step();
  check(done(), "waiting for a play state timed out");
}
void zeroPwm() { for (uint8_t pin : MOTOR_PINS) check(fakePwm[pin] == 0, "motor input should be zero"); }
void ball(bool visible) { Wire.ir[0x49] = visible ? 5 : 0; Wire.ir[0x4C] = 120; }
int main() {
  setup();
  until([] { return imu.healthy(millis()) && boundary.healthy(millis()); });
#ifdef TEST_LINUX_FULL
  executeCommand('g');
#else
  executeCommand('f');
  until([] { return boundary.capturing(); });
  until([] { return !boundary.capturing(); });
  for (int i = 0; i < 4; ++i) {
    Wire.clear(i, 1000); executeCommand(char('1' + i));
    until([] { return !boundary.capturing(); }); Wire.clear(i, 100);
  }
#endif
  until([] { return boundary.calibrated() && !boundary.capturing() && !boundary.saving(); });
#ifndef TEST_LINUX_FULL
  executeCommand('g');
#endif
  check(enabled, "calibrated play must enable");
  ball(false);
  int rounds = 0;
  bool wasPause = false;
  uint32_t start = millis();
  while (uint32_t(millis() - start) < 10000) {
    step();
    check(!isState("SEARCH_DONE"), "no permanent search completion is allowed");
    if (isState("SEARCH_PAUSE") && !wasPause) ++rounds;
    wasPause = isState("SEARCH_PAUSE");
  }
  check(rounds >= 4 && enabled, "search must keep repeating for more than four rounds");
  ball(true); until([] { return isState("FOLLOW"); });
  check(wheels.fl > 0 && wheels.fr > 0, "centred ball should follow forward");
  const int8_t expectedForward[4] = {-1, 0, 1, 0};
  const int8_t expectedStrafe[4] = {0, -1, 0, 1};
  for (int i = 0; i < 4; ++i) {
    Wire.clear(i, 1000); until([] { return isState("LINE_ESCAPE"); });
    auto expected = drive::mix({expectedForward[i] * boundarycfg::ESCAPE_PWM,
                               expectedStrafe[i] * boundarycfg::ESCAPE_PWM, 0},
                              boundarycfg::ESCAPE_WHEEL_CAP);
    check(wheels.fl == expected.fl && wheels.fr == expected.fr &&
          wheels.bl == expected.bl && wheels.br == expected.br, "escape must move away from the detected side");
    Wire.clear(i, 100); until([] { return isState("LINE_CLEARED"); }); zeroPwm();
    until([] { return isState("FOLLOW"); });
  }
  Wire.clear(0, 1000); Wire.clear(1, 1000);
  until([] { return boundary.hitMask() == 3 && isState("LINE_ESCAPE"); });
  check(wheels.fl < 0 && wheels.br < 0, "adjacent front/right hits should retreat diagonally");
  Wire.clear(1, 100);
  until([] { return boundary.hitMask() == 1 && isState("LINE_ESCAPE"); });
  until([] { return isState("LINE_HOLD"); }); zeroPwm();
  until([] { return isState("LINE_ESCAPE"); });
  check(enabled, "long boundary encounter should recover without disabling play");
  Wire.clear(2, 1000);
  until([] { return (boundary.hitMask() & 5) == 5 && isState("LINE_HOLD"); }); zeroPwm();
  for (int i = 0; i < 20; ++i) { step(); zeroPwm(); }
  Wire.clear(2, 100); until([] { return isState("LINE_ESCAPE"); });
  check(wheels.fl < 0 && wheels.fr < 0, "resolved ambiguity must resume moving away from the remaining front line");
  Wire.clear(0, 100); ball(false);
  until([] { return isState("LINE_CLEARED"); });
  until([] { return isState("SEARCH"); });
  Wire.clear(3, 1000); until([] { return isState("LINE_ESCAPE"); });
  check(wheels.fl > 0 && wheels.fr < 0, "line must override search even without a ball");
  Wire.clear(3, 6500);
  for (int i = 0; i < 15; ++i) step();
  check(isState("LINE_ESCAPE") || isState("LINE_HOLD"), "calibrated bright saturation must remain a boundary detection");
  Wire.clear(3, 100); until([] { return isState("SEARCH"); });
  Wire.irPresent = false; until([] { return isState("IR_FAULT"); }); zeroPwm();
  Wire.irPresent = true; until([] { return isState("SEARCH"); });
  Wire.missingApds = 1; until([] { return isState("LINE_SENSOR_FAULT"); }); zeroPwm();
  Wire.missingApds = -1; executeCommand('b');
  until([] { return imu.healthy(millis()) && boundary.healthy(millis()); });
#ifndef TEST_LINUX_FULL
  executeCommand('g');
#endif
  until([] { return isState("SEARCH"); });
  Serial.input.push_back('s'); Serial.input.push_back('\n'); step(); zeroPwm();
  for (int i = 0; i < 200; ++i) { step(); zeroPwm(); }
  check(!enabled, "manual stop must remain stopped during continuous search");
#ifdef TEST_LINUX_FULL
  executeCommand('r');
#else
  executeCommand('g');
#endif
  until([] { return isState("SEARCH"); });
  Wire.clear(0, 1000); until([] { return isState("LINE_ESCAPE"); });
  Serial.input.push_back('s'); Serial.input.push_back('\n'); step(); zeroPwm();
  for (int i = 0; i < 50; ++i) { step(); zeroPwm(); }
  check(!enabled, "manual stop must override active line escape");
#ifdef TEST_LINUX_FULL
  executeCommand('r');
#else
  executeCommand('g');
#endif
  until([] { return isState("LINE_ESCAPE"); });
  Wire.mpuPresent = false;
  until([] { return isState("MPU_FAULT"); }); zeroPwm();
  std::cout << "PASS: repeated search, follow/reacquisition, four escape directions, corners, timeout retry,\n"
               "      ambiguity hold/recovery, saturation, escape during search, sensor faults, and manual stops.\n";
}
