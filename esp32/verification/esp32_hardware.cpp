#include "fakes/Arduino.h"
#include "fakes/Wire.h"
#include <cstdlib>
#include <iostream>
#include <algorithm>
uint64_t fakeUs=0;
int fakePwm[80]={};
bool fakeAttached[80]={};
int fakeAttachFailurePin=-1,fakeWriteFailurePin=-1;
FakeSerial Serial;
FakeWire Wire;
#include "../ESP32HardwareTest/ESP32HardwareTest.ino"
void check(bool good,const char *message) {
  if (!good) { std::cerr<<"FAIL "<<message<<'\n'<<Serial.output; std::exit(1); }
}
void zero() { for (uint8_t pin:MOTOR_PINS) check(fakePwm[pin]==0,"motor output must be zero"); }
void step() { fakeUs+=4000; loop(); }
void command(const char *text) { while (*text) Serial.input.push_back(*text++); step(); }
int main() {
  setup(); zero();
  for (int i=0;i<300;i++) { step(); zero(); }
  check(Serial.output.find("IR MAIN")!=std::string::npos,"IR remains on main rails");
  check(Serial.output.find("MPU MAIN")!=std::string::npos,"MPU remains on main rails");
  command("1"); zero(); check(!active,"partial motor command must not run");
  command("\n"); check(active && fakePwm[25]==70,"FL forward pulse");
  Wire.failMux=true;
  while (active) step(); zero(); check(uint32_t(millis()-pulseStart)<=405,"pulse timer must not block on I2C");
  Wire.failMux=false;
  command("2\n"); check(active && fakePwm[16]==70 && fakePwm[17]==0,"FR forward uses GPIO16");
  command("6\n"); check(active && fakePwm[17]==70 && fakePwm[16]==0,"FR reverse uses GPIO17");
  check(fakePwm[27]==0 && fakePwm[14]==0,"old FR pins remain unused");
  command("3\n"); check(active && fakePwm[33]==70,"BL sign inversion");
  command("s\n"); zero(); check(!active,"stop overrides pulse");
  command("8\n"); check(active && fakePwm[19]==70,"BR reverse pulse");
  command("11\n"); zero(); check(!active,"invalid multi-character command stops");
  command("1\n"); fakeWriteFailurePin=25;
  command("s\n"); zero(); check(!pwmReady,"write failure must detach all PWM outputs");
  std::cout<<"PASS: hardware-test main-rail routing, idle stop, pin/sign pulse mapping, bounded pulses,\n"
              "      newline validation, manual stop, PWM failure shutdown.\n";
}
