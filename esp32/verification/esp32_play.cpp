#include "fakes/Arduino.h"
#include "fakes/Wire.h"
#include "fakes/Preferences.h"
#include <cstdlib>
#include <iostream>
#include <algorithm>
uint64_t fakeUs=0;
int fakePwm[80]={};
bool fakeAttached[80]={};
int fakeAttachFailurePin=-1,fakeWriteFailurePin=-1;
FakeSerial Serial;
FakeWire Wire;
FakeNVS fakeNvs;
#include "../ESP32SoccerFull/ESP32SoccerFull.ino"
void check(bool good,const char *message) {
  if (!good) { std::cerr<<"FAIL "<<message<<'\n'<<Serial.output; std::exit(1); }
}
bool mode(const char *text) { return state && std::string(reinterpret_cast<const char *>(state))==text; }
void zero() { for (uint8_t pin:MOTOR_PINS) check(fakePwm[pin]==0,"all motor inputs must be zero"); }
void step() { fakeUs+=4000; loop(); }
template<class Predicate> void until(Predicate done) {
  int remaining=2500;
  while (!done() && remaining-->0) step();
  check(done(),"state timeout");
}
void reboot() {
  fakeUs=0; Serial=FakeSerial(); Wire=FakeWire();
  std::fill(fakePwm,fakePwm+80,0); std::fill(fakeAttached,fakeAttached+80,false);
  calibstore::prefs=Preferences(); calibstore::ready=false; calibstore::generation=0;
  imu=MpuYaw(); boundary=BoundarySensors(); lineEscape=line::Escape();
  bus::isolated=false; pwmReady=busInitialized=enabled=headingLocked=searchStarted=false;
  seenStorageGeneration=0; lastBallSeenMs=searchStartMs=lastIrPollMs=lastReportMs=0;
  preferredSide=1; targetYaw=0; lastDirection=0; lastStrength=0;
  wheels={0,0,0,0}; reportedMpuError=MpuYaw::OK; setup();
}
void ball(bool present) { Wire.ir[0x49]=present ? 5 : 0; Wire.ir[0x4C]=120; }
int main() {
  reboot();
  const uint8_t expected[]={25,26,16,17,32,33,18,19};
  check(std::equal(MOTOR_PINS,MOTOR_PINS+8,expected),"ESP32 pin map");
  until([] { return imu.healthy(millis()) && boundary.healthy(millis()); }); zero();
  check(fakeNvs.blobs.count("gyro") && !fakeNvs.blobs.count("line"),"separate gyro blob");
  executeCommand('g'); check(boundary.preparing(),"green preparation"); zero();
  until([] { return boundary.calibrated() && !boundary.capturing() && !boundary.saving() && imu.healthy(millis()); });
  check(fakeNvs.blobs.count("line") && imu.error()==MpuYaw::OK,"NVS flash latency must not fault gyro");
  auto saved=fakeNvs.blobs; unsigned writes=fakeNvs.writes;
  reboot();
  check(imu.loaded() && boundary.calibrated(),"both NVS calibrations reload");
  check(fakeNvs.writes==writes,"boot load must not rewrite flash");
  until([] { return mode("SEARCH"); });
  int pauses=0; bool previous=false; uint32_t start=millis();
  while (uint32_t(millis()-start)<10000) { step(); if (mode("SEARCH_PAUSE")&&!previous) ++pauses; previous=mode("SEARCH_PAUSE"); }
  check(pauses>=4,"continuous repeat search");
  ball(true); until([] { return mode("FOLLOW"); });
  check(fakePwm[FL1]>0 && fakePwm[FR1]>0 && fakePwm[BL2]>0 && fakePwm[BR1]>0,"forward plus BL inversion");
  const float forward[]={-1,0,1,0},strafe[]={0,-1,0,1};
  for (int i=0;i<4;i++) {
    Wire.clear(i,1000); until([] { return mode("LINE_ESCAPE"); });
    auto wanted=drive::mix({forward[i]*boundarycfg::ESCAPE_PWM,strafe[i]*boundarycfg::ESCAPE_PWM,0},boundarycfg::ESCAPE_WHEEL_CAP);
    check(wheels.fl==wanted.fl && wheels.fr==wanted.fr && wheels.bl==wanted.bl && wheels.br==wanted.br,"escape side");
    Wire.clear(i,100); until([] { return mode("FOLLOW"); });
  }
  Wire.clear(0,1000); until([] { return mode("LINE_ESCAPE"); });
  until([] { return mode("LINE_HOLD"); }); zero();
  until([] { return mode("LINE_ESCAPE"); });
  Wire.clear(2,1000); until([] { return (boundary.hitMask()&5)==5 && mode("LINE_HOLD"); }); zero();
  Wire.clear(2,100); until([] { return mode("LINE_ESCAPE"); });
  Wire.clear(0,100); until([] { return mode("FOLLOW"); });
  Wire.transientMpuFailures=1; unsigned retries=imu.recoveredReads();
  until([&] { return imu.recoveredReads()>retries; });
  check(imu.healthy(millis()),"one transient read retries without fake gyro data");
  Wire.irPresent=false; until([] { return mode("IR_FAULT"); }); zero();
  Wire.irPresent=true; until([] { return mode("FOLLOW"); });
  Wire.failMux=true; until([] { return mode("LINE_MUX_FAULT"); }); zero();
  Wire.failMux=false; executeCommand('b'); until([] { return mode("FOLLOW"); });
  Serial.input.push_back('s'); Serial.input.push_back('\n'); step(); zero();
  for (int i=0;i<80;i++) { step(); zero(); } check(!enabled,"manual stop");
  fakeNvs.blobs=saved; fakeNvs.blobs["gyro"][10]^=1; reboot();
  check(!imu.loaded() && imu.calibrating(),"invalid CRC triggers recalibration");
  check(boundary.calibrated(),"bad gyro must preserve line blob");
  fakeNvs.blobs=saved; reboot(); until([] { return imu.healthy(millis()); });
  executeCommand('c'); fakeNvs.failPut=true;
  until([] { return mode("NVS_FAULT"); }); zero(); fakeNvs.failPut=false;
  fakeNvs.blobs=saved; fakeAttachFailurePin=FR1; reboot(); step();
  check(mode("PWM_FAULT"),"attach failure must block startup"); zero(); fakeAttachFailurePin=-1;
  reboot(); ball(true); until([] { return mode("FOLLOW"); });
  fakeWriteFailurePin=FL1; step();
  until([] { return mode("PWM_FAULT"); }); zero(); fakeWriteFailurePin=-1;
  fakeNvs.failBegin=true; reboot(); step(); check(mode("NVS_FAULT"),"NVS initialization fault"); zero(); fakeNvs.failBegin=false;
  fakeNvs.blobs=saved; reboot(); Wire.mpuPresent=false;
  until([] { return mode("MPU_FAULT"); }); zero();
  std::cout<<"PASS: ESP32 direct IR/MPU rails, APDS mux, eight PWM pins, NVS save/load and flash stalls,\n"
              "      search/follow/escape, retries, CRC, storage/PWM/I2C faults, manual stop.\n";
}
