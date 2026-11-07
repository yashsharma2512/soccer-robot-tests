#include "fakes/Arduino.h"
#include "fakes/Wire.h"
#include "fakes/Preferences.h"
#include <cstdlib>
#include <iostream>
#include <vector>
uint64_t fakeUs=0;
int fakePwm[80]={};bool fakeAttached[80]={};
int fakeAttachFailurePin=-1,fakeWriteFailurePin=-1;
FakeSerial Serial;FakeWire Wire;FakeNVS fakeNvs;
void check(bool good,const char *message) { if(!good) { std::cerr<<"FAIL "<<message<<'\n'<<Serial.output;std::exit(1); } }
void zero() { const uint8_t pins[]={25,26,16,17,32,33,18,19};for(uint8_t p:pins) check(fakePwm[p]==0,"all motor inputs zero"); }
class CheckedPreferences:public Preferences {
 public:
  size_t putBytes(const char *key,const void *data,size_t length) { zero();return Preferences::putBytes(key,data,length); }
  bool remove(const char *key) { zero();return Preferences::remove(key); }
};
#define Preferences CheckedPreferences
#include "../ESP32SoccerAP/PlayRuntime.h"
#undef Preferences
std::vector<std::string> events;
void playEvent(const char *message) { events.push_back(message); }
void step() { fakeUs+=5000;motors.tick();stepPlay();motors.tick(); }
bool mode(const char *text) { return state && std::string(reinterpret_cast<const char *>(state))==text; }
template<class Predicate> void until(Predicate p) {
  int limit=3000;while(!p() && limit-->0) step();check(p(),"state timeout");
}
void command(char c) { check(requests.submit(c),"control request accepted");step(); }
void ball(bool present) { Wire.ir[0x49]=present ? 5 : 0;Wire.ir[0x4C]=120; }
int main() {
  initializePlay();zero();
  until([] { return imu.healthy(millis()) && boundary.healthy(millis()); });
  check(!enabled && !boundary.calibrated(),"first boot cannot drive before green calibration");zero();
  command('f');check(requests.blocked(),"combined gyro/green calibration busy");
  check(!requests.submit('r'),"resume rejected while calibration busy");
  until([] { return !requests.blocked() && calibrationStage==0 && boundary.calibrated() && imu.healthy(millis()); });
  check(!enabled && !motors.state().enabled,"combined calibration remains stopped");zero();
  check(fakeNvs.blobs.count("gyro") && fakeNvs.blobs.count("line"),"both records saved");
  ball(true);command('r');until([] { return mode("FOLLOW") && motors.state().active; });
  check(fakePwm[25]>0 && fakePwm[16]>0 && fakePwm[33]>0 && fakePwm[18]>0,"forward pins and BL inversion");
  check(fakePwm[14]==0 && fakePwm[27]==0,"old FR pins unused");
  // Calibration is requested during active movement, not only after manual stop.
  command('g');check(!enabled && boundary.preparing(),"AP green request stops active play");zero();
  until([] { return !requests.blocked() && boundary.calibrated() && imu.healthy(millis()); });
  check(!enabled,"no auto-resume after green capture");zero();
  command('r');until([] { return mode("FOLLOW"); });
  const float forward[]={-1,0,1,0},strafe[]={0,-1,0,1};
  for(int i=0;i<4;i++) {
    Wire.clear(i,1000);until([] { return mode("LINE_ESCAPE"); });
    auto expected=drive::mix({forward[i]*boundarycfg::ESCAPE_PWM,strafe[i]*boundarycfg::ESCAPE_PWM,0},boundarycfg::ESCAPE_WHEEL_CAP);
    check(wheels.fl==expected.fl && wheels.fr==expected.fr && wheels.bl==expected.bl && wheels.br==expected.br,"line escape moves away from mapped side");
    Wire.clear(i,100);until([] { return mode("FOLLOW"); });
  }
  Wire.clear(0,1000);until([] { return mode("LINE_ESCAPE"); });until([] { return mode("LINE_HOLD"); });zero();
  until([] { return mode("LINE_ESCAPE"); });Wire.clear(2,1000);until([] { return (boundary.hitMask()&5)==5 && mode("LINE_HOLD"); });zero();
  Wire.clear(0,100);Wire.clear(2,100);until([] { return mode("FOLLOW"); });
  ball(false);int pauses=0;bool previous=false;uint32_t start=millis();
  while(uint32_t(millis()-start)<10000) { step();if(mode("SEARCH_PAUSE")&&!previous) ++pauses;previous=mode("SEARCH_PAUSE"); }
  check(pauses>=4,"search repeats without a terminal state while no HTTP loop runs");
  ball(true);until([] { return mode("FOLLOW"); });
  Wire.irPresent=false;until([] { return mode("IR_FAULT"); });zero();
  Wire.irPresent=true;until([] { return mode("FOLLOW"); });
  command('c');zero();check(!enabled && imu.calibrating(),"gyro calibration stops active play");
  command('s');check(!imu.calibrating() && !enabled && !requests.blocked(),"STOP cancels gyro calibration");zero();
  command('b');until([] { return imu.healthy(millis()) && !requests.blocked(); });check(!enabled,"rescan remains stopped");
  command('r');until([] { return mode("FOLLOW"); });
  // The motor worker continues without any controller/HTTP service calls.
  for(int i=0;i<21;i++) { fakeUs+=5000;motors.tick(); }
  zero();check(motors.state().watchdog && !motors.state().enabled,"100 ms stale command watchdog");
  step();check(!enabled && mode("DRIVE_WATCHDOG"),"watchdog disables play and does not automatically restart");
  command('b');until([] { return imu.healthy(millis()) && !requests.blocked(); });zero();
  command('r');until([] { return mode("FOLLOW") && motors.state().active; });
  check(!motors.state().watchdog,"explicit resume clears watchdog after healthy rescan");
  command('s');zero();
  check(requests.submit('r'),"queue old resume");motors.tick();char oldCommand;uint32_t oldGeneration;
  check(requests.take(oldCommand,oldGeneration) && oldCommand=='r',"claim old resume");
  check(requests.submit('s') && !requests.permit(oldGeneration),"new STOP invalidates a previously claimed Resume");step();zero();
  check(!enabled,"STOP remains authoritative");
  // Stored calibration permits the configured one-time automatic startup.
  unsigned writes=fakeNvs.writes;initializePlay();
  until([] { return enabled && mode("FOLLOW"); });
  check(imu.loaded() && fakeNvs.writes==writes,"saved boot auto-run without rewriting flash");
  Wire.failMux=true;until([] { return mode("LINE_MUX_FAULT"); });zero();
  Wire.failMux=false;command('b');until([] { return readyToResume() && !requests.blocked(); });command('r');until([] { return mode("FOLLOW"); });
  Wire.mpuPresent=false;until([] { return mode("MPU_FAULT"); });zero();
  Wire.mpuPresent=true;command('b');until([] { return readyToResume() && !requests.blocked(); });command('r');until([] { return mode("FOLLOW"); });
  fakeWriteFailurePin=16;until([] { return !motors.state().ready && !enabled; });zero();
  fakeWriteFailurePin=-1;PlayMotor isolated;check(isolated.begin() && isolated.permit(),"isolated watchdog test setup");
  check(isolated.request(80,80,80,80),"queue command before stalled worker");fakeUs+=150000;isolated.tick();zero();
  check(isolated.state().watchdog && !isolated.state().active,"an expired queued command never energizes motors");
  fakeUs=uint64_t(UINT32_MAX-40)*1000;check(isolated.permit() && isolated.request(80,80,80,80),"watchdog request across clock wrap");isolated.tick();
  check(isolated.state().active,"fresh wrapped command activates");fakeUs+=100000;isolated.tick();zero();
  check(isolated.state().watchdog,"watchdog expires across millis wrap");
  std::cout<<"PASS: AP full-play runtime, combined/stopped calibration, flash-zero invariant, continuous search, all line escapes, faults, watchdog, STOP generations, saved auto-start.\n";
}
