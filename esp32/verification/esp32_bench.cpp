#include "fakes/Arduino.h"
#include "fakes/Wire.h"
#include "fakes/Preferences.h"
#include <cstdlib>
#include <iostream>
uint64_t fakeUs=0;
int fakePwm[80]={}; bool fakeAttached[80]={};
int fakeAttachFailurePin=-1,fakeWriteFailurePin=-1;
FakeSerial Serial; FakeWire Wire; FakeNVS fakeNvs;
#include "../ESP32BenchDashboard/BenchMotor.h"
#include "../ESP32BenchDashboard/BenchLogic.h"
#include "../ESP32BenchDashboard/BenchSensors.h"
void check(bool value,const char *message) { if(!value) { std::cerr<<"FAIL "<<message<<'\n'<<Serial.output; std::exit(1); } }
void zero() { for(uint8_t p:BenchMotor::pins) check(fakePwm[p]==0,"all motor PWM zero"); }
int main() {
  BenchMotor motor; check(motor.begin(),"PWM init"); zero();
  BenchMotor::Command cmd={{80,80,80,80},400};
  check(!motor.pulse(cmd),"boot must be disarmed");
  check(motor.arm() && motor.pulse(cmd),"explicit arm accepts pulse");
  check(!motor.pulse(cmd),"queued pulse cannot be overwritten/extended"); motor.tick();
  check(fakePwm[25]==80 && fakePwm[16]==80 && fakePwm[33]==80 && fakePwm[18]==80,"main pin map and BL sign");
  check(fakePwm[14]==0 && fakePwm[27]==0,"old FR pins unused");
  // No network loop or sensor service is called while advancing the motor worker.
  for(int i=0;i<79;i++) { fakeUs+=5000; motor.tick(); }
  check(motor.state().active,"pulse not cut early"); fakeUs+=5000; motor.tick(); zero();
  check(motor.idle(),"400 ms independent cutoff");
  cmd.duration=501; check(!motor.pulse(cmd),"reject overlong pulse"); cmd.duration=400;
  cmd.wheel[0]=121; check(!motor.pulse(cmd),"reject excessive PWM");cmd.wheel[0]=80;
  check(motor.pulse(cmd),"second pulse after first completed"); motor.stop(); motor.tick(); zero();
  check(!motor.state().armed && !motor.pulse(cmd),"stop wins over queued motion");
  check(motor.arm(),"rearm"); fakeUs+=29900000; check(motor.pulse(cmd),"pulse near arm expiry");motor.tick();
  fakeUs+=100000; motor.tick();zero();check(!motor.state().armed,"arm expiry cuts active pulse");
  fakeUs=uint64_t(UINT32_MAX-200)*1000;check(motor.arm() && motor.pulse(cmd),"arm across millis wrap");motor.tick();
  fakeUs+=400000;motor.tick();zero();check(motor.idle(),"pulse deadline across wrap");
  int value=0;check(benchlogic::number("120",30,120,value)&&value==120,"valid numeric input");
  for(const char *bad:{"","-1","121","80.5","80x","9999999999"," 80"}) check(!benchlogic::number(bad,30,120,value),"reject malformed number");
  int16_t wheel[4];check(benchlogic::motion("right",80,wheel)&&wheel[0]==80&&wheel[1]==-80&&wheel[2]==-80&&wheel[3]==80,"strafe mixer");
  check(benchlogic::motion("rotatePlus",80,wheel)&&wheel[0]==80&&wheel[1]==-80&&wheel[2]==80&&wheel[3]==-80,"rotation mixer");
  check(!benchlogic::motion("typo",80,wheel),"unknown target rejected");
  check(motor.arm() && motor.pulse(cmd),"failure test pulse");motor.tick();fakeWriteFailurePin=25;motor.stop();motor.tick();zero();
  check(!motor.state().ready,"runtime PWM failure detaches all outputs");fakeWriteFailurePin=-1;
  BenchMotor broken;fakeAttachFailurePin=17;check(!broken.begin(),"attachment failure rejected");zero();fakeAttachFailurePin=-1;

  MpuYaw imu;BoundarySensors boundary;
  calibstore::begin();check(boundary.begin() && imu.startCalibration(true),"sensor init accepts APDS9E and MPU70");
  uint32_t seenGeneration=calibstore::generation;
  auto service=[&] { fakeUs+=5000;synchronizeGyroStorage(imu,seenGeneration);imu.service();boundary.service();synchronizeGyroStorage(imu,seenGeneration); };
  for(int i=0;i<650;i++) service();
  check(imu.healthy(millis()) && imu.raw[2]==16384 && imu.raw[6]==131 && imu.samples>=500,"fresh raw MPU telemetry/calibration");
  check(boundary.id(0)==0x9E && boundary.clear(0)==100 && boundary.fresh(0),"APDS telemetry");
  check(boundary.startField(),"green capture starts");for(int i=0;i<850;i++)service();
  check(boundary.calibrated() && imu.healthy(millis()) && fakeNvs.blobs.count("line") && fakeNvs.blobs.count("gyro"),"shared NVS blobs and flash resync");
  check(boundary.startField(),"repeat green capture erases old record");for(int i=0;i<850;i++)service();
  check(boundary.calibrated() && imu.healthy(millis()),"60 ms pre-service erase must resync gyro as well as save");
  MpuYaw loaded;BoundarySensors loadedLine;check(loadedLine.begin() && loaded.startCalibration(true) && loaded.loaded() && loadedLine.calibrated(),"saved records reload");
  Wire.clear(0,1000);for(int i=0;i<10;i++) service();check(boundary.hitMask()&1,"white line telemetry");
  check(boundary.startField(),"new capture before cancellation");boundary.cancelCapture();
  check(!boundary.capturing() && !boundary.saving() && !boundary.calibrated(),"cancel abandons pending line calibration");
  Wire.transientMpuFailures=1;for(int i=0;i<5;i++)service();check(imu.failedTransactions>=1 && imu.recoveredReads()>=1 && imu.lastWireStatus==2,"recovered I2C error retained in telemetry");
  Wire.mpuPresent=false;service();check(!imu.healthy(millis()) && imu.error()==MpuYaw::READ_FAILED,"persistent MPU fault visible");
  Wire.missingApds=2;for(int i=0;i<30;i++)service();check(!boundary.fresh(2) && boundary.errors(2)>0,"APDS failure gets stale and counter");
  Wire.mpuPresent=true;fakeNvs.failPut=true;MpuYaw saveFailure;
  check(saveFailure.startCalibration(false),"start gyro save-failure case");
  for(int i=0;i<650;i++) { fakeUs+=5000;saveFailure.service(); }
  check(saveFailure.error()==MpuYaw::SAVE_FAILED && !calibstore::ready,"failed NVS write is visible and not calibrated");
  zero();std::cout<<"PASS: bench arm/stop/deadline/wrap/PWM-failure, parser/mixer, raw sensor telemetry, shared NVS, I2C faults.\n";
}
