#pragma once
#include "PlayController.h"
#include "PlayRequests.h"
PlayRequests requests(motors);
uint32_t controlSequence=0;
bool startupAuto=false,previousWatchdog=false;
uint8_t calibrationStage=0;
void playEvent(const char *message);
bool permitPlay() { return requests.permit(controlSequence); }
void initializePlay() { setupPlay();startupAuto=espconfig::AUTO_RUN_WHEN_READY; }
void stepPlay() {
    char command;
    if(requests.take(command,controlSequence)) {
      startupAuto=false;calibrationStage=0;
      // take() only succeeds after the motor worker confirms zero output.
      executeCommand(command=='f' ? 'c' : command);
      if(command=='f') calibrationStage=1;
      playEvent(enabled ? "Play resumed" : "Stopped; Resume required after calibration");
    }
    if(busInitialized) pollIR();
    servicePlay();
    if(calibrationStage==1 && !imu.calibrating() && !imu.saving()) {
      if(imu.healthy(millis())) {
        calibrationStage=boundary.startField() ? 2 : 0;
        playEvent(calibrationStage ? "Gyro saved; GREEN capture starts in 3 seconds" : "Combined calibration: GREEN start failed");
      } else if(imu.error()!=MpuYaw::OK) { calibrationStage=0;playEvent("Combined calibration: gyro failed"); }
    }
    if(calibrationStage==2 && !boundary.capturing() && !boundary.saving()) {
      calibrationStage=0;playEvent(boundary.calibrated() ? "Both calibrations saved; press Resume" : "GREEN calibration failed; remain stopped");
    }
    auto motor=motors.state();
    if(motor.watchdog) { enabled=false;startupAuto=false;state=F("DRIVE_WATCHDOG"); }
    if(motor.watchdog && !previousWatchdog) playEvent("Drive watchdog expired; explicit Resume required");
    previousWatchdog=motor.watchdog;
    if(startupAuto && readyToResume() && !requests.pending() && motors.idle()) {
      executeCommand('r');if(enabled) { startupAuto=false;playEvent("Configured automatic startup: play enabled"); }
    }
    requests.busy(imu.calibrating() || imu.saving() || boundary.capturing() || boundary.saving() || calibrationStage);
}
