# APDS boundary avoidance

This is part of **MegaBallSearchNoTurn**, the simple follower. It uses four APDS9960-compatible sensors on a TCA9548A at 0x70, with the MPU and IR sensor on the upstream Mega SDA D20 / SCL D21 bus. No NeoPixel library or output is used.

## Edit the mounting map first

Open **BoundaryConfig.h**. The supplied positions are examples requested by the user, not verified chassis positions:

| Sensor / white command | TCA channel | Example side |
| --- | --- | --- |
| 1 | 0 | FRONT |
| 2 | 1 | RIGHT |
| 3 | 2 | REAR |
| 4 | 3 | LEFT |

Edit `CHANNELS` and `SIDES` to match the wiring and physical placement. Side constants also include `line::FRONT_LEFT`, `line::FRONT_RIGHT`, `line::REAR_LEFT`, and `line::REAR_RIGHT`. For example, four corner sensors may use `{line::FRONT_LEFT, line::FRONT_RIGHT, line::REAR_RIGHT, line::REAR_LEFT}`. Incorrect sides produce an incorrect retreat direction. Duplicate/out-of-range channels and invalid side values block initialization.

Keep the existing motor wiring: FL 8/6, FR 9/7, BL 2/3, BR 10/11. APDS power and sensor-side I2C must remain at the appropriate 3.3 V levels, with common ground and the previously checked TCA bus-level arrangement. Each APDS needs its own mux channel. All four configured sensors are required; missing sensors cannot be bypassed by `g`.

## Calibrate before starting

Select Mega 2560, upload the complete sketch folder, and open Serial Monitor at **115200 baud, Newline**. It starts with motors stopped. Keep the robot still during MPU calibration. Begin with wheels lifted or motor power disabled.

1. Put **all four sensors over the normal playing surface** at their final mounted height and lighting. Keep the robot completely still and send **f**. It first collects 500 gyro samples, saves the gyro offset, prints **MPU calibration saved to EEPROM**, and then starts the field capture automatically. Keep still and wait for all four `FIELD sensor ...` lines. A gyro/save failure aborts the sequence before field capture; fix it and retry `f`.
2. Put **sensor 1** over the actual white tape at the same height. Send **1** and wait for its `WHITE sensor ... trip=... release=...` result.
3. Repeat for sensor **2**, **3**, and **4**, sending the matching number. Only that sensor needs to be over white for its capture. Keep every sensor connected and receiving usable light throughout.
4. Wait for **LINE calibration saved**. Move every sensor back over the playing surface. Send **g** to enable following after all readings and the gyro are healthy.

Each capture uses 32 fresh samples. Low contrast, inverted brightness, excessive noise, read failure, or saturation cannot produce an enabled calibration. A capture times out after eight seconds if enough fresh data is unavailable. `LOW CONTRAST/NOISY` means fix illumination/height and repeat `f` plus the white captures. `SATURATED` means reduce gain or adjust lighting, then recalibrate. The colour sensor needs visible light; its proximity IR LED does not supply white illumination.

The calibration derives separate per-sensor thresholds from measured field and white means. White must be at least 15% brighter than the field, with a gap of at least 8 counts and six times the sum of the measured standard deviations. White asserts at 45% of that gap above the field mean and releases at 25%; this hysteresis prevents rapid toggling near the threshold. These are starting criteria to validate on the actual tape and field.

Valid thresholds are saved automatically to EEPROM **bytes 0-29**, with a CRC, version, channel list, gain, and integration-time signature. Saving updates one byte per loop with motors stopped, invalidating the record first and committing it last. Saved calibration loads after reboot; wait for the MPU and line sensors, then use `g`. Starting a new capture invalidates the old saved record, so an interrupted recalibration cannot silently restore old thresholds. Changing gain/time or channel wiring requires recalibration. Repeat calibration if lighting, mounting, or sensors change.

The gyro offset is saved separately at **bytes 64-79**. Its record includes a CRC, version, MPU address/identity, gyro range, sample divider, and filter setting. The firmware checks the sensor before loading; invalid records trigger a fresh stationary gyro calibration and save. Loading still waits for fresh data before allowing `g`. Gyro EEPROM writes update one byte per loop, commit the marker last, and verify the result before field capture can start. `f` and `c` always recalibrate rather than reuse the saved bias. Motors remain stopped during calibration/saving, and `s` cancels the pending gyro/field sequence.

Only the Z-axis zero-rate offset is saved, not an absolute heading. Yaw starts at zero after reboot. Bias can change with temperature; use `c` while still to refresh just the gyro, or `f` to redo the gyro and field procedure. With no valid gyro record, startup automatically calibrates/saves it; this does not automatically capture field/white values.

## Priority and retreat

- Ball searching/following only runs with calibrated, fresh, healthy line sensors and a healthy MPU.
- A white reading takes control away from both searching and following, even when no ball is visible.
- Front white commands backward retreat; rear white commands forward retreat; right white commands left; left white commands right. Adjacent/corner detections combine into a diagonal retreat, with gyro heading correction.
- Escape direction is recomputed from current detections. Opposing edge detections brake in `LINE_HOLD`; the program reevaluates automatically and resumes escape when the direction becomes unambiguous.
- Once all sensors drop below their release thresholds, the robot brakes while confirming 120 ms stable clear. It then resumes following if the ball is visible, or starts a fresh search after the normal 200 ms pause.
- A retreat lasting 650 ms gets a 120 ms brake, then automatically retries using fresh readings. There is no latched boundary timeout. Manual stop still cancels escape and requires `g` to resume.
- A calibrated sensor that saturates during play counts as a bright boundary and prints `SAT`; saturation during calibration is rejected. Missing, stale, failed, or unconfigured line-sensor data sets wheel output to zero. A mux deselection failure blocks use of the shared MPU/IR bus until isolation recovers. APDS configuration loss requires `b` to rescan/reinitialize.

`BoundaryConfig.h` exposes retreat PWM, wheel caps, clear time, timeout, sampling, and stale-data limits. Defaults are retreat PWM 70, retreat wheel cap 100, and following wheel cap 90. These are not measured motor calibration. Start with wheels lifted and verify the resulting direction for each sensor before a slow floor test.

The main program uses about **16.7 ms integration**, faster than the standalone 103 ms bench test. It polls one mux channel per 5 ms slot without waiting for conversions, checks AVALID, and reads Clear from the RGBC burst. Readings older than 100 ms block movement. Wire operations retain their 25 ms timeout; a transaction already in progress cannot be interrupted by serial stop. Actual detection and stopping distance depend on sensor placement, latency, surface contrast, chassis speed, and braking. This detects local brightness contrast; it does not determine the robot's global position or guarantee that it remains in bounds.

## Commands and telemetry

| Command | Action |
| --- | --- |
| s / x / 0 / space | Immediate stop; cancel capture/retreat |
| g + Enter | Start only when gyro, sensors, calibration, and EEPROM save are ready |
| f + Enter | Stop; calibrate/save gyro first, then measure field under all sensors |
| 1 / 2 / 3 / 4 + Enter | Stop; measure white under the selected sensor |
| c + Enter | Stop; recalibrate/save gyro; does not erase completed line thresholds |
| b + Enter | Stop; rescan APDS sensors and load saved gyro/line records; calibrate/save gyro if its record is invalid |

Calibration commands always stop movement. Use a separate `g` after completion. `c` or stop during a capture cancels that unfinished capture. Use `f` before new white captures after reboot/rescan, because the stored record contains thresholds rather than the original field sample statistics.

Telemetry includes `line=0x...`, a bit mask in sensor order (1, 2, 4, 8), and `C=a/b/c/d`. An exclamation mark indicates unusable/stale data; `L1=...` etc. names missing, read, saturation, configuration, or stale/wait faults. Relevant states are `LINE_CAL_REQUIRED`, `LINE_CALIBRATING`, `LINE_SAVING`, `LINE_SENSOR_FAULT`, `LINE_MUX_FAULT`, `LINE_ESCAPE`, `LINE_CLEARED`, and `LINE_HOLD`. `STOPPED` requires `g`; `MPU_FAULT` requires fixing the sensor and stationary `c` recalibration. Valid ball readings use `FOLLOW`; no ball cycles through `WAIT_BALL`, `SEARCH`, and `SEARCH_PAUSE` with continued polling, without a terminal search state.

## Verification

The software checks in `verification/boundary_scenarios.cpp` evaluate the actual BoundaryLogic.h functions using AVR C++14 constant expressions. They cover all 16 edge masks, mounting validity, calibration contrast and hysteresis, motion guards, timed escape retry, clear timing, automatic ambiguity recovery, wraparound, and CRC. Executable `play_workflow.cpp` tests exercise actual-sketch search/follow/escape behaviour for both maintained play entry points. Firmware uses the normal Arduino C++11 setting. Physical validation is still required; see the bench checklist in `verification/README.md`.
