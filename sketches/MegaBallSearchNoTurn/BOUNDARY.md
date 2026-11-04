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

Keep the existing motor wiring: FL 8/6, FR 9/7, BL 4/2, BR 3/5. APDS power and sensor-side I2C must remain at the appropriate 3.3 V levels, with common ground and the previously checked TCA bus-level arrangement. Each APDS needs its own mux channel. All four configured sensors are required; missing sensors cannot be bypassed by `g`.

## Calibrate before starting

Select Mega 2560, upload the complete sketch folder, and open Serial Monitor at **115200 baud, Newline**. It starts with motors stopped. Keep the robot still during MPU calibration. Begin with wheels lifted or motor power disabled.

1. Put **all four sensors over the normal playing surface** at their final mounted height and lighting. Send **f**. Wait for the four `FIELD sensor ...` lines.
2. Put **sensor 1** over the actual white tape at the same height. Send **1** and wait for its `WHITE sensor ... trip=... release=...` result.
3. Repeat for sensor **2**, **3**, and **4**, sending the matching number. Only that sensor needs to be over white for its capture. Keep every sensor connected and receiving usable light throughout.
4. Wait for **LINE calibration saved**. Move every sensor back over the playing surface. Send **g** to enable following after all readings and the gyro are healthy.

Each capture uses 32 fresh samples. Low contrast, inverted brightness, excessive noise, read failure, or saturation cannot produce an enabled calibration. A capture times out after eight seconds if enough fresh data is unavailable. `LOW CONTRAST/NOISY` means fix illumination/height and repeat `f` plus the white captures. `SATURATED` means reduce gain or adjust lighting, then recalibrate. The colour sensor needs visible light; its proximity IR LED does not supply white illumination.

The calibration derives separate per-sensor thresholds from measured field and white means. White must be at least 15% brighter than the field, with a gap of at least 8 counts and six times the sum of the measured standard deviations. White asserts at 45% of that gap above the field mean and releases at 25%; this hysteresis prevents rapid toggling near the threshold. These are starting criteria to validate on the actual tape and field.

Valid thresholds are saved automatically to EEPROM **bytes 0-29**, with a CRC, version, channel list, gain, and integration-time signature. Saving updates one byte per loop with motors stopped, invalidating the record first and committing it last. Saved calibration loads after reboot; wait for the MPU and line sensors, then use `g`. Starting a new capture invalidates the old saved record, so an interrupted recalibration cannot silently restore old thresholds. Changing gain/time or channel wiring requires recalibration. Repeat calibration if lighting, mounting, or sensors change.

## Priority and retreat

- Ball following only runs with calibrated, fresh, healthy line sensors and a healthy MPU.
- A white reading immediately takes control away from ball following, even when no ball is visible.
- Front white commands backward retreat; rear white commands forward retreat; right white commands left; left white commands right. Adjacent/corner detections combine into a diagonal retreat, with gyro heading correction.
- The detected sides remain remembered during a retreat. A newly detected opposite edge stops rather than choosing an ambiguous direction.
- Retreat continues until all sensors have been below their release thresholds for 120 ms. It stops for that control pass, then normal IR following can resume automatically.
- A retreat lasting 650 ms, or opposing edge detections, stops and latches `LINE_BLOCKED`. Reposition the robot onto the field, check the mapping, and send `g` to retry. Manual stop also cancels a retreat.
- Any missing, stale, saturated, or failed line-sensor data sets wheel output to zero. A mux deselection failure blocks use of the shared MPU/IR bus until isolation recovers. APDS configuration loss requires `b` to rescan/reinitialize.

`BoundaryConfig.h` exposes retreat PWM, wheel caps, clear time, timeout, sampling, and stale-data limits. Defaults are retreat PWM 70, retreat wheel cap 100, and following wheel cap 90. These are not measured motor calibration. Start with wheels lifted and verify the resulting direction for each sensor before a slow floor test.

The main program uses about **16.7 ms integration**, faster than the standalone 103 ms bench test. It polls one mux channel per 5 ms slot without waiting for conversions, checks AVALID, and reads Clear from the RGBC burst. Readings older than 100 ms block movement. Wire operations retain their 25 ms timeout; a transaction already in progress cannot be interrupted by serial stop. Actual detection and stopping distance depend on sensor placement, latency, surface contrast, chassis speed, and braking. This detects local brightness contrast; it does not determine the robot's global position or guarantee that it remains in bounds.

## Commands and telemetry

| Command | Action |
| --- | --- |
| s / x / 0 / space | Immediate stop; cancel capture/retreat |
| g + Enter | Start only when gyro, sensors, calibration, and EEPROM save are ready |
| f + Enter | Stop; measure field under all sensors |
| 1 / 2 / 3 / 4 + Enter | Stop; measure white under the selected sensor |
| c + Enter | Stop; recalibrate gyro; does not erase completed line thresholds |
| b + Enter | Stop; rescan APDS sensors, load saved line calibration, and restart gyro calibration |

Calibration commands always stop movement. Use a separate `g` after completion. `c` or stop during a capture cancels that unfinished capture. Use `f` before new white captures after reboot/rescan, because the stored record contains thresholds rather than the original field sample statistics.

Telemetry includes `line=0x...`, a bit mask in sensor order (1, 2, 4, 8), and `C=a/b/c/d`. An exclamation mark indicates unusable/stale data; `L1=...` etc. names missing, read, saturation, configuration, or stale/wait faults. Relevant states are `LINE_CAL_REQUIRED`, `LINE_CALIBRATING`, `LINE_SAVING`, `LINE_SENSOR_FAULT`, `LINE_MUX_FAULT`, `LINE_ESCAPE`, `LINE_CLEARED`, and `LINE_BLOCKED`. `STOPPED` requires `g`; `MPU_FAULT` requires fixing the sensor and stationary `c` recalibration. Valid ball readings use `FOLLOW`; no ball uses `WAIT_BALL` with continued polling.

## Verification

The software checks in `verification/boundary_scenarios.cpp` evaluate the actual BoundaryLogic.h functions using AVR C++14 constant expressions. They cover all 16 edge masks, mounting validity, calibration contrast and hysteresis, motion guards, bounded retreat, clear timing, explicit reset, wraparound, and CRC. Firmware uses the normal Arduino C++11 setting. Physical validation is still required; see the bench checklist in `verification/README.md`.
