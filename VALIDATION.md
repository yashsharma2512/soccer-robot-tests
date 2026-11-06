# Validation

## Continuous play and automatic line escape - 5 November 2026

The main `MegaBallSearchNoTurn` and the green-only `test code linux full`
entry point now repeat short sideways/forward searches until a valid ball
appears. Each 1.95-second cycle has a 200 ms pause; neither program has a
terminal SEARCH_DONE. Ball detection returns to FOLLOW on the next valid
IR poll. White-line escape owns the drive before either search or follow.
Current adjacent detections command diagonal escape; opposing detections
hold zero PWM until resolved. After 650 ms escape brakes for 120 ms and
retries; 120 ms stable clear releases play automatically. Calibrated bright
saturation counts as a boundary, while calibration saturation is rejected.
Manual stop, calibration, missing calibration, and sensor faults still stop
motion. Main f/white/EEPROM and Linux full c/green/auto-start workflows remain.

Both actual-sketch `play_workflow.cpp` host runs passed: over four consecutive
search rounds, follow/reacquisition, all four escape directions, corners,
timeout retry, stable clear, ambiguity recovery, saturation, search/line
priority, IR/APDS/MPU faults, APDS rescan, and manual stop during both search
and escape. The main gyro-first `calibration_workflow.cpp` regression passed.
All 33 AVR C++14 boundary assertions passed, including all 16 edge masks,
automatic recovery, hysteresis, movement guards, timer rollover, and CRC.
Host tests use MSVC C++14 with simulated Wire/EEPROM and byte-write latency.

Both sketches compiled for Mega 2560 using Arduino AVR Boards 1.8.6 and
AVR GCC 7.3.0-atmel3.6.1-arduino7:

| Entry point | Flash bytes | Static RAM bytes |
| --- | ---: | ---: |
| MegaBallSearchNoTurn | 23,908 | 834 |
| test code linux full | 23,872 | 812 |

No firmware upload or physical motion test was performed. Verify the editable
sensor-side mapping and all four escape directions on the chassis before play.
The separate `MegaBallSearchBoundedNoTurn_linux` retains its bounded-search
behaviour and is not the continuous-play entry point.

## Gyro EEPROM and sequential calibration - 5 November 2026

The integrated main sketch now saves the stationary Z-gyro offset at EEPROM
bytes 64-79, separately from the APDS record at 0-29. Startup/rescan checks
the connected MPU and loads a valid record; otherwise it calibrates/saves
while stopped. Saved offsets require fresh live data before movement. The
record checks version, address, identity, gyro configuration, finite/ranged
bias, and CRC. Writes invalidate first, update one byte per loop, commit
last, and verify the result. Heading itself is not persisted.

`f` now forces a fresh gyro calibration/save before automatically starting
the four-sensor field capture. White captures still use `1` through `4` and
save line thresholds when complete. `c` refreshes/saves only the gyro.
Pending calibration/saving blocks `g`; stop cancels the deferred field
sequence, and gyro/save failures cannot start field capture.

The actual main sketch passed executable host simulation in
`verification/calibration_workflow.cpp`, using MSVC 19.44 C++14 and fake
Arduino/Wire/EEPROM interfaces. Checks include gyro-first ordering, line and
gyro save/reload, zero PWM and explicit enable, no EEPROM rewrite on load,
fresh-data and hardware requirements, stop during sampling/saving, not-still
failure, interrupted commits, invalid CRC/identity/configuration/NaN records,
negative bias, failed write verification, and separate EEPROM address ranges.
Byte writes advance simulated time. This does not validate physical bus or
sensor timing.

The Mega firmware compiled with Arduino AVR Boards 1.8.6 / AVR GCC
7.3.0-atmel3.6.1-arduino7: **23,050 bytes flash**, **768 bytes static RAM**.
No upload or hardware test was performed. A saved bias may need stationary
recalibration when temperature changes; use `c` or the complete `f` procedure.

## Motor wiring and polarity - 6 November 2026

All ten repository sketches use FL 6/7, FR 4/5, BL 11/10, and BR 2/3.
The four driving configurations retain signs FL +1, FR +1, BL -1, BR +1;
sensor diagnostics hold those same eight pins LOW. Wiring tables and inline
pin maps were updated together, and a source audit found no previous motor map.

All ten sketches compiled for `arduino:avr:mega` with Arduino AVR Boards
1.8.8. The integrated main sketch uses **23,908 bytes flash** and **834 bytes
static RAM**. Both play-workflow host tests passed with the revised map.
No firmware was uploaded and no physical motion was tested; verify each
wheel's direction with the chassis lifted.

## APDS boundary priority - 5 November 2026

`MegaBallSearchNoTurn` now polls four APDS sensors on TCA channels 0-3,
with editable side/corner mapping in BoundaryConfig.h. It starts stopped,
requires valid field/white calibration, and gives line escape priority over
IR following. Missing, stale, saturated, or failed line readings block
movement. Retreat is limited to 650 ms and requires 120 ms stable clear;
opposing detections or timeout latch a stop. Calibration is stored in a
30-byte checksummed EEPROM record, updated one byte per loop while stopped.
The standalone Linux bounded-search variant was not modified.

**33 compile-time assertions passed** against actual BoundaryLogic.h
functions, using AVR GCC C++14. These cover all 16 physical edge masks,
cardinal/corner mapping and invalid channels, contrast/noise rejection,
hysteresis, manual/calibration/fault motion guards, stable-clear timing,
timeout latching/reset, newly detected corners/opposing sides, timer
rollover, and the standard CRC-16/CCITT-FALSE check vector. This evaluates
the pure control logic; it does not execute the I2C driver, EEPROM hardware,
or a complete simulated robot.

The main firmware compiled for `arduino:avr:mega` with Arduino AVR Boards
**1.8.6** and AVR GCC **7.3.0-atmel3.6.1-arduino7**: **21,470 bytes flash**,
**743 bytes static RAM**. No firmware was uploaded or physically tested.
Per-sensor field/white calibration, actual side mapping, lighting contrast,
gyro/retreat direction, sensor timing, and stopping distance remain to be
checked on the robot. See [boundary instructions](sketches/MegaBallSearchNoTurn/BOUNDARY.md)
and the [bench checklist](verification/README.md).

## Simple following and automatic reacquisition - 4 November 2026

The main sketch now uses `FOLLOW` for a valid IR ball reading and
`WAIT_BALL` with all wheel outputs zero when the ball is unseen. The timed
search, search completion flags, and IR-error latch were removed. IR
transactions continue every 50 ms while enabled with a healthy MPU,
including during `WAIT_BALL` and following an IR read error. A valid later
reading enables following without a serial retry command. Manual stop
remains latched until `g`; an MPU fault still requires `c` recalibration.

Serial commands are now parsed as single-character lines, with immediate
stop characters discarding the rest of their line. Unused search/turn
helpers and their former search-pattern test were removed. The MPU6500
identity fix, motor pins, direction mapping, and heading correction remain.

The updated main sketch compiled for **Arduino Mega 2560**
(`arduino:avr:mega`) with Arduino AVR Boards **1.8.6** and AVR GCC
**7.3.0-atmel3.6.1-arduino7**: **13,956 bytes flash**, **522 bytes static RAM**.
Whitespace checks passed and active source was checked for removed search
state and IR fault-latch references. No automated runtime test, firmware
upload, or physical robot test was performed for this change. The
[bench regression checklist](verification/README.md) covers loss and
reappearance after a long wait, transient IR failures, manual stop/resume,
MPU faults, and heading feedback.

## MPU6500 identity fix - 3 November 2026

The user-provided identity-test output reports `0x70 / 0x70 / 0x70 STABLE`
at I2C address `0x68`. The previous main reader rejected this ID with error
2 (`MPU_WRONG_ID`). The corrected reader accepts the same exact IDs as the
standalone MPU-only test (`0x68`, `0x70`, `0x74`), requires three matching
reads, configures the MPU6500-family accelerometer filter, and verifies
that filter during runtime configuration checks. Unknown identities and
unstable reads remain blocked.

The updated `MegaBallSearchNoTurn` compiled successfully for
`arduino:avr:mega` with Arduino AVR Boards **1.8.6** and AVR GCC
**7.3.0-atmel3.6.1-arduino7**: **14,456 bytes flash**, **590 bytes static RAM**.
The standalone identity diagnostic also compiled: **5,586 bytes flash**,
**412 bytes static RAM**. Whitespace checks passed.

The screenshot establishes the live identity reading only. No updated
main firmware was uploaded or physically tested during this fix; gyro
calibration, yaw accuracy, and heading feedback remain to be checked on the
robot. The ball-following/search logic and motor pin map were not changed.

## Original search update - 1 November 2026

`MegaBallSearchNoTurn`, `MegaIROnlyTest`, and `MegaI2CTest` were compiled for **Arduino Mega 2560** (`arduino:avr:mega`) using Arduino AVR Boards **1.8.8** and AVR GCC **7.3.0-atmel3.6.1-arduino7**. The search-pattern checks passed with the host C++11 compiler. The other two sketches only changed their motor pins (and the I2C scanner's identity label); their sensor read sequences were preserved.

| Sketch | Flash bytes / 253,952 | Static RAM bytes / 8,192 |
| --- | ---: | ---: |
| MegaBallSearchNoTurn | 13,718 | 571 |
| MegaIROnlyTest | 5,666 | 412 |
| MegaI2CTest | 6,180 | 412 |

The search check verifies both preferred sides, each search boundary, that the sequence ends on time, and that no search leg commands backward travel or a turn. See [verification](verification/README.md).

No board upload or physical motion test was performed during this repository update. The sketch cannot locate field boundaries or the own goal; timed search legs cannot guarantee a return to the same position. Test gyro correction and motor polarity with wheels lifted before floor operation.
