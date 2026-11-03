# Validation

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
