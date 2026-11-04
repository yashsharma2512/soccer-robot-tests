# Standalone test validation - 5 November 2026

All five current hardware-test sketches compiled for **Arduino Mega 2560** (`arduino:avr:mega`) after the motor wiring/polarity update, using Arduino AVR Boards **1.8.6** and AVR GCC **7.3.0-atmel3.6.1-arduino7**.

| Sketch | Flash bytes / 253,952 | Static RAM bytes / 8,192 |
| --- | ---: | ---: |
| MegaMotorTestCurrent | 4,770 | 243 |
| MegaMPUOnlyTest | 9,592 | 462 |
| MegaIROnlyTest | 5,620 | 412 |
| MegaI2CTest | 6,180 | 412 |
| MegaAPDSTcaTest | 7,364 | 425 |

Current motor inputs: **FL 8/6, FR 9/7, BL 2/3, BR 10/11**. Motor-test signs are **FL +1, FR +1, BL -1, BR +1**. All sensor/diagnostic sketches hold those same eight output pins LOW. The motor-only test uses no sensor input and stops after each 800 ms pulse. Searches verified that the old pin map no longer appears in the repository's current source or instructions.

These are the latest maintained copies of the standalone tests delivered during the hardware bring-up session; pin declarations and diagnostic stop-pin arrays were updated together. They use built-in Arduino/Wire functionality and the APDS test no longer requires NeoPixels. No firmware was uploaded and no physical motion, sensor, or white-line test was performed during this update. Compilation does not establish correct physical wiring or chassis direction. The root validation document records the integrated main sketch's separate checks.
