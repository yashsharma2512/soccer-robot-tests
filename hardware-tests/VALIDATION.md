# Standalone test validation - 2 November 2026

All four exact sketches in this folder compiled successfully for **Arduino Mega 2560** (`arduino:avr:mega`) with installed **Arduino AVR Boards 1.8.6** and AVR GCC **7.3.0-atmel3.6.1-arduino7**.

| Sketch | Flash bytes / 253,952 | Static RAM bytes / 8,192 |
| --- | ---: | ---: |
| MegaMotorTestCurrent | 4,764 | 243 |
| MegaMPUOnlyTest | 9,592 | 462 |
| MegaIROnlyTest | 5,620 | 412 |
| MegaI2CTest | 6,180 | 412 |

The motor and MPU-only sketches were copied from the latest delivered test files, with SHA-256 equality checked after copying. The IR-only sketch is the latest standalone test pasted in the chat. The I2C scanner is a copy of the repository's current scanner, also checked by SHA-256.

All tests use the current motor pin map: FL 8/6, FR 9/7, BL 4/2, BR 3/5. The three sensor/diagnostic sketches hold these motor outputs LOW. The motor-only test has no IR/MPU dependency and stops after each 800 ms pulse. Existing integrated sketches were not modified.

No firmware was uploaded and no physical motion or live sensor test was performed during this update. Successful compilation does not establish correct wiring, sensor identity, gyro accuracy, motor polarity, or real chassis motion. The original root validation document and integrated search checks are separate from these standalone compilation results.
