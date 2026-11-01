# Validation - 1 November 2026

The exact four sketches copied into this repository compiled successfully for **Arduino Mega 2560** (`arduino:avr:mega`) with **Arduino AVR Boards 1.8.6** and AVR GCC **7.3.0-atmel3.6.1-arduino7**.

| Sketch | Flash bytes / 253,952 | Static RAM bytes / 8,192 |
| --- | ---: | ---: |
| MegaIROnlyTest | 5,666 | 412 |
| MegaMotorOnlyTest | 4,764 | 243 |
| MegaMPUSingleFile | 15,914 | 530 |
| MegaI2CTest | 6,088 | 412 |

Each `.ino` was copied without changing its contents, and the source and repository copy SHA-256 hashes were compared.

**37 existing software assertions passed** against the actual `MegaMPUSingleFile.ino` method bodies, compiled as C++14 constant expressions. The normal sketches use the core's ordinary C++11 setting. The assertions cover wheel mixing/normalization, feedback sign and yaw wrap, turn targets/damping, timed navigation, settling, timer rollover, and implemented/rejected sensor IDs. See [verification instructions](verification/README.md).

Repository contents were reviewed to exclude combined IR/driving programs, ball following, autonomous ball search, build products, and local machine configuration. The included IR-only sketch holds all motor outputs LOW. The movement sketches do not read the IR sensor.

No firmware was uploaded and no physical movement was run during repository preparation. The IR-only test was previously reported working on the Mega. The connected GY-521's actual chip identity and the reported wrong-ID failure remain unresolved; neither compilation nor the software assertions establish successful live IMU operation. Motor polarity, feedback direction, gains, gyro stability, and physical stopping/turning accuracy still require checks on the chassis.
