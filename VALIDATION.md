# Validation - 1 November 2026

`MegaBallSearchNoTurn`, `MegaIROnlyTest`, and `MegaI2CTest` were compiled for **Arduino Mega 2560** (`arduino:avr:mega`) using Arduino AVR Boards **1.8.8** and AVR GCC **7.3.0-atmel3.6.1-arduino7**. The search-pattern checks passed with the host C++11 compiler. The other two sketches only changed their motor pins (and the I2C scanner's identity label); their sensor read sequences were preserved.

| Sketch | Flash bytes / 253,952 | Static RAM bytes / 8,192 |
| --- | ---: | ---: |
| MegaBallSearchNoTurn | 13,718 | 571 |
| MegaIROnlyTest | 5,666 | 412 |
| MegaI2CTest | 6,180 | 412 |

The search check verifies both preferred sides, each search boundary, that the sequence ends on time, and that no search leg commands backward travel or a turn. See [verification](verification/README.md).

No board upload or physical motion test was performed during this repository update. The sketch cannot locate field boundaries or the own goal; timed search legs cannot guarantee a return to the same position. Test gyro correction and motor polarity with wheels lifted before floor operation.
