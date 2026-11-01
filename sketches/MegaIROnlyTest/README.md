# HiTechnic IR-only test

Open `MegaIROnlyTest.ino` for Mega 2560. Use Serial Monitor at **115200 baud**. Keep the sensor's already working power and wiring setup; Mega SDA is D20 and SCL is D21.

The sketch reads six AC bytes from register `0x49` at 7-bit I2C address `0x08`. It prints direction, derived strength, and S1-S5 every 100 ms using Wire, without an external HiTechnic library. It preserves the current sensor mode.

Direction `0` means no detected direction; `5` is center, `1`-`4` are left, and `6`-`9` are right. Strength follows the original baseline library's direction-dependent calculation.

All eight motor control outputs are held LOW. The MPU is not initialized. This is a sensor reading test and contains no movement logic.
