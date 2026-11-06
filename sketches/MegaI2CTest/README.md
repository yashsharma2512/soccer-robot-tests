# Mega I2C and identity test

Open `MegaI2CTest.ino` for Mega 2560. Set Serial Monitor to **115200 baud**. Mega SDA is D20 and SCL is D21. The motor outputs use the current pin map (FL 6/7, FR 4/5, BL 11/10, BR 2/3) and remain LOW.

The sketch scans 7-bit I2C addresses at 100 kHz. For a device acknowledging `0x68` or `0x69`, it reads register `0x75` three times. A stable `0x74` result matches the identity accepted by the current ball-search sketch's MPU reader. The bus address `0x68` and chip identity `0x74` are separate values.

This is only a diagnostic. It does not initialize the IMU or move motors. Check wiring and voltage levels if reads are unstable or the I2C bus reports errors.
