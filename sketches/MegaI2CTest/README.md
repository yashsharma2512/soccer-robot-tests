# Mega I2C and identity test

Open `MegaI2CTest.ino` for Mega 2560. Set Serial Monitor to **115200 baud**. Mega SDA is **D20** and SCL is **D21**.

For the current IMU diagnosis, turn motor power off and connect only the GY-521 to the I2C bus, with the correct module power, common ground, and signal voltage levels. See [MPU wiring notes](../MegaMPUSingleFile/README.md).

The sketch scans 7-bit addresses `0x01` through `0x7E` at 100 kHz, repeating every three seconds. For a device acknowledging `0x68` or `0x69`, it reads register `0x75` three times and reports whether the values agree. It does not initialize or calibrate the IMU and does not filter identities.

Example format only; these are not measurements from this robot:

```text
Found device at 0x68
Address 0x68 WHO_AM_I (register 0x75), three reads: 0x68, 0x68, 0x68 (stable)
```

MPU6050's expected WHO_AM_I is `0x68`; MPU6500's is `0x70`. Both can use bus address `0x68` or `0x69`. The bus address and chip identity are separate values. A stable unexpected identity needs its own verified driver configuration; do not bypass the identity check. Unstable readings or bus errors require resolving the electrical/read problem first.

The eight motor control outputs stay LOW. Save the complete scan and identity output when diagnosing the unresolved MPU ID issue.
