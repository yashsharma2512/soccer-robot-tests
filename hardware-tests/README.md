# Standalone hardware tests

The latest motor, MPU-only, IR-only, and APDS9960/TCA test programs from the hardware bring-up session, plus the I2C scanner. Each subfolder is a separate Arduino sketch. Upload only one at a time; none of these tests combines IR detection with motor movement.

Select **Arduino Mega or Mega 2560 / ATmega2560**. Open the `.ino` whose name matches its folder, then set Serial Monitor to **115200 baud**. Use **Newline** for the motor test.

| Folder | Purpose |
| --- | --- |
| [MegaMotorTestCurrent](MegaMotorTestCurrent) | Individual wheel pulses and brief chassis movement; no sensors required |
| [MegaMPUOnlyTest](MegaMPUOnlyTest) | Three identity reads, gyro calibration, acceleration, gyro Z rate, and relative yaw; motor outputs LOW |
| [MegaIROnlyTest](MegaIROnlyTest) | AC direction, baseline-derived strength, and five IR channels; motor outputs LOW |
| [MegaI2CTest](MegaI2CTest) | I2C scanner and three raw MPU identity reads; motor outputs LOW |
| [MegaAPDSTcaTest](MegaAPDSTcaTest) | All eight TCA9548A channels: APDS identity, configuration, and Clear/RGB readings for field/white comparison; motor outputs LOW |

## Shared connections

| Wheel | Input 1 | Input 2 |
| --- | --- | --- |
| FL | D8 | D6 |
| FR | D9 | D7 |
| BL | D2 | D3 |
| BR | D10 | D11 |

Motor-test polarity defaults: `FL_SIGN=1`, `FR_SIGN=1`, `BL_SIGN=-1`, `BR_SIGN=1`.

Each MDD3A channel uses two PWM inputs. Connect Mega ground, both driver grounds, and motor battery negative together. Motors receive power through the drivers' motor supply terminals. When powering the Mega by USB, leave driver 5VO outputs unconnected.

Mega **SDA is D20** and **SCL is D21**. Retain the sensor power connections already verified for the respective modules. A standard regulator-equipped GY-521 accepts 5 V at VCC, but its sensor-side I2C signals are 3.3 V: use appropriate bidirectional level shifting to the Mega bus. Disconnect the other sensor for isolated MPU/IR testing and keep motor power off during sensor tests.

## Motor test

Lift the wheels first. Motors start stopped. Each accepted movement runs for **800 ms at PWM 80**, then stops automatically.

| Wheel | Positive | Reverse |
| --- | --- | --- |
| FL | `1` | `5` |
| FR | `2` | `6` |
| BL | `3` | `7` |
| BR | `4` | `8` |

After checking individual wheels, use `w` forward, `b` backward, `a` left, `d` right, `q` rotate left, and `e` rotate right. These directions assume the wheel mapping and polarity are correct. Change only a reversed wheel's named `*_SIGN` constant to `-1` when needed.

Send one command per line. `s`, `x`, `0`, or space stops early without waiting for newline and discards the rest of that line. `?` prints help.

## MPU-only test

Startup prints three WHO_AM_I reads at responding addresses `0x68`/`0x69`. The test accepts stable IDs `0x68`, `0x70`, and `0x74`; unknown or unstable identities remain blocked. Keep the module still for about three seconds of calibration, then rotate it by hand to check relative yaw. `c` recalibrates and `z` zeros yaw.

The output columns are Ax/Ay/Az in g, gyro Z in degrees/second, and yaw in degrees. At rest the acceleration magnitude should be about 1 g and gyro Z near zero. Yaw is relative, wraps at +/-180 degrees, and drifts over time. Calibration checks gross acceleration and Z-axis instability; it does not prove perfect stillness. A sensor fault stops reporting until `c` retries.

## IR-only test

The test reads six AC bytes at register `0x49` from 7-bit address `0x08`, preserving the sensor's existing frequency setting. Output is direction, derived strength, and S1-S5. Direction `0` means no detected direction, `5` is center, `1`-`4` are left, and `6`-`9` are right.

## I2C test

The scanner repeats every three seconds. For responding MPU addresses `0x68`/`0x69`, it prints three raw reads of register `0x75`. The I2C address and WHO_AM_I value are separate numbers. This test does not configure or calibrate the MPU.

## Validation

See [VALIDATION.md](VALIDATION.md). Software compilation does not verify physical wiring, sensor behavior, or motor movement.
