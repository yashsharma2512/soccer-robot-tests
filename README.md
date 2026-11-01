# Soccer robot tests

Arduino Mega 2560 hardware tests for a four-motor soccer robot with two Cytron MDD3A drivers, a HiTechnic IR Seeker V2, and a GY-521 IMU module.

This snapshot covers the individual sensor and motor tests through the latest MPU motor/navigation test and I2C diagnostic. **IR-controlled movement, ball following, and autonomous ball search are excluded.**

## Sketches

Each folder is a separate program. Open its matching `.ino` file and upload only one program at a time.

| Sketch | Purpose | Status |
| --- | --- | --- |
| [MegaIROnlyTest](sketches/MegaIROnlyTest) | IR direction, derived strength, and five raw AC channels; motor outputs LOW | Reported working on the Mega |
| [MegaMotorOnlyTest](sketches/MegaMotorOnlyTest) | Individual FL/FR/BL/BR motor pulses and chassis movement | Motor baseline; independent of sensors |
| [MegaMPUSingleFile](sketches/MegaMPUSingleFile) | Motor pulses, relative heading hold, 90-degree turns, and timed navigation | Experimental: the connected IMU's identity issue remains unresolved |
| [MegaI2CTest](sketches/MegaI2CTest) | I2C scan plus three raw WHO_AM_I reads at responding MPU addresses | Current diagnostic; motor outputs LOW |

The latest MPU test is kept as a single file. Earlier duplicate navigation versions and combined IR/driving programs are omitted.

## Upload

1. Open the selected sketch in Arduino IDE.
2. Select **Arduino Mega or Mega 2560**, processor **ATmega2560**, and the board's port.
3. Upload, then set Serial Monitor to **115200 baud** and **Newline**.

These sketches use Arduino's built-in APIs and Wire library. No external sensor library is required. Validation used Arduino AVR Boards **1.8.6**, which supplies the Wire timeout functions used here.

## Wiring and test order

| Motor | Input 1 | Input 2 |
| --- | --- | --- |
| FL: front left | FL1 = D10 | FL2 = D11 |
| FR: front right | FR1 = D4 | FR2 = D5 |
| BL: back left | BL1 = D8 | BL2 = D9 |
| BR: back right | BR1 = D3 | BR2 = D2 |

Each MDD3A motor channel uses **two PWM control inputs**. Connect Mega ground, both driver grounds, and battery negative together. Motor power goes to the drivers' motor supply terminals. When powering the Mega by USB, leave driver 5VO outputs unconnected.

Mega I2C is **SDA D20, SCL D21**. Keep the already working IR wiring for the IR-only test. For the GY-521 connection and IMU checks, see [the MPU instructions](sketches/MegaMPUSingleFile/README.md).

Start with wheels lifted and use the motor-only sketch to check wheel identities and polarity. It does not require a working MPU. While the IMU ID is unresolved, use the I2C test with motor power off and only the GY-521 attached to the I2C bus. Attempt MPU-controlled movement after the sensor is identified and calibrated and the feedback direction has been checked.

The four-wheel mixer preserves the original drivetrain pattern:

```text
FL = forward + strafe + rotation
FR = forward - strafe - rotation
BL = forward - strafe + rotation
BR = forward + strafe - rotation
```

## Commands

Send one command per line. `s`, `x`, `0`, or space stops movement without waiting for a newline. `?` prints help.

| Command | Motor-only test | MPU test |
| --- | --- | --- |
| `1` / `5` | FL positive / negative | Same, after valid gyro calibration |
| `2` / `6` | FR positive / negative | Same, after valid gyro calibration |
| `3` / `7` | BL positive / negative | Same, after valid gyro calibration |
| `4` / `8` | BR positive / negative | Same, after valid gyro calibration |
| `w` / `b` | Forward / backward, 800 ms | Forward / backward with heading hold, 1.5 s |
| `a` / `d` | Strafe left / right, 800 ms | Strafe left / right with heading hold, 1.5 s |
| `q` / `e` | Rotate left / right, 800 ms | Request a relative left / right 90-degree turn |
| `g` | Unused | Hold current heading for 3 s |
| `n` | Unused | Timed forward/right/backward/left sequence |
| `c` | Unused | Stop and recalibrate while stationary |
| `z` | Unused | Stop and zero relative yaw |

Individual motor pulses use PWM 80 for 800 ms. Adjust only the relevant `FL_SIGN`, `FR_SIGN`, `BL_SIGN`, or `BR_SIGN` if a correctly wired wheel needs its polarity reversed.

## Validation and limits

See [VALIDATION.md](VALIDATION.md) for the compilation results and existing navigation assertions. Compilation does not verify physical motion or fix the current IMU identity problem. Relative gyro yaw drifts; timed navigation has no position or distance feedback.
