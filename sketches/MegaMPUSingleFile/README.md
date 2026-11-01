# MPU motor and navigation test (experimental)

**The connected GY-521's identity problem is unresolved.** This is the latest test program, not a confirmed hardware fix. Run [MegaI2CTest](../MegaI2CTest) first to obtain actual scan and WHO_AM_I results. Use [MegaMotorOnlyTest](../MegaMotorOnlyTest) for motor testing that does not depend on a healthy IMU.

Open `MegaMPUSingleFile.ino` for Mega 2560. It contains all configuration and logic in one `.ino` and requires only built-in Wire. Serial Monitor is **115200 baud, Newline**. The IR sensor is not used.

## GY-521 connection

| Module | Mega / connection |
| --- | --- |
| VCC | 5 V for a standard regulator-equipped GY-521 board |
| GND | Common ground with Mega and both motor drivers |
| SDA | D20 via a bidirectional I2C level shifter |
| SCL | D21 via a bidirectional I2C level shifter |
| AD0 | LOW for address `0x68`; HIGH for `0x69` |
| INT | Unused |

The regulator-equipped GY-521's VCC input and its sensor-side I2C logic have different voltage requirements. Use a 3.3 V / 5 V bidirectional I2C level shifter: MPU on LV, Mega on HV, and common grounds. Mount the board flat with sensor Z approximately vertical. For this isolated test, disconnect the IR sensor from the bus.

References: [GY-521 module specifications](https://www.hotmcu.com/gy521-mpu6050-3axis-acceleration-gyroscope-6dofmodule-p-83.html), [I2C logic levels](https://learn.adafruit.com/working-with-i2c-devices?view=all), [MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), [MPU6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf).

## Identity and calibration

Startup and `c` read WHO_AM_I three times at responding addresses `0x68`/`0x69`. Only stable `0x68` (MPU6050) and `0x70` (MPU6500) identities are implemented. The MPU6500 path also configures its separate accelerometer filter register. Unknown IDs remain blocked.

Keep the robot stationary for approximately three seconds during calibration. The program uses 500 fresh samples, a +/-250 degrees/s gyro range, and 131 LSB/(degree/s). Gross movement can fail calibration; a slow constant rotation may escape its stillness checks.

All movement, including individual motor pulses, requires a calibrated and fresh gyro. Outputs start stopped. Read failures, stale data, and changed sensor configuration stop active motion. After a fault, use `c` while stationary, then explicitly choose a new movement.

## Commands and checks

See the [root command table](../../README.md#commands). Motor pins and `FL_SIGN`, `FR_SIGN`, `BL_SIGN`, `BR_SIGN` match the motor-only baseline.

1. Check wheel identity and polarity with the separate motor-only test.
2. With motors disabled or wheels lifted, check that calibration completes and stationary yaw is reasonably steady.
3. Rotate the chassis counterclockwise by hand while stopped. Positive yaw should increase; verify mounting and `GYRO_Z_SIGN` if it does not.
4. Test `g` with wheels lifted. Feedback should oppose heading displacement. Stop and correct the mapping/sign if it reinforces displacement.
5. Test `w`, `b`, `a`, `d`, then `q` and `e`, in a clear area. Tune gains and minimum turning PWM as needed.
6. Attempt `n` after the individual movements work.

The translation base is PWM 80, wheel ceiling 120, heading correction ceiling 40, and turning ceiling 60. A turn requests +/-90 degrees, requires +/-3 degrees and <=8 degrees/s for 200 ms to settle, and times out after six seconds. Check the physical angle separately.

Yaw is relative gyro integration and drifts over time and temperature. The `n` sequence uses timed 1.5-second legs with 0.5-second pauses; it has no distance feedback and cannot guarantee a closed square. Wire calls have a 25 ms timeout; a serial stop cannot interrupt an I2C operation already in progress.
