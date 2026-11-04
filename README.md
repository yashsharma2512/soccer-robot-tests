# Soccer robot tests

Arduino Mega 2560 sketches for a four-motor soccer robot with two Cytron MDD3A drivers, a HiTechnic IR Seeker V2, and an MPU-compatible IMU. Open one sketch folder at a time in Arduino IDE; each folder is a separate program.

| Sketch | Purpose |
| --- | --- |
| [MegaBallSearchNoTurn](sketches/MegaBallSearchNoTurn) | Simple IR follower with MPU heading correction and calibrated APDS boundary priority |
| [MegaIROnlyTest](sketches/MegaIROnlyTest) | IR direction and strength diagnostic; motors held LOW |
| [MegaI2CTest](sketches/MegaI2CTest) | I2C scan and three MPU identity reads; motors held LOW |

The integrated movement sketch is `MegaBallSearchNoTurn`. The latest standalone motor, MPU-only, IR-only, and I2C tests are collected separately in **[hardware-tests](hardware-tests/README.md)**, with wiring and serial commands. The sketches use the motor pin map supplied for the current robot:

| Motor | Input 1 | Input 2 |
| --- | --- | --- |
| Front left | D8 | D6 |
| Front right | D9 | D7 |
| Back left | D4 | D2 |
| Back right | D3 | D5 |

Each MDD3A channel uses two PWM control inputs. Connect Mega ground and both driver grounds together. Motor power goes to the drivers' motor supply terminals. Mega I2C is SDA D20 and SCL D21. Use the same IR power and signal wiring that worked in the IR-only test, and appropriate bus-voltage handling for the IMU.

## Ball follower

Open `sketches/MegaBallSearchNoTurn/MegaBallSearchNoTurn.ino` with all six headers, select **Arduino Mega 2560**, upload, and open Serial Monitor at **115200 baud, Newline**. Edit the placeholder sensor positions in **BoundaryConfig.h** first. The program starts stopped and requires four healthy, calibrated APDS sensors before `g` can enable movement. Keep the robot still during gyro calibration. The MPU reader accepts stable IDs `0x68`, `0x70`, and `0x74`; this robot's latest diagnostic reported `0x70` at address `0x68`.

Calibrate the field with `f`, then white for each sensor with `1`, `2`, `3`, and `4`. Wait for the calibration-save message, reposition onto the field, then use `g`. Thresholds are saved to EEPROM. White-line detection overrides ball following with a short retreat; stale/faulty sensors block movement. [Boundary mapping, calibration, and commands](sketches/MegaBallSearchNoTurn/BOUNDARY.md).

If the ball disappears, the robot stops and keeps reading IR, resuming when the ball returns. IR read failures stop and retry; MPU faults require stationary `c` recalibration. There is no timed ball search. Send `s` to stop. [Main sketch instructions](sketches/MegaBallSearchNoTurn/README.md).

**Lift the wheels for the first test.** Check wheel polarity, gyro correction, and the retreat direction for each APDS sensor before floor testing. APDS sensors detect local brightness contrast; the MPU measures relative heading. These do not provide global position or guaranteed boundary/goal avoidance. See [validation](VALIDATION.md).
