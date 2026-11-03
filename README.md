# Soccer robot tests

Arduino Mega 2560 sketches for a four-motor soccer robot with two Cytron MDD3A drivers, a HiTechnic IR Seeker V2, and an MPU-compatible IMU. Open one sketch folder at a time in Arduino IDE; each folder is a separate program.

| Sketch | Purpose |
| --- | --- |
| [MegaBallSearchNoTurn](sketches/MegaBallSearchNoTurn) | Current ball follower and bounded sideways/forward search with MPU heading correction |
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

Open `sketches/MegaBallSearchNoTurn/MegaBallSearchNoTurn.ino`, select **Arduino Mega 2560**, upload, and open Serial Monitor at **115200 baud**. Keep the robot still during gyro calibration. The MPU reader accepts stable identities `0x68`, `0x70`, and `0x74`, matching the standalone MPU test. The latest diagnostic from this robot reports `0x70` at address `0x68`; the earlier main reader rejected that ID as error 2 (`MPU_WRONG_ID`). The updated reader configures the MPU6500-family accelerometer filter and prints three identity reads plus named errors. It starts following a valid IR signal automatically after calibration.

If the ball signal disappears, the robot keeps its facing direction and performs one bounded sideways/forward search. It never commands a search turn or blind reverse. The search ends after about 2.85 seconds if no ball is found. Send `s` to stop, `g` to resume/retry, or `c` to recalibrate. [Detailed behavior and first-test instructions](sketches/MegaBallSearchNoTurn/README.md).

**Lift the wheels for the first test.** Check wheel polarity and gyro correction direction before floor testing. The MPU measures relative heading, not position; the robot cannot know its exact location, field boundaries, or own goal from these sensors. See [validation](VALIDATION.md).
