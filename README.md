# Soccer robot tests

For the classic **ESP32-WROOM-32** port, use the separate **[esp32 folder](esp32/README.md)**
and its **[ESP32 wiring guide](esp32/WIRING.md)**. Its GPIO map differs from the Mega map below.
The **[ESP32 AP bench dashboard](esp32/ESP32BenchDashboard/README.md)** adds live
sensor monitoring, saved calibration and explicitly armed motor pulses from a phone.

Arduino Mega 2560 sketches for a four-motor soccer robot with two Cytron MDD3A drivers, a HiTechnic IR Seeker V2, and an MPU-compatible IMU. Open one sketch folder at a time in Arduino IDE; each folder is a separate program.

| Sketch | Purpose |
| --- | --- |
| [MegaBallSearchNoTurn](sketches/MegaBallSearchNoTurn) | Continuous IR search/follow with MPU heading correction and calibrated APDS escape |
| [Test code Linux full](sketches/test%20code%20linux%20full) | Single-file automatic play variant with green-only calibration; c=gyro, g=green, r=resume |
| [MegaIROnlyTest](sketches/MegaIROnlyTest) | IR direction and strength diagnostic; motors held LOW |
| [MegaI2CTest](sketches/MegaI2CTest) | I2C scan and three MPU identity reads; motors held LOW |

The integrated movement sketch is `MegaBallSearchNoTurn`. The latest standalone motor, MPU-only, IR-only, and I2C tests are collected separately in **[hardware-tests](hardware-tests/README.md)**, with wiring and serial commands. The sketches use the motor pin map supplied for the current robot:

| Motor | Input 1 | Input 2 |
| --- | --- | --- |
| Front left | D6 | D7 |
| Front right | D4 | D5 |
| Back left | D11 | D10 |
| Back right | D2 | D3 |

Motor polarity: `FL_SIGN=1`, `FR_SIGN=1`, `BL_SIGN=-1`, `BR_SIGN=1`. All movement sketches use these defaults; diagnostic sketches hold the same eight pins LOW.

Each MDD3A channel uses two PWM control inputs. Connect Mega ground and both driver grounds together. Motor power goes to the drivers' motor supply terminals. Mega I2C is SDA D20 and SCL D21. Use the same IR power and signal wiring that worked in the IR-only test, and appropriate bus-voltage handling for the IMU.

## Ball follower

Open `sketches/MegaBallSearchNoTurn/MegaBallSearchNoTurn.ino` with all six headers, select **Arduino Mega 2560**, upload, and open Serial Monitor at **115200 baud, Newline**. Edit the placeholder sensor positions in **BoundaryConfig.h** first. The program starts stopped and requires four healthy, calibrated APDS sensors before `g` can enable movement. Keep the robot still during gyro calibration. The MPU reader accepts stable IDs `0x68`, `0x70`, and `0x74`; this robot's latest diagnostic reported `0x70` at address `0x68`.

Keep still over the field and send `f`: the gyro is calibrated and saved first, then field capture starts automatically. Wait for all four field results; capture white for each sensor with `1`, `2`, `3`, and `4`. Wait for `LINE calibration saved`, reposition onto the field, then use `g`. Gyro offset and line thresholds are saved separately to EEPROM and loaded after reboot; `c` refreshes/saves only the gyro. White-line detection overrides ball following with a short retreat; stale/faulty sensors block movement. [Boundary mapping, calibration, and commands](sketches/MegaBallSearchNoTurn/BOUNDARY.md).

If the ball disappears, the robot repeats 1.95-second sideways/forward searches with 200 ms pauses until the ball returns. White-line detections override search/follow, command movement away, and automatically release play after a stable clear. Long escapes pause/retry; ambiguous opposite detections hold until resolved. IR read failures stop and retry; MPU faults require stationary `c` recalibration. Send `s` to stop. [Main sketch instructions](sketches/MegaBallSearchNoTurn/README.md).

**Lift the wheels for the first test.** Check wheel polarity, gyro correction, and the retreat direction for each APDS sensor before floor testing. APDS sensors detect local brightness contrast; the MPU measures relative heading. These do not provide global position or guaranteed boundary/goal avoidance. See [validation](VALIDATION.md).
