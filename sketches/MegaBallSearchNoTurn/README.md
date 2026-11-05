# Continuous ball search/follow with APDS boundary escape

Open `MegaBallSearchNoTurn.ino` with all six `.h` files in this folder. Select **Arduino Mega 2560**, upload, and set Serial Monitor to **115200 baud, Newline**. Only built-in Wire and EEPROM are required. No NeoPixels are used.

The robot follows the HiTechnic IR ball with MPU heading correction. With no ball it waits 200 ms, searches sideways/forward for 1.95 s, pauses 200 ms, and repeats until the ball appears. A valid ball immediately returns to following. IR read errors stop and retry automatically. APDS white detection overrides both search and following with movement away from the detected side; missing or stale line data blocks movement.

**First edit the example channel/position map in BoundaryConfig.h.** The user requested editable placeholder positions: CH0 front, CH1 right, CH2 rear, CH3 left. Check the map on the actual robot before enabling motion.

**The program starts stopped.** Keep the robot still with all APDS sensors over the playing field and send `f`. It calibrates and saves the gyro first, then automatically captures the field. Wait for all four `FIELD sensor` results before using `1`, `2`, `3`, `4` with the corresponding sensor over white. Wait for `LINE calibration saved`, reposition onto the field, then send `g`. Both gyro offset and line thresholds are retained in EEPROM and loaded on later starts. Full instructions, corner mappings, threshold behavior, fault states, and tuning are in **[BOUNDARY.md](BOUNDARY.md)**.

## Connections

| Wheel | Input 1 | Input 2 |
| --- | ---: | ---: |
| FL | 8 | 6 |
| FR | 9 | 7 |
| BL | 2 | 3 |
| BR | 10 | 11 |

Motor polarity in `RobotConfig.h`: FL +1, FR +1, **BL -1**, BR +1.

Mega SDA is D20 and SCL is D21. IR address is 0x08; MPU is upstream at 0x68/0x69; TCA is 0x70, with one APDS at 0x39 per configured channel. Retain the previously checked sensor power and bus-level wiring. APDS IDs 0xAB, 0x9C, and the observed 0x9E are supported. The MPU6500 fix remains: supported stable IDs are 0x68, 0x70, and 0x74; this robot reported 0x70 at address 0x68.

## Commands

Send one command per line. `s`, `x`, `0`, or space stops without waiting for newline and discards the rest of that line.

- `g`: enable continuous searching/following only when all sensors/calibration are ready.
- `f`: stop; calibrate and save the gyro while still, then automatically capture the field under all four APDS sensors.
- `1` / `2` / `3` / `4`: stop and capture white for the corresponding sensor.
- `c`: stop; recalibrate and save just the gyro while stationary, preserving completed line thresholds.
- `b`: stop, rescan the line sensors, and reload saved gyro/line calibration. A missing/invalid gyro record triggers fresh calibration and saving.

Gyro saving uses EEPROM bytes 64-79, separate from the line record at 0-29, with a CRC and sensor/configuration checks. A loaded offset still requires fresh MPU data before movement. Saving an offset does not save an absolute compass heading: yaw starts at zero each boot. Temperature can change gyro bias; use stationary `c`, or the full `f` procedure, if heading drifts. `s` cancels a pending gyro/field procedure; retry `f` or `c` if gyro calibration was cancelled.

With wheels lifted, show white to each sensor and check that escape points away from its physical side. All sensors must clear for 120 ms before following/search resumes automatically. A long escape brakes after 650 ms for 120 ms, then retries from current readings. Opposing edges hold the motors at zero in `LINE_HOLD` until there is an unambiguous direction or a stable clear. No manual restart is required for an ordinary boundary encounter. `s` still stops until a separate `g`.

Search uses the existing sideways/forward movement pattern and gyro heading hold, with no deliberate turn. The starting sweep side follows the last ball sighting. Sensor field of view and travel path limit which ball positions can be found. Edit search PWM/timings near the top of the `.ino`; edit retreat settings and sensor positions in `BoundaryConfig.h`.

Calibration and tests do not establish physical stopping distance or guarantee boundary containment. See [validation](../../VALIDATION.md) and the [bench checklist](../../verification/README.md).
