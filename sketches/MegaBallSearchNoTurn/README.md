# Simple ball follower with APDS boundary priority

Open `MegaBallSearchNoTurn.ino` with all six `.h` files in this folder. Select **Arduino Mega 2560**, upload, and set Serial Monitor to **115200 baud, Newline**. Only built-in Wire and EEPROM are required. No NeoPixels are used.

The robot follows the HiTechnic IR ball with MPU heading correction. With no ball it stops and keeps reading, automatically following again when the signal returns. IR read errors stop and retry automatically. APDS boundary detection overrides following with a bounded retreat; missing or stale line data blocks movement.

**First edit the example channel/position map in BoundaryConfig.h.** The user requested editable placeholder positions: CH0 front, CH1 right, CH2 rear, CH3 left. Check the map on the actual robot before enabling motion.

**The program starts stopped.** Calibrate with `f` over the playing field, then `1`, `2`, `3`, `4` with the corresponding sensor over white. Wait for `LINE calibration saved`, reposition onto the field, then send `g`. Calibration is retained in EEPROM for later starts. Full instructions, corner mappings, threshold behavior, fault states, and tuning are in **[BOUNDARY.md](BOUNDARY.md)**.

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

- `g`: enable following only when all sensors/calibration are ready.
- `f`: stop and capture the field under all four APDS sensors.
- `1` / `2` / `3` / `4`: stop and capture white for the corresponding sensor.
- `c`: stop and recalibrate the MPU while stationary.
- `b`: stop, rescan the line sensors, and restart gyro calibration.

With wheels lifted, show white to each sensor and check that retreat points away from its physical side. After all sensors clear for 120 ms, the robot resumes normal following. A retreat timeout or opposing edges stops in `LINE_BLOCKED`; reposition onto the field before `g`. There is no blind ball-search movement. A ball outside the forward sensor's view may remain unseen.

Calibration and tests do not establish physical stopping distance or guarantee boundary containment. See [validation](../../VALIDATION.md) and the [bench checklist](../../verification/README.md).
