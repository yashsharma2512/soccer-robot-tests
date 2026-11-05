# Test code Linux full

Open `test code linux full.ino` from this folder in Arduino IDE and select
**Arduino Mega 2560**. The sketch is self-contained. Set Serial Monitor to
**115200 baud** and **New Line**. The motors stay braked until the saved
calibrations and live sensor readings pass the safety checks; then the main
ball-following code runs automatically.

## Serial commands

| Send | Action |
| --- | --- |
| `c` | Recalibrate and save the MPU gyro. Keep the robot still until it finishes; normal running then resumes automatically. |
| `g` | Calibrate all four boundary sensors over the green/dark field only. There is a 3-second placement window, then 32 readings per sensor. Normal running resumes after `LINE calibration saved` if all sensors are healthy. |
| `r` | Resume after a manual stop, or restart the current search cycle. |
| `s` | Stop immediately and brake the motors. `x`, `0`, and space also stop. |
| `b` | Rescan boundary sensors and reload saved calibration. |

Send each letter followed by Enter. A single complete command is required;
`s` stops immediately. On later boots, valid MPU and line calibrations load
from EEPROM and the robot starts automatically once ready. This version uses
green-only calibration record version 2, so an older white-plus-green line
calibration must be replaced with one `g` capture.

Before running the robot, set `boundarycfg::SIDES` in the sketch to match the physical
positions of TCA9548A channels 0–3. The values currently shown are examples:
front, right, rear, left. With the wheels lifted, check that each sensor reports
the correct `line=0x...` hit and that the line escape moves away from that edge.

## Bright boundary rule

Each sensor learns its own green level. It treats a reading at least 25% above
that level, 8 noise deviations above it, and 20 clear counts above it as a
bright boundary. The threshold has hysteresis. A sensor saturation during play
also counts as bright and appears as `SAT` in the serial readings. Saturation
while collecting green is rejected; it cannot give a useful baseline. A
communication fault, missing sensor, stale reading, or changed configuration
still stops the motors. If the green readings are too bright or noisy, the
sketch refuses to save and prints the affected sensor number.

This is a brightness rule, so other bright patches can also trigger boundary
escape. It cannot establish that a surface is specifically white without a
white reference. Test the threshold across several green and white spots at
the robot's actual sensor height before playing. Saturation during white
calibration no longer matters because this sketch has no white capture.

## Hardware and movement

- Mega I2C: SDA 20, SCL 21; TCA9548A at `0x70`; four APDS sensors on channels 0–3.
- HiTechnic IR Seeker V2: `0x08`; MPU: `0x68` or `0x69`.
- MDD3A motor inputs: FL 8/6, FR 9/7, BL 2/3, BR 10/11. Back-left polarity is inverted in software. LOW/LOW brakes each motor.

The robot follows a detected ball while holding its MPU heading. After a
200 ms lost-ball pause, it searches sideways and forward for 1.95 s, pauses
200 ms, and repeats until the ball is detected.
Boundary escape has priority. A front sensor commands backward motion; rear
commands forward; right commands left; left commands right. Corner hits combine
the two directions. If opposing sides trigger together, the robot brakes in
`LINE_HOLD` and reevaluates on the next sensor readings. It resumes when the
direction becomes unambiguous; it never chooses a direction into another
currently active boundary. A retreat lasting 650 ms
gets a 120 ms brake before automatically trying again. Once all boundary
sensors have been clear for 120 ms, normal ball following resumes, or a fresh
ball search starts after the usual 200 ms lost-ball pause.

Ball search repeats continuously while the ball remains unseen; a ball sighting
immediately resumes following. Send `r` to resume after `s`.
Communication faults, stale boundary data, missing calibration, and MPU faults
still brake the robot until healthy. The code compiles for Mega 2560; motor
direction and boundary behavior still need testing on the physical robot.

## MPU read failures

`id=0x70` is a supported MPU identity. `MPU_READ_FAILED` means an I2C transfer
failed after the MPU was found. This sketch retries each MPU read once. The
`mpu_retries=` count increases when the second attempt succeeds. If both
attempts fail, the motors brake and an `MPU I2C` line prints the register,
failed step, Wire status, byte count, and timeout flag. A retry can absorb an
occasional glitch, but repeated errors need a wiring or power fix. Compare
`mpu_retries` with motor power connected and disconnected; also check common
ground, MPU supply voltage, SDA/SCL connections, and routing away from motor
wires. Send `b` to rescan after correcting a persistent read fault.
