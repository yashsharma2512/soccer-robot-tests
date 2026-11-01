# Ball search without search turns or blind reverse

Open `MegaBallSearchNoTurn/MegaBallSearchNoTurn.ino` as the Arduino sketch,
select **Arduino Mega 2560**, and upload. Serial Monitor is **115200 baud**.
This version uses both the HiTechnic IR Seeker V2 and the MPU yaw reader on
Mega SDA D20 / SCL D21. The MPU reader accepts the `0x74` identity reported
by your module. Keep the robot still until `MPU ready; holding heading.`

Motor inputs are the pins you supplied:

| Wheel | Input 1 | Input 2 |
|---|---:|---:|
| Front left | 8 | 6 |
| Front right | 9 | 7 |
| Back left | 4 | 2 |
| Back right | 3 | 5 |

The IR sensor should face the robot's forward direction. It retains the AC
mode and 600/1200 Hz setting already on the sensor and reads the same six
bytes from register `0x49` as your working IR-only test.

## What it does

- With a valid ball direction, it drives forward and/or sideways toward the
  signal. Directions 1 and 9 now strafe; they do not command a search turn.
- With no signal, it makes a bounded, square-like sweep: sideways toward the
  last sighting, forward, sideways across, forward, sideways back across,
  forward, and sideways to the starting lane. The sweep takes about 2.85 s
  of commanded motion. It then stops and reports `SEARCH_DONE` if still blind.
- The MPU corrects yaw during translation to keep the heading it had after
  calibration. The search commands no rotation and no backward travel.
- `s`, `x`, `0`, or space stops immediately; `g` resumes and starts a new
  search if needed. `c` recalibrates the MPU while stationary. An IR read
  fault stops movement until `g`; an MPU fault stops movement until `c`.

The sketch starts searching automatically after calibration if no ball is
visible. **Lift the wheels for the first test.** Confirm positive yaw increases
when rotating the robot counterclockwise by hand, and check wheel direction
before putting it on the floor. Change `GYRO_Z_SIGN` or the individual
`*_SIGN` constants in `RobotConfig.h` if those tests show reversed signs.

The MPU measures heading, not position. Timed legs cannot return the robot
exactly to its prior location or identify field boundaries or your own goal.
Avoiding blind reverse reduces the specific self-goal risk you described,
but this robot cannot guarantee goal or obstacle avoidance with these sensors.
A ball wholly behind the fixed forward-facing IR sensor may remain unseen
without a rear sensor or an intentional change of heading.

The sketch and search-pattern checks compiled/passed for Mega 2560. No board
upload or physical motion test was performed here.
