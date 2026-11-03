# Simple ball follower with MPU heading hold

Open `MegaBallSearchNoTurn.ino` with its three `.h` files in this folder. Select **Arduino Mega 2560**, upload, and set Serial Monitor to **115200 baud, Newline**. The existing folder name is retained so the project path stays familiar.

## Behavior

- **Ball visible:** `FOLLOW` drives toward the IR direction with MPU yaw correction.
- **Ball unseen:** `WAIT_BALL` sets all wheel PWM to zero and keeps reading the IR sensor every 50 ms.
- **Ball visible again:** automatically returns to `FOLLOW`, including after a long wait. No `g` command is needed for reacquisition.
- **IR read error or invalid direction:** `IR_FAULT` stops the motors and retries on the next poll. A successful read returns to `WAIT_BALL` or `FOLLOW` automatically.
- **MPU failure:** `MPU_FAULT` stops movement. Correct the sensor issue and send `c` while stationary to recalibrate.

There is no timed search, search completion state, deliberate turn, or blind reverse. The robot stays still when it cannot see the ball. A ball behind the forward-facing sensor may remain unseen until it enters the sensor's view.

The old sketch already continued polling after `SEARCH_DONE`; its IR-error latch could block all further reads until `g`. This version removes that latch and the entire search sequence. Electrical bus failures and MPU faults can still prevent following; telemetry distinguishes them from `WAIT_BALL`.

## Startup and commands

Lift the wheels for initial testing. Keep the robot still through approximately three seconds of gyro calibration. Following becomes enabled automatically after calibration, but the wheels remain stopped until a valid ball signal is seen.

| Command | Action |
| --- | --- |
| `s`, `x`, `0`, space | Immediate stop; remains stopped until `g` |
| `g` + Enter | Enable following and capture the current heading |
| `c` + Enter | Stop and recalibrate the MPU; preserves whether following was enabled |

Send one command per line. Stop discards the rest of its line; a word such as `stopping` cannot accidentally enable following through its final `g`. Invalid or multi-character commands leave following disabled.

## Wiring and sensor setup

| Wheel | Input 1 | Input 2 |
| --- | ---: | ---: |
| FL | 8 | 6 |
| FR | 9 | 7 |
| BL | 4 | 2 |
| BR | 3 | 5 |

Mega I2C is **SDA D20, SCL D21**. Use the sensor power and signal-level connections already checked for the robot, with common ground shared by the Mega and both MDD3A drivers.

The MPU fix is retained: stable IDs `0x68`, `0x70`, and `0x74` are supported. This robot's latest diagnostic showed ID `0x70` at address `0x68`. Startup prints three identity reads and telemetry includes named MPU errors and the ID. Error 2 means `MPU_WRONG_ID`; unknown or unstable identities block movement.

The IR read remains six AC bytes from register `0x49` at address `0x08`, using the baseline strength calculation. Its frequency setting is preserved. A valid direction `1`-`9` with strength at least `MIN_STRENGTH` qualifies; its current value `1` is a baseline threshold, not calibrated noise rejection. Gyro heading is relative and drifts; these sensors do not provide position or field-boundary detection.

## First check

After calibration, show the ball: expect `FOLLOW` with nonzero PWM. Hide it for more than five seconds: expect `WAIT_BALL` with zero PWM. Show it again: expect `FOLLOW` automatically. Repeat on both sides, then verify `s` remains stopped until a separate `g` command. See the [bench checklist](../../verification/README.md) and [validation record](../../VALIDATION.md).
