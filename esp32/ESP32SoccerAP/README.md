# Full ESP32 soccer program with AP control

This is the AP version of the Linux full / ESP32SoccerFull driving logic:
HiTechnic ball following, repeating no-turn search, MPU heading hold, and
APDS white-line escape with priority over following/searching.

Open **ESP32SoccerAP.ino**, with all **six `.h` files** beside it. Select
**ESP32 Dev Module**, using Espressif Arduino core **3.x**. No external
libraries are required. Target: classic **ESP32-WROOM-32**.

**This program automatically starts playing after boot when saved calibration
and all required sensor readings are valid.** To start paused instead, change
`espconfig::AUTO_RUN_WHEN_READY=false` in `PlaySensors.h`. Upload with motor
power off and use the separate bench dashboard to verify hardware first.
After any AP/Serial calibration, rescan or manual STOP, play remains paused
until you explicitly press **Resume play**.

## AP and calibration

1. Join Wi-Fi **SoccerPlay-xxxx**, password **SoccerPlay32**. Stay connected
   despite the phone's no-internet message.
2. Open **http://192.168.4.1/**. No router or internet connection is needed.
3. The page continuously shows play state, logical wheel PWM, MPU raw data /
   yaw / faults, HiTechnic direction and detector strengths, and APDS colour /
   threshold / line readings. Monitoring itself does not pause the robot.
4. **Calibrate gyro then GREEN + save** stops the drive first, calibrates and
   saves the stationary gyro, then provides 3 seconds to put all four APDS
   sensors over green at mounted height before collecting/saving 32 samples
   per sensor. Keep the robot stationary during the gyro phase. There is no
   white capture. Individual gyro/green buttons are also available.
5. Test each sensor over the actual white tape, then position the robot on
   the field and press **Resume play**. Resume is rejected unless IR, gyro,
   APDS data, calibration and PWM are ready. Direction 0 is a valid no-ball
   IR reading and permits searching.

Calibration can be requested **during active following/searching/escape**.
The request invalidates queued motor commands immediately; the control task
waits for the motor worker to confirm zero output before touching calibration
storage. STOP cancels a pending/active calibration. Cancelling a capture does
not restore an old calibration record that was already erased.

Gyro and line data use the same checksummed **Preferences/NVS** namespace
`soccer32`, keys `gyro` and `line`, as ESP32SoccerFull and ESP32BenchDashboard.
Saved values survive reset and are loaded without rewriting flash. Gyro
calibration is automatic on boot if no valid bias is stored; missing green
calibration keeps the robot stopped. Sensor heights, gain or lighting changes
require a fresh green capture. APDS thresholds detect brightness above the
field, not a specific white colour.

Closing/disconnecting the webpage **does not stop autonomous play**. The AP
is an optional monitor/control interface; use STOP while connected or the
motor power switch when you need to stop without a working connection.

## Wiring and editable configuration

| Motor | MDD3A input 1 / input 2 | Sign |
| --- | --- | --- |
| FL | GPIO25 / GPIO26 | +1 |
| FR | **GPIO16 / GPIO17** | +1 |
| BL | GPIO32 / GPIO33 | **-1** |
| BR | GPIO18 / GPIO19 | +1 |

**SDA21 / SCL22** are the main I2C rails. HiTechnic **0x08** and MPU
**0x68/0x69** stay directly on them. TCA **0x70** shares those rails;
only APDS **0x39** devices go behind TCA channels **0..3**.
Your MPU identity **0x70** and APDS identity **0x9E** are accepted.

In `PlaySensors.h`, edit `boundarycfg::SIDES` to match the actual APDS mounts.
The initial CH0/1/2/3 positions are **front/right/rear/left** examples. Incorrect
mapping produces an incorrect escape direction. Motor signs are `cfg::*_SIGN`
in this same header. GPIO pairs are `PlayMotor::pins` in `PlayMotor.h`.
Chase/search PWM and heading-correction limits are in `PlayController.h`.
AP credentials and `ENABLE_AP` are near the top of the `.ino`.

Use the [complete ESP32 wiring guide](../WIRING.md). ESP32-facing I2C signals
must be **3.3 V**; retain the tested HiTechnic power wiring and use an I2C
level shifter where required for its data pull-ups. Use common grounds and a
clean electronics supply; leave both MDD3A **5Vo** outputs disconnected from
electronics rails. Wi-Fi adds supply load, so this software does not establish
that the earlier battery-triggered MPU fault is fixed. Keep motor power off
while uploading and use external **10 kΩ pull-downs** on driver inputs during
reset/upload. No NeoPixels are used.

## Driving and fault handling

- A visible ball selects translation from the baseline HiTechnic AC sectors.
  Heading correction holds the relative MPU yaw; there is no deliberate turn
  even for sectors 1 and 9. Yaw drifts over time and is not a compass heading.
- When the ball disappears, sideways/forward searches repeat with brief pauses
  until it returns. There is no terminal search-completed state.
- White lines override search/follow and move away from the mapped active side.
  Opposite-side detections hold still. Long escapes pause/retry; a stable clear
  releases normal play. The robot requires healthy calibrated APDS and gyro
  data before moving. Line escape retains priority when the IR reading fails.
- IR polling continues while stopped or calibrating, so the page can monitor
  the ball without enabling movement. Invalid/stale IR prevents normal
  searching/following and Resume. A later valid read recovers automatically.
- MPU faults latch; use stationary gyro calibration or **Rescan / reload**,
  then Resume. Missing/config-failed APDS sensors also require rescan. Mux
  isolation faults can block direct MPU/IR accesses; check the mux before
  attributing every reported read failure to the MPU itself.
- The motor worker runs independently at nominal 5 ms intervals. Commands
  older than **100 ms** expire and stop/disarm the drive. Old queued commands
  cannot restart it. A watchdog fault requires explicit Resume after sensors
  are healthy; a controller stall may also require MPU rescan/recalibration.
- Runtime PWM failure detaches all eight PWM outputs and holds them LOW.
  Reset is required after fixing the driver/PWM initialization problem.

HTTP handling, sensor/control service and motor PWM run in separate tasks.
Only the control task accesses I2C/NVS; only the motor worker writes PWM after
initialization. STOP has a request generation that invalidates older Resume
claims, preventing a queued Resume from overriding a newer Stop.

## Serial commands

115200 baud, **Newline**. One complete character per line, except immediate
STOP (`s`, `x`, `0`, or space).

| Command | Action |
| --- | --- |
| `r` | Resume after all readiness checks |
| `s` | Stop and cancel calibration |
| `c` | Stop; calibrate/save gyro; remain stopped |
| `g` | Stop; prepare/capture/save green; remain stopped |
| `f` | Stop; calibrate/save gyro, then green; remain stopped |
| `b` | Stop; rescan/reload; remain stopped |
| `p` | Print one cached JSON snapshot |
| `m` | Add battery-switch event marker |

The page offers the same controls, an event list and **46-column CSV** export
of its last 2,000 samples. Changing the AP password is recommended before
using it around other teams; actions are restricted to the dashboard's per-boot
token in addition to Wi-Fi access.

## First physical run

Use [ESP32BenchDashboard](../ESP32BenchDashboard/README.md) first. Confirm battery
switching does not corrupt I2C, every motor's direction, each APDS side and
white-tape contrast. For the new full program, start paused for initial lifted
wheel tests, verify heading correction and all escape directions, then conduct
a low-speed field test with a reachable motor power switch.

See [VALIDATION.md](VALIDATION.md) for software checks and their limits.
