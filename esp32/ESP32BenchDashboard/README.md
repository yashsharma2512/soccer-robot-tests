# ESP32 motor + sensor AP bench

Open **ESP32BenchDashboard.ino**, keeping all four `.h` files beside it. Select
**ESP32 Dev Module** with **esp32 by Espressif Systems 3.x**. This sketch uses
only bundled libraries. It is for the classic **ESP32-WROOM-32**, not WROVER,
C3 or S3. No NeoPixels are used.

## Connect

1. Upload with the motor battery OFF. Power the ESP32 from USB initially.
2. Join Wi-Fi **SoccerBench-xxxx**; the suffix identifies your board.
   Password: **SoccerBench32**. The network deliberately has no internet;
   tell your phone to stay connected if it asks.
3. Open **http://192.168.4.1/** in the browser. This is an ordinary HTTP page,
   not HTTPS. It needs no router, cloud account, or external web assets.
4. Wait for startup gyro calibration, keeping the robot still. Motors stay
   disarmed on every boot. A valid saved gyro record is loaded instead.

The page polls cached readings roughly three times per second. MPU sampling
and I2C run in a separate task. Motor PWM has its own higher-priority task,
serviced nominally every 5 ms; a slow HTTP connection cannot extend a pulse.
No network handler accesses I2C or writes calibration directly.

## Wiring

| Connection | ESP32 GPIO |
| --- | --- |
| FL MDD3A input 1 / input 2 | 25 / 26 |
| FR MDD3A input 1 / input 2 | **16 / 17** |
| BL MDD3A input 1 / input 2 | 32 / 33 |
| BR MDD3A input 1 / input 2 | 18 / 19 |
| Main SDA / SCL | **21 / 22** |

Motor signs: FL +1, FR +1, **BL -1**, BR +1; edit `BenchMotor.h` if necessary.
HiTechnic IR **0x08** and MPU **0x68/0x69** remain directly on the main I2C
rails. TCA **0x70** is on those same rails; APDS **0x39** devices go separately
behind TCA channels **0, 1, 2, 3**. Default sides are front/right/rear/left;
edit `boundarycfg::SIDES` in `BenchSensors.h` to match the actual mounts.

Keep the ESP32-facing I2C bus at **3.3 V**. Use a bidirectional I2C level
shifter for any HiTechnic branch whose data pull-ups are 5 V. Preserve your
tested HiTechnic power wiring. TCA VCC and APDS modules use 3.3 V; a GY-521
breakout's regulator supply and SDA/SCL pull-up voltages must be checked
separately. Read [the complete wiring guide](../WIRING.md).

Use a common ground and a clean electronics supply. Leave **both MDD3A 5Vo
outputs disconnected** from the electronics power rails. Wi-Fi adds load,
so the earlier battery-triggered MPU fault still needs a physical power/noise
test. For standalone operation use a suitable regulated 5 V supply at the
board's 5 V input; initially use USB alone. Do not connect raw motor-battery
voltage to a 5 V/3.3 V pin or combine board power sources.

Firmware cannot control motor inputs during the bootloader/upload. Keep the
motor battery off while uploading and use external **10 kΩ pull-downs** on
all eight driver control inputs to hold them LOW during reset.

## Motor tests

Raise the wheels. Tick **Wheels are raised**, then **Arm for 30 s**. Choose
PWM **30–120** and duration **100–500 ms**; defaults are 80 and 400 ms.
Test FL/FR/BL/BR separately in both directions before testing all-forward,
all-back, strafe or rotation. Rotation +/− follows the mixer convention;
verify its physical direction yourself. If a motor does not start, raise PWM
within this range and check its driver/power wiring.

Every click is one bounded pulse; commands while a pulse is queued/running
are rejected. Arm expires after 30 seconds, including during a pulse.
**STOP** stops and disarms; recalibration/rescan also disarms. The display
shows requested logical PWM, not measured motor speed: there are no encoders.
Sensor faults remain visible but do not prohibit an explicitly armed bench
pulse, which lets you compare faults before and during motor operation.

This is a bench test. White detection is displayed; it does not initiate
an escape, and there is no ball following. The motor battery switch remains
the physical way to remove drive power; the webpage's STOP needs a connection.

## Sensors and calibration

- **MPU:** shows address, supported identity (0x68/0x70/0x74), relative yaw,
  Z rate, raw ax/ay/az, raw temperature and gx/gy/gz, sample age, failed
  transactions and retry recoveries. Your observed ID **0x70** is accepted.
  Calibrate gyro with the robot stationary. It collects 500 samples, verifies
  stillness, saves bias and verifies the saved record. Yaw is relative, not
  magnetic heading, and will drift.
- **HiTechnic:** AC direction, baseline-compatible strength and five detector
  strengths. Move a pulsed IR ball left/centre/right: expect 1–4 / 5 / 6–9.
  With no ball, a successful read reports direction 0. A failed read is
  separately marked stale; retained numbers are not treated as fresh.
- **APDS:** accepts IDs AB/9C/**9E**; shows clear/R/G/B, per-channel freshness,
  errors, threshold/release and white status. Press **Capture GREEN + save**;
  there are 3 seconds to position all four sensors over the green field at
  mounted height, then 32 samples per sensor are collected. No white capture.
  Test actual white tape under each sensor afterward. Thresholding detects
  brighter-than-field surfaces; it does not classify a specific colour.
  Saturated/noisy green captures are rejected. Bright saturation after valid
  calibration counts as a boundary. Recalibrate after gain/height/light changes.

Calibration uses checksummed Preferences/NVS records in namespace `soccer32`,
keys `gyro` and `line`. They are compatible with **ESP32SoccerFull** on the
same board, and survive reset without repeated boot writes. Recalibrating
replaces that sensor's prior record; cancelling does not restore an erased
record. Flash writes occur only after the motor worker confirms PWM is zero.
The gyro requires fresh samples after storage stalls before reporting healthy.

Gyro faults latch until **Rescan / reload** or **Calibrate gyro**. IR polling
recovers automatically. Missing/config-failed APDS channels need a rescan.
The backend checks TCA isolation before direct MPU/IR access; a missing or
failed mux can therefore also block those readings. Resolve the mux fault
before attributing such read failures to the MPU itself.

## Suggested comprehensive run

1. Motor battery off: wait for fresh MPU/IR/APDS readings; check all IDs and
   channels, calibrate gyro and green, then test white tape under each sensor.
2. Reboot; verify saved calibration reloads and motors remain disarmed.
3. Press **Mark battery switch**, switch motor power on while still disarmed,
   and watch MPU failure count, last Wire status and sensor ages. This tests
   switching with stopped motors. It does not isolate attached motors from
   the driver electrically.
4. With wheels raised, arm and test each wheel/direction, then whole-robot
   movement pulses. Check sensor freshness while motors run.
5. During a pulse, disconnect the phone or close the page. The pulse should
   still end by its configured duration plus scheduling latency. Reconnect
   and test STOP. Verify the 30-second arm expiry.
6. If switching alone causes MPU failures, power down and physically unplug
   all motor leads from the driver output terminals, then repeat with the
   motors disarmed. Compare clean USB power and a suitable regulated supply.
   Do not make wiring changes while powered. Set `ENABLE_AP=false` for a
   Serial-only run to compare whether enabling Wi-Fi affects supply stability.
7. Download CSV before closing the page. It holds the last 2,000 samples
   collected by that browser; events/counters reset on board reboot.

## Serial alternative

115200 baud. Commands act immediately; Enter is optional.

| Command | Action |
| --- | --- |
| `a` | Arm for 30 s; raise wheels first |
| `1` `2` `3` `4` | FL FR BL BR forward, PWM80 / 400 ms |
| `5` `6` `7` `8` | FL FR BL BR reverse, PWM80 / 400 ms |
| `f` / `v` | All forward / all back, PWM80 / 400 ms |
| `s` / `x` / `0` / space | Stop, disarm and cancel pending calibration |
| `c` / `g` | Calibrate/save gyro / green |
| `b` | Rescan/reload saved calibration |
| `m` | Add battery-switch event marker |
| `p` | Print one JSON status snapshot |

The AP password is editable near the top of the sketch. Keep access limited
to people running the test. Action POSTs require the dashboard's per-boot
token, preventing an unrelated browser page from issuing a simple form action.

## Later integration into the main program

AP monitoring can run while the main program follows/searches for the ball
and gives boundary escape priority. Selecting calibration should first
suspend driving and disarm motors, then capture/save, and require an explicit
Resume after fresh calibrated readings return. This bench implements the
monitoring and stopped-calibration pieces; autonomous integration is the next
stage after the physical tests pass.

See [VALIDATION.md](VALIDATION.md) for the exact checks performed in software.

Implementation references: Espressif's [Wi-Fi AP API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html),
[Preferences/NVS API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html)
and [LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).
