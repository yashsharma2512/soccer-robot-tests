# ESP32-WROOM-32 soccer robot

This folder ports the Linux full sketch to the classic **ESP32 DevKit / ESP32-WROOM-32**.
It uses the same four-motor mixer, HiTechnic IR following, repeated sideways/forward
search, MPU heading hold, green-only APDS calibration, and white-line escape priority.
IR and MPU remain on the **main I2C rails**. Only the four APDS sensors use the TCA.
No NeoPixels or Bluetooth are used. The optional bench dashboard uses Wi-Fi AP mode.

| Open this sketch | Purpose |
| --- | --- |
| [ESP32HardwareTest](ESP32HardwareTest/ESP32HardwareTest.ino) | Read all sensors; explicitly requested 400 ms motor pulses only |
| [ESP32BenchDashboard](ESP32BenchDashboard/README.md) | Offline AP webpage, live sensor telemetry, saved calibration, armed motor pulses |
| [ESP32SoccerFull](ESP32SoccerFull/ESP32SoccerFull.ino) | Full continuous search/follow/escape program, saved calibration |

The original sketches are self-contained `.ino` files; the bench dashboard's
four `.h` files must stay beside its `.ino`. All require only libraries bundled
with **esp32 by Espressif Systems 3.x**. Select **ESP32 Dev Module**, not Arduino
Nano ESP32, ESP32-C3, or ESP32-S3. Tested compilation uses Espressif core 3.3.12.

Read **[WIRING.md](WIRING.md)** before connecting the ESP32. The Mega motor pin map
does not apply here. GPIO21 is SDA, GPIO22 is SCL, and the ESP32-facing bus is 3.3V.

## First test

1. With motor battery off, wire the sensors and power the ESP32 through USB.
   Upload `ESP32HardwareTest`; open Serial Monitor at **115200 baud, Newline**.
   It never starts the motors automatically. Check APDS channels 0-3, IR at
   0x08, and MPU at 0x68/0x69. Your observed MPU ID 0x70 and APDS ID 0x9E are accepted.
2. Lift the wheels. Check battery switching while all motor outputs are zero.
   Send `1`-`4` for FL/FR/BL/BR forward pulses, `5`-`8` for reverse, and `s` to stop.
   Each pulse ends after 400 ms; the test avoids I2C transfers during a pulse.
   `t` stops and reinitializes the APDS sensors.
3. Fix wiring/power/I2C faults before proceeding. Migrating to ESP32 does not
   establish that the earlier battery-triggered MPU fault has been resolved.

## Full program

Edit the example positions in `boundarycfg::SIDES` inside `ESP32SoccerFull.ino`.
Default sensor order is front, right, rear, left on TCA CH0, CH1, CH2, CH3.
Corner positions and a different channel list are supported. Incorrect mapping
commands the wrong escape direction.

**The full program automatically runs after valid calibration and live readings.**
Set `espconfig::AUTO_RUN_WHEN_READY=false` to start stopped after boot instead.
The hardware test remains the first sketch to upload for a new wiring setup.

| Command + Enter | Full-program action |
| --- | --- |
| `c` | Stop, calibrate the stationary gyro, save its offset, then resume when ready |
| `g` | Stop; allow 3 seconds to place all four sensors on green; collect/save green; resume when ready |
| `r` | Resume after a manual stop; restart the search cycle |
| `s` | Stop immediately; `x`, `0`, and space also stop |
| `b` | Stop, rescan APDS, reinitialize MPU, load saved calibration, resume when ready |

Commands require one complete character per line, except immediate stop.
Use `c`, wait for `MPU calibration saved to NVS`, then `g` with all sensors at
their mounted playing height over the green field. Motor battery can stay off
for calibration. Wait for `LINE calibration saved`, then send `s`. Once power
switching is verified and the robot is positioned for play, send `r`.
An invalid/missing gyro record also triggers stationary gyro calibration on boot.
Green calibration is required once on this ESP32; Mega EEPROM values are not imported.

The green-only threshold identifies brightness above the learned field, not a
specific white colour. No white capture is required. Bright saturation during
play counts as a boundary; saturated/weak/noisy green calibration is rejected.
Verify detection on the actual field and tape before enabling motion.

## During play and saving

With a ball, FOLLOW uses the original AC direction/strength read and heading hold.
Without a ball, SEARCH repeats 1.95-second sideways/forward sweeps with 200 ms pauses.
There is no terminal search state. White detection overrides either mode and
commands escape away from the detected side. Stable clear for 120 ms releases
play. An escape lasting 650 ms brakes for 120 ms then retries. Opposite-side
detections hold still until unambiguous; manual stop remains stopped until `r`.

Gyro and line calibration save separately in **Preferences/NVS**, namespace
`soccer32`, keys `gyro` and `line`. Records have CRC/version/config checks and
read-back verification. Flash is written only while stopped, not on every loop.
After flash writes the gyro requires fresh data and reseeds its integration clock;
storage latency is not integrated as a false heading change. Yaw itself resets
at boot. Invalid records request calibration; an NVS write failure stops the robot.
Preferences is Espressif's supported persistent-storage replacement for EEPROM.
[Preferences API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html).

PWM is eight independent LEDC channels at 16 kHz, 8-bit resolution (0-255).
Startup attachment or runtime PWM-write failures disable all motor outputs.
I2C uses 100 kHz and a 25 ms transaction timeout. The ESP32 Wire API does not
provide the AVR timeout flag, so diagnostics show status/byte counts and the
configured timeout bound. Failed MPU reads retry once, then stop on failure.
All APDS channels are deselected before main-rail IR/MPU reads.

See [VALIDATION.md](VALIDATION.md) for actual build and simulation results.
