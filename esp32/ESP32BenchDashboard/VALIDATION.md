# Bench dashboard validation — 7 November 2026

Target: `esp32:esp32:esp32` (classic ESP32 Dev Module), Espressif Arduino
core **3.3.12**, built-in Wire/Preferences/WiFi/WebServer libraries.

The final firmware compiled successfully: **995,575 / 1,310,720 flash bytes**
(75%) and **49,668 / 327,680 static RAM bytes** (15%). Remaining RAM is also
used at runtime by Wi-Fi, task stacks, HTTP buffers and telemetry strings.

## Checks passed

`esp32/verification/esp32_bench.cpp` compiles the actual motor controller,
input/mixer helpers and sensor backend with host doubles for Arduino,
register-level I2C, Preferences and FreeRTOS queue/task creation. It checks:

- Disarmed boot; explicit arm; rejection of overlapping queued pulses.
- FL25/26, FR16/17, BL32/33 with reversed sign, BR18/19; old FR pins unused.
- 400 ms pulse cutoff while no network/sensor loop runs, using the motor
  worker's actual `tick()` function at 5 ms intervals.
- Stop overriding queued motion, arm expiry cutting active motion, and pulse
  timing across the 32-bit `millis()` wrap.
- Invalid numeric input, unknown movement targets, PWM/duration bounds,
  strafe/rotation mixing, PWM attachment failure and runtime output failure.
- MPU ID70/APDS ID9E, fresh raw MPU and APDS telemetry, stationary calibration,
  shared NVS gyro/line records, reload, and white-line indication.
- Simulated 80 ms NVS writes and 60 ms erases. Gyro clock resynchronization
  is tested both after saving and before service following a repeat green
  capture that erases a prior record.
- Capture cancellation, failed NVS writes, transient MPU read recovery,
  persistent MPU read failure, retained error metadata, and APDS stale/error
  reporting.

`esp32/verification/bench_page.cjs` executes the **actual embedded JavaScript**
with a small DOM/network double. Checks passed for sensor rendering, all
14 motor buttons, disarmed/busy/offline control gating, per-boot token header
on action POSTs, and consistent **44-column CSV** output.

These tests simulate controller logic; they do not emulate FreeRTOS scheduling
or a physical Wi-Fi stack. Firmware compilation checks the actual core APIs.
The in-app browser could not reach the Windows localhost preview server, so
visual browser layout verification was unavailable in this session.

## Reproduce

From the repository root, in a Visual Studio x64 Developer Command Prompt:

```bat
cl /nologo /std:c++14 /EHsc /W4 /DARDUINO_ARCH_ESP32 /DCONFIG_IDF_TARGET_ESP32 /Iesp32\verification\fakes esp32\verification\esp32_bench.cpp /Fo%TEMP%\esp32_bench.obj /Fe%TEMP%\esp32_bench.exe
%TEMP%\esp32_bench.exe
node esp32\verification\bench_page.cjs
arduino-cli compile --fqbn esp32:esp32:esp32 esp32\ESP32BenchDashboard
```

No hardware was uploaded or operated during these checks. Physical motor
direction, AP connectivity, electrical noise, supply stability, sensor mounting,
white-tape contrast and pulse timing must still be checked on the robot using
the sequence in [README.md](README.md).
