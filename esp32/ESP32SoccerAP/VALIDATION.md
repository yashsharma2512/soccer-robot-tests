# AP full-program validation — 7 November 2026

Target: classic ESP32 Dev Module (`esp32:esp32:esp32`), Espressif Arduino
core **3.3.12**. Only bundled Arduino/ESP32 libraries are used.

The final firmware compiled successfully: **997,351 / 1,310,720 flash bytes**
(76%) and **49,772 / 327,680 static RAM bytes** (15%). Wi-Fi, task stacks,
HTTP buffers and telemetry strings also consume RAM dynamically at runtime.

## Software checks passed

`verification/esp32_ap_play.cpp` includes the **actual** PlayRuntime,
PlayController, sensor backend, request arbitration and motor worker. Arduino,
I2C device registers, Preferences and FreeRTOS task/queue creation are doubled;
the test invokes the real worker `tick()` function explicitly.

- First boot without green calibration remains stopped.
- Combined gyro-then-green calibration saves both records and remains stopped.
- Calibration can be requested while the motors are following the ball.
- Every simulated NVS write/erase asserts that **all eight physical PWM inputs
  are already zero**, including re-calibration with an existing stored record.
- Resume is rejected during calibration; manual STOP cancels calibration.
- Rescan loads stored calibration and remains stopped until Resume.
- Saved-record startup performs the configured automatic run without rewriting
  calibration flash.
- FL25/26, FR16/17, BL32/33 with BL polarity inversion, BR18/19; old FR pins unused.
- Ball following, more than four repeated search cycles, and ball reacquisition.
- All four mapped line-escape directions, timeout pause/retry, opposite-side
  hold, stable-clear release, and return to following.
- IR failure/recovery, mux failure/rescan, persistent MPU failure/recovery,
  and runtime PWM failure shutdown.
- Motor watchdog cuts output after a 100 ms stale command, independently of
  the controller/HTTP loop; recovery requires an explicit Resume.
- A stale command sitting in the queue never energizes the outputs. Expiry
  also works across the 32-bit `millis()` wrap.
- A newer STOP invalidates an older Resume request even if the controller
  already claimed that Resume before the STOP arrived.

`verification/play_page.cjs` executes the actual embedded JavaScript with a
small DOM/network double. It checks live play-state rendering, calibration
buttons remaining available during active play, Resume health/busy/offline
gating, watchdog display, authenticated action POSTs, and consistent
**46-column CSV** output.

## Reproduce

From the repository root, in a Visual Studio x64 Developer Command Prompt:

```bat
cl /nologo /std:c++14 /EHsc /W4 /DARDUINO_ARCH_ESP32 /DCONFIG_IDF_TARGET_ESP32 /Iesp32\verification\fakes esp32\verification\esp32_ap_play.cpp /Fo%TEMP%\esp32_ap_play.obj /Fe%TEMP%\esp32_ap_play.exe
%TEMP%\esp32_ap_play.exe
node esp32\verification\play_page.cjs
arduino-cli compile --fqbn esp32:esp32:esp32 esp32\ESP32SoccerAP
```

Host tests verify controller logic, not real FreeRTOS scheduling, Wi-Fi/AP
connectivity, sensor electrical timing, or physical wheel direction. No
hardware was uploaded or operated. Supply stability, battery-switch noise,
mounting, gyro correction, line contrast and real motor cutoff must be tested
on the robot. Use the bench dashboard first, then a paused/lifted-wheel test
of this program before field operation.
