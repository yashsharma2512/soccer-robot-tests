# ESP32 validation - 7 November 2026

The optional AP bench dashboard has its own **[validation record](ESP32BenchDashboard/VALIDATION.md)**.

Both self-contained sketches compiled for **ESP32 Dev Module**
(`esp32:esp32:esp32`) with **Espressif Arduino core 3.3.12**.

FR now uses **GPIO16/GPIO17** on WROOM-32. Both builds and host tests passed
again with the revised map; hardware-test scenarios explicitly check FR
forward on GPIO16, reverse on GPIO17, and no use of the previous GPIO27/14.
No physical upload-time motor behaviour has been tested. See the wiring
guide's pull-down and motor-power instructions for reset/upload handling.

| Sketch | Flash bytes / 1,310,720 | Static RAM bytes / 327,680 |
| --- | ---: | ---: |
| ESP32SoccerFull | 328,311 | 24,080 |
| ESP32HardwareTest | 305,031 | 23,704 |

## Software simulation

The C++14 host tests include the actual `.ino` programs with test doubles for
Arduino timing/serial, Wire device registers, LEDC outputs, and Preferences.
The fake bus requires IR/MPU access on the main rails with the mux deselected;
only APDS sensors respond behind their selected TCA channels.

`verification/esp32_play.cpp` passed checks for:

- Eight ESP32 PWM pins, independent outputs, and BL polarity inversion.
- Stationary gyro and green-only line calibration; distinct NVS keys.
- Saved-record reload without flash rewriting; gyro CRC corruption rejection.
- 80 ms flash-write and 60 ms erase stalls without false MPU sample-gap faults;
  fresh gyro samples are required after storage.
- More than four continuous search cycles, ball following, all four mapped
  escape directions, timeout retry, stable clear, and opposite-side hold/recovery.
- A transient MPU retry; persistent MPU failure, IR failure, mux failure, and rescan.
- Manual stop; failed NVS open/write, PWM attachment failure, and runtime PWM
  failure all keeping motor outputs zero.

`verification/esp32_hardware.cpp` passed checks for main-rail sensor routing,
no automatic motion, newline validation, motor pulse pin/sign mapping, a
400 ms pulse deadline with an unavailable I2C mux, immediate manual stop,
invalid command shutdown, and runtime PWM failure shutdown.

Run from the repository root in a Visual Studio x64 Developer Command Prompt:

```bat
cl /nologo /std:c++14 /EHsc /W4 /DARDUINO_ARCH_ESP32 /DCONFIG_IDF_TARGET_ESP32 /Iesp32\verification\fakes esp32\verification\esp32_play.cpp /Fo%TEMP%\esp32_play.obj /Fe%TEMP%\esp32_play.exe
%TEMP%\esp32_play.exe
cl /nologo /std:c++14 /EHsc /W4 /DARDUINO_ARCH_ESP32 /DCONFIG_IDF_TARGET_ESP32 /Iesp32\verification\fakes esp32\verification\esp32_hardware.cpp /Fo%TEMP%\esp32_hardware.obj /Fe%TEMP%\esp32_hardware.exe
%TEMP%\esp32_hardware.exe
```

For firmware builds:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 esp32/ESP32SoccerFull
arduino-cli compile --fqbn esp32:esp32:esp32 esp32/ESP32HardwareTest
```

Compilation and simulation do not execute an ESP32, physical NVS flash, real
I2C transactions, level shifters, or the robot. No firmware was uploaded and
no physical motor/sensor test was performed. Check the actual breakout supply
voltages, pull-ups, side mapping, motor polarity, battery-switching behaviour,
and low-speed stopping distance with the hardware test before full play.
