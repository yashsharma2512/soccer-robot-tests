# Navigation assertions

`navigation_scenarios.cpp` includes the actual single-file MPU sketch and evaluates **37 `static_assert` checks** with the AVR C++ compiler. Successful compilation means all assertions passed; no robot or simulator is run.

The existing checks cover wheel mixing and normalization, yaw feedback and wrap, turn targets and damping, timed navigation legs, settling and timer rollover, plus accepted and rejected IMU IDs. They do not test electrical behavior, live I2C transactions, or physical movement.

Build this file separately from the Arduino sketches. The assertions need C++14; the ordinary Arduino firmware still builds with the core's C++11 setting.

For an Arduino AVR Boards 1.8.6 installation, adapt the compiler and core paths below. Run from the repository root; write the object file into a local `build` folder, which is ignored by Git.

```powershell
$avrCompiler = '<avr-gcc toolchain folder>\bin\avr-g++.exe'
$avrCore = '<Arduino AVR Boards 1.8.6 folder>'
New-Item -ItemType Directory -Force build | Out-Null
& $avrCompiler -Os -std=gnu++14 -mmcu=atmega2560 `
  -DF_CPU=16000000L -DARDUINO=10819 `
  -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR `
  -I (Join-Path $avrCore 'cores\arduino') `
  -I (Join-Path $avrCore 'variants\mega') `
  -I (Join-Path $avrCore 'libraries\Wire\src') `
  -Wall -Wextra -Werror -c verification\navigation_scenarios.cpp `
  -o build\navigation_scenarios.o
if ($LASTEXITCODE -ne 0) { throw 'Navigation assertions failed' }
```

Compile each normal sketch with Arduino IDE's Verify button or `arduino-cli compile --fqbn arduino:avr:mega sketches/<sketch-name>`. Use an installed core with Wire timeout support.
