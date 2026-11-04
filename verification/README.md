# Boundary and follower verification

## Software checks

`boundary_scenarios.cpp` includes the actual BoundaryLogic.h and evaluates 33 `static_assert` checks as C++14 constant expressions. Successful compilation means the checked scenarios passed; no robot is run. The normal firmware builds with Arduino's ordinary C++11 setting.

From the repository root, with the installed AVR compiler on PATH:

```sh
avr-g++ -Os -std=gnu++14 -mmcu=atmega2560 -Wall -Wextra -Werror -c verification/boundary_scenarios.cpp -o boundary_scenarios.o
arduino-cli compile --fqbn arduino:avr:mega sketches/MegaBallSearchNoTurn
```

The logic scenarios check all 16 physical edge combinations, channel/corner mapping, calibration contrast/noise, hysteresis, movement permissions, retreat timeout and stable clear, new corner/opposing hits, reset, millis rollover, and CRC. They do not execute live I2C or EEPROM operations or establish physical motion. The old search-pattern checks were removed with the search implementation.

## Bench regression checklist

These checks require the robot and have **not** been run by the coding agent. Begin with wheels lifted or motor power off, while preserving the sensors' actual field height and lighting.

1. Edit BoundaryConfig.h to match each sensor's physical side/corner and channel. Check APDS startup prints four ready sensors and a stable supported MPU ID. Without line calibration, `g` must refuse movement with all PWM zero.
2. Run `f` with all sensors over the field; then `1`, `2`, `3`, `4` with the corresponding sensor over white. Wait for each capture result. Confirm strong contrast and the final EEPROM-save message. Invalid/noisy contrast must leave movement blocked.
3. Put all sensors back over the field. Wait for stationary gyro calibration, then send `g`. Show/hide the ball: expect FOLLOW/WAIT_BALL. Hide it for at least five seconds and show it again; following should resume automatically without a search command.
4. While a ball is visible, put white under each APDS sensor individually. Verify LINE_ESCAPE overrides following, and that wheel direction would move the chassis away from that sensor's actual side. Check front, rear, left, right, and configured corners before floor tests.
5. Remove the white tape from view. Escape should continue briefly, clear after 120 ms, stop for that control pass, then return to ordinary IR behavior. Reappearing white should restart the clear interval. With no ball it should return to WAIT_BALL.
6. Leave white under a sensor for more than 650 ms. Expect LINE_BLOCKED and zero PWM, remaining stopped even after the tape is removed. Reposition onto the field and use `g`. Opposing edge detections must also stop rather than command an ambiguous escape.
7. With motor power off, interrupt one APDS connection. Expect zero PWM and a named line fault/stale marker. Reconnect; use `b` if configuration was lost. A mux isolation error must also stop motors. If the shared-bus fault affected the MPU, use stationary `c` after repair. `g` cannot bypass missing or stale line data.
8. Send `s` during follow and during escape. It must remain stopped until a separate `g` line. Send `stopping` and `gg`; neither may enable motion. A partial `g` without newline must not move the robot. Stop during a new line capture must not produce a valid partial calibration.
9. Restart after a complete calibration. Confirm thresholds load from EEPROM and the program still starts stopped. Changing APDS gain/integration/channel configuration must invalidate the saved calibration. Starting `f` then resetting before calibration finishes must not restore old thresholds.
10. With motor power off, interrupt the MPU. Expect MPU_FAULT and zero PWM. Restore it and use `c` while still. Check gyro correction direction with wheels lifted before enabling slow floor motion.

Only after these checks, test at low speed on the actual line with room to stop. Colour sensing latency, Wire timeouts, sensor geometry, motor stiction, chassis speed, and stopping distance all affect containment. See [BOUNDARY.md](../sketches/MegaBallSearchNoTurn/BOUNDARY.md) and [VALIDATION.md](../VALIDATION.md).
