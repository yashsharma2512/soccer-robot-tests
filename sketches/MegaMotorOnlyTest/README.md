# Mega motor-only test

Open `MegaMotorOnlyTest.ino`, select Mega 2560 / ATmega2560, and use Serial Monitor at **115200 baud, Newline**. This standalone sketch uses no IR or MPU code.

Start with wheels lifted. Outputs start LOW. Each movement runs at PWM 80 for 800 ms and then stops automatically.

| Motor | Input 1 | Input 2 | Positive | Negative |
| --- | --- | --- | --- | --- |
| FL: front left | D10 | D11 | `1` | `5` |
| FR: front right | D4 | D5 | `2` | `6` |
| BL: back left | D8 | D9 | `3` | `7` |
| BR: back right | D3 | D2 | `4` | `8` |

Test `1` through `8` first. Correct the wiring or pin map if the wrong wheel moves. Change only that wheel's named `*_SIGN` from `1` to `-1` if its direction needs reversing. Positive output applies PWM to input 1 and holds input 2 LOW before any sign inversion.

After checking individual wheels:

- `w` / `b`: forward / backward.
- `a` / `d`: strafe left / right.
- `q` / `e`: rotate left / right.
- `s`, `x`, `0`, or space: stop immediately when received.
- `?`: print help while stopped.

Send one command per line. An unrecognized or multi-character command leaves the robot stopped. A stop character also discards the rest of its line.

Use the common-ground and motor-power connections in the [main README](../../README.md). Stopping sets both control inputs LOW; actual stopping distance depends on the chassis and driver.
