# Simple follower verification

Compile the main sketch for `arduino:avr:mega` with Arduino IDE or Arduino CLI. The former search-pattern test was removed with the search implementation; it no longer describes the active control flow.

## Bench regression checklist

These checks require the robot and have **not** been run by the coding agent. Begin with wheels lifted and the sensor facing a working pulsed IR ball.

1. Keep the robot still during MPU calibration. With no ball visible, expect `WAIT_BALL` and PWM `0/0/0/0`.
2. Show the ball in the center, then on either side. Expect `FOLLOW`, a valid direction/strength, and nonzero wheel PWM.
3. Hide the ball for at least five seconds, exceeding the former search duration. Expect `WAIT_BALL`, zero PWM, and continuing telemetry. Show it again without serial input; expect automatic `FOLLOW`. Repeat several times.
4. With motor power off, temporarily disconnect only the IR sensor signal connection using the existing electrical setup. Expect `IR_FAULT` and zero PWM. Reconnect it; if the MPU and shared bus remain healthy, IR polling should recover automatically. A shared-bus failure that also faults the MPU requires stationary `c` recalibration.
5. Send `s` while the ball is visible. The robot must remain `STOPPED`, including after moving/hiding/showing the ball. Send `g` followed by Enter to resume. A partial `g` without newline must not resume movement.
6. Send `stopping` followed by Enter. It must remain stopped. Send an invalid multi-character line such as `gg`; it must remain stopped until a separate `g` line.
7. With motor power off, interrupt the MPU connection. Expect `MPU_FAULT` and zero PWM. Restore it, keep stationary, and use `c`. Confirm the named error clears and calibration finishes before following resumes.
8. Check heading feedback with wheels lifted: a small hand rotation while following should cause correction opposing the displacement. Verify wheel polarity and `GYRO_Z_SIGN` before floor testing.

Successful compilation does not establish sensor visibility, electrical reliability, gyro accuracy, or real chassis movement. See [VALIDATION.md](../VALIDATION.md) for measured software build results.
