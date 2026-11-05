# Linux-named bounded search sketch

Open `MegaBallSearchBoundedNoTurn_linux.ino` as an Arduino Mega 2560 sketch.
It is self-contained; no extra header files are needed. Use Serial Monitor at
115200 baud with Newline selected.

This version combines the main robot sketch's four APDS boundary sensors,
EEPROM-saved gyro calibration, and corrected motor wiring with the separate
bounded ball search. It starts with the motors stopped. The APDS sensors use
TCA9548A channels 0–3, and their example side positions are defined by
`boundarycfg::SIDES` near the top of the `.ino` file. Edit those positions to
match the physical robot before enabling movement.

## Setup

1. Lift the wheels and send `f` followed by Enter while keeping the robot still
   for gyro calibration and saving. When the gyro is saved, a **3-second FIELD
   preparation window** begins. Move all four APDS sensors over the normal
   green playing surface at their mounted height. Hold them still once the
   sketch prints `LINE: collecting FIELD now`.
2. After all four `FIELD sensor` results appear, send `1`, `2`, `3`, or `4`
   for the sensor you want to capture over white. Each command provides a
   **3-second WHITE preparation window** to reposition that sensor. Hold it
   still when `LINE: collecting WHITE now` appears. Wait for its `WHITE sensor`
   result before starting the next one. Wait for `LINE calibration saved` after
   all four sensors have been captured.
3. Put every sensor back over the field and send `g` followed by Enter.
   A saved gyro offset and line calibration can reload on later boots, but the
   robot still starts stopped and requires `g`.

The motor inputs are FL 8/6, FR 9/7, BL 2/3, BR 10/11. The back-left motor
polarity is inverted in software. Mega I2C uses SDA 20 and SCL 21.
During preparation, capture, manual stop, and boundary faults, both MDD3A
inputs for each motor are held LOW. The MDD3A datasheet calls LOW/LOW **brake**;
this sketch already brakes when it sets wheel PWM to zero.

## Ball search and boundary priority

When a valid IR ball signal disappears, the robot stops for 200 ms and then
makes a timed sideways/forward raster while holding the MPU heading. The sweep
starts toward the last-seen ball side, lasts at most 1.95 s, and commands no
blind reverse or deliberate turn. A valid ball signal immediately returns to
`FOLLOW`. If the sweep finishes without a ball, `SEARCH_DONE` stops movement
but continues IR polling. A later ball sighting resumes following; `g` starts
a fresh search if the ball is still unseen.

White-line detection takes priority over both following and search. A line
escape or sensor fault cancels the current search and prevents another blind
sweep at that boundary. A front-line escape can command a short, sensor-triggered
backward retreat. Check each sensor's mapped side and the resulting retreat
direction with the wheels lifted before trying the robot on the floor.

The MPU and all four boundary sensors must be healthy before movement.
Calibration and saving stop the motors. An MPU fault requires stationary `c`
recalibration. An IR read error stops motion and retries the read. The field
capture and line threshold procedure otherwise matches the main sketch's
`sketches/MegaBallSearchNoTurn/BOUNDARY.md`.

The sketch compiles for Arduino Mega 2560. Its physical behavior, stopping
distance, and boundary containment have not been tested on the robot.
