# Classic ESP32-WROOM-32 wiring

Use GPIO numbers printed on the ESP32 board. This map is for classic WROOM-32;
it is not the Mega pin map. Both sketches in this folder use these connections.

## Two MDD3A drivers and four motors

The following connections go to the drivers' **control input headers**. Each
motor connects to the corresponding motor output terminal pair.

| Motor | Driver/channel | Input A GPIO | Input B GPIO | Software sign |
| --- | --- | ---: | ---: | ---: |
| FL | Driver 1 / M1 | 25 | 26 | +1 |
| FR | Driver 1 / M2 | 16 | 17 | +1 |
| BL | Driver 2 / M1 | 32 | 33 | -1 |
| BR | Driver 2 / M2 | 18 | 19 | +1 |

The MDD3A accepts 3.3V control signals directly. The code uses 16 kHz PWM,
within its specified PWM range. These inputs use PWM/PWM, not PWM/DIR.
For a stopped motor both inputs are LOW.
[Cytron MDD3A specifications](https://www.cytron.io/cytron/p-3amp-4v-16v-dc-motor-driver-2-channels).

Add a 10 kOhm pull-down from each of the eight driver control inputs to GND
to keep them LOW while the ESP32 resets or is disconnected.

FR now uses GPIO16/GPIO17. These are available on the confirmed WROOM-32 board;
they are reserved for PSRAM on WROVER boards. Disconnect the old FR wires from
GPIO27/GPIO14 and reconnect the driver's two FR control inputs to GPIO16/GPIO17.
Keep motor power OFF during uploading. Firmware cannot hold outputs LOW while
the chip is reset or in its bootloader; pin changes alone do not guarantee that.
The full sketch can also auto-run after boot when calibration is valid; set
`espconfig::AUTO_RUN_WHEN_READY=false` if you want `r` before it starts.
[Espressif GPIO16/17 board restrictions](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html).

## Main I2C rails: IR and MPU stay here

| Main-rail connection | ESP32 / destination |
| --- | --- |
| SDA | GPIO21; MPU SDA; TCA SDA; HiTechnic SDA connection described below |
| SCL | GPIO22; MPU SCL; TCA SCL; HiTechnic SCL connection described below |
| GND | ESP32 GND, all sensor grounds, both driver grounds, battery negative |

The ESP32-facing SDA/SCL rails and their pull-ups must be **3.3V**. Do not
connect a 5V-pulled-up I2C rail directly to ESP32 GPIO. Keep the wires short
and route them away from motor/battery wires.
[ESP32 electrical limits](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf).

**HiTechnic IR Seeker V2:** retain the connector power wires and supply that
already worked on the Mega. Reuse the known SDA/SCL wires, not an assumed
RJ12 cable colour order. The IR address remains 0x08 on the main bus.
If its SDA/SCL pull up to 5V, connect a bidirectional I2C level shifter:

| Level-shifter terminal | Connection |
| --- | --- |
| LV | ESP32 3V3 |
| HV | The IR side's regulated 5V logic supply |
| GND | Common GND |
| LV1 | Main SDA rail / GPIO21 |
| HV1 | HiTechnic SDA |
| LV2 | Main SCL rail / GPIO22 |
| HV2 | HiTechnic SCL |

This keeps IR on the same logical I2C bus; no TCA channel is used for IR.
An ordinary resistor divider is unsuitable for bidirectional SDA. If the IR
data lines are already verified to be 3.3V, the level shifter is unnecessary.
Do not change an already working IR power arrangement merely to power it
from the ESP32's 3V3 pin.

**MPU breakout:** SDA -> main SDA, SCL -> main SCL, GND -> common GND. Retain
its appropriate regulated supply connection. A regulator-equipped GY-521 can
use its 5V VCC/VIN input, while the MPU chip and SDA/SCL pull-ups operate at
3.3V; a bare/3.3V-only module must use 3.3V. Verify the breakout's supply and
pull-up arrangement rather than putting 5V on raw MPU pins. This wiring
requires 3.3V on its SDA/SCL connection to the ESP32. Leave AD0 at its existing
known level; the code probes both 0x68 and 0x69. INT is not used.

## TCA9548A and four APDS sensors

| TCA terminal | Connection |
| --- | --- |
| VCC/VIN | ESP32 3V3 |
| GND | Common GND |
| SDA | Main SDA / GPIO21 |
| SCL | Main SCL / GPIO22 |
| A0, A1, A2 | GND, selecting address 0x70 |
| RESET / RST | Hold HIGH at 3.3V; use the breakout's pull-up or a 10 kOhm pull-up |

| APDS example position | TCA SDA terminal | TCA SCL terminal |
| --- | --- | --- |
| Front | SD0 | SC0 |
| Right | SD1 | SC1 |
| Back | SD2 | SC2 |
| Left | SD3 | SC3 |

All four APDS VCC inputs go to 3.3V and GND to common GND. INT is not used.
Use a different channel for each identical 0x39 device. TCA channels 4-7
are unused. The main program's `CHANNELS` and `SIDES` are editable examples;
match them to the actual physical mounting.

Each active I2C segment needs pull-ups to its own correct supply. Many
breakouts include them; check before adding extra parallel resistors.
Typical starting values for a short 100 kHz bus are 4.7 kOhm per SDA/SCL.
The TCA operates at 3.3V here; it does not generate a 3.3V sensor supply.
[TI TCA9548A documentation](https://www.ti.com/product/TCA9548A).

## Power connections

- Battery positive -> both drivers' VB+ terminals.
- Battery negative -> both drivers' VB- terminals and common ground.
- Leave **both driver 5Vo outputs unconnected**. They are rated only 200 mA
  each; do not parallel them or use them to power the complete ESP32/sensor set.
- Initially power the ESP32 through USB. For standalone use, provide a clean
  regulated supply suitable for the board and the whole electronics load.
  Use only one board power input at a time; disconnect USB before feeding
  an external 5V supply to the 5V/VIN input, following the board's markings.
- Never connect the motor battery directly to ESP32 3V3 or GPIO.
- Connect driver power returns directly to the battery/distribution ground;
  use a separate ground branch for the ESP32 and sensors. Keep motor current
  out of thin breadboard/ESP32 ground wiring while maintaining common ground.

[Espressif DevKitC power options](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html).

Use the hardware-test sketch first, with wheels lifted for commanded motor
pulses. Test switching the motor supply while it reports sensor readings.
Software cannot establish that the previous battery-induced bus disturbance
has been corrected; full play requires stable live sensors.
