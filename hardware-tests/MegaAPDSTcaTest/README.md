# Mega APDS9960 / TCA9548A test

Open `MegaAPDSTcaTest.ino` and select **Arduino Mega 2560**. Serial Monitor is **115200 baud**. Install **Adafruit NeoPixel** through Arduino IDE Library Manager; Wire is built in. The test uses **16 RGB NeoPixels on D16** as steady white surface illumination. The four motor channels remain LOW; keep motor power off for this stationary test.

This assumes **one TCA9548A** at address **0x70**. If its address pins select another address, change `TCA_ADDRESS` to `0x71` through `0x77`. The program scans all eight channels automatically, so the occupied channel numbers do not need to be known first.

## Connections

| Connection | Destination |
| --- | --- |
| Mega SDA D20 | TCA upstream SDA |
| Mega SCL D21 | TCA upstream SCL |
| Mega GND | TCA GND and all sensor grounds |
| TCA VCC/VIN | 3.3 V for the level arrangement described below |
| Each APDS VCC | 3.3 V for ordinary 3.3 V APDS breakouts |
| Sensor 0 SDA/SCL | TCA SD0/SC0 |
| Sensor 1 SDA/SCL | TCA SD1/SC1 |
| Additional sensors | One per separate SDn/SCn channel, up to channel 7 |
| APDS INT | Unconnected; the program polls |
| Mega D16 | First NeoPixel DIN through a 330-470 ohm series resistor |
| NeoPixel +5V | Regulated 5 V LED supply |
| NeoPixel GND | LED supply ground and common Mega/sensor ground |

Ordinary APDS9960 boards require **3.3 V power and sensor-side I2C**, unlike the Mega's 5 V bus. The TCA9548A can translate between upstream 5 V and downstream 3.3 V when powered and pulled up appropriately: use TCA VCC 3.3 V, upstream pull-ups to the Mega bus's 5 V reference, and downstream pull-ups to 3.3 V. Check the particular breakout's installed pull-ups and voltage ratings. A suitable bidirectional level shifter before an all-3.3-V TCA/sensor bus is another valid arrangement. Do not put 5 V pull-ups on APDS signal lines.

TCA RESET must remain high during operation. A0/A1/A2 should select the intended address. The normal all-low selection is 0x70. Never connect two APDS sensors to the same channel; they share address 0x39. Do not connect their SDA/SCL directly to the upstream bus.

Sources: [TI TCA9548A voltage translation](https://www.ti.com/product/TCA9548A), [SparkFun APDS voltage requirements](https://learn.sparkfun.com/tutorials/apds-9960-rgb-and-gesture-sensor-hookup-guide/hardware-hookup), [TCA breakout pinout](https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/pinouts).

## White NeoPixel illumination

All 16 pixels are set to equal red, green, and blue values of **32/255**, controlled by `WHITE_LEVEL`. They stay white during scanning and readings. The program sends the LED data once in setup, before starting Wire transactions; it does not animate or change colour when a line is detected. This keeps the light source constant for measuring the field and white tape.

Assumed pixel type: standard **RGB, GRB order, 800 kHz**. For RGBW hardware, set the appropriate RGBW/GRBW type in the constructor instead. Pin 16 is also Mega TX2, so do not use Serial2 for another device on this pin.

Aim the light at the surface under the APDS sensors rather than directly into their lenses. Keep `WHITE_LEVEL`, sensor height, and APDS gain fixed while collecting field/white readings. If readings say `SATURATED`, reduce `WHITE_LEVEL` or gain and repeat the comparison. If both surfaces read very low, check light placement and adjust brightness as needed. These readings do not yet select a line-detection threshold.

Power ordinary 5 V NeoPixels from a regulated supply with adequate current and common ground; keep APDS sensor power/signals at 3.3 V. Do not connect LED power to D16 or the APDS 3.3 V supply. Adafruit's conservative full-white estimate is up to 60 mA per RGB pixel, or about **0.96 A for 16** at full output; the default level here is much lower. Allow supply headroom if brightness will be increased. Fit a 500-1000 uF capacitor across LED power near the strip, with suitable voltage rating. See [NeoPixel power guidance](https://learn.adafruit.com/adafruit-neopixel-uberguide/powering-neopixels) and [data resistor/common-ground guidance](https://learn.adafruit.com/adafruit-neopixel-uberguide/best-practices).

## Test procedure

1. Upload, confirm all 16 NeoPixels light steady white, and check the startup scan. Each connected channel should print its ID followed by `READY`. IDs `0xAB`, `0x9C`, and `0x9E` are accepted. This robot's scan showed `0x9E` on channels 0-3; the earlier diagnostic rejected that ID before initialization. Empty channels are normal. The count of ready channels should match the physical APDS sensors.
2. The live table prints channel number, **Clear, Red, Green, Blue**, and a status every 500 ms. `OK` means a valid, unsaturated data read; it does not mean a white line was detected.
3. With sensors at their actual mounted height, move white tape/paper underneath one sensor at a time. Verify the corresponding channel's readings change. Repeat for every connected sensor to verify channel mapping.
4. For each sensor, record several readings over the actual playing surface and several over the actual white line. Keep the height and illumination the same. These measurements are needed before choosing a line threshold for each channel.
5. Send `r` to rescan/reinitialize after reconnecting a sensor or changing wiring.

The colour engine measures visible light; the built-in APDS IR LED is for proximity/gesture sensing and is not a white illuminator. The NeoPixels supply visible illumination for the field/white comparison. Ensure that light actually reaches the surface viewed by each sensor. [Broadcom sensor description](https://www.broadcom.com/products/optical-sensors/integrated-ambient-light-and-proximity-sensors/apds-9960).

No white/non-white threshold is guessed in this diagnostic. A bright object, lamp, or change of mounting height can change Clear; white-line detection must be checked against the field under the intended lighting.

## Read errors

- `MUX ERROR`: the configured TCA address did not accept/read back the channel selection. Check address, SDA/SCL, power, RESET, and the selected downstream bus. A stuck-low downstream bus may require a hardware TCA reset or power cycle before `r` can work.
- `Device 0x39 found with all channels OFF`: a device with the APDS address is on the upstream bus. Fix the topology before counting sensors; otherwise one upstream APDS could appear falsely on every channel.
- `unsupported ID`: the observed ID is printed; the device is not configured as APDS9960.
- `configuration failed`: register writes/readback did not complete correctly.
- `I2C READ FAILED` / `RGBC READ FAILED`: that channel's transaction failed. Read attempts continue, while mux selection errors stop the scan until `r`.
- `NOT READY`: the ALS valid bit is not set. Occasional occurrences during startup/reinitialization can happen; persistent ones require checking configuration/power.
- `SATURATED`: the saturation flag or full-scale data was observed. Reduce `GAIN_VALUE` or illumination, upload, and repeat the field/white comparison. Gain codes are 0=1x, 1=4x, 2=16x, 3=64x.

## Configuration and validation

Each selected APDS is configured for colour/ambient light only: PON+AEN, about 103 ms integration (`ATIME=219`), and 4x gain (`GAIN_VALUE=1`). Proximity and gesture are disabled. Reads check AVALID and decode the eight RGBC bytes from 0x94 in low/high pairs. The TCA control mask is read back after every selection, with at most one channel enabled. Channels are disabled after each reporting cycle.

Register references: [manufacturer datasheet](https://cdn-learn.adafruit.com/assets/assets/000/045/848/original/Avago-APDS-9960-datasheet.pdf), [SparkFun APDS register definitions and original supported IDs](https://github.com/sparkfun/SparkFun_APDS-9960_Sensor_Arduino_Library/blob/master/src/SparkFun_APDS9960.h). Support for `0x9E` is based on the [merged ESPHome driver change](https://github.com/esphome/esphome/pull/9392), whose author reported successful operation after accepting this ID. Register configuration/readback, AVALID, saturation, and mux-selection checks remain enabled. Accepting an ID does not establish white-line contrast or full sensor health; verify the live colour readings on each channel.

Recompiled with `0x9E` ID support and NeoPixel illumination on **5 November 2026** for `arduino:avr:mega` with Arduino AVR Boards 1.8.6, AVR GCC 7.3.0-atmel3.6.1-arduino7, and **Adafruit NeoPixel 1.15.2**: **9,060 bytes flash**, **457 bytes static RAM**. The 16 RGB pixels also use a 48-byte dynamically allocated pixel buffer. No firmware was uploaded and no physical LED/sensor/white-line readings were taken by the coding agent. These are bench diagnostic settings; the test is not connected to the driving logic.
