# TCS34725 soccer for Arduino Mega

Open [MegaTCSSoccer.ino](MegaTCSSoccer/MegaTCSSoccer.ino) and select **Arduino Mega or Mega 2560** with **ATmega2560**. Install **Adafruit NeoPixel** in the Arduino IDE. The sketch uses Wire to read the TCS34725 and EEPROM for the motor direction settings.

## Wiring

Connect the TCA9548A at `0x70` to **D20 (SDA)** and **D21 (SCL)**. Connect one **TCS34725 at `0x29`** to each channel.

| Sensor | TCA9548A channel |
| --- | --- |
| Front | CH0 |
| Left | CH1 |
| Back | CH3 |
| Right | CH2 |

Follow the power and logic-level specifications for your breakout. The TCS34725 chip uses a 2.7–3.6 V supply. A suitable breakout or correctly configured TCA9548A level translation connects the sensor to the Mega I²C bus. [TCS34725 specifications](https://ams-osram.com/products/sensor-solutions/ambient-light-color-spectral-proximity-sensors/ams-tcs34725-color-sensor) · [TCA9548A datasheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf)

| Motor | Input A | Input B |
| --- | --- | --- |
| Front left | D9 | D7 |
| Front right | D5 | D6 |
| Back left | D11 | D13 |
| Back right | D10 | D12 |

Keep motor power off during upload and reset. The Mega bootloader can pulse D13. [Arduino bootloader source](https://github.com/arduino/ArduinoCore-avr/blob/master/bootloaders/stk500v2/stk500boot.c)

HiTechnic IR (`0x08`) and MPU (`0x68`) connect to the main I²C rails. The ground NeoPixel strip uses **D15**.

## Startup and sensor check

1. Place all four sensors over green and keep the robot still at startup. The sketch captures green and calibrates the gyro automatically.
2. AUTO begins when all four sensors have valid green references and recent readings. It pauses the drive if any of the four readings drops out.
3. For a stationary check, send `s` through Serial Monitor at **115200 baud** with **New Line**. Send `l` to read all four sensors. Compare `GREEN` over turf and `LINE` over the actual tape. The output also shows `C/green` (clear brightness relative to calibration), `distance` (normalized color distance), and `SATURATED` when a channel clips. Send `g` to recalibrate over green, then `a` to resume AUTO.

The sensors integrate in parallel at **9.6 ms** integration time, plus sensor overhead. The sketch schedules mux reads one at a time and reads status and RGBC in a single I²C transaction. Measure detection at the mounted height and intended speed. [TCS3472 timing and register reference](https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf)

The initial gain is **1×**. If the readings clip at the top of the range, adjust `TCS_LIGHT_BRIGHTNESS` or your breakout’s illumination. If the readings are too dim, `TCS_GAIN = 0x01` selects **4×**. Recalibrate over green after changing the lighting, height or gain.

The green color tolerance is now **0.05** (previously 0.08). A reading at least **1.50×** as bright as its green calibration is also treated as a line. Check `l` over several green and white spots with the motors stopped: green should have `distance <= 0.05` and `C/green < 1.50`; white should exceed one of these limits. If ordinary green triggers `LINE`, tune these constants from your readings instead of increasing sensor gain blindly.

Other Serial commands carry over from the Mega OG sketch. [Mega motor and gyro commands](../OG%20test%20Mega/README.md#optional-serial-commands)
