# Arduino Mega 2560 port of OG test

Open [MegaOGTest/MegaOGTest.ino](MegaOGTest/MegaOGTest.ino) in Arduino IDE and select **Arduino Mega or Mega 2560**. Install the **Adafruit NeoPixel** library. Wire and EEPROM are included with the Arduino AVR core.

This port keeps the ESP32 sketch's HiTechnic IR ball following, sideways and forward ball search, MPU heading hold, APDS green calibration and white-line escape. It has no Wi-Fi, dashboard, or OTA. It starts in AUTO after startup calibration without a Serial Monitor connection. Keep the robot still over green during startup calibration.

## Pins

| Motor | Input A | Input B | Default direction |
| --- | ---: | ---: | --- |
| Front left | D6 | D7 | Normal |
| Front right | D4 | D5 | Normal |
| Back left | D11 | D10 | Reversed |
| Back right | D2 | D3 | Normal |

Mega I²C is SDA D20 and SCL D21. The HiTechnic IR Seeker V2 is at `0x08` (AC mode), the MPU at `0x68`, and the TCA9548A at `0x70`. APDS channels are **front 3, left 2, back 1, right 0**. The 16-LED NeoPixel strip uses D15 in this port; move that connection from the ESP32 pin if needed.

## Optional Serial commands

Set Serial Monitor to **115200 baud** and **New Line**. The robot does not require Serial to play.

| Command | Action |
| --- | --- |
| `a` | AUTO play |
| `s` | Stop and remain MANUAL |
| `g` | Stop and recalibrate all four sensors over green; send `a` to play |
| `c` | Stop and recalibrate the gyro while still; send `a` to play |
| `p` | Print the four saved motor directions |
| `d1`–`d4` | Reverse FL, FR, BL, or BR and save to Mega EEPROM |
| `t1f`–`t4f` | Test one motor forward at PWM 100 for 700 ms |
| `t1r`–`t4r` | Test one motor reverse at PWM 100 for 700 ms |

The direction settings are saved on the Mega itself. ESP32 flash settings do not transfer to Mega EEPROM. After a motor test or direction change, the robot stays stopped in MANUAL until `a` is sent or the Mega is restarted.
