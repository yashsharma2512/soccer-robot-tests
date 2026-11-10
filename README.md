# Soccer robot firmware

Choose the sketch for your controller:

- [ESP32 OG test](OG%20test/ESP32OmniSoccer_HiTechnicIR_linux/ESP32OmniSoccer_HiTechnicIR_linux.ino): open its folder in Arduino IDE and select **ESP32 Dev Module**.
- [Arduino Mega OG test](OG%20test%20Mega/README.md): open `MegaOGTest/MegaOGTest.ino` and select **Arduino Mega or Mega 2560**. This version has no Wi-Fi, dashboard, or OTA.

Both versions need the **Adafruit NeoPixel** library. Wire and each board's storage library come with its Arduino core.

The robot starts playing automatically after startup calibration. Opening the dashboard is not required. If you save motor directions or run a manual motor test, select **AUTO** in the dashboard to resume immediately. A reboot starts in AUTO using the saved motor directions.

## ESP32 wiring

| Wheel | PWM input A | PWM input B | Default direction |
| --- | ---: | ---: | --- |
| Front left (FL) | GPIO25 | GPIO26 | Normal |
| Front right (FR) | GPIO16 | GPIO17 | Normal |
| Back left (BL) | GPIO32 | GPIO33 | Reversed |
| Back right (BR) | GPIO18 | GPIO19 | Normal |

The HiTechnic IR Seeker V2 uses I²C address `0x08` in AC mode. ESP32 SDA is GPIO21 and SCL is GPIO22. The sketch also uses an MPU6050 at `0x68`, a TCA9548A at `0x70` for the four APDS line sensors, and a 16-LED NeoPixel strip on GPIO15. [Mega pin map and Serial commands](OG%20test%20Mega/README.md).

## Dashboard

The ESP32 sketch creates the `SoccerMagnumOpus` Wi-Fi access point. Use the credentials configured near the top of the sketch and open the access point IP shown on Serial Monitor at 115200 baud. The dashboard shows HiTechnic IR readings, manual controls, individual motor tests, and saved motor directions.

When the ball is lost, the robot searches with sideways sweeps and forward steps while keeping its heading. Line escape can override the search.

Earlier sketches and tests remain available in this repository's Git history.
