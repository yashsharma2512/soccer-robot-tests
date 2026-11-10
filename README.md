# Soccer robot firmware

The current program is [ESP32OmniSoccer_HiTechnicIR_linux.ino](OG%20test/ESP32OmniSoccer_HiTechnicIR_linux/ESP32OmniSoccer_HiTechnicIR_linux.ino). Open its folder in Arduino IDE and select **ESP32 Dev Module**. Install the **Adafruit NeoPixel** library; the other included libraries come with the ESP32 Arduino core.

The robot starts playing automatically after startup calibration. Opening the dashboard is not required. If you save motor directions or run a manual motor test, select **AUTO** in the dashboard to resume immediately. A reboot starts in AUTO using the saved motor directions.

## Wiring used by this sketch

| Wheel | PWM input A | PWM input B | Default direction |
| --- | ---: | ---: | --- |
| Front left (FL) | GPIO25 | GPIO26 | Normal |
| Front right (FR) | GPIO16 | GPIO17 | Normal |
| Back left (BL) | GPIO32 | GPIO33 | Reversed |
| Back right (BR) | GPIO18 | GPIO19 | Normal |

The HiTechnic IR Seeker V2 uses I²C address `0x08` in AC mode. ESP32 SDA is GPIO21 and SCL is GPIO22. The sketch also uses an MPU6050 at `0x68`, a TCA9548A at `0x70` for the four APDS line sensors, and a 16-LED NeoPixel strip on GPIO15.

## Dashboard

The sketch creates the `SoccerMagnumOpus` Wi-Fi access point. Use the credentials configured near the top of the sketch and open the access point IP shown on Serial Monitor at 115200 baud. The dashboard shows sensor readings and lets you control the robot, test one motor at a time, and save each motor's Normal/Reversed direction to ESP32 flash.

When the ball is lost, the robot searches with sideways sweeps and forward steps while keeping its heading. Line escape can override the search.

Earlier sketches and tests remain available in this repository's Git history.
