#include <Arduino.h>
#include <Wire.h>

const uint8_t IR_ADDRESS = 0x08;

// Latest motor pin map: FL, FR, BL, BR.
// These outputs remain LOW throughout the test.
const uint8_t MOTOR_PINS[] = {8, 6, 9, 7, 2, 3, 10, 11};

void setup() {
  for (uint8_t i = 0; i < sizeof(MOTOR_PINS); ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);

  delay(200);

  Serial.println(F("MEGA IR ONLY TEST. SDA=20 SCL=21"));
  Serial.println(F("Dir\tStrength\tS1\tS2\tS3\tS4\tS5"));
}

void loop() {
  delay(100);

  // Read AC direction and five AC signal strengths.
  // Preserve the sensor's existing frequency setting.
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(uint8_t(0x49));

  uint8_t error = Wire.endTransmission();

  if (error != 0 || Wire.getWireTimeoutFlag()) {
    Serial.print(F("IR I2C error: "));
    Serial.println(error);
    return;
  }

  uint8_t received = Wire.requestFrom(IR_ADDRESS, uint8_t(6));

  if (received != 6 || Wire.getWireTimeoutFlag()) {
    Serial.print(F("IR read failed. Bytes received: "));
    Serial.println(received);

    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t data[6];

  for (uint8_t i = 0; i < 6; ++i) {
    data[i] = Wire.read();
  }

  uint8_t direction = data[0];

  if (direction > 9) {
    Serial.print(F("Invalid direction: "));
    Serial.println(direction);
    return;
  }

  // Same derived strength calculation as the baseline library.
  uint16_t strength = 0;

  if (direction >= 1) {
    uint8_t index = direction / 2 + 1;

    if (direction & 1) {
      strength = data[index];
    } else {
      strength =
        (uint16_t(data[index - 1]) + data[index]) / 2;
    }
  }

  Serial.print(direction);
  Serial.print('\t');
  Serial.print(strength);

  for (uint8_t i = 1; i < 6; ++i) {
    Serial.print('\t');
    Serial.print(data[i]);
  }

  Serial.println();
}
