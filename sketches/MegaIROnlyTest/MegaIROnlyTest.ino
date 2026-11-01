// Arduino Mega 2560: HiTechnic IR Seeker V2 ONLY.
// SDA = 20, SCL = 21. Serial Monitor = 115200 baud.
// Motor outputs are held LOW. MPU6050 is not initialized or used.
#include <Wire.h>

const uint8_t IR_ADDRESS = 0x08;
const uint8_t MOTOR_PINS[8] = {10, 11, 4, 5, 8, 9, 3, 2};

void setup() {
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000); // Same default speed as your working library sketch.
  Wire.setWireTimeout(25000, true);
  delay(200);
  // Keep the sensor's current mode; do not write any configuration register.
  Serial.println(F("IR ONLY: Mega SDA=20 SCL=21. Motor outputs LOW."));
  Serial.println(F("Dir\tStrength\tS1\tS2\tS3\tS4\tS5"));
}

void loop() {
  delay(100); // Same reporting interval as your working baseline.
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(uint8_t(0x49)); // AC direction, followed by five AC strengths.
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.print(F("I2C error "));
    Serial.print(error);
    Serial.println(F(" reading address 0x08"));
    return;
  }

  uint8_t received = Wire.requestFrom(IR_ADDRESS, uint8_t(6));
  if (Wire.getWireTimeoutFlag() || received != 6) {
    Serial.print(F("Read failed: received "));
    Serial.print(received);
    Serial.println(F(" / 6 bytes"));
    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t data[6];
  for (uint8_t i = 0; i < 6; ++i) data[i] = Wire.read();

  // Same derived Strength calculation as your original library.
  uint16_t strength = 0;
  uint8_t direction = data[0];
  if (direction >= 1 && direction <= 9) {
    uint8_t index = direction / 2 + 1;
    if (direction & 1) strength = data[index];
    else strength = (uint16_t(data[index - 1]) + data[index]) / 2;
  }

  Serial.print(direction);
  Serial.print('\t');
  Serial.print(strength);
  for (uint8_t i = 1; i < 6; ++i) {
    Serial.print('\t');
    Serial.print(data[i]);
  }
  if (direction > 9) Serial.print(F("  INVALID DIRECTION BYTE"));
  Serial.println();
}
