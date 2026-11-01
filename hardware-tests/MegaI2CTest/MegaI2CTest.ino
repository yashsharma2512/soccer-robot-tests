// Arduino Mega 2560 I2C scanner + raw MPU identity test.
// SDA = 20, SCL = 21. Serial Monitor = 115200 baud.
// No MPU library, initialization, calibration, or motor movement.
#include <Wire.h>

const uint8_t FL1 = 8, FL2 = 6;
const uint8_t FR1 = 9, FR2 = 7;
const uint8_t BL1 = 4, BL2 = 2;
const uint8_t BR1 = 3, BR2 = 5;
const uint8_t MOTOR_PINS[8] = {FL1, FL2, FR1, FR2, BL1, BL2, BR1, BR2};

void printHex(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 16) Serial.print('0');
  Serial.print(value, HEX);
}

void printIdentity(uint8_t address) {
  Serial.print(F("Address "));
  printHex(address);
  Serial.print(F(" WHO_AM_I (register 0x75), three reads: "));

  uint8_t values[3] = {};
  bool complete = true;

  for (uint8_t i = 0; i < 3; ++i) {
    if (i) Serial.print(F(", "));
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(address);
    Wire.write(uint8_t(0x75));
    uint8_t error = Wire.endTransmission(false);
    if (error != 0) {
      Serial.print(F("ERR "));
      Serial.print(error);
      complete = false;
      continue;
    }

    uint8_t received = Wire.requestFrom(address, uint8_t(1));
    if (Wire.getWireTimeoutFlag() || received != 1) {
      Serial.print(F("READ FAILED"));
      while (Wire.available()) Wire.read();
      complete = false;
      continue;
    }
    values[i] = Wire.read();
    printHex(values[i]);
    delay(5);
  }

  if (complete) {
    Serial.print(values[0] == values[1] && values[1] == values[2]
                     ? F(" (stable)") : F(" (UNSTABLE)"));
    if (values[0] == values[1] && values[1] == values[2]) {
      if ((values[0] & 0x7E) == 0x68)
        Serial.print(F("; MPU-6050 compatible ID"));
      else if (values[0] == 0x74)
        Serial.print(F("; MPU-6515/6500-family ID"));
    }
  }
  Serial.println();
}

void setup() {
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  delay(200);
  Serial.println(F("MEGA I2C TEST: SDA=20, SCL=21; motor outputs LOW."));
}

void loop() {
  Serial.println(F("\nScanning addresses 0x01 to 0x7E..."));
  uint8_t found = 0;
  bool busError = false;
  bool found68 = false, found69 = false;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      ++found;
      Serial.print(F("Found device at "));
      printHex(address);
      Serial.println();
      if (address == 0x68) found68 = true;
      if (address == 0x69) found69 = true;
    } else if (error != 2) {
      Serial.print(F("Bus error "));
      Serial.print(error);
      Serial.print(F(" at "));
      printHex(address);
      Serial.println();
      busError = true;
      break;
    }
  }

  if (!found && !busError) Serial.println(F("No I2C devices found."));
  if (!busError) {
    if (found68) printIdentity(0x68);
    if (found69) printIdentity(0x69);
    if (!found68 && !found69) Serial.println(F("No device at MPU addresses 0x68/0x69."));
  }
  Serial.println(F("Scan complete. Repeating in 3 seconds."));
  delay(3000);
}
