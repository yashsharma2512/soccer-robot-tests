// Mega 2560 + one TCA9548A + one APDS9960 per occupied channel.
// Wire only. Motor outputs stay LOW. Serial Monitor: 115200 baud.
#include <Arduino.h>
#include <Wire.h>

const uint8_t TCA_ADDRESS = 0x70; // Change if A0/A1/A2 select 0x71..0x77.
const uint8_t APDS_ADDRESS = 0x39;
const uint8_t ATIME_VALUE = 219;  // Approximately 103 ms integration.
const uint8_t GAIN_VALUE = 1;     // 0=1x, 1=4x, 2=16x, 3=64x.
const uint8_t MOTOR_PINS[] = {8, 6, 9, 7, 2, 3, 10, 11};
bool present[8] = {};
bool muxReady = false;
uint32_t lastReport = 0;

uint8_t ping(uint8_t address) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return Wire.getWireTimeoutFlag() ? 5 : error;
}

bool setMux(uint8_t mask) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(mask);
  if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) return false;
  uint8_t received = Wire.requestFrom(TCA_ADDRESS, uint8_t(1));
  if (received != 1 || Wire.getWireTimeoutFlag()) {
    while (Wire.available()) Wire.read();
    return false;
  }
  return uint8_t(Wire.read()) == mask;
}

bool readAPDS(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(APDS_ADDRESS, count);
  if (received != count || Wire.getWireTimeoutFlag()) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeAPDS(uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
}

bool initAPDS() {
  // Disable gesture/proximity. Enable only power and the colour engine.
  if (!writeAPDS(0x80, 0) || !writeAPDS(0xAB, 0) ||
      !writeAPDS(0x81, ATIME_VALUE) || !writeAPDS(0x8F, GAIN_VALUE) ||
      !writeAPDS(0x8D, 0x60) || !writeAPDS(0x90, 0x41) ||
      !writeAPDS(0x9F, 0) || !writeAPDS(0x80, 1)) return false;
  delay(10);
  if (!writeAPDS(0x80, 3)) return false;
  uint8_t enabled = 0, integration = 0, gain = 0;
  return readAPDS(0x80, &enabled, 1) && enabled == 3 &&
         readAPDS(0x81, &integration, 1) && integration == ATIME_VALUE &&
         readAPDS(0x8F, &gain, 1) && gain == GAIN_VALUE;
}

void muxFault() {
  muxReady = false;
  Serial.println(F("MUX ERROR: check power/wiring/reset; send r to rescan."));
}

void scanSensors() {
  muxReady = false;
  for (uint8_t ch = 0; ch < 8; ++ch) present[ch] = false;
  Serial.print(F("Testing TCA at 0x")); Serial.println(TCA_ADDRESS, HEX);
  if (!setMux(0)) { muxFault(); return; }
  uint8_t upstream = ping(APDS_ADDRESS);
  if (upstream == 0) {
    Serial.println(F("Device 0x39 found with all channels OFF."));
    Serial.println(F("Put APDS sensors on separate mux channels, not upstream SDA/SCL."));
    return;
  }
  if (upstream != 2) { muxFault(); return; }

  uint8_t found = 0;
  for (uint8_t ch = 0; ch < 8; ++ch) {
    if (!setMux(uint8_t(1U << ch))) { muxFault(); return; }
    Serial.print(F("CH")); Serial.print(ch); Serial.print(F(": "));
    uint8_t error = ping(APDS_ADDRESS);
    if (error == 2) { Serial.println(F("empty (no 0x39)")); continue; }
    if (error != 0) { Serial.println(F("I2C bus error")); continue; }
    uint8_t id = 0;
    if (!readAPDS(0x92, &id, 1)) { Serial.println(F("ID read failed")); continue; }
    Serial.print(F("ID=0x")); Serial.print(id, HEX);
    // 0x9E modules are also supported by the ESPHome APDS9960 driver.
    if (id != 0xAB && id != 0x9C && id != 0x9E) {
      Serial.println(F(" unsupported ID")); continue;
    }
    if (!initAPDS()) { Serial.println(F(" configuration failed")); continue; }
    present[ch] = true;
    ++found;
    Serial.println(F(" READY"));
  }
  if (!setMux(0)) { muxFault(); return; }
  muxReady = true;
  Serial.print(F("Found ")); Serial.print(found);
  Serial.println(F(" APDS channel(s). Move each sensor over field, then white tape."));
  Serial.println(F("CH\tClear\tRed\tGreen\tBlue\tStatus"));
  delay(150);
}

uint16_t littleEndian(const uint8_t *data) {
  return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

void setup() {
  for (uint8_t i = 0; i < sizeof(MOTOR_PINS); ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(25000, true);
  Serial.println(F("MEGA APDS/TCA TEST: SDA=20 SCL=21. Motors LOW. r=rescan"));
  scanSensors();
}

void loop() {
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    char command = Serial.read();
    if (command == 'r' || command == 'R') scanSensors();
  }
  if (!muxReady || uint32_t(millis() - lastReport) < 500) return;
  lastReport = millis();
  for (uint8_t ch = 0; ch < 8; ++ch) {
    if (!present[ch]) continue;
    if (!setMux(uint8_t(1U << ch))) { muxFault(); return; }
    Serial.print(ch); Serial.print('\t');
    uint8_t status = 0, data[8];
    if (!readAPDS(0x93, &status, 1)) {
      Serial.println(F("I2C READ FAILED")); continue;
    }
    if (!(status & 1)) { Serial.println(F("NOT READY")); continue; }
    if (!readAPDS(0x94, data, 8)) {
      Serial.println(F("RGBC READ FAILED")); continue;
    }
    uint16_t clear = littleEndian(data), red = littleEndian(data + 2);
    uint16_t green = littleEndian(data + 4), blue = littleEndian(data + 6);
    Serial.print(clear); Serial.print('\t');
    Serial.print(red); Serial.print('\t');
    Serial.print(green); Serial.print('\t');
    Serial.print(blue); Serial.print('\t');
    bool saturated = (status & 0x80) || clear == 65535 || red == 65535 ||
                     green == 65535 || blue == 65535;
    Serial.println(saturated ? F("SATURATED: reduce gain/light") : F("OK"));
    if (status & 0x80) {
      Wire.clearWireTimeoutFlag();
      Wire.beginTransmission(APDS_ADDRESS); Wire.write(uint8_t(0xE6));
      if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag())
        Serial.println(F("Saturation flag clear failed; send r."));
    }
  }
  if (!setMux(0)) muxFault();
}
