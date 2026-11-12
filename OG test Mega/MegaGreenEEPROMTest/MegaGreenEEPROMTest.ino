// Arduino Mega 2560. Serial Monitor at 115200 baud.
// g captures green and saves it. p reads the saved record. Reset reloads it.
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Wire.h>
#include <stddef.h>
#include <string.h>

#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega or Mega 2560 with the ATmega2560 processor.
#endif

constexpr uint8_t muxAddress = 0x70;
constexpr uint8_t apdsAddress = 0x39;
constexpr uint8_t channels[] = {3, 2, 1, 0};
const char *const positions[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
constexpr uint8_t motorPins[] = {6, 7, 4, 5, 11, 10, 2, 3};
constexpr uint8_t integrationTime = 0xFC;
constexpr uint8_t gain = 0x01;  // 4x, matching MegaOGTest.
constexpr uint8_t lightBrightness = 255;  // Set to 0 for an unlit setup.
constexpr uint8_t samplesPerSensor = 4;
Adafruit_NeoPixel groundLight(16, 15, NEO_GRB + NEO_KHZ800);

struct GreenReading {
  float red;
  float green;
  float blue;
  float clear;
};

struct GreenRecord {
  GreenReading readings[4];
  uint8_t channelMap[4];
  uint8_t version;
  uint8_t atime;
  uint8_t alsGain;
  uint8_t brightness;
  uint16_t checksum;
};

// Bytes 128–202. The OG motor direction settings use bytes 0–1.
constexpr int eepromBase = 128;
constexpr uint8_t committedMarker = 0xA7;
constexpr uint8_t recordVersion = 1;
static_assert(sizeof(GreenRecord) == 74, "Unexpected green EEPROM record size");

uint16_t checksum(const GreenRecord &record) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < offsetof(GreenRecord, checksum); ++i) {
    crc ^= uint16_t(bytes[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = uint16_t((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool loadGreen(GreenRecord &record) {
  if (EEPROM.read(eepromBase) != committedMarker) return false;
  EEPROM.get(eepromBase + 1, record);
  return record.version == recordVersion &&
         record.atime == integrationTime &&
         record.alsGain == gain &&
         record.brightness == lightBrightness &&
         memcmp(record.channelMap, channels, sizeof(channels)) == 0 &&
         record.checksum == checksum(record);
}

bool saveGreen(GreenRecord &record) {
  record.version = recordVersion;
  record.atime = integrationTime;
  record.alsGain = gain;
  record.brightness = lightBrightness;
  memcpy(record.channelMap, channels, sizeof(channels));
  record.checksum = checksum(record);

  GreenRecord readback;
  if (loadGreen(readback) && memcmp(&record, &readback, sizeof(record)) == 0) {
    return true;
  }

  EEPROM.update(eepromBase, 0);
  EEPROM.put(eepromBase + 1, record);
  EEPROM.get(eepromBase + 1, readback);
  if (memcmp(&record, &readback, sizeof(record)) != 0) return false;
  EEPROM.update(eepromBase, committedMarker);
  return loadGreen(readback);
}

bool selectMux(uint8_t mask) {
  Wire.beginTransmission(muxAddress);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

bool writeApds(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(apdsAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readApds(uint8_t reg, uint8_t *bytes, uint8_t count) {
  Wire.beginTransmission(apdsAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(apdsAddress, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) {
    bytes[i] = uint8_t(Wire.read());
  }
  return true;
}

bool readColorWord(uint8_t reg, uint16_t &value) {
  uint8_t bytes[2];
  if (!readApds(reg, bytes, sizeof(bytes))) return false;
  value = uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8);
  return true;
}

bool captureSensor(uint8_t channel, GreenReading &reading) {
  if (!selectMux(uint8_t(1U << channel))) return false;
  delayMicroseconds(250);
  uint8_t id;
  if (!readApds(0x92, &id, 1)) return false;
  if (id != 0x9E && id != 0xAB && id != 0x9C) return false;

  // ENABLE off, configure ATIME and gain, then enable the colour engine.
  if (!writeApds(0x80, 0) || !writeApds(0x81, integrationTime) ||
      !writeApds(0x8F, gain) || !writeApds(0x80, 0x03)) return false;

  uint8_t accepted = 0;
  for (uint8_t sample = 0; sample < samplesPerSensor; ++sample) {
    delay(15);  // Allow a new conversion between readings.
    uint8_t status;
    uint16_t clear, red, green, blue;
    if (!readApds(0x93, &status, 1) || (status & 0x81) != 0x01 ||
        !readColorWord(0x94, clear) || !readColorWord(0x96, red) ||
        !readColorWord(0x98, green) || !readColorWord(0x9A, blue)) continue;

    const float total = float(red) + green + blue;
    if (clear == 0 || total == 0) continue;
    reading.red += red / total;
    reading.green += green / total;
    reading.blue += blue / total;
    reading.clear += clear;
    ++accepted;
  }
  if (accepted < 2) return false;
  reading.red /= accepted;
  reading.green /= accepted;
  reading.blue /= accepted;
  reading.clear /= accepted;
  return true;
}

void printGreen(const GreenRecord &record) {
  for (uint8_t i = 0; i < sizeof(channels); ++i) {
    const GreenReading &reading = record.readings[i];
    Serial.print(positions[i]);
    Serial.print(F(" CH"));
    Serial.print(channels[i]);
    Serial.print(F(" R="));
    Serial.print(reading.red, 4);
    Serial.print(F(" G="));
    Serial.print(reading.green, 4);
    Serial.print(F(" B="));
    Serial.print(reading.blue, 4);
    Serial.print(F(" Clear="));
    Serial.println(reading.clear, 1);
  }
}

void showSavedGreen() {
  GreenRecord record;
  if (loadGreen(record)) {
    Serial.println(F("LOADED GREEN FROM EEPROM"));
    printGreen(record);
  } else {
    Serial.println(F("No valid saved GREEN calibration. Place sensors over green and send g."));
  }
}

void captureAndSaveGreen() {
  Serial.println(F("Capturing GREEN on all four channels. Keep the robot over green."));
  GreenRecord record = {};
  bool allReady = true;
  for (uint8_t i = 0; i < sizeof(channels); ++i) {
    if (captureSensor(channels[i], record.readings[i])) continue;
    allReady = false;
    Serial.print(F("Capture failed on "));
    Serial.print(positions[i]);
    Serial.print(F(" CH"));
    Serial.println(channels[i]);
  }
  if (!selectMux(0)) allReady = false;
  if (!allReady) {
    Serial.println(F("GREEN capture failed. The previous EEPROM record was kept."));
    return;
  }
  if (!saveGreen(record)) {
    Serial.println(F("EEPROM verification failed."));
    return;
  }
  Serial.println(F("SAVED GREEN TO EEPROM (readback verified)"));
  printGreen(record);
}

void setup() {
  for (uint8_t pin : motorPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin();  // Mega SDA D20, SCL D21.
  Wire.setWireTimeout(25000, true);
  groundLight.begin();
  groundLight.setBrightness(lightBrightness);
  groundLight.fill(groundLight.Color(255, 255, 255));
  groundLight.show();
  delay(120);
  selectMux(0);
  Serial.println(F("Mega APDS GREEN EEPROM test. g=capture/save, p=read saved values."));
  showSavedGreen();
}

void loop() {
  if (!Serial.available()) return;
  const char command = char(Serial.read());
  if (command == 'g' || command == 'G') captureAndSaveGreen();
  if (command == 'p' || command == 'P') showSavedGreen();
}
