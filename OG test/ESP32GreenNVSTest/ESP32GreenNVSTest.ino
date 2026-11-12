// Classic ESP32-WROOM-32. Serial Monitor at 115200 baud.
// Connect to SoccerGreenTest for the dashboard. Serial g saves and p reloads.
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <stddef.h>
#include <string.h>

#if !defined(ARDUINO_ARCH_ESP32) || !defined(CONFIG_IDF_TARGET_ESP32)
#error Select ESP32 Dev Module for a classic ESP32-WROOM-32.
#endif

constexpr uint8_t muxAddress = 0x70;
constexpr uint8_t apdsAddress = 0x39;
constexpr uint8_t channels[] = {3, 2, 1, 0};
const char *const positions[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
constexpr uint8_t motorPins[] = {25, 26, 16, 17, 32, 33, 18, 19};
constexpr uint8_t integrationTime = 0xFC;
constexpr uint8_t gain = 0x01;  // 4x, matching the ESP32 OG sketch.
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
  uint16_t signature;
  uint16_t checksum;
};

constexpr char storageNamespace[] = "og-green-test";
constexpr char storageKey[] = "green";
constexpr uint16_t recordSignature = 0x4752;
constexpr uint8_t recordVersion = 1;
static_assert(sizeof(GreenRecord) == 76, "Unexpected green NVS record size");
Preferences greenStorage;
bool storageReady = false;

enum class CaptureResult : uint8_t { None, Saved, SensorFailed, WriteFailed, StorageUnavailable };
constexpr char dashboardSsid[] = "SoccerGreenTest";
constexpr char dashboardPassword[] = "soccer123";
WebServer dashboard(80);
bool dashboardReady = false;
String captureToken;
CaptureResult lastWebCapture = CaptureResult::None;

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
  if (!storageReady || !greenStorage.isKey(storageKey) ||
      greenStorage.getBytesLength(storageKey) != sizeof(record) ||
      greenStorage.getBytes(storageKey, &record, sizeof(record)) != sizeof(record)) {
    return false;
  }
  return record.signature == recordSignature &&
         record.version == recordVersion &&
         record.atime == integrationTime &&
         record.alsGain == gain &&
         record.brightness == lightBrightness &&
         memcmp(record.channelMap, channels, sizeof(channels)) == 0 &&
         record.checksum == checksum(record);
}

bool saveGreen(GreenRecord &record) {
  if (!storageReady) return false;
  record.signature = recordSignature;
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
  if (greenStorage.putBytes(storageKey, &record, sizeof(record)) != sizeof(record)) {
    return false;
  }
  return loadGreen(readback) && memcmp(&record, &readback, sizeof(record)) == 0;
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
  if (!storageReady) {
    Serial.println(F("Could not open green calibration storage."));
    return;
  }
  GreenRecord record;
  if (loadGreen(record)) {
    Serial.println(F("LOADED GREEN FROM NVS"));
    printGreen(record);
  } else {
    Serial.println(F("No valid saved GREEN calibration. Place sensors over green and send g."));
  }
}

CaptureResult captureAndSaveGreen(GreenRecord &record) {
  if (!storageReady) return CaptureResult::StorageUnavailable;
  Serial.println(F("Capturing GREEN on all four channels. Keep the robot over green."));
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
  if (!allReady) return CaptureResult::SensorFailed;
  return saveGreen(record) ? CaptureResult::Saved : CaptureResult::WriteFailed;
}

const __FlashStringHelper *captureMessage(CaptureResult result) {
  switch (result) {
    case CaptureResult::Saved: return F("SAVED GREEN TO NVS (readback verified)");
    case CaptureResult::SensorFailed: return F("GREEN capture failed. The previous NVS record was kept.");
    case CaptureResult::WriteFailed: return F("NVS verification failed.");
    case CaptureResult::StorageUnavailable: return F("Could not open green calibration storage.");
    default: return F("");
  }
}

CaptureResult runGreenCapture() {
  GreenRecord record = {};
  const CaptureResult result = captureAndSaveGreen(record);
  Serial.println(captureMessage(result));
  if (result == CaptureResult::Saved) printGreen(record);
  return result;
}

void showDashboard() {
  GreenRecord record;
  const bool saved = loadGreen(record);
  const CaptureResult result = dashboard.hasArg("capture") ? lastWebCapture : CaptureResult::None;
  String page;
  page.reserve(3600);
  page += R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Green calibration test</title>
<style>
:root{color-scheme:dark;font:16px system-ui;background:#101820;color:#edf4f7}
body{max-width:42rem;margin:0 auto;padding:1.25rem}
h1{font-size:1.65rem;margin:0 0 .75rem}p{line-height:1.5}
.actions{display:flex;flex-wrap:wrap;gap:.65rem;margin:1.25rem 0}
button,a{display:inline-block;border:1px solid #567568;border-radius:.5rem;padding:.65rem .8rem;font:inherit;color:#edf4f7}
button{background:#216046;cursor:pointer}a{text-decoration:none;background:#213139}
.status{padding:.65rem .8rem;background:#22313b;border-radius:.5rem}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{padding:.55rem .25rem;text-align:right;border-bottom:1px solid #3a4e56}
th:first-child,td:first-child{text-align:left}
small{display:block;margin-top:.9rem;color:#b9c9d0}
</style></head><body>
<h1>Green calibration test</h1>
<p>Place all four sensors over the green field before capturing.</p>
<div class="actions"><form method="post" action="/calibrate">
<input type="hidden" name="token" value=")HTML";
  page += captureToken;
  page += F("\"><button type=\"submit\">Calibrate &amp; Save Green</button></form>"
            "<a href=\"/\">Load Saved Green</a></div><p class=\"status\" role=\"status\">");
  if (result != CaptureResult::None) {
    page += captureMessage(result);
  } else if (!storageReady) {
    page += captureMessage(CaptureResult::StorageUnavailable);
  } else {
    page += saved ? F("LOADED GREEN FROM NVS") : F("No valid saved green calibration yet.");
  }
  page += F("</p>");
  if (saved) {
    page += F("<table aria-label=\"Saved green readings\"><thead><tr><th>Sensor</th>"
              "<th>R</th><th>G</th><th>B</th><th>Clear</th></tr></thead><tbody>");
    for (uint8_t i = 0; i < sizeof(channels); ++i) {
      const GreenReading &reading = record.readings[i];
      page += F("<tr><th scope=\"row\">");
      page += positions[i];
      page += F(" CH");
      page += String(channels[i]);
      page += F("</th><td>");
      page += String(reading.red, 4);
      page += F("</td><td>");
      page += String(reading.green, 4);
      page += F("</td><td>");
      page += String(reading.blue, 4);
      page += F("</td><td>");
      page += String(reading.clear, 1);
      page += F("</td></tr>");
    }
    page += F("</tbody></table><small>Power-cycle, reconnect, and compare these saved readings.</small>");
  }
  page += F("</body></html>");
  dashboard.sendHeader("Cache-Control", "no-store");
  dashboard.send(200, "text/html; charset=utf-8", page);
}

void startDashboard() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  dashboardReady = WiFi.softAP(dashboardSsid, dashboardPassword);
  if (!dashboardReady) {
    Serial.println(F("Dashboard Wi-Fi startup failed. Serial commands are available."));
    return;
  }
  captureToken = String(esp_random(), HEX) + String(esp_random(), HEX);
  dashboard.on("/", HTTP_GET, showDashboard);
  dashboard.on("/calibrate", HTTP_POST, []() {
    if (dashboard.arg("token") != captureToken) {
      dashboard.send(403, "text/plain", "Reload the dashboard and try again.");
      return;
    }
    lastWebCapture = runGreenCapture();
    dashboard.sendHeader("Cache-Control", "no-store");
    dashboard.sendHeader("Location", "/?capture=1");
    dashboard.send(303, "text/plain", "");
  });
  dashboard.begin();
  Serial.print(F("Join Wi-Fi "));
  Serial.print(dashboardSsid);
  Serial.print(F(" and open http://"));
  Serial.println(WiFi.softAPIP());
}

void setup() {
  for (uint8_t pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  Serial.begin(115200);
  if (Wire.begin(21, 22, 100000)) {
    Wire.setTimeOut(25);
    selectMux(0);
  } else {
    Serial.println(F("I2C initialization failed."));
  }
  storageReady = greenStorage.begin(storageNamespace);
  groundLight.begin();
  groundLight.setBrightness(lightBrightness);
  groundLight.fill(groundLight.Color(255, 255, 255));
  groundLight.show();
  delay(120);
  Serial.println(F("ESP32 APDS GREEN NVS test. g=capture/save, p=read saved values."));
  showSavedGreen();
  startDashboard();
}

void loop() {
  if (dashboardReady) dashboard.handleClient();
  if (!Serial.available()) {
    delay(1);
    return;
  }
  const char command = char(Serial.read());
  if (command == 'g' || command == 'G') runGreenCapture();
  if (command == 'p' || command == 'P') showSavedGreen();
}
