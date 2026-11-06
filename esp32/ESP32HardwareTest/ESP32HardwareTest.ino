// Classic ESP32-WROOM-32 wiring test. Motors stay LOW unless explicitly pulsed.
// Same wiring as ESP32SoccerFull. No calibration/NVS changes.
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#if !defined(ARDUINO_ARCH_ESP32) || !defined(CONFIG_IDF_TARGET_ESP32)
#error Select ESP32 Dev Module for a classic ESP32-WROOM-32.
#endif
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error Use esp32 by Espressif Systems 3.x.
#endif
const uint8_t MOTOR_PINS[8]={25,26,16,17,32,33,18,19};
const int8_t MOTOR_SIGNS[4]={1,1,-1,1};
const uint8_t TCA_ADDR=0x70;
const uint8_t APDS_CHANNELS[4]={0,1,2,3}; // FRONT, RIGHT, BACK, LEFT examples.
bool pwmReady=false, wireReady=false, active=false;
void pwmFailure();
uint32_t pulseStart=0,lastPoll=0;
uint8_t sensorSlot=0;

void allOff() {
  active=false;
  for (uint8_t pin:MOTOR_PINS) {
    if (pwmReady) { if (!ledcWrite(pin,0)) { pwmFailure(); return; } }
    else { pinMode(pin,OUTPUT); digitalWrite(pin,LOW); }
  }
}
void pwmFailure() {
  pwmReady=false; active=false;
  for (uint8_t pin:MOTOR_PINS) { ledcDetach(pin); pinMode(pin,OUTPUT); digitalWrite(pin,LOW); }
  Serial.println(F("PWM fault; all outputs disabled."));
}
bool mux(uint8_t mask) {
  Wire.beginTransmission(TCA_ADDR); Wire.write(mask);
  if (Wire.endTransmission()!=0) return false;
  return Wire.requestFrom(TCA_ADDR,size_t(1))==1 && uint8_t(Wire.read())==mask;
}
bool readReg(uint8_t address,uint8_t reg,uint8_t *data,uint8_t count,bool repeated=true) {
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(!repeated)!=0) return false;
  if (Wire.requestFrom(address,size_t(count))!=count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i=0;i<count;i++) data[i]=uint8_t(Wire.read());
  return true;
}
bool writeReg(uint8_t address,uint8_t reg,uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}
void initializeSensors() {
  allOff();
  if (!wireReady || !mux(0)) { Serial.println(F("TCA 0x70 unavailable.")); return; }
  for (uint8_t i=0;i<4;i++) {
    if (!mux(uint8_t(1U<<APDS_CHANNELS[i]))) { mux(0); return; }
    uint8_t id=0;
    bool good=readReg(0x39,0x92,&id,1) && (id==0xAB || id==0x9C || id==0x9E);
    good=good && writeReg(0x39,0x80,0) && writeReg(0x39,0xAB,0) &&
        writeReg(0x39,0x81,250) && writeReg(0x39,0x8F,1) && writeReg(0x39,0x80,1);
    delay(10); good=good && writeReg(0x39,0x80,3);
    bool closed=mux(0);
    Serial.print(F("APDS CH")); Serial.print(APDS_CHANNELS[i]);
    Serial.print(F(" ID=0x")); Serial.print(id,HEX);
    Serial.println(good && closed ? F(" READY") : F(" FAILED"));
  }
}
void pulse(uint8_t command) {
  allOff();
  if (!pwmReady) return;
  uint8_t motor=(command-1)%4;
  int value=(command<=4 ? 70 : -70)*MOTOR_SIGNS[motor];
  uint8_t pin=MOTOR_PINS[motor*2+(value>0 ? 0 : 1)];
  if (!ledcWrite(pin,abs(value))) { pwmFailure(); return; }
  pulseStart=millis(); active=true;
  Serial.print(F("400 ms pulse: "));
  Serial.print(motor==0 ? F("FL") : motor==1 ? F("FR") : motor==2 ? F("BL") : F("BR"));
  Serial.println(command<=4 ? F(" forward") : F(" reverse"));
}
void serialCommands() {
  static char pending=0;
  static bool bad=false,discard=false;
  for (uint8_t budget=0;budget<32 && Serial.available();budget++) {
    char c=char(Serial.read());
    if (c=='s'||c=='S'||c=='x'||c=='X'||c=='0'||c==' ') {
      allOff(); pending=0; bad=false; discard=true; Serial.println(F("Stopped."));
    } else if (c=='\r'||c=='\n') {
      if (!discard && pending) {
        if (!bad && pending>='1' && pending<='8') pulse(uint8_t(pending-'0'));
        else if (!bad && (pending=='t'||pending=='T')) initializeSensors();
        else { allOff(); Serial.println(F("Invalid command; stopped.")); }
      }
      pending=0; bad=discard=false;
    } else if (!discard) { if (pending) bad=true; else pending=c; }
  }
}
void pollSensor() {
  uint8_t slot=sensorSlot; sensorSlot=(sensorSlot+1)%6;
  uint8_t channel=slot<4 ? APDS_CHANNELS[slot] : 0;
  if (!mux(slot<4 ? uint8_t(1U<<channel) : 0)) { mux(0); Serial.println(F("MUX SELECT FAILED")); return; }
  bool good=false;
  if (slot<4) {
    uint8_t status=0,data[8]={};
    good=readReg(0x39,0x93,&status,1) && (status&1) && readReg(0x39,0x94,data,8);
    Serial.print(F("APDS CH")); Serial.print(channel); Serial.print(F(" C="));
    if (good) Serial.print(uint16_t(data[0])|(uint16_t(data[1])<<8));
  } else if (slot==4) {
    uint8_t data[6]={}; good=readReg(0x08,0x49,data,6,false) && data[0]<=9;
    Serial.print(F("IR MAIN dir=")); if (good) Serial.print(data[0]);
  } else {
    uint8_t id=0,address=0x68;
    good=readReg(address,0x75,&id,1);
    if (!good) { address=0x69; good=readReg(address,0x75,&id,1); }
    Serial.print(F("MPU MAIN addr=0x")); Serial.print(address,HEX);
    Serial.print(F(" ID=0x")); if (good) Serial.print(id,HEX);
    good=good && (id==0x68 || id==0x70 || id==0x74);
  }
  bool closed=mux(0);
  Serial.println(good && closed ? F(" OK") : F(" READ/CONFIG/MUX FAILED"));
}
void setup() {
  for (uint8_t pin:MOTOR_PINS) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
  Serial.begin(115200);
  bool attached=true;
  for (uint8_t i=0;i<8;i++) {
    if (!ledcAttachChannel(MOTOR_PINS[i],16000,8,i) || !ledcWrite(MOTOR_PINS[i],0)) { attached=false; break; }
  }
  if (!attached) pwmFailure(); else pwmReady=true;
  wireReady=Wire.begin(21,22,100000); Wire.setTimeOut(25);
  Serial.println(F("ESP32 HARDWARE TEST. 115200 baud, Newline. No automatic motion."));
  Serial.println(F("1..4=FL/FR/BL/BR forward; 5..8=reverse; s=stop; t=sensor reinitialize."));
  Serial.println(F("Motor pulses stop after 400 ms. Lift wheels before testing."));
  if (wireReady) initializeSensors();
}
void loop() {
  serialCommands();
  if (active && uint32_t(millis()-pulseStart)>=400) allOff();
  // Do not perform blocking I2C work during a timed motor pulse.
  if (!active && wireReady && uint32_t(millis()-lastPoll)>=100) {
    lastPoll=millis(); pollSensor();
  }
  delay(1);
}
