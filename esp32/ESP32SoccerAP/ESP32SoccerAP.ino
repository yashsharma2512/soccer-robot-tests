// Classic ESP32-WROOM-32 full soccer program with an offline AP dashboard.
#include "PlayRuntime.h"
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/semphr.h>
#include "PlayPage.h"

const bool ENABLE_AP=true;
const char *AP_PASSWORD="SoccerPlay32";
WebServer server(80);
SemaphoreHandle_t snapshotMutex;
String snapshot="{}",events[16],apiKey,apName;
uint8_t eventNext=0,eventCount=0;
String jsonBool(bool value) { return value ? "true" : "false"; }
void event(const String &message) {
  xSemaphoreTake(snapshotMutex,portMAX_DELAY);
  events[eventNext]=String(millis())+" ms: "+message;
  eventNext=(eventNext+1)%16;if(eventCount<16) ++eventCount;
  xSemaphoreGive(snapshotMutex);
}
bool submit(char command) {
  bool accepted=requests.submit(command);
  if(accepted) event(String("Requested ")+command+"; drive paused");
  return accepted;
}
void playEvent(const char *message) { event(message); }
void control(void *) {
  initializePlay();
  uint32_t lastPublish=0;
  int previousMpu=-1;bool previousLine=false;
  for(;;) {
    stepPlay();
    uint32_t now=millis();
    if (int(imu.error())!=previousMpu) {
      previousMpu=int(imu.error()); event(String("MPU: ")+String(imu.errorName(imu.error())));
      imu.printLastReadFailure();
    }
    bool lineReady=boundary.calibrated();
    if (lineReady!=previousLine) { previousLine=lineReady; event(lineReady ? "Line thresholds ready" : "Line thresholds invalid"); }
    if (uint32_t(now-lastPublish)>=100) {
      lastPublish=now;
      uint16_t strength=0;
      if (ir[0]) { uint8_t i=ir[0]/2+1; strength=(ir[0]&1) ? ir[i] : (uint16_t(ir[i-1])+ir[i])/2; }
      String s; s.reserve(4000);
      s="{\"ms\":"+String(now)+",\"playState\":\""+String(state ? state : F("STARTING"))+"\",\"playing\":"+jsonBool(enabled)+
        ",\"canResume\":"+jsonBool(readyToResume() && !requests.blocked())+",\"busy\":"+jsonBool(requests.blocked())+",\"nvs\":"+jsonBool(calibstore::ready)+
        ",\"mux\":"+jsonBool(bus::isolated)+",\"mpu\":{\"ok\":"+jsonBool(imu.healthy(now))+
        ",\"id\":"+String(imu.identity())+",\"address\":"+String(imu.address())+
        ",\"error\":\""+String(imu.errorName(imu.error()))+"\",\"calibrating\":"+jsonBool(imu.calibrating())+
        ",\"progress\":"+String(imu.progress())+",\"loaded\":"+jsonBool(imu.loaded())+
        ",\"yaw\":"+String(imu.yaw(),2)+",\"rate\":"+String(imu.rate(),2)+",\"bias\":"+String(imu.biasRaw(),2)+
        ",\"age\":"+String(imu.sampleAge())+",\"samples\":"+String(imu.samples)+
        ",\"failedTransactions\":"+String(imu.failedTransactions)+",\"recovered\":"+String(imu.recoveredReads())+
        ",\"wireStatus\":"+String(imu.lastWireStatus)+",\"bytes\":"+String(imu.lastReceived)+
        ",\"register\":"+String(imu.lastRegister)+",\"raw\":[";
      for (uint8_t i=0;i<7;i++) { if(i) s+=','; s+=String(imu.raw[i]); }
      s+=" ]},\"ir\":{\"ok\":"+jsonBool(irOK && uint32_t(now-irFresh)<=150)+",\"direction\":"+String(ir[0])+
        ",\"strength\":"+String(strength)+",\"age\":"+String(uint32_t(now-irFresh))+
        ",\"good\":"+String(irGood)+",\"failures\":"+String(irFailures)+",\"wireStatus\":"+String(irStatus)+
        ",\"bytes\":"+String(irBytes)+",\"detectors\":[";
      for (uint8_t i=1;i<6;i++) { if(i>1) s+=','; s+=String(ir[i]); }
      s+=" ]},\"lineCalibrated\":"+jsonBool(boundary.calibrated())+",\"lineCapturing\":"+jsonBool(boundary.capturing())+
         ",\"lineSaving\":"+jsonBool(boundary.saving())+",\"apds\":[";
      for(uint8_t i=0;i<4;i++) {
        if(i) s+=',';
        s+="{\"channel\":"+String(boundarycfg::CHANNELS[i])+",\"side\":"+String(boundarycfg::SIDES[i])+
          ",\"id\":"+String(boundary.id(i))+",\"ok\":"+jsonBool(boundary.fresh(i))+
          ",\"age\":"+String(boundary.age(i))+",\"clear\":"+String(boundary.clear(i))+
          ",\"rgb\":["+String(boundary.rgb(i,0))+","+String(boundary.rgb(i,1))+","+String(boundary.rgb(i,2))+
          "],\"white\":"+jsonBool((boundary.hitMask()&(1<<i))!=0)+",\"threshold\":"+String(boundary.threshold(i))+
          ",\"release\":"+String(boundary.release(i))+",\"calibrated\":"+jsonBool(boundary.valid(i))+
          ",\"progress\":"+String(boundary.progress(i))+",\"errors\":"+String(boundary.errors(i))+
          ",\"fault\":"+String(boundary.fault(i))+"}";
      }
      s+="]}";
      xSemaphoreTake(snapshotMutex,portMAX_DELAY); snapshot=s; xSemaphoreGive(snapshotMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

String statusJSON() {
  xSemaphoreTake(snapshotMutex,portMAX_DELAY);
  String result=snapshot;
  if (result.endsWith("}")) result.remove(result.length()-1);
  result+=",\"events\":[";
  for(uint8_t i=0;i<eventCount;i++) {
    if(i) result+=',';
    // Event messages are firmware constants and numbers only, without quotes.
    result+='"'; result+=events[(eventNext+16-eventCount+i)%16]; result+='"';
  }
  xSemaphoreGive(snapshotMutex);
  PlayMotor::State m=motors.state();
  result+="],\"motor\":{\"ready\":"+jsonBool(m.ready)+",\"enabled\":"+jsonBool(m.enabled)+
    ",\"active\":"+jsonBool(m.active || m.stopping)+",\"wheel\":[";
  for(uint8_t i=0;i<4;i++) { if(i) result+=','; result+=String(m.wheel[i]); }
  result+="],\"watchdog\":"+jsonBool(m.watchdog)+"}}"; return result;
}
void reply(int code,const String &message) { server.send(code,"text/plain",message); }
void action() {
  if(server.header("X-Bench-Key")!=apiKey) { reply(403,"Reload this device dashboard");return; }
  String action=server.arg("action");
  if(action=="marker") { event("BATTERY SWITCH marker");reply(200,"Marker recorded");return; }
  char command=action=="stop" ? 's' : action=="resume" ? 'r' : action=="gyro" ? 'c' :
    action=="green" ? 'g' : action=="rescan" ? 'b' : action=="fullcal" ? 'f' : 0;
  if(!command) { reply(400,"Unknown action");return; }
  bool accepted=submit(command);
  reply(accepted ? 202 : 409,accepted ? "Queued; check play state. Calibration stays stopped until Resume." : "Busy: STOP cancels current calibration");
}
void handleSerial() {
  static char pending=0;static bool extra=false,discard=false;
  for(uint8_t budget=0;budget<32 && Serial.available();budget++) {
    char c=char(Serial.read());if(c>='A' && c<='Z') c+='a'-'A';
    if(c=='\r' || c=='\n') {
      if(!discard && pending) {
        if(pending=='p' && !extra) Serial.println(statusJSON());
        else if(pending=='m' && !extra) event("BATTERY SWITCH marker");
        else if(extra || !submit(pending)) Serial.println("Rejected command or busy; s stops");
      }
      pending=0;extra=discard=false;
    } else if(c=='s' || c=='x' || c=='0' || c==' ') {
      submit('s');pending=0;discard=true;
    } else if(!discard) { if(pending) extra=true;else pending=c; }
  }
}
void setup() {
  for(uint8_t pin:PlayMotor::pins) { digitalWrite(pin,LOW);pinMode(pin,OUTPUT); }
  Serial.begin(115200);snapshotMutex=xSemaphoreCreateMutex();
  if(!snapshotMutex) for(;;) delay(1000);
  apiKey=String(esp_random(),HEX)+String(esp_random(),HEX);
  apName="SoccerPlay-"+String(uint32_t(ESP.getEfuseMac())&0xFFFF,HEX);
  if(ENABLE_AP) {
    WiFi.persistent(false);WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
    event(WiFi.softAP(apName.c_str(),AP_PASSWORD,1,false,2) ? "AP online: 192.168.4.1" : "AP initialization FAILED");
    const char *headers[]={"X-Bench-Key"};server.collectHeaders(headers,1);
    server.on("/",HTTP_GET,[]() {
      String page=FPSTR(PLAY_PAGE);page.replace("__KEY__",apiKey);
      server.sendHeader("Cache-Control","no-store");server.send(200,"text/html",page);
    });
    server.on("/api/status",HTTP_GET,[]() { server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",statusJSON()); });
    server.on("/api/action",HTTP_POST,action);
    server.onNotFound([]() { reply(404,"Not found"); });server.begin();
    Serial.println(apName+" password: "+AP_PASSWORD+" URL: http://192.168.4.1");
  }
#if CONFIG_FREERTOS_UNICORE
  const int core=0;
#else
  const int core=1;
#endif
  if(xTaskCreatePinnedToCore(control,"soccerControl",14336,nullptr,2,nullptr,core)!=pdPASS)
    event("Control task failed; motors remain LOW");
  Serial.println("FULL PLAY: r resume; s STOP; c gyro; g GREEN; f gyro then GREEN; b rescan; p JSON. Newline.");
}
void loop() { handleSerial();if(ENABLE_AP) server.handleClient();delay(2); }
