// Classic ESP32-WROOM-32. Arduino ESP32 core 3.x. No external libraries.
#include "BenchSensors.h"
#include "BenchMotor.h"
#include "BenchLogic.h"
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/semphr.h>
#include "BenchPage.h"

const bool ENABLE_AP=true; // false: Serial-only comparison, same sensor/motor test
const char *AP_PASSWORD="SoccerBench32";
BenchMotor motors;
MpuYaw imu;
BoundarySensors boundary;
WebServer server(80);
SemaphoreHandle_t snapshotMutex;
QueueHandle_t jobs;
portMUX_TYPE gateLock=portMUX_INITIALIZER_UNLOCKED;
bool busy=true, pending=false;
String snapshot="{}", events[16], apiKey, apName;
uint8_t eventNext=0, eventCount=0;
enum Job : uint8_t { GYRO, GREEN, RESCAN, CANCEL_JOB };

void event(const String &message) {
  if (!snapshotMutex) return;
  xSemaphoreTake(snapshotMutex,portMAX_DELAY);
  events[eventNext]=String(millis())+" ms: "+message;
  eventNext=(eventNext+1)%16; if (eventCount<16) ++eventCount;
  xSemaphoreGive(snapshotMutex);
}
bool blocked() {
  portENTER_CRITICAL(&gateLock); bool result=busy || pending; portEXIT_CRITICAL(&gateLock);
  return result;
}
void submit(Job job) {
  portENTER_CRITICAL(&gateLock); pending=true; portEXIT_CRITICAL(&gateLock);
  motors.stop(); // Never write flash until worker confirms PWM is zero.
  xQueueOverwrite(jobs,&job);
}
void stopAll() { submit(CANCEL_JOB); event("STOP: motors disarmed; calibration cancelled"); }
String jsonBool(bool value) { return value ? "true" : "false"; }

void sensors(void *) {
  Wire.begin(espconfig::SDA_PIN,espconfig::SCL_PIN,100000);
  Wire.setTimeOut(espconfig::I2C_TIMEOUT_MS);
  calibstore::begin(); boundary.begin(); imu.startCalibration(true);
  uint32_t lastIR=0,lastPublish=0,irFresh=0,irGood=0,irFailures=0;
  uint8_t ir[6]={},irStatus=0,irBytes=0;
  bool irOK=false;
  int previousMpu=-1; bool previousLine=false;
  uint32_t seenGeneration=calibstore::generation;
  for (;;) {
    Job job;
    if (motors.idle() && xQueueReceive(jobs,&job,0)==pdTRUE) {
      portENTER_CRITICAL(&gateLock); pending=false; busy=true; portEXIT_CRITICAL(&gateLock);
      imu.cancelCalibration(); boundary.cancelCapture();
      if (job==GYRO) { imu.startCalibration(false); event("Gyro calibration requested: KEEP STILL"); }
      if (job==GREEN) event(boundary.startField() ? "Green capture: 3 seconds to position sensors" : "Green capture rejected: sensors not ready");
      if (job==RESCAN) { boundary.begin(); imu.startCalibration(true); event("Rescan complete; saved calibration loaded if valid"); }
    }
    synchronizeGyroStorage(imu,seenGeneration);
    imu.service(); boundary.service();
    synchronizeGyroStorage(imu,seenGeneration);
    uint32_t now=millis();
    if (uint32_t(now-lastIR)>=30) {
      lastIR=now; uint8_t next[6];
      irOK=bus::readDevice(0x08,0x49,next,6,irStatus,irBytes,false) && next[0]<=9;
      if (irOK) { memcpy(ir,next,6); irFresh=millis(); ++irGood; }
      else ++irFailures;
    }
    now=millis(); // Transactions advance time; never subtract a newer sample from an older clock.
    portENTER_CRITICAL(&gateLock);
    busy=imu.calibrating() || imu.saving() || boundary.capturing() || boundary.saving();
    portEXIT_CRITICAL(&gateLock);
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
      s="{\"ms\":"+String(now)+",\"busy\":"+jsonBool(blocked())+",\"nvs\":"+jsonBool(calibstore::ready)+
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
  BenchMotor::State m=motors.state();
  result+="],\"motor\":{\"ready\":"+jsonBool(m.ready)+",\"armed\":"+jsonBool(m.armed)+
    ",\"active\":"+jsonBool(m.active || m.queued || m.stopping)+",\"wheel\":[";
  for(uint8_t i=0;i<4;i++) { if(i) result+=','; result+=String(m.wheel[i]); }
  result+="]}}"; return result;
}
bool pulse(const String &target,int pwm,int duration,bool reverse) {
  if (blocked()) return false;
  BenchMotor::Command c={{0,0,0,0},uint16_t(duration)};
  const char *names[]={"FL","FR","BL","BR"}; bool individual=false;
  for(uint8_t i=0;i<4;i++) if(target==names[i]) { c.wheel[i]=reverse ? -pwm : pwm; individual=true; }
  if (!individual && !benchlogic::motion(target.c_str(),pwm,c.wheel)) return false;
  bool accepted=motors.pulse(c);
  if(accepted) event("Motor pulse "+target+" PWM="+String(pwm)+" duration="+String(duration)+" ms"+(reverse ? " reverse" : ""));
  return accepted;
}
void reply(int code,const String &message) { server.send(code,"text/plain",message); }
void action() {
  if(server.header("X-Bench-Key")!=apiKey) { reply(403,"Reload this device's dashboard"); return; }
  String cmd=server.arg("action");
  if(cmd=="stop") { stopAll(); reply(200,"Stopped and disarmed"); return; }
  if(cmd=="marker") { event("BATTERY SWITCH marker"); reply(200,"Marker recorded"); return; }
  if(cmd=="arm") {
    if(server.arg("raised")!="yes" || blocked() || !motors.arm()) { reply(409,"Raise wheels; wait for calibration and motor stop"); return; }
    event("Motors armed for 30 seconds"); reply(200,"Armed for 30 seconds"); return;
  }
  if(cmd=="gyro" || cmd=="green" || cmd=="rescan") {
    if(blocked() || !motors.idle()) { reply(409,"Stop first; wait until calibration is idle"); return; }
    submit(cmd=="gyro" ? GYRO : (cmd=="green" ? GREEN : RESCAN)); reply(202,"Queued; motors disarmed"); return;
  }
  if(cmd=="pulse") {
    int pwm=0,ms=0;
    if(!benchlogic::number(server.arg("pwm").c_str(),30,120,pwm) ||
       !benchlogic::number(server.arg("ms").c_str(),100,500,ms) ||
       (server.arg("reverse")!="0" && server.arg("reverse")!="1")) { reply(400,"PWM 30..120; duration 100..500 ms; reverse 0/1"); return; }
    bool accepted=pulse(server.arg("target"),pwm,ms,server.arg("reverse")=="1");
    reply(accepted ? 200 : 409,accepted ? "Pulse accepted" : "Pulse rejected: arm first; wait for idle; use a valid target"); return;
  }
  reply(400,"Unknown action");
}
void setup() {
  Serial.begin(115200);
  snapshotMutex=xSemaphoreCreateMutex(); jobs=xQueueCreate(1,sizeof(Job));
  if(!snapshotMutex || !jobs) { for(uint8_t p:BenchMotor::pins) { pinMode(p,OUTPUT); digitalWrite(p,LOW); } for(;;) delay(1000); }
  motors.begin();
#if CONFIG_FREERTOS_UNICORE
  const int core=0;
#else
  const int core=1;
#endif
  if(xTaskCreatePinnedToCore(sensors,"benchSensors",12288,nullptr,2,nullptr,core)!=pdPASS) {
    motors.stop(); event("Sensor task failed; motor arming blocked");
  }
  apiKey=String(esp_random(),HEX)+String(esp_random(),HEX);
  apName="SoccerBench-"+String(uint32_t(ESP.getEfuseMac())&0xFFFF,HEX);
  if(ENABLE_AP) {
    WiFi.persistent(false); WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
    bool ap=WiFi.softAP(apName.c_str(),AP_PASSWORD,1,false,2);
    event(ap ? "AP online: 192.168.4.1" : "AP initialization FAILED");
    const char *headers[]={"X-Bench-Key"}; server.collectHeaders(headers,1);
    server.on("/",HTTP_GET,[]() {
      String page=FPSTR(BENCH_PAGE); page.replace("__KEY__",apiKey);
      server.sendHeader("Cache-Control","no-store"); server.send(200,"text/html",page);
    });
    server.on("/api/status",HTTP_GET,[]() { server.sendHeader("Cache-Control","no-store"); server.send(200,"application/json",statusJSON()); });
    server.on("/api/action",HTTP_POST,action);
    server.onNotFound([]() { reply(404,"Not found"); }); server.begin();
  }
  Serial.println("BENCH ONLY. No automatic movement. Wheels raised before a=ARM.");
  Serial.println("a arm; 1..4 FL/FR/BL/BR forward; 5..8 reverse; f/v all forward/back; s STOP");
  Serial.println("c gyro; g green; b rescan; m battery marker; p JSON status");
  if(ENABLE_AP) Serial.println(apName+" password: "+AP_PASSWORD+" URL: http://192.168.4.1");
  else Serial.println("AP disabled; Serial-only bench");
}
void loop() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='s' || c=='x' || c=='0' || c==' ') stopAll();
    else if(c=='a') { bool ok=!blocked() && motors.arm(); Serial.println(ok ? "ARMED 30 seconds" : "Arm rejected: busy/not ready"); }
    else if(c>='1' && c<='8') { const char *n[]={"FL","FR","BL","BR"}; Serial.println(pulse(n[(c-'1')%4],80,400,c>='5') ? "Pulse accepted" : "Pulse rejected"); }
    else if(c=='f' || c=='v') Serial.println(pulse(c=='f' ? "forward" : "back",80,400,false) ? "Pulse accepted" : "Pulse rejected");
    else if(c=='c' || c=='g' || c=='b') { if(!blocked() && motors.idle()) submit(c=='c' ? GYRO : (c=='g' ? GREEN : RESCAN)); else Serial.println("Busy: s stops/cancels first"); }
    else if(c=='m') event("BATTERY SWITCH marker");
    else if(c=='p') Serial.println(statusJSON());
  }
  if(ENABLE_AP) server.handleClient();
  delay(2);
}
