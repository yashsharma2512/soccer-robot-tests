#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Only this worker writes motor PWM after initialization. Its 5 ms service
// task runs independently of HTTP handling and blocking sensor transfers.
class BenchMotor {
 public:
  struct Command { int16_t wheel[4]; uint16_t duration; };
  struct State {
    bool ready=false,armed=false,active=false,queued=false,stopping=false;
    uint32_t armUntil=0,started=0; uint16_t duration=0;
    int16_t wheel[4]={0,0,0,0};
  };
  static constexpr uint8_t pins[8]={25,26,16,17,32,33,18,19};
  bool begin() {
    for (uint8_t pin:pins) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
    for (uint8_t i=0;i<8;i++) {
      if (!ledcAttachChannel(pins[i],16000,8,i) || !ledcWrite(pins[i],0)) { disable(); return false; }
    }
    queue_=xQueueCreate(1,sizeof(Command));
    if (!queue_) { disable(); return false; }
    state_.ready=true;
#if CONFIG_FREERTOS_UNICORE
    const int core=0;
#else
    const int core=1;
#endif
    if (xTaskCreatePinnedToCore(task,"benchMotor",3072,this,3,&task_,core)!=pdPASS) {
      disable(); return false;
    }
    return true;
  }
  State state() {
    portENTER_CRITICAL(&lock_); State result=state_; portEXIT_CRITICAL(&lock_);
    if (result.armed && int32_t(millis()-result.armUntil)>=0) result.armed=false;
    return result;
  }
  bool idle() { State s=state(); return !s.active && !s.queued && !s.stopping; }
  bool arm() {
    portENTER_CRITICAL(&lock_);
    bool good=state_.ready && !state_.active && !state_.queued && !state_.stopping;
    if (good) { state_.armed=true; state_.armUntil=millis()+30000UL; }
    portEXIT_CRITICAL(&lock_); return good;
  }
  void stop() {
    portENTER_CRITICAL(&lock_);
    state_.armed=false; state_.queued=false; state_.stopping=true;
    bool ready=state_.ready;
    portEXIT_CRITICAL(&lock_);
    if (ready) { Command command={{0,0,0,0},0}; xQueueOverwrite(queue_,&command); }
    else { portENTER_CRITICAL(&lock_); state_.stopping=false; portEXIT_CRITICAL(&lock_); }
  }
  bool pulse(const Command &command) {
    if (command.duration<100 || command.duration>500) return false;
    for (int16_t value:command.wheel) if (value<-120 || value>120) return false;
    portENTER_CRITICAL(&lock_);
    bool good=state_.ready && state_.armed && int32_t(millis()-state_.armUntil)<0 &&
        !state_.active && !state_.queued && !state_.stopping;
    if (good) state_.queued=true;
    portEXIT_CRITICAL(&lock_);
    if (!good) return false;
    if (xQueueOverwrite(queue_,&command)!=pdPASS) { stop(); return false; }
    return true;
  }
  // Called exclusively by the worker; host tests call this same method directly.
  void tick() {
    State before=state();
    if (!before.ready) return;
    Command command;
    if (xQueueReceive(queue_,&command,0)==pdTRUE) {
      State s=state();
      bool run=command.duration && s.armed && !s.stopping;
      if (!output(run ? command.wheel : zeros_)) return;
      portENTER_CRITICAL(&lock_);
      bool stillAllowed=run && state_.armed && !state_.stopping && int32_t(millis()-state_.armUntil)<0;
      state_.queued=false; state_.active=stillAllowed;
      if (!stillAllowed) state_.stopping=true;
      if (stillAllowed) {
        state_.started=millis(); state_.duration=command.duration;
        for (uint8_t i=0;i<4;i++) state_.wheel[i]=command.wheel[i];
      }
      portEXIT_CRITICAL(&lock_);
      if (!stillAllowed) finish();
    }
    State s=state();
    if (s.stopping || (s.active && (!s.armed || uint32_t(millis()-s.started)>=s.duration))) finish();
  }
 private:
  static constexpr int16_t zeros_[4]={0,0,0,0};
  State state_;
  portMUX_TYPE lock_=portMUX_INITIALIZER_UNLOCKED;
  QueueHandle_t queue_=nullptr;
  TaskHandle_t task_=nullptr;
  static void task(void *context) {
    BenchMotor *self=static_cast<BenchMotor *>(context);
    for (;;) { self->tick(); vTaskDelay(pdMS_TO_TICKS(5)); }
  }
  void disable() {
    for (uint8_t pin:pins) { ledcDetach(pin); pinMode(pin,OUTPUT); digitalWrite(pin,LOW); }
    portENTER_CRITICAL(&lock_); state_=State{}; portEXIT_CRITICAL(&lock_);
  }
  bool output(const int16_t *logical) {
    const int8_t signs[4]={1,1,-1,1};
    for (uint8_t m=0;m<4;m++) {
      int16_t value=logical[m]*signs[m];
      if (!ledcWrite(pins[m*2],0) || !ledcWrite(pins[m*2+1],0) ||
          (value>0 && !ledcWrite(pins[m*2],value)) ||
          (value<0 && !ledcWrite(pins[m*2+1],-value))) { disable(); return false; }
    }
    return true;
  }
  void finish() {
    if (!output(zeros_)) return;
    portENTER_CRITICAL(&lock_);
    state_.active=state_.queued=state_.stopping=false;
    for (int16_t &value:state_.wheel) value=0;
    portEXIT_CRITICAL(&lock_);
  }
};
constexpr uint8_t BenchMotor::pins[8];
constexpr int16_t BenchMotor::zeros_[4];
