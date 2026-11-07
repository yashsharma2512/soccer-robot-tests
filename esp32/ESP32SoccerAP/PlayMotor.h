#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// The controller refreshes commands; the independent worker expires them.
// HTTP never writes PWM. halt() invalidates commands already in the queue.
class PlayMotor {
 public:
  static constexpr uint8_t pins[8]={25,26,16,17,32,33,18,19};
  static const uint16_t WATCHDOG_MS=100;
  struct State {
    bool ready=false,enabled=false,active=false,stopping=false,watchdog=false;
    uint32_t generation=0,updated=0;
    int16_t wheel[4]={};
  };
  struct Command { int16_t wheel[4]; uint32_t created,generation; };
  bool begin() {
    for(uint8_t pin:pins) { digitalWrite(pin,LOW);pinMode(pin,OUTPUT); }
    for(uint8_t i=0;i<8;i++) if(!ledcAttachChannel(pins[i],16000,8,i) || !ledcWrite(pins[i],0)) { disable();return false; }
    queue_=xQueueCreate(1,sizeof(Command));if(!queue_) { disable();return false; }
    portENTER_CRITICAL(&lock_);state_.ready=true;portEXIT_CRITICAL(&lock_);
#if CONFIG_FREERTOS_UNICORE
    const int core=0;
#else
    const int core=1;
#endif
    if(xTaskCreatePinnedToCore(task,"playMotor",3072,this,3,nullptr,core)!=pdPASS) { disable();return false; }
    return true;
  }
  State state() { portENTER_CRITICAL(&lock_);State s=state_;portEXIT_CRITICAL(&lock_);return s; }
  bool idle() { State s=state();return !s.active && !s.stopping; }
  bool permit() {
    portENTER_CRITICAL(&lock_);
    bool ok=state_.ready && !state_.active && !state_.stopping;
    if(ok) { ++state_.generation;state_.enabled=true;state_.watchdog=false;state_.updated=millis(); }
    portEXIT_CRITICAL(&lock_);return ok;
  }
  void halt() {
    portENTER_CRITICAL(&lock_);
    state_.enabled=false;++state_.generation;
    state_.stopping=state_.ready;
    portEXIT_CRITICAL(&lock_);
  }
  bool request(int16_t fl,int16_t fr,int16_t bl,int16_t br) {
    Command c={{fl,fr,bl,br},millis(),0};
    for(int16_t value:c.wheel) if(value<-100 || value>100) { halt();return false; }
    portENTER_CRITICAL(&lock_);
    bool ok=state_.ready && state_.enabled && !state_.stopping;
    c.generation=state_.generation;
    portEXIT_CRITICAL(&lock_);
    if(!ok) return false;
    if(xQueueOverwrite(queue_,&c)!=pdPASS) { halt();return false; }
    return true;
  }
  void tick() {
    State s=state();if(!s.ready) return;
    Command c;
    if(xQueueReceive(queue_,&c,0)==pdTRUE) {
      s=state();
      if(s.enabled && !s.stopping && c.generation==s.generation) {
        if(uint32_t(millis()-c.created)>=WATCHDOG_MS) expire();
        else {
          if(!output(c.wheel)) return;
          portENTER_CRITICAL(&lock_);
          bool valid=state_.enabled && !state_.stopping && c.generation==state_.generation;
          if(valid) {
            state_.updated=c.created;state_.active=false;
            for(uint8_t i=0;i<4;i++) { state_.wheel[i]=c.wheel[i];state_.active|=c.wheel[i]!=0; }
          } else state_.stopping=true;
          portEXIT_CRITICAL(&lock_);
        }
      }
    }
    s=state();
    if(s.active && uint32_t(millis()-s.updated)>=WATCHDOG_MS) { expire();s=state(); }
    if(!s.enabled || s.stopping) {
      const int16_t zero[4]={};if(!output(zero)) return;
      portENTER_CRITICAL(&lock_);
      state_.active=false;state_.stopping=false;
      for(int16_t &value:state_.wheel) value=0;
      portEXIT_CRITICAL(&lock_);
    }
  }
 private:
  State state_;
  portMUX_TYPE lock_=portMUX_INITIALIZER_UNLOCKED;
  QueueHandle_t queue_=nullptr;
  static void task(void *context) {
    auto self=static_cast<PlayMotor *>(context);
    for(;;) { self->tick();vTaskDelay(pdMS_TO_TICKS(5)); }
  }
  void expire() {
    portENTER_CRITICAL(&lock_);
    state_.watchdog=true;state_.enabled=false;state_.stopping=true;++state_.generation;
    portEXIT_CRITICAL(&lock_);
  }
  void disable() {
    for(uint8_t pin:pins) { ledcDetach(pin);pinMode(pin,OUTPUT);digitalWrite(pin,LOW); }
    portENTER_CRITICAL(&lock_);state_=State{};portEXIT_CRITICAL(&lock_);
  }
  bool output(const int16_t *wheel) {
    const int8_t signs[4]={cfg::FL_SIGN,cfg::FR_SIGN,cfg::BL_SIGN,cfg::BR_SIGN};
    for(uint8_t i=0;i<4;i++) {
      int16_t value=wheel[i]*signs[i];
      if(!ledcWrite(pins[i*2],0) || !ledcWrite(pins[i*2+1],0) ||
         (value>0 && !ledcWrite(pins[i*2],value)) ||
         (value<0 && !ledcWrite(pins[i*2+1],-value))) { disable();return false; }
    }
    return true;
  }
};
constexpr uint8_t PlayMotor::pins[8];
