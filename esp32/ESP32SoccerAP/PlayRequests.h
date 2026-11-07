#pragma once
// One pending control request. STOP overwrites pending work and invalidates an
// older Resume claim. Lock order is requests -> motor, never the reverse.
class PlayRequests {
 public:
  explicit PlayRequests(PlayMotor &motor):motor_(motor) {}
  bool submit(char command) {
    if(command!='s' && command!='r' && command!='c' && command!='g' && command!='b' && command!='f') return false;
    portENTER_CRITICAL(&lock_);
    bool ok=command=='s' || (!pending_ && !busy_);
    if(ok) { ++generation_;command_=command;pending_=true;motor_.halt(); }
    portEXIT_CRITICAL(&lock_);return ok;
  }
  bool take(char &command,uint32_t &generation) {
    portENTER_CRITICAL(&lock_);
    bool ok=pending_ && motor_.idle();
    if(ok) { command=command_;generation=generation_;pending_=false;busy_=true; }
    portEXIT_CRITICAL(&lock_);return ok;
  }
  bool permit(uint32_t generation) {
    portENTER_CRITICAL(&lock_);
    bool ok=!pending_ && generation==generation_ && motor_.permit();
    portEXIT_CRITICAL(&lock_);return ok;
  }
  bool blocked() { portENTER_CRITICAL(&lock_);bool b=pending_ || busy_;portEXIT_CRITICAL(&lock_);return b; }
  bool pending() { portENTER_CRITICAL(&lock_);bool b=pending_;portEXIT_CRITICAL(&lock_);return b; }
  void busy(bool value) { portENTER_CRITICAL(&lock_);busy_=value;portEXIT_CRITICAL(&lock_); }
 private:
  PlayMotor &motor_;
  portMUX_TYPE lock_=portMUX_INITIALIZER_UNLOCKED;
  bool busy_=true,pending_=false;
  char command_='s';uint32_t generation_=0;
};
