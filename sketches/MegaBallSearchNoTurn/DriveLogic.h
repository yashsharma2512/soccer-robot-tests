#ifndef MEGA_MPU_NAV_LOGIC_H
#define MEGA_MPU_NAV_LOGIC_H
#include <stdint.h>
#ifdef NAV_CONSTEXPR_TEST
#define NAV_FN constexpr
#else
#define NAV_FN inline
#endif

namespace drive {
struct Motion { float forward, strafe, rotation; };
struct Wheels { int16_t fl, fr, bl, br; };
struct NavigationLeg { Motion motion; uint8_t number; bool paused, finished; };

NAV_FN float absolute(float value) { return value < 0 ? -value : value; }
NAV_FN float limit(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}
NAV_FN float wrap180(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}
NAV_FN int16_t rounded(float value) {
  return int16_t(value >= 0 ? value + 0.5f : value - 0.5f);
}
NAV_FN float yawCorrection(float current, float target, float rate,
                           float kp, float kd, float sign, float maximum) {
  return limit(sign * (kp * wrap180(current - target) + kd * rate), -maximum, maximum);
}
NAV_FN Wheels mix(Motion motion, int16_t ceiling) {
  // Original FastRoboCup drivetrain equations.
  float fl = motion.forward + motion.strafe + motion.rotation;
  float fr = motion.forward - motion.strafe - motion.rotation;
  float bl = motion.forward - motion.strafe + motion.rotation;
  float br = motion.forward + motion.strafe - motion.rotation;
  float biggest = absolute(fl);
  if (absolute(fr) > biggest) biggest = absolute(fr);
  if (absolute(bl) > biggest) biggest = absolute(bl);
  if (absolute(br) > biggest) biggest = absolute(br);
  if (biggest > ceiling) {
    float scale = ceiling / biggest;
    fl *= scale; fr *= scale; bl *= scale; br *= scale;
  }
  return {rounded(fl), rounded(fr), rounded(bl), rounded(br)};
}

NAV_FN float turnCorrection(float current, float target, float rate,
                            float kp, float kd, float sign, float minimum, float maximum) {
  float error = wrap180(current - target);
  float correction = yawCorrection(current, target, rate, kp, kd, sign, maximum);
  // Assist low-speed stiction only when moving slowly and correcting toward
  // the target. Do not override a damping command that is braking the turn.
  if (absolute(error) > 3 && absolute(rate) < 15 &&
      absolute(correction) < minimum && correction * (error * sign) > 0)
    correction = correction > 0 ? minimum : -minimum;
  return correction;
}

class TurnSettler {
 public:
  NAV_FN TurnSettler() : active_(false), since_(0) {}
  NAV_FN void reset() { active_ = false; since_ = 0; }
  NAV_FN bool update(float error, float rate, uint32_t now) {
    if (absolute(error) > 3 || absolute(rate) > 8) { active_ = false; return false; }
    if (!active_) { active_ = true; since_ = now; }
    return uint32_t(now - since_) >= 200;
  }
 private:
  bool active_;
  uint32_t since_;
};

NAV_FN NavigationLeg navigationLeg(uint32_t elapsed, uint32_t moveMs,
                                   uint32_t pauseMs, float speed) {
  uint32_t period = moveMs + pauseMs;
  if (period == 0 || elapsed / period >= 4) return {{0,0,0},4,false,true};
  uint8_t number = elapsed / period;
  bool paused = elapsed % period >= moveMs;
  if (paused) return {{0,0,0},number,true,false};
  if (number == 0) return {{speed,0,0},number,false,false};
  if (number == 1) return {{0,speed,0},number,false,false};
  if (number == 2) return {{-speed,0,0},number,false,false};
  return {{0,-speed,0},number,false,false};
}
}
#undef NAV_FN
#endif
