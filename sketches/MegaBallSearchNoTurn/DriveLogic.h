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

}
#undef NAV_FN
#endif
