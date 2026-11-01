#define NAV_CONSTEXPR_TEST
#include "../sketches/MegaMPUSingleFile/MegaMPUSingleFile.ino"

constexpr bool same(drive::Wheels w, int fl, int fr, int bl, int br) {
  return w.fl == fl && w.fr == fr && w.bl == bl && w.br == br;
}
static_assert(same(drive::mix({0,0,0},120),0,0,0,0), "Stop pattern");
static_assert(same(drive::mix({80,0,0},120),80,80,80,80), "Forward pattern");
static_assert(same(drive::mix({-80,0,0},120),-80,-80,-80,-80), "Backward pattern");
static_assert(same(drive::mix({0,80,0},120),80,-80,-80,80), "Right strafe pattern");
static_assert(same(drive::mix({0,-80,0},120),-80,80,80,-80), "Left strafe pattern");
static_assert(same(drive::mix({0,0,40},120),40,-40,40,-40), "Clockwise correction pattern");
static_assert(same(drive::mix({80,80,40},120),120,-24,24,72), "Preserve ratios under normalization");
static_assert(same(drive::mix({255,255,255},120),120,-40,40,40), "Bound all wheel outputs");

static_assert(drive::yawCorrection(10,0,0,2.5,0,1,40) == 25, "Preserve baseline feedback sign");
static_assert(drive::yawCorrection(179,-179,0,2.5,0,1,40) == -5, "Short yaw wrap correction");
static_assert(drive::yawCorrection(-179,179,0,2.5,0,1,40) == 5, "Opposite yaw wrap correction");
static_assert(drive::yawCorrection(90,0,0,2.5,0,1,40) == 40, "Bound heading correction");
static_assert(drive::wrap180(170 + 90) == -100, "Left quarter-turn across wrap");
static_assert(drive::wrap180(-170 - 90) == 100, "Right quarter-turn across wrap");
static_assert(drive::turnCorrection(0,90,0,2.5,0.8,1,30,60) == -60, "Left target must request counterclockwise motion");
static_assert(drive::turnCorrection(0,-90,0,2.5,0.8,1,30,60) == 60, "Right target must request clockwise motion");
static_assert(drive::turnCorrection(5,0,0,2.5,0.8,1,30,60) == 30, "Minimum correction can overcome low-speed stiction");
static_assert(drive::turnCorrection(5,0,-20,2.5,0.8,1,30,60) < 0, "Minimum correction must not override damping");

static_assert(drive::navigationLeg(0,1500,500,80).motion.forward == 80, "Sequence starts forward");
static_assert(drive::navigationLeg(1499,1500,500,80).motion.forward == 80, "Forward duration");
static_assert(drive::navigationLeg(1500,1500,500,80).paused, "Pause between legs");
static_assert(drive::navigationLeg(2000,1500,500,80).motion.strafe == 80, "Second leg is right");
static_assert(drive::navigationLeg(4000,1500,500,80).motion.forward == -80, "Third leg is backward");
static_assert(drive::navigationLeg(6000,1500,500,80).motion.strafe == -80, "Fourth leg is left");
static_assert(drive::navigationLeg(7999,1500,500,80).paused, "Final pause remains stopped");
static_assert(drive::navigationLeg(8000,1500,500,80).finished, "Sequence must terminate");
static_assert(drive::navigationLeg(0,0,0,80).finished, "Zero-duration configuration must stop safely");

constexpr bool settlingNeedsTime() {
  drive::TurnSettler settle;
  if (settle.update(1,0,0)) return false;
  if (settle.update(1,0,199)) return false;
  return settle.update(1,0,200);
}
constexpr bool overshootRestartsSettling() {
  drive::TurnSettler settle;
  settle.update(1,0,0);
  if (settle.update(4,0,150)) return false;
  if (settle.update(1,0,200)) return false;
  if (settle.update(1,0,399)) return false;
  return settle.update(1,0,400);
}
constexpr bool fastMotionCannotSettle() {
  drive::TurnSettler settle;
  if (settle.update(0,20,0)) return false;
  return !settle.update(0,20,500);
}
constexpr bool settlingSurvivesRollover() {
  drive::TurnSettler settle;
  uint32_t start = UINT32_MAX - 100;
  if (settle.update(1,0,start)) return false;
  return settle.update(1,0,start + 200);
}
constexpr bool sequenceElapsedSurvivesRollover() {
  uint32_t start = UINT32_MAX - 100;
  uint32_t now = start + 2000;
  return drive::navigationLeg(uint32_t(now - start),1500,500,80).motion.strafe == 80;
}
static_assert(settlingNeedsTime(), "Turn must remain settled for 200ms");
static_assert(overshootRestartsSettling(), "Overshoot must restart settle timer");
static_assert(fastMotionCannotSettle(), "Passing through target at speed must not count as settled");
static_assert(settlingSurvivesRollover(), "Turn settling must survive millis rollover");
static_assert(sequenceElapsedSurvivesRollover(), "Sequence timing must survive millis rollover");

static_assert(MpuYaw::isSupportedId(0x68), "MPU6050 must be supported");
static_assert(MpuYaw::isSupportedId(0x70), "MPU6500 must be supported");
static_assert(!MpuYaw::isSupportedId(0x00), "Zero ID must stay blocked");
static_assert(!MpuYaw::isSupportedId(0xFF), "All-ones ID must stay blocked");
static_assert(!MpuYaw::isSupportedId(0x12), "Unimplemented IMU must stay blocked");
