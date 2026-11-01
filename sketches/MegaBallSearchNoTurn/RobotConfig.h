#ifndef MEGA_MPU_NAV_CONFIG_H
#define MEGA_MPU_NAV_CONFIG_H
#include <stdint.h>
namespace cfg {
enum Mode { IDLE, HEADING_HOLD, TRANSLATE, TURN, NAVIGATION };
static const uint16_t GYRO_POLL_MS = 5;
static const uint16_t GYRO_STALE_MS = 50;
static const uint16_t GYRO_CAL_SAMPLES = 500;
static const uint32_t GYRO_CAL_TIMEOUT_MS = 7000;
static const float GYRO_Z_SIGN = 1.0f; // Positive yaw should mean counterclockwise.
// Set an individual sign to -1 if that wheel spins backward for positive PWM.
static const int8_t FL_SIGN = 1;
static const int8_t FR_SIGN = 1;
static const int8_t BL_SIGN = 1;
static const int8_t BR_SIGN = 1;
static const float YAW_CORRECTION_SIGN = 1.0f; // Same convention as FastRoboCup.
static const float HOLD_KP = 2.5f;
static const float HOLD_KD = 0.0f;
static const float MAX_HOLD_PWM = 40.0f;
static const float TURN_KP = 2.5f;
static const float TURN_KD = 0.8f;
static const float MIN_TURN_PWM = 30.0f;
static const float MAX_TURN_PWM = 60.0f;
static const uint16_t CONTROL_MS = 20;
static const uint16_t REPORT_MS = 200;
static const uint16_t MOVE_MS = 1500;
static const uint16_t PAUSE_MS = 500;
static const uint16_t HOLD_MS = 3000;
static const uint16_t TURN_TIMEOUT_MS = 6000;
static const uint8_t BASE_PWM = 80;
static const uint8_t MAX_PWM = 120;
}
#endif
