#ifndef MEGA_MPU_NAV_CONFIG_H
#define MEGA_MPU_NAV_CONFIG_H
#include <stdint.h>
namespace cfg {
static const uint16_t GYRO_POLL_MS = 5;
static const uint16_t GYRO_STALE_MS = 50;
static const uint16_t GYRO_CAL_SAMPLES = 500;
static const uint32_t GYRO_CAL_TIMEOUT_MS = 7000;
static const float GYRO_Z_SIGN = 1.0f; // Positive yaw should mean counterclockwise.
// Set an individual sign to -1 if that wheel spins backward for positive PWM.
static const int8_t FL_SIGN = 1;
static const int8_t FR_SIGN = 1;
static const int8_t BL_SIGN = -1;
static const int8_t BR_SIGN = 1;
static const float YAW_CORRECTION_SIGN = 1.0f; // Same convention as FastRoboCup.
static const float HOLD_KP = 2.5f;
static const float HOLD_KD = 0.0f;
}
#endif
