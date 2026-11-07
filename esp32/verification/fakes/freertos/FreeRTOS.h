#pragma once
#include <stdint.h>
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdPASS 1
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
