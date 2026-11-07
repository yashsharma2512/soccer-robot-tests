#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
inline int xTaskCreatePinnedToCore(void (*)(void *),const char *,unsigned,void *,unsigned,TaskHandle_t *,int) { return pdPASS; }
inline void vTaskDelay(unsigned ms) { delay(ms); }
