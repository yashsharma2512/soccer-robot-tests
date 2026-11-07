#pragma once
#include <vector>
#include <cstring>
#include "FreeRTOS.h"
struct FakeQueue { std::vector<unsigned char> bytes; bool present=false; };
typedef FakeQueue *QueueHandle_t;
inline QueueHandle_t xQueueCreate(int length,unsigned size) {
  if(length!=1) return nullptr;
  auto q=new FakeQueue; q->bytes.resize(size); return q;
}
inline int xQueueOverwrite(QueueHandle_t q,const void *data) { memcpy(q->bytes.data(),data,q->bytes.size()); q->present=true; return pdPASS; }
inline int xQueueReceive(QueueHandle_t q,void *data,int) {
  if(!q->present) return 0;
  memcpy(data,q->bytes.data(),q->bytes.size()); q->present=false; return pdTRUE;
}
