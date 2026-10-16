#pragma once
// Host adapters only. Production SingleTagRuntime.cpp owns all tested decisions.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace RuntimeFake {
inline uint64_t us=1000000;
inline unsigned ticks=0,stopAfter=0;
inline std::function<void()> onTick;
struct Stop {};
inline void check(bool ok,const char* reason) { if(!ok) throw std::runtime_error(reason); }
}
inline int64_t esp_timer_get_time() { return static_cast<int64_t>(RuntimeFake::us); }
inline int esp_reset_reason() { return 1; }
struct FakeESP {
  uint32_t getFreeHeap() { return UINT32_MAX; }
  uint32_t getMinFreeHeap() { return UINT32_MAX; }
};
inline FakeESP ESP;
struct FakeSerial {
  std::string output;
  size_t allowance=256;
  void setTxBufferSize(unsigned) {}
  void setRxBufferSize(unsigned) {}
  void begin(unsigned baud) { RuntimeFake::check(baud==921600,"UART baud changed"); }
  size_t read(uint8_t*,size_t) { return 0; }
  size_t availableForWrite() { return allowance; }
  size_t write(const uint8_t* bytes,size_t count) {
    RuntimeFake::check(count<=allowance && count<=256,"unbounded UART write");
    output.append(reinterpret_cast<const char*>(bytes),count); return count;
  }
};
inline FakeSerial Serial;
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
constexpr int pdTRUE=1, pdFALSE=0;
#define pdMS_TO_TICKS(ms) (ms)
struct StaticQueue_t {
  size_t capacity=0,itemSize=0;
  std::deque<std::vector<uint8_t>> items;
};
using QueueHandle_t=StaticQueue_t*;
inline QueueHandle_t xQueueCreateStatic(size_t capacity,size_t itemSize,uint8_t*,StaticQueue_t* queue) {
  queue->capacity=capacity; queue->itemSize=itemSize; queue->items.clear(); return queue;
}
inline int xQueueSend(QueueHandle_t queue,const void* item,unsigned wait) {
  RuntimeFake::check(wait==0,"RF producer must never wait for a queue");
  if(queue->items.size()>=queue->capacity) return pdFALSE;
  const auto* bytes=static_cast<const uint8_t*>(item);
  queue->items.emplace_back(bytes,bytes+queue->itemSize); return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t queue,void* item,unsigned wait) {
  RuntimeFake::check(wait==0,"queue consumer must not block");
  if(queue->items.empty()) return pdFALSE;
  std::memcpy(item,queue->items.front().data(),queue->itemSize); queue->items.pop_front(); return pdTRUE;
}
inline void xQueueOverwrite(QueueHandle_t queue,const void* item) {
  RuntimeFake::check(queue->capacity==1,"overwrite is only for latest RAW");
  queue->items.clear(); xQueueSend(queue,item,0);
}
inline uint32_t uxQueueMessagesWaiting(QueueHandle_t queue) { return queue->items.size(); }
inline void xTaskCreatePinnedToCore(void (*)(void*),const char*,unsigned,void*,unsigned,void*,int) {}
inline void vTaskDelay(unsigned milliseconds) {
  RuntimeFake::us+=1000ULL*milliseconds;
  if(RuntimeFake::onTick) RuntimeFake::onTick();
  if(++RuntimeFake::ticks>=RuntimeFake::stopAfter) throw RuntimeFake::Stop{};
}
constexpr int UART_NUM_0=0,ESP_OK=0;
inline int uart_wait_tx_done(int,unsigned timeout) {
  RuntimeFake::check(timeout==0,"UART drain check must not block"); return ESP_OK;
}
struct cJSON {
  cJSON* child=nullptr;
  cJSON* next=nullptr;
  char* string=nullptr;
  char* valuestring=nullptr;
  double valuedouble=0;
};
// Input parser is not under test in this adapter; Serial.read returns no input.
inline cJSON* cJSON_GetObjectItemCaseSensitive(cJSON*,const char*) { return nullptr; }
inline bool cJSON_IsNumber(cJSON*) { return false; }
inline bool cJSON_IsString(cJSON*) { return false; }
inline bool cJSON_IsObject(cJSON*) { return false; }
inline bool cJSON_IsArray(cJSON*) { return false; }
inline bool cJSON_IsBool(cJSON*) { return false; }
inline bool cJSON_IsTrue(cJSON*) { return false; }
inline cJSON* cJSON_ParseWithOpts(const char*,const char**,bool) { return nullptr; }
inline void cJSON_Delete(cJSON*) {}
namespace JetsonTagDisplay { inline bool ready() { return true; } }
