#pragma once
#include <stdint.h>

namespace JetsonTagDisplay {

// Starts the display worker. ready() becomes true after the panel ACKs.
bool begin();
bool ready();

// Nonblocking mailbox update only: no I2C, drawing, allocation, delay or RTOS
// queue wait. Full, on-time A1-A4 ranging within 1500 ms means connected.
// The worker expires that state independently, even without further updates.
void render(bool uwbReady, uint8_t validMask, bool force = false);

}  // namespace JetsonTagDisplay
