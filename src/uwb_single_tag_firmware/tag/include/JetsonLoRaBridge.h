#pragma once

#include <stdint.h>

namespace JetsonLoRaBridge {

// UWB and the onboard SX1262 share the global Arduino SPI object. Call this
// before every DW3000 transaction after the LoRa service has run.
void selectUwbSpi();

bool begin();
bool ready();

struct Diagnostics {
  uint32_t txAttempts = 0;
  uint32_t txDone = 0;
  uint32_t txTimeouts = 0;
  uint32_t watchdogTimeouts = 0;
  uint32_t recoveries = 0;
  uint32_t rxPackets = 0;
  uint32_t rxErrors = 0;
  uint32_t lastTxDoneMs = 0;
  uint32_t lastRxMs = 0;
  bool hwVerified = false;
  bool txBusy = false;
};

// TX done is a local radio completion event, NOT a ground-station ACK.
Diagnostics diagnostics();

// Accepts newline-delimited Jetson pose messages from USB serial and forwards
// LoRa downlink packets back to Jetson as typed JSON lines.
void serviceSerialInput();
void service(uint32_t uwbCycleSequence);

}  // namespace JetsonLoRaBridge
