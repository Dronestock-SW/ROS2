#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "Br101Dw3000Bridge.h"

class Br101Dw3000Port {
public:
  static Br101Dw3000Port& instance();

  bool begin(const HeltecDw3000Pins& pins = HeltecDw3000Pins());
  void end();

  void hardReset();
  void wakeup();
  void wakeupPinLow();

  bool initInterrupt();
  void enableInterrupt();
  void disableInterrupt();
  bool interruptEnabled() const;
  int irqState() const;

  void setFastRate(bool fast);
  bool ready() const;

  int32_t spiWrite(uint16_t headerLength, const uint8_t* headerBuffer,
                   uint16_t bodyLength, const uint8_t* bodyBuffer);
  int32_t spiRead(uint16_t headerLength, uint8_t* headerBuffer,
                  uint16_t readLength, uint8_t* readBuffer);
  int32_t spiWriteCrc(uint16_t headerLength, const uint8_t* headerBuffer,
                      uint16_t bodyLength, const uint8_t* bodyBuffer,
                      uint8_t crc8);

  decaIrqStatus_t mutexOn();
  void mutexOff(decaIrqStatus_t s);

  SPIClass& spi();
  const HeltecDw3000Pins& pins() const;

private:
  Br101Dw3000Port();
  static void IRAM_ATTR irqThunk();
  void IRAM_ATTR handleIrq();

  SPIClass* spi_;
  HeltecDw3000Pins pins_;
  bool ready_;
  bool fastRate_;
  bool irqAttached_;
  bool irqEnabled_;
  portMUX_TYPE mutex_;
  SPISettings slowSettings_;
  SPISettings fastSettings_;
};

namespace Br101Dw3000 {
  bool begin(const HeltecDw3000Pins& pins = HeltecDw3000Pins());
  bool resetAndProbe(uint32_t* devid = nullptr);
  bool initialise(const dwt_config_t& cfg, const dwt_txconfig_t* txcfg = nullptr, bool withInterrupt = false);
  void spiSlow();
  void spiFast();
  void printStatus(const char* tag);
}

