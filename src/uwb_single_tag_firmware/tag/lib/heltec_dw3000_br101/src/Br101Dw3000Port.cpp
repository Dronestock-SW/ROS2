#include "Br101Dw3000Port.h"
#include "UwbDsTwrProtocol.h"
#include "log.h"

#ifdef UWB_SINGLE_TAG
// ESP32-S3 SPI3 is dedicated to DW3000; the Heltec SX1262 owns global SPI (SPI2).
static SPIClass singleTagUwbSpi(HSPI);
#endif

Br101Dw3000Port& Br101Dw3000Port::instance() {
  static Br101Dw3000Port port;
  return port;
}

Br101Dw3000Port::Br101Dw3000Port()
#ifdef UWB_SINGLE_TAG
: spi_(&singleTagUwbSpi),
#else
: spi_(&SPI),
#endif
  ready_(false),
  fastRate_(false),
  irqAttached_(false),
  irqEnabled_(false),
  mutex_(portMUX_INITIALIZER_UNLOCKED),
  slowSettings_(HELTEC_DW3000_SPI_SLOW_HZ, MSBFIRST, SPI_MODE0),
  fastSettings_(HELTEC_DW3000_SPI_FAST_HZ, MSBFIRST, SPI_MODE0) {}

bool Br101Dw3000Port::begin(const HeltecDw3000Pins& pins) {
  pins_ = pins;

  pinMode(pins_.cs, OUTPUT);
  digitalWrite(pins_.cs, HIGH);

  if (pins_.irq >= 0) {
    pinMode(pins_.irq, INPUT_PULLDOWN);
  }

  if (pins_.rst >= 0) {
    pinMode(pins_.rst, INPUT_PULLUP);
  }

  if (pins_.wakeup >= 0) {
    pinMode(pins_.wakeup, OUTPUT);
    digitalWrite(pins_.wakeup, LOW);
  }

  spi_->end();
  spi_->begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs);

  ready_ = true;
  fastRate_ = false;
  return true;
}

void Br101Dw3000Port::end() {
  if (irqAttached_ && pins_.irq >= 0) {
    detachInterrupt(digitalPinToInterrupt(pins_.irq));
  }
  irqAttached_ = false;
  irqEnabled_ = false;
  ready_ = false;
  spi_->end();
}

void Br101Dw3000Port::hardReset() {
  if (pins_.rst < 0) return;
  pinMode(pins_.rst, OUTPUT);
  digitalWrite(pins_.rst, LOW);
  delayMicroseconds(HELTEC_DW3000_RESET_LOW_US);
  // DW3000 RSTn is bidirectional: release it; never drive it high.
  pinMode(pins_.rst, INPUT);
  delay(HELTEC_DW3000_RESET_RECOVERY_MS);
  delay(HELTEC_DW3000_BOOT_WAIT_MS);
}

void Br101Dw3000Port::wakeup() {
  if (pins_.wakeup >= 0) {
    digitalWrite(pins_.wakeup, HIGH);
    delayMicroseconds(HELTEC_DW3000_WAKEUP_PULSE_US);
    digitalWrite(pins_.wakeup, LOW);
  } else {
    begin();
    digitalWrite(pins_.cs, LOW);
    delayMicroseconds(HELTEC_DW3000_WAKEUP_PULSE_US);
    digitalWrite(pins_.cs, HIGH);
  }
  delay(HELTEC_DW3000_POST_WAKE_MS);
}

void Br101Dw3000Port::wakeupPinLow() {
  if (pins_.wakeup >= 0) {
    digitalWrite(pins_.wakeup, LOW);
  }
}

void IRAM_ATTR Br101Dw3000Port::irqThunk() {
  Br101Dw3000Port::instance().handleIrq();
}

void IRAM_ATTR Br101Dw3000Port::handleIrq() {
  if (!irqEnabled_ || pins_.irq < 0) return;
  while (digitalRead(pins_.irq)) {
    dwt_isr();
  }
}

bool Br101Dw3000Port::initInterrupt() {
  if (pins_.irq < 0) return false;
  detachInterrupt(digitalPinToInterrupt(pins_.irq));
  attachInterrupt(digitalPinToInterrupt(pins_.irq), irqThunk, RISING);
  irqAttached_ = true;
  irqEnabled_ = true;
  return true;
}

void Br101Dw3000Port::enableInterrupt() {
  irqEnabled_ = true;
}

void Br101Dw3000Port::disableInterrupt() {
  irqEnabled_ = false;
}

bool Br101Dw3000Port::interruptEnabled() const {
  return irqEnabled_;
}

int Br101Dw3000Port::irqState() const {
  return pins_.irq >= 0 ? digitalRead(pins_.irq) : 0;
}

void Br101Dw3000Port::setFastRate(bool fast) {
  fastRate_ = fast;
}

bool Br101Dw3000Port::ready() const {
  return ready_;
}

decaIrqStatus_t Br101Dw3000Port::mutexOn() {
  portENTER_CRITICAL(&mutex_);
  return 0;
}

void Br101Dw3000Port::mutexOff(decaIrqStatus_t) {
  portEXIT_CRITICAL(&mutex_);
}

int32_t Br101Dw3000Port::spiWrite(uint16_t headerLength, const uint8_t* headerBuffer,
                                  uint16_t bodyLength, const uint8_t* bodyBuffer) {
  if (!ready_) return DWT_ERROR;
  decaIrqStatus_t s = mutexOn();
  spi_->beginTransaction(fastRate_ ? fastSettings_ : slowSettings_);
  digitalWrite(pins_.cs, LOW);
  const uint32_t length = static_cast<uint32_t>(headerLength) + bodyLength;
  if (fastRate_ && length <= 64U) {
    // Use one hardware transfer for short register/frame transactions. The
    // ESP32 HAL loads 32-bit words, so pad and align the complete buffer.
    alignas(4) uint8_t bytes[64] = {};
    if (headerBuffer) memcpy(bytes, headerBuffer, headerLength);
    if (bodyBuffer) memcpy(bytes + headerLength, bodyBuffer, bodyLength);
    spi_->transferBytes(bytes, nullptr, length);
  } else {
    for (uint16_t i = 0; i < headerLength; ++i)
      spi_->transfer(headerBuffer ? headerBuffer[i] : 0x00);
    for (uint16_t i = 0; i < bodyLength; ++i)
      spi_->transfer(bodyBuffer ? bodyBuffer[i] : 0x00);
  }
  digitalWrite(pins_.cs, HIGH);
  spi_->endTransaction();
  mutexOff(s);
  return DWT_SUCCESS;
}

int32_t Br101Dw3000Port::spiRead(uint16_t headerLength, uint8_t* headerBuffer,
                                 uint16_t readLength, uint8_t* readBuffer) {
  if (!ready_) return DWT_ERROR;
  decaIrqStatus_t s = mutexOn();
  spi_->beginTransaction(fastRate_ ? fastSettings_ : slowSettings_);
  digitalWrite(pins_.cs, LOW);
  const uint32_t length = static_cast<uint32_t>(headerLength) + readLength;
  if (fastRate_ && length <= 64U) {
    alignas(4) uint8_t tx[64] = {}, rx[64] = {};
    if (headerBuffer) memcpy(tx, headerBuffer, headerLength);
    spi_->transferBytes(tx, rx, length);
    if (readLength) memcpy(readBuffer, rx + headerLength, readLength);
  } else {
    for (uint16_t i = 0; i < headerLength; ++i)
      spi_->transfer(headerBuffer ? headerBuffer[i] : 0x00);
    for (uint16_t i = 0; i < readLength; ++i)
      readBuffer[i] = spi_->transfer(0x00);
  }
  digitalWrite(pins_.cs, HIGH);
  spi_->endTransaction();
  mutexOff(s);
  return DWT_SUCCESS;
}

int32_t Br101Dw3000Port::spiWriteCrc(uint16_t headerLength, const uint8_t* headerBuffer,
                                     uint16_t bodyLength, const uint8_t* bodyBuffer,
                                     uint8_t) {
  return spiWrite(headerLength, headerBuffer, bodyLength, bodyBuffer);
}

SPIClass& Br101Dw3000Port::spi() { return *spi_; }
const HeltecDw3000Pins& Br101Dw3000Port::pins() const { return pins_; }

namespace Br101Dw3000 {

bool begin(const HeltecDw3000Pins& pins) {
  return Br101Dw3000Port::instance().begin(pins);
}

bool resetAndProbe(uint32_t* devid) {
  auto& port = Br101Dw3000Port::instance();
  port.hardReset();
  port.setFastRate(false);
  uint32_t id = dwt_readdevid();
  if (devid) *devid = id;
  return id == 0xDECA0302 || id == 0xDECA0312;
}

bool initialise(const dwt_config_t& cfg, const dwt_txconfig_t* txcfg, bool withInterrupt) {
  auto& port = Br101Dw3000Port::instance();
  port.setFastRate(false);
  if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
    return false;
  }
  dwt_configure(const_cast<dwt_config_t*>(&cfg));
  if (txcfg) {
    dwt_configuretxrf(const_cast<dwt_txconfig_t*>(txcfg));
  }
  dwt_setrxantennadelay(UwbDsTwr::kRxAntDly);
  dwt_settxantennadelay(UwbDsTwr::kTxAntDly);
  port.setFastRate(true);
  if (withInterrupt) {
    port.initInterrupt();
  }
  return true;
}

void spiSlow() { Br101Dw3000Port::instance().setFastRate(false); }
void spiFast() { Br101Dw3000Port::instance().setFastRate(true); }

void printStatus(const char* tag) {
  uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
  Serial.printf("[%s] SYS_STATUS=0x%08lX irq=%d\n", tag, (unsigned long)st, Br101Dw3000Port::instance().irqState());
}

} // namespace Br101Dw3000

extern "C" {

int dw3000_hw_init(void) {
  return Br101Dw3000Port::instance().begin() ? 0 : -1;
}

int dw3000_hw_init_interrupt(void) {
  return Br101Dw3000Port::instance().initInterrupt() ? 0 : -1;
}

void dw3000_hw_fini(void) {
  Br101Dw3000Port::instance().end();
}

void dw3000_hw_reset(void) {
  Br101Dw3000Port::instance().hardReset();
}

void dw3000_hw_wakeup(void) {
  Br101Dw3000Port::instance().wakeup();
}

void dw3000_hw_wakeup_pin_low(void) {
  Br101Dw3000Port::instance().wakeupPinLow();
}

void dw3000_hw_interrupt_enable(void) {
  Br101Dw3000Port::instance().enableInterrupt();
}

void dw3000_hw_interrupt_disable(void) {
  Br101Dw3000Port::instance().disableInterrupt();
}

bool dw3000_hw_interrupt_is_enabled(void) {
  return Br101Dw3000Port::instance().interruptEnabled();
}

int dw3000_spi_init(void) {
  return Br101Dw3000Port::instance().begin() ? 0 : -1;
}

void dw3000_spi_fini(void) {
  Br101Dw3000Port::instance().end();
}

void dw3000_spi_wakeup(void) {
  Br101Dw3000Port::instance().wakeup();
}

void dw3000_spi_speed_slow(void) {
  Br101Dw3000Port::instance().setFastRate(false);
}

void dw3000_spi_speed_fast(void) {
  Br101Dw3000Port::instance().setFastRate(true);
}

int32_t dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
                        uint16_t readLength, uint8_t* readBuffer) {
  return Br101Dw3000Port::instance().spiRead(headerLength, headerBuffer, readLength, readBuffer);
}

int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
                         uint16_t bodyLength, const uint8_t* bodyBuffer) {
  return Br101Dw3000Port::instance().spiWrite(headerLength, headerBuffer, bodyLength, bodyBuffer);
}

int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
                             uint16_t bodyLength, const uint8_t* bodyBuffer,
                             uint8_t crc8) {
  return Br101Dw3000Port::instance().spiWriteCrc(headerLength, headerBuffer, bodyLength, bodyBuffer, crc8);
}

void dw3000_spi_trace_output(void) {}

}

