#include "dw3000_shared_functions.h"
#include "UwbDsTwrProtocol.h"

extern "C" {
extern const struct dwt_probe_s dw3000_probe_interf;
}

SPISettings _fastSPI(HELTEC_DW3000_SPI_FAST_HZ, MSBFIRST, SPI_MODE0);

dwt_txconfig_t txconfig_options = {
  0x34,
  0x0FDFDFDFD, // JOHNNA FULL POWER
  0
};

void spiBegin(uint8_t, uint8_t) {
  Br101Dw3000Port::instance().begin();
  dwt_probe(const_cast<dwt_probe_s*>(&dw3000_probe_interf));
}

void spiSelect(uint8_t) {
}

void spiWakeup() {
  Br101Dw3000Port::instance().wakeup();
}

dwt_config_t makerfabsDefaultConfig() {
  return UwbDsTwr::defaultConfig();
}

dwt_txconfig_t makerfabsDefaultTxConfig() {
  return txconfig_options;
}

