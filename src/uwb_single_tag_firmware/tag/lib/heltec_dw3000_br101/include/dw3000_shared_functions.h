#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "Br101Dw3000Bridge.h"
#include "Br101Dw3000Port.h"

extern SPISettings _fastSPI;
extern dwt_txconfig_t txconfig_options;

void spiBegin(uint8_t irq, uint8_t rst);
void spiSelect(uint8_t cs);
void spiWakeup();
dwt_config_t makerfabsDefaultConfig();
dwt_txconfig_t makerfabsDefaultTxConfig();

