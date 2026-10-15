#pragma once

#include "Br101Dw3000Config.h"

extern "C" {
#include "../vendor/br101/platform/dw3000.h"
#include "../vendor/br101/platform/dw3000_hw.h"
#include "../vendor/br101/platform/dw3000_spi.h"
#include "dw3000_deca_regs.h"
#include "dw3000_deca_vals.h"
}

static inline uint32_t dwt_read32bitreg(uint32_t address) {
  return dwt_read_reg(address);
}

static inline void dwt_write32bitreg(uint32_t address, uint32_t data) {
  dwt_write_reg(address, data);
}

