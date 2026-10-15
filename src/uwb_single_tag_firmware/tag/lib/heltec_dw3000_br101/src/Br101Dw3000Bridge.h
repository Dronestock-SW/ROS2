#pragma once

#include "Br101Dw3000Config.h"

extern "C" {
#include "../vendor/br101/platform/dw3000.h"
#include "../vendor/br101/platform/dw3000_hw.h"
#include "../vendor/br101/platform/dw3000_spi.h"
#include "dw3000_deca_regs.h"
#include "dw3000_deca_vals.h"
}

// Arduino sketches often use the older Decawave helper names directly.
// Map those to the BR101 driver's public register access functions.
static inline uint32_t dwt_read32bitreg(uint32_t address) {
  return dwt_read_reg(address);
}

static inline void dwt_write32bitreg(uint32_t address, uint32_t data) {
  dwt_write_reg(address, data);
}

// Older Decawave examples sometimes use a generic RX frame length mask name.
#ifndef RXFLEN_MASK
#define RXFLEN_MASK RX_FINFO_STD_RXFLEN_MASK
#endif

// Some examples omit the segment argument and assume the default compatibility path.
static inline void dwt_readrxtimestamp(uint8_t* timestamp) {
  dwt_readrxtimestamp(timestamp, DWT_COMPAT_NONE);
}

static inline uint32_t dwt_readrxtimestamplo32() {
  return dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
}

// Compatibility helpers used by older DW3000 Arduino examples/sketches.
// The BR101 driver exposes timestamp readers through the Decawave API,
// but not the legacy convenience wrappers.
static inline uint64_t get_rx_timestamp_u64() {
  uint8_t ts[5] = {0};
  dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
  uint64_t value = 0;
  for (int i = 4; i >= 0; --i) {
    value = (value << 8) | ts[i];
  }
  return value;
}

static inline uint64_t get_tx_timestamp_u64() {
  uint8_t ts[5] = {0};
  dwt_readtxtimestamp(ts);
  uint64_t value = 0;
  for (int i = 4; i >= 0; --i) {
    value = (value << 8) | ts[i];
  }
  return value;
}

