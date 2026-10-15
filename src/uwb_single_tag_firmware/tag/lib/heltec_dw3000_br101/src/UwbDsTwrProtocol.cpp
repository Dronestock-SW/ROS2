#include "UwbDsTwrProtocol.h"
#include "dw3000_shared_functions.h"

namespace UwbDsTwr {

namespace {
constexpr double kSpeedOfLight = 299702547.0;
}

void writeShortAddress(uint8_t* frame, uint16_t destAddr, uint16_t srcAddr) {
  frame[3] = static_cast<uint8_t>(kPanId & 0xFF);
  frame[4] = static_cast<uint8_t>(kPanId >> 8);
  frame[5] = static_cast<uint8_t>(destAddr & 0xFF);
  frame[6] = static_cast<uint8_t>(destAddr >> 8);
  frame[7] = static_cast<uint8_t>(srcAddr & 0xFF);
  frame[8] = static_cast<uint8_t>(srcAddr >> 8);
}

uint16_t readAddress(const uint8_t* frame, uint8_t lowIndex) {
  return static_cast<uint16_t>(frame[lowIndex]) | (static_cast<uint16_t>(frame[lowIndex + 1]) << 8);
}

bool frameMatches(const uint8_t* frame, uint16_t frameLen, uint8_t funcCode, uint16_t destAddr, uint16_t srcAddr) {
  if (frameLen < kCommonLen) return false;
  if (frame[0] != 0x41 || frame[1] != 0x88) return false;
  if (frame[3] != static_cast<uint8_t>(kPanId & 0xFF) || frame[4] != static_cast<uint8_t>(kPanId >> 8)) return false;
  if (readAddress(frame, 5) != destAddr) return false;
  if (readAddress(frame, 7) != srcAddr) return false;
  return frame[9] == funcCode;
}

void writeTs40(uint8_t* buffer, uint64_t ts) {
  for (int i = 0; i < 5; ++i) buffer[i] = static_cast<uint8_t>(ts >> (i * 8));
}

uint64_t readTs40(const uint8_t* buffer) {
  uint64_t ts = 0;
  for (int i = 0; i < 5; ++i) ts |= (static_cast<uint64_t>(buffer[i]) << (i * 8));
  return ts;
}

void writeDistanceMm(uint8_t* frame, uint32_t distanceMm, bool valid) {
  frame[kReportDistanceMmIdx + 0] = static_cast<uint8_t>(distanceMm & 0xFF);
  frame[kReportDistanceMmIdx + 1] = static_cast<uint8_t>((distanceMm >> 8) & 0xFF);
  frame[kReportDistanceMmIdx + 2] = static_cast<uint8_t>((distanceMm >> 16) & 0xFF);
  frame[kReportDistanceMmIdx + 3] = static_cast<uint8_t>((distanceMm >> 24) & 0xFF);
  frame[kReportDistanceStatusIdx] = valid ? 1 : 0;
  frame[kReportDistanceStatusIdx + 1] = 0;
}

uint32_t readDistanceMm(const uint8_t* frame) {
  return static_cast<uint32_t>(frame[kReportDistanceMmIdx + 0])
       | (static_cast<uint32_t>(frame[kReportDistanceMmIdx + 1]) << 8)
       | (static_cast<uint32_t>(frame[kReportDistanceMmIdx + 2]) << 16)
       | (static_cast<uint32_t>(frame[kReportDistanceMmIdx + 3]) << 24);
}

uint32_t waitForStatus(uint32_t mask, uint32_t timeoutMs) {
  const unsigned long startMs = millis();
  while (true) {
    uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
    if (st & mask) return st;
    if (timeoutMs > 0 && millis() - startMs >= timeoutMs) return 0;
    delay(1);
  }
}

bool readIncomingFrame(uint8_t* buffer, uint16_t bufferSize, uint16_t& frameLen) {
  frameLen = static_cast<uint16_t>(dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_STD_RXFLEN_MASK);
  if (frameLen == 0 || frameLen > bufferSize) return false;
  dwt_readrxdata(buffer, frameLen, 0);
  return true;
}

void clearRxState() {
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
}

void clearTxState() {
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
}

void armImmediateRx() {
  clearRxState();
  clearTxState();
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

float computeDistanceFromFinal(const uint8_t* finalMsg, uint64_t pollRxTs, uint64_t respTxTs, uint64_t finalRxTs) {
  const uint64_t pollTxTs = readTs40(finalMsg + kFinalPollTxTsIdx);
  const uint64_t respRxTs = readTs40(finalMsg + kFinalRespRxTsIdx);
  const uint64_t finalTxTs = readTs40(finalMsg + kFinalFinalTxTsIdx);

  const double Ra = static_cast<double>(respRxTs - pollTxTs);
  const double Rb = static_cast<double>(finalRxTs - respTxTs);
  const double Da = static_cast<double>(finalTxTs - respRxTs);
  const double Db = static_cast<double>(respTxTs - pollRxTs);
  const double tof_dtu = ((Ra * Rb) - (Da * Db)) / (Ra + Rb + Da + Db);
  const double distance_m = tof_dtu * DWT_TIME_UNITS * kSpeedOfLight;
  return static_cast<float>(distance_m);
}

dwt_config_t defaultConfig() {
  dwt_config_t cfg = {
    5,
    DWT_PLEN_1024,
    DWT_PAC16,
    9,
    9,
    DWT_SFD_DW_8,
    DWT_BR_850K,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (1024 + 1 + 8 - 16),    // = 1017
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
  };
  return cfg;
}

dwt_txconfig_t defaultTxConfig() {
  return txconfig_options;
}

} // namespace UwbDsTwr

