#pragma once

#include <Arduino.h>
#include "Br101Dw3000Bridge.h"

namespace UwbDsTwr {

static constexpr uint16_t kPanId = 0xDECA;
static constexpr uint16_t kTagShortAddr = 0x0005;
static constexpr uint16_t kAnchorA1 = 0x00A1;
static constexpr uint16_t kAnchorA2 = 0x00A2;
static constexpr uint16_t kAnchorA3 = 0x00A3;

static constexpr uint16_t kTxAntDly = 16385;
static constexpr uint16_t kRxAntDly = 16385;

static constexpr uint8_t kFuncPoll   = 0xE0;
static constexpr uint8_t kFuncResp   = 0xE1;
static constexpr uint8_t kFuncFinal  = 0xE2;
static constexpr uint8_t kFuncReport = 0xE3;

static constexpr uint8_t kCommonLen = 10;
static constexpr uint8_t kSeqIdx = 2;
static constexpr uint8_t kRespPollRxTsIdx = 10;
static constexpr uint8_t kRespRespTxTsIdx = 14;
static constexpr uint8_t kFinalPollTxTsIdx = 10;
static constexpr uint8_t kFinalRespRxTsIdx = 14;
static constexpr uint8_t kFinalFinalTxTsIdx = 18;
static constexpr uint8_t kReportDistanceMmIdx = 10;
static constexpr uint8_t kReportDistanceStatusIdx = 14;
static constexpr uint8_t kReportPollRxTsIdx = 10;
static constexpr uint8_t kReportRespTxTsIdx = 14;
static constexpr uint8_t kReportFinalRxTsIdx = 18;
static constexpr uint8_t kReportStatusIdx = 22;
static constexpr uint8_t kReportLen = 23;

static constexpr uint32_t kPollRxToRespTxDlyUus = 2400;
static constexpr uint32_t kFinalRxTimeoutUus = 20000;
static constexpr uint32_t kFinalRxToReportTxDlyUus = 2400;

static constexpr uint32_t kPollTxToRespRxDlyUus = 500;
static constexpr uint32_t kFlightRespRxTimeoutUus = 12000;
static constexpr uint32_t kFlightReportRxTimeoutUus = 12000;
static constexpr uint32_t kFlightWaitTimeoutMs = 15;
static constexpr uint32_t kDebugRespRxTimeoutUus = 180000;
static constexpr uint32_t kDebugReportRxTimeoutUus = 180000;
static constexpr uint32_t kDebugWaitTimeoutMs = 360;
// Deprecated: do not use the generic names in flight code.
static constexpr uint32_t kRespRxTimeoutUus = kFlightRespRxTimeoutUus;
static constexpr uint32_t kRespRxToFinalTxDlyUus = 16000;
static constexpr uint32_t kFinalTxToReportRxDlyUus = 500;
static constexpr uint32_t kReportRxTimeoutUus = kFlightReportRxTimeoutUus;

static constexpr uint32_t kWaitTimeoutMs = kFlightWaitTimeoutMs;
static constexpr uint8_t kRxBufLen = 32;

void writeShortAddress(uint8_t* frame, uint16_t destAddr, uint16_t srcAddr);
uint16_t readAddress(const uint8_t* frame, uint8_t lowIndex);
bool frameMatches(const uint8_t* frame, uint16_t frameLen, uint8_t funcCode, uint16_t destAddr, uint16_t srcAddr);

void writeTs40(uint8_t* buffer, uint64_t ts);
uint64_t readTs40(const uint8_t* buffer);

void writeDistanceMm(uint8_t* frame, uint32_t distanceMm, bool valid);
uint32_t readDistanceMm(const uint8_t* frame);

uint32_t waitForStatus(uint32_t mask, uint32_t timeoutMs);
bool readIncomingFrame(uint8_t* buffer, uint16_t bufferSize, uint16_t& frameLen);
void clearRxState();
void clearTxState();
void armImmediateRx();

float computeDistanceFromFinal(const uint8_t* finalMsg, uint64_t pollRxTs, uint64_t respTxTs, uint64_t finalRxTs);

dwt_config_t defaultConfig();
dwt_txconfig_t defaultTxConfig();

} // namespace UwbDsTwr

