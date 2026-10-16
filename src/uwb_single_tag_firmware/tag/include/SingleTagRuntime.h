#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../../common/SingleTagProtocol.h"
#include "SingleTagDiagnostics.h"

#ifndef SINGLE_TAG_RATE_HZ
#define SINGLE_TAG_RATE_HZ 50
#endif
#if SINGLE_TAG_RATE_HZ != 40 && SINGLE_TAG_RATE_HZ != 50
#error "Only explicitly tested 40 and 50 Hz candidates are supported"
#endif

namespace SingleTag {
constexpr uint32_t kPeriodUs = 1000000U/SINGLE_TAG_RATE_HZ;
#if SINGLE_TAG_BENCH_CONTROLS
constexpr const char* kVersion = "uwb-single-tag-v2.0-r5-recovery-bench-20261016";
#else
constexpr const char* kVersion = "uwb-single-tag-v2.0-r5-recovery-20261016";
#endif
constexpr const char* kLayout = "warehouse-rectangle-6p3x4p6-z0p15-20261007";
enum Failure : uint8_t {
  Ok=0, NotReady=1, PollStart=2, PollTimeout=3, ResponseTimeout=4,
  ResponseInvalid=5, FinalStart=6, FinalTimeout=7, ReportTimeout=8,
  ReportInvalid=9, TofInvalid=10, RangeInvalid=11, Deadline=12, ClockInvalid=13
};
struct Sample {
  float distance;
  uint64_t timeUs;
  uint32_t transaction;
  uint32_t firstTransaction;
  uint16_t rfMapSpanUs;
  uint8_t failure, attempts;
  uint8_t firstFailure, cause, firstCause;
};
struct Cycle {
  uint32_t boot, session, seq, skipped, recoveries;
  uint64_t startUs, endUs;
  uint8_t mask;
  bool ready, deadlineMissed;
  Sample samples[4];
};
struct Pose {
  bool received, fix;
  uint32_t boot, session, seq, sourceSeq, hostSession;
  uint64_t sampleUs;
  float x, y;
  uint8_t quality;
  char reason[32];
};
struct RadioStatus {
  bool ready, busy, lastTxFix;
  uint8_t lastTxLength;
  uint32_t attempts, done, timeouts, watchdog, recoveries, rx, rxErrors;
  uint32_t lastDoneMs, lastRxMs, serviceMaxUs;
};
void beginRuntime(uint32_t boot);
void publishCycle(const Cycle& cycle);
Cycle currentCycle();
Pose currentPose();
void setRadioStatus(const RadioStatus& status);
RadioStatus radioStatus();
bool clockSynchronized();
uint32_t clockGeneration();
uint64_t uwbPausedUntil();
bool takeUwbRecovery();
bool takeLoRaRecovery();
bool takeLoRaTest();
void queueAux(const char* line);
void startLoRaTask();
}
