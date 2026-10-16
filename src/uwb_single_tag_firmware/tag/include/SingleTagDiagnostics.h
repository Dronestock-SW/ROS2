#pragma once
#include <stdint.h>
#include <stddef.h>

namespace SingleTag {
// Additive USB diagnostics. RF protocol and legacy Failure values do not change.
enum class DiagnosticCause : uint8_t {
  None=0, NotReady=1, TxStartRejected=2, RxTimeout=3, RxError=4,
  RadioDeadline=5, McuGuard=6, FrameLength=7, FrameIdentity=8,
  ReportRejected=9, TofInvalid=10, RangeInvalid=11, ClockMapInvalid=12,
  Budget=13, Success=14
};
constexpr unsigned kDiagnosticCauses=15;
enum class DiagnosticStage : uint8_t {
  NotStarted=0, PollTx=1, ResponseRx=2, FinalTx=3, ReportRx=4,
  Distance=5, ClockMap=6, Done=7
};
struct WaitDiagnostic {
  uint64_t startUs, endUs;
  uint32_t radioStart, radioEnd, deadline, status;
  DiagnosticStage stage;
  // False means no progress was OBSERVED, not proof of a stopped device.
  bool rfProgress;
};
struct AttemptDiagnostic {
  uint32_t boot, session, seq, transaction;
  uint64_t startUs, endUs, retryEligibleUs;
  uint32_t radioStart, radioEnd;
  WaitDiagnostic wait;
  uint8_t anchor, attempt, failure;
  DiagnosticCause cause;
  DiagnosticStage stage;
  bool pollTxConfirmed;
};
enum class RecoveryEvent : uint8_t { Scheduled, InitStart, InitSuccess, InitFailed };
enum class RecoveryReason : uint8_t { Startup, McuGuard, TxFaultStreak, BenchRequest };
struct RecoveryDiagnostic {
  uint32_t boot, session, seq, recoveries;
  uint64_t atUs, detectedUs, nextCycleUs, eligibleUs;
  RecoveryEvent event;
  RecoveryReason reason;
};
struct DiagnosticEvent {
  bool recovery;
  AttemptDiagnostic attempt;
  RecoveryDiagnostic reset;
};
constexpr unsigned kDiagnosticQueueDepth=16;
constexpr uint64_t kDiagnosticMaxAgeUs=2000000;

void recordAttemptDiagnostic(const AttemptDiagnostic& diagnostic);
void recordRecoveryDiagnostic(const RecoveryDiagnostic& diagnostic);
}
