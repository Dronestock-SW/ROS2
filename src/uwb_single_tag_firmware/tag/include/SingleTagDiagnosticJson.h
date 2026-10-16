#pragma once
#include "SingleTagDiagnostics.h"
#include <stdio.h>

namespace SingleTag {
inline const char* recoveryEventName(RecoveryEvent event) {
  switch(event) {
    case RecoveryEvent::Scheduled: return "scheduled";
    case RecoveryEvent::InitStart: return "init_start";
    case RecoveryEvent::InitSuccess: return "init_success";
    case RecoveryEvent::InitFailed: return "init_failed";
  }
  return "unknown";
}
inline const char* recoveryReasonName(RecoveryReason reason) {
  switch(reason) {
    case RecoveryReason::Startup: return "startup";
    case RecoveryReason::McuGuard: return "mcu_guard";
    case RecoveryReason::TxFaultStreak: return "tx_fault_streak";
    case RecoveryReason::BenchRequest: return "bench_request";
  }
  return "unknown";
}
inline size_t serializeDiagnostic(const DiagnosticEvent& event,char* line,size_t cap) {
  int n;
  if(event.recovery) {
    const RecoveryDiagnostic& d=event.reset;
    n=snprintf(line,cap,"{\"type\":\"uwb_rf_recovery\",\"schema\":2,\"diag_version\":1,\"tag_id\":\"6\","
        "\"boot_id\":%lu,\"session\":%lu,\"seq\":%lu,\"event\":\"%s\",\"reason\":\"%s\","
        "\"at_us\":%llu,\"detected_us\":%llu,\"next_cycle_us\":%llu,\"recoveries\":%lu}\n",
        (unsigned long)d.boot,(unsigned long)d.session,(unsigned long)d.seq,
        recoveryEventName(d.event),recoveryReasonName(d.reason),(unsigned long long)d.atUs,
        (unsigned long long)d.detectedUs,(unsigned long long)d.nextCycleUs,(unsigned long)d.recoveries);
  } else {
    const AttemptDiagnostic& d=event.attempt;
    n=snprintf(line,cap,"{\"type\":\"uwb_rf_attempt\",\"schema\":2,\"diag_version\":1,\"tag_id\":\"6\","
        "\"boot_id\":%lu,\"session\":%lu,\"seq\":%lu,\"anchor\":%u,\"attempt\":%u,"
        "\"transaction\":%lu,\"failure\":%u,\"cause\":%u,\"stage\":%u,"
        "\"start_us\":%llu,\"end_us\":%llu,\"radio_start\":%lu,\"radio_end\":%lu,"
        "\"status\":%lu,\"wait_stage\":%u,\"wait_start_us\":%llu,\"wait_end_us\":%llu,"
        "\"wait_radio_start\":%lu,\"wait_radio_end\":%lu,\"deadline_radio\":%lu,"
        "\"rf_progress\":%s,\"poll_tx_confirmed\":%s}\n",
        (unsigned long)d.boot,(unsigned long)d.session,(unsigned long)d.seq,d.anchor,d.attempt,
        (unsigned long)d.transaction,d.failure,(unsigned)d.cause,(unsigned)d.stage,
        (unsigned long long)d.startUs,(unsigned long long)d.endUs,(unsigned long)d.radioStart,
        (unsigned long)d.radioEnd,(unsigned long)d.wait.status,(unsigned)d.wait.stage,(unsigned long long)d.wait.startUs,
        (unsigned long long)d.wait.endUs,(unsigned long)d.wait.radioStart,(unsigned long)d.wait.radioEnd,
        (unsigned long)d.wait.deadline,d.wait.rfProgress?"true":"false",d.pollTxConfirmed?"true":"false");
  }
  // A truncated line is never sent as a JSON message.
  return n>0 && (size_t)n<cap?(size_t)n:0;
}

// Template keeps the formatter host-testable without Arduino or FreeRTOS.
template<class CycleType>
size_t serializeRawDiagnosticFields(const CycleType& c,char* line,size_t cap) {
  const int n=snprintf(line,cap,",\"diag_version\":1,\"transaction\":[%lu,%lu,%lu,%lu],"
      "\"first_transaction\":[%lu,%lu,%lu,%lu],\"first_failure\":[%u,%u,%u,%u],"
      "\"cause\":[%u,%u,%u,%u],\"first_cause\":[%u,%u,%u,%u]",
      (unsigned long)c.samples[0].transaction,(unsigned long)c.samples[1].transaction,
      (unsigned long)c.samples[2].transaction,(unsigned long)c.samples[3].transaction,
      (unsigned long)c.samples[0].firstTransaction,(unsigned long)c.samples[1].firstTransaction,
      (unsigned long)c.samples[2].firstTransaction,(unsigned long)c.samples[3].firstTransaction,
      c.samples[0].firstFailure,c.samples[1].firstFailure,c.samples[2].firstFailure,c.samples[3].firstFailure,
      c.samples[0].cause,c.samples[1].cause,c.samples[2].cause,c.samples[3].cause,
      c.samples[0].firstCause,c.samples[1].firstCause,c.samples[2].firstCause,c.samples[3].firstCause);
  return n>0 && (size_t)n<cap?(size_t)n:0;
}
}
