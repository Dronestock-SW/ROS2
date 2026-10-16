// Runs the production Arduino translation unit against deterministic adapters.
// These are software fault-injection tests, not RF, board, or timing validation.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "Arduino.h"
#include "Br101Dw3000Port.h"
#include "SingleTagRuntime.h"
#include "SingleTagDiagnosticJson.h"

namespace Fake {
enum class Fault {
  Good, PollStart, PollTxTimeout, ResponseRxTimeout, ResponseRxError,
  ResponseDeadline, ResponseClockStopped, ResponseInvalid, FinalStart,
  FinalTxTimeout, ReportRxTimeout, ReportRxError, ReportInvalid,
  ResponseOversized, ReportRejected, TofInvalid, RangeZero, ClockMapWide
};
struct Plan { unsigned anchor; Fault fault; };
struct Sent { unsigned anchor; uint32_t session, transaction; uint64_t at; };
uint64_t us = 1000000;
uint32_t status = 0, frozenRadio = 0, randomValue = 100;
uint64_t txDue = 0, rxDue = 0;
uint32_t rxEvent = 0, pollTx = 0, responseRx = 0, finalTx = 0, txLow = 0;
uint32_t txHigh = 0, rxLow = 0;
bool radioFrozen = false, recoveryRequested = false;
uint64_t pausedUntil = 0;
unsigned initCalls = 0, initFailures = 0, displayCalls = 0;
unsigned idleCalls=0,delayOnIdle=0;
uint32_t idleDelayUs=0,pauseJumpUs=0;
bool wideMapRead=false;
uint8_t txFrame[64] = {}, rxFrame[64] = {};
uint16_t rxLength = 0;
uint8_t phase = 0;
Fault activeFault = Fault::Good;
std::deque<Plan> plans;
std::vector<Sent> sent;
std::vector<SingleTag::Cycle> published;
std::vector<SingleTag::AttemptDiagnostic> attempts;
std::vector<SingleTag::RecoveryDiagnostic> recoveryEvents;

void require(bool condition, const std::string& why) {
  if (!condition) throw std::runtime_error(why);
}
uint32_t highAt(uint64_t at) {
  return static_cast<uint32_t>((at * 1248ULL) / 5ULL);
}
uint32_t lowAt(uint64_t at) {
  return static_cast<uint32_t>((at * 638976ULL) / 10ULL);
}
void events() {
  if (txDue && us >= txDue) {
    status |= SYS_STATUS_TXFRS_BIT_MASK;
    txDue = 0;
  }
  if (rxDue && us >= rxDue) {
    status |= rxEvent;
    rxDue = 0;
  }
}
void step(uint64_t amount = 1) { us += amount; events(); }
void enqueue(unsigned anchor, Fault fault) { plans.push_back({anchor, fault}); }
void four(Fault fourth = Fault::Good) {
  for (unsigned i = 0; i < 4; ++i) enqueue(i, i == 3 ? fourth : Fault::Good);
}
void scheduleRx(Fault timeout, Fault error, Fault deadline, uint64_t at) {
  if (activeFault == deadline || activeFault == Fault::ResponseClockStopped) return;
  rxEvent = activeFault == timeout ? SYS_STATUS_RXFTO_BIT_MASK :
      activeFault == error ? SYS_STATUS_RXFCE_BIT_MASK : SYS_STATUS_RXFCG_BIT_MASK;
  rxDue = at;
}
}

uint32_t millis() { return static_cast<uint32_t>(Fake::us / 1000); }
void delay(uint32_t ms) { Fake::step(static_cast<uint64_t>(ms) * 1000); }
void delayMicroseconds(uint32_t us) {
  Fake::step(us+Fake::pauseJumpUs); Fake::pauseJumpUs=0;
}
int64_t esp_timer_get_time() { return static_cast<int64_t>(Fake::us); }
uint32_t esp_random() { return ++Fake::randomValue; }
dwt_probe_s dw3000_probe_interf;
int dw3000_hw_init() {
  ++Fake::initCalls;
  if (Fake::initFailures) { --Fake::initFailures; return -1; }
  return 0;
}
void dw3000_hw_reset() { Fake::radioFrozen = false; Fake::status = 0; }
int dwt_probe(dwt_probe_s*) { return DWT_SUCCESS; }
bool dwt_checkidlerc() { return true; }
void dwt_softreset(int) {}
int dwt_initialise(int) { return DWT_SUCCESS; }
int dwt_configure(dwt_config_t*) { return DWT_SUCCESS; }
void dwt_setrxaftertxdelay(uint32_t) {}
void dwt_configuretxrf(dwt_txconfig_t*) {}
void dwt_settxantennadelay(uint16_t) {}
void dwt_setrxantennadelay(uint16_t) {}
void dwt_setlnapamode(int) {}
void dwt_setleds(int) {}
void dwt_forcetrxoff() {
  Fake::step(); Fake::txDue = Fake::rxDue = 0;
  if(++Fake::idleCalls==Fake::delayOnIdle) Fake::step(Fake::idleDelayUs);
}
void dwt_write32bitreg(uint32_t reg, uint32_t value) {
  Fake::step();
  Fake::require(reg == SYS_STATUS_ID, "unexpected write register");
  Fake::status &= ~value;
}
uint32_t dwt_read32bitreg(uint32_t reg) {
  Fake::step();
  if (reg == SYS_STATUS_ID) return Fake::status;
  if (reg == RX_FINFO_ID) return Fake::rxLength + FCS_LEN;
  throw std::runtime_error("unexpected read register");
}
uint32_t dwt_readsystimestamphi32() {
  Fake::step();
  if(Fake::wideMapRead) { Fake::step(501); Fake::wideMapRead=false; }
  return Fake::radioFrozen ? Fake::frozenRadio : Fake::highAt(Fake::us);
}
void dwt_readrxdata(uint8_t* bytes, uint16_t length, uint16_t offset) {
  Fake::step();
  Fake::require(length + offset <= sizeof(Fake::rxFrame), "fake RX read bounds");
  std::memcpy(bytes, Fake::rxFrame + offset, length);
  if(Fake::phase==ST_FINAL && Fake::activeFault==Fake::Fault::ClockMapWide) Fake::wideMapRead=true;
}
void dwt_setrxtimeout(uint16_t timeout) {
  Fake::require(timeout == ST_RX_TIMEOUT_UUS, "production RX timeout changed");
  Fake::step();
}
void dwt_writetxdata(uint16_t length, const uint8_t* bytes, uint16_t offset) {
  Fake::step();
  Fake::require(length + offset <= sizeof(Fake::txFrame), "fake TX write bounds");
  std::memcpy(Fake::txFrame + offset, bytes, length);
}
void dwt_writetxfctrl(uint16_t, uint16_t, int) { Fake::step(); }
int dwt_starttx(int mode) {
  using namespace Fake;
  step();
  require((mode & DWT_RESPONSE_EXPECTED) != 0, "production response-expected changed");
  phase = txFrame[9];
  const unsigned anchor = st_read16(txFrame + 5) - 0xA1;
  const uint32_t sid = st_read32(txFrame + 11), txn = st_read32(txFrame + 15);
  require(anchor < 4, "unexpected anchor address");
  if (phase == ST_POLL) {
    activeFault = Fault::Good;
    if (!plans.empty()) {
      require(plans.front().anchor == anchor, "unexpected attempt order");
      activeFault = plans.front().fault; plans.pop_front();
    }
    sent.push_back({anchor, sid, txn, us});
    if (activeFault == Fault::PollStart) return DWT_ERROR;
    txLow = pollTx = lowAt(us + 100); txHigh = highAt(us + 100);
    if (activeFault != Fault::PollTxTimeout) txDue = us + 100;
    responseRx = lowAt(us + 500); rxLow = responseRx;
    st_header(rxFrame, ST_RESP, ST_TAG, 0xA1 + anchor, sid, txn);
    rxLength = ST_HEADER_LEN;
    if (activeFault == Fault::ResponseInvalid) rxFrame[15] ^= 1;
    if (activeFault == Fault::ResponseOversized) rxLength=64;
    scheduleRx(Fault::ResponseRxTimeout, Fault::ResponseRxError,
               Fault::ResponseDeadline, us + 500);
    return DWT_SUCCESS;
  }
  require(phase == ST_FINAL, "unexpected transmitted function");
  if (activeFault == Fault::FinalStart) return DWT_ERROR;
  txLow = finalTx = lowAt(us + 100); txHigh = highAt(us + 100);
  if (activeFault != Fault::FinalTxTimeout) txDue = us + 100;
  st_header(rxFrame, ST_REPORT, ST_TAG, 0xA1 + anchor, sid, txn);
  // Both peers share a fake timebase; 640 DTU propagation gives ~3 m.
  const uint32_t propagation=activeFault==Fault::RangeZero?0:640;
  st_write32(rxFrame + ST_POLL_RX_IDX, pollTx + propagation);
  st_write32(rxFrame + ST_RESP_TX_IDX, responseRx - propagation);
  st_write32(rxFrame + ST_FINAL_RX_IDX, finalTx + propagation);
  rxFrame[ST_STATUS_IDX] = 1;
  rxLength = ST_REPORT_LEN;
  if (activeFault == Fault::ReportInvalid) rxFrame[15] ^= 1;
  if (activeFault == Fault::ReportRejected) rxFrame[ST_STATUS_IDX]=0;
  if (activeFault == Fault::TofInvalid) st_write32(rxFrame + ST_POLL_RX_IDX,pollTx+0x70000000);
  scheduleRx(Fault::ReportRxTimeout, Fault::ReportRxError,
             Fault::ResponseDeadline, us + 500);
  return DWT_SUCCESS;
}
uint32_t dwt_readtxtimestamplo32() { Fake::step(); return Fake::txLow; }
uint32_t dwt_readtxtimestamphi32() { Fake::step(); return Fake::txHigh; }
uint32_t dwt_readrxtimestamplo32(int) { Fake::step(); return Fake::rxLow; }
namespace Br101Dw3000 { void spiFast() {} }
namespace UwbDsTwr {
dwt_config_t defaultConfig() { return {}; }
dwt_txconfig_t defaultTxConfig() { return {}; }
void clearTxState() {
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
  if (Fake::phase == ST_POLL && Fake::activeFault == Fake::Fault::ResponseClockStopped) {
    Fake::radioFrozen = true; Fake::frozenRadio = Fake::highAt(Fake::us);
  }
}
void clearRxState() { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD |
                                     SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR); }
}
namespace JetsonTagDisplay {
void begin() {}
void render(bool, uint8_t, bool) { ++Fake::displayCalls; }
}
namespace SingleTag {
void beginRuntime(uint32_t) {}
bool takeUwbRecovery() {
  const bool result = Fake::recoveryRequested; Fake::recoveryRequested = false; return result;
}
uint64_t uwbPausedUntil() { return Fake::pausedUntil; }
void publishCycle(const Cycle& cycle) { Fake::published.push_back(cycle); }
void recordAttemptDiagnostic(const AttemptDiagnostic& diagnostic) { Fake::attempts.push_back(diagnostic); }
void recordRecoveryDiagnostic(const RecoveryDiagnostic& diagnostic) { Fake::recoveryEvents.push_back(diagnostic); }
}

// Production decisions and state are deliberately not reimplemented here.
#include "../../src/main.cpp"

namespace Test {
using Fake::require;
using Fake::Fault;
using SingleTag::Sample;
void start() {
  setup();
  require(ready, "setup must initialize the adapter");
  require(SingleTag::kPeriodUs == 20000, "50 Hz period must remain 20 ms");
  require(ST_CYCLE_BUDGET_US == 18000, "cycle budget must remain 18 ms");
  require(nextStartUs - Fake::us == 100000, "startup quiet period must remain 100 ms");
}
SingleTag::Cycle cycle() {
  Fake::us = std::max(Fake::us, nextStartUs);
  const auto before = Fake::published.size(); loop();
  require(Fake::published.size() == before + 1, "loop must publish one completed cycle");
  return Fake::published.back();
}
Sample one(Fault fault, unsigned anchor = 0) {
  Fake::enqueue(anchor, fault);
  Sample sample = {}; attempt(anchor, sample); return sample;
}
void goodSample(const Sample& s) {
  require(s.failure == SingleTag::Ok, "expected successful measurement, failure=" + std::to_string(s.failure));
  require(s.distance > 2.99f && s.distance < 3.01f, "DS-TWR timestamps must yield ~3 m");
  require(s.timeUs > 0 && s.timeUs <= Fake::us, "sample must retain a past source time");
}
void normal() {
  start(); Fake::four(); const auto c = cycle();
  require(c.mask == 15 && c.ready, "all four anchors must succeed");
  require(!c.deadlineMissed && c.endUs - c.startUs < 18000, "normal cycle must fit budget");
  require(Fake::sent.size() == 4, "normal cycle must have no retry");
  for (unsigned i = 0; i < 4; ++i) {
    goodSample(c.samples[i]); require(c.samples[i].attempts == 1, "normal attempt count");
    require(Fake::sent[i].anchor == i, "first-pass anchor order");
  }
}
void responseFault(Fault fault) {
  start(); const auto before = Fake::us; const auto s = one(fault);
  require(s.failure == SingleTag::ResponseTimeout, "response fault keeps legacy failure code");
  require(std::isnan(s.distance), "failure cannot fabricate a distance");
  require(Fake::us - before < 3000, "response failure must be bounded");
  require(ready == (fault != Fault::ResponseClockStopped), "only stopped-clock guard disables ready here");
  const auto& d = Fake::attempts.back();
  using SingleTag::DiagnosticCause;
  const auto expected = fault == Fault::ResponseRxTimeout ? DiagnosticCause::RxTimeout :
      fault == Fault::ResponseRxError ? DiagnosticCause::RxError :
      fault == Fault::ResponseDeadline ? DiagnosticCause::RadioDeadline : DiagnosticCause::McuGuard;
  require(d.cause == expected && s.cause == static_cast<uint8_t>(expected), "cause must identify the injected stop");
  require(d.stage == SingleTag::DiagnosticStage::ResponseRx && d.pollTxConfirmed, "response failure retains confirmed POLL-TX stage");
  require(d.wait.endUs >= d.wait.startUs && d.endUs >= d.startUs, "diagnostic intervals are ordered");
  require(d.wait.rfProgress == (fault != Fault::ResponseClockStopped), "RF progress observation is explicit");
  if (fault == Fault::ResponseRxTimeout) require(d.wait.status & SYS_STATUS_RXFTO_BIT_MASK, "raw RX timeout flag preserved");
  if (fault == Fault::ResponseRxError) require(d.wait.status & SYS_STATUS_RXFCE_BIT_MASK, "raw RX error flag preserved");
  if (fault == Fault::ResponseClockStopped) {
    require(Fake::recoveryEvents.back().event == SingleTag::RecoveryEvent::Scheduled &&
            Fake::recoveryEvents.back().reason == SingleTag::RecoveryReason::McuGuard,
            "MCU guard schedules an observable recovery");
  }
}
void reportFault(Fault fault) {
  start(); const auto s = one(fault);
  require(s.failure == SingleTag::ReportTimeout && ready, "report RX fault classification");
  require(std::isnan(s.distance), "report fault cannot retain previous distance");
  const auto& d = Fake::attempts.back();
  require(d.stage == SingleTag::DiagnosticStage::ReportRx && d.pollTxConfirmed, "report failure stage retained");
  require(d.cause == (fault == Fault::ReportRxTimeout ? SingleTag::DiagnosticCause::RxTimeout :
                     SingleTag::DiagnosticCause::RxError), "report RX cause separated");
}
void retry(bool succeeds) {
  start(); Fake::four(Fault::ResponseRxTimeout);
  Fake::enqueue(3, succeeds ? Fault::Good : Fault::ResponseRxError);
  const auto c = cycle();
  require(Fake::sent.size() == 5, "exactly one optional retry");
  require(Fake::sent.back().anchor == 3, "A4 retry follows all first attempts");
  require(c.samples[3].attempts == 2, "retry attempt count");
  require(c.mask == (succeeds ? 15 : 7), "retry result mask");
  require(!c.deadlineMissed, "retry must fit original cycle budget");
  require(Fake::sent[3].transaction != Fake::sent[4].transaction, "retry must have a new transaction");
  require(c.samples[3].firstFailure == SingleTag::ResponseTimeout &&
          c.samples[3].firstCause == static_cast<uint8_t>(SingleTag::DiagnosticCause::RxTimeout),
          "first failure and first cause survive both retry outcomes");
  require(c.samples[3].firstTransaction == Fake::sent[3].transaction &&
          c.samples[3].transaction == Fake::sent[4].transaction, "both transaction identifiers survive retry");
  require(Fake::attempts.size() == 5 && Fake::attempts[3].attempt == 1 &&
          Fake::attempts[4].attempt == 2, "each attempt has its own diagnostic record");
  require(Fake::attempts[3].cause == SingleTag::DiagnosticCause::RxTimeout &&
          Fake::attempts[4].cause == (succeeds ? SingleTag::DiagnosticCause::Success :
                                     SingleTag::DiagnosticCause::RxError), "retry diagnostic does not overwrite first stop");
  if (succeeds) goodSample(c.samples[3]);
  else require(c.samples[3].failure == SingleTag::ResponseTimeout, "final retry failure");
}
void oneRetryOnly() {
  start();
  Fake::enqueue(0, Fault::ResponseRxTimeout); Fake::enqueue(1, Fault::ResponseRxTimeout);
  Fake::enqueue(2, Fault::Good); Fake::enqueue(3, Fault::Good); Fake::enqueue(0, Fault::Good);
  const auto c = cycle();
  require(Fake::sent.size() == 5 && c.samples[1].attempts == 1, "two failures still get only one retry");
  require(c.mask == 13, "A2 remains invalid when A1 uses retry");
}
void consecutiveTx(Fault fault, uint8_t expected) {
  start();
  require(one(fault).failure == expected && ready, "first TX fault does not reinitialize");
  require(one(fault, 1).failure == expected && !ready, "second consecutive TX fault disables ready");
  require(Fake::initCalls == 1, "attempt must not bypass natural recovery delay");
  require(Fake::recoveryEvents.back().event == SingleTag::RecoveryEvent::Scheduled &&
          Fake::recoveryEvents.back().reason == SingleTag::RecoveryReason::TxFaultStreak,
          "TX fault streak has a distinct scheduled recovery reason");
  const auto& d = Fake::attempts.back();
  require(d.stage == (fault == Fault::PollStart || fault == Fault::PollTxTimeout ?
                     SingleTag::DiagnosticStage::PollTx : SingleTag::DiagnosticStage::FinalTx), "TX failure stage");
  require(d.pollTxConfirmed == (fault == Fault::FinalStart || fault == Fault::FinalTxTimeout), "POLL completion evidence");
}
void txStreakReset() {
  start(); one(Fault::PollStart); goodSample(one(Fault::Good, 1)); one(Fault::PollStart, 2);
  require(ready && txFaults == 1, "success breaks consecutive TX fault streak");
}
void makeNotReady() {
  one(Fault::PollStart); one(Fault::PollStart, 1);
  require(!ready, "test precondition: two actual TX failures");
}
void naturalRecovery() {
  start(); const uint32_t oldSession = session; makeNotReady();
  const uint64_t due = static_cast<uint64_t>(lastRecoveryMs) * 1000 + 1500000;
  Fake::us = due - 1000; loop();
  require(!ready && Fake::initCalls == 1, "1499 ms must not recover");
  Fake::us = due; loop();
  require(ready && Fake::initCalls == 2 && recoveries == 1, "1500 ms must attempt natural recovery");
  require(session != oldSession && transaction == 0, "successful recovery creates a new radio session");
  const auto& recovered = Fake::recoveryEvents.back();
  require(recovered.event == SingleTag::RecoveryEvent::InitSuccess &&
          recovered.nextCycleUs - recovered.atUs == 100000, "natural recovery retains 100 ms quiet period");
  require(recovered.atUs >= recovered.detectedUs + 1500000, "natural recovery event includes actual detection-to-init latency");
  const auto count = Fake::published.size(); Fake::us = nextStartUs - 1; loop();
  require(Fake::published.size() == count, "quiet period must not emit a new measurement cycle");
  Fake::four(); const auto c = cycle(); require(c.mask == 15, "all links can measure after recovery");
}
void failedInitialization() {
  start(); makeNotReady(); Fake::initFailures = 1;
  Fake::us = static_cast<uint64_t>(lastRecoveryMs) * 1000 + 1500000; loop();
  require(!ready && Fake::initCalls == 2 && recoveries == 1, "failed initializer must remain not ready");
  require(Fake::recoveryEvents.back().event == SingleTag::RecoveryEvent::InitFailed,
          "failed initialization is reported separately from recovery success");
  const uint32_t failedAt = lastRecoveryMs;
  Fake::us = static_cast<uint64_t>(failedAt) * 1000 + 1499000; loop();
  require(Fake::initCalls == 2, "initializer failure must preserve 1500 ms retry spacing");
  Fake::us = static_cast<uint64_t>(failedAt) * 1000 + 1500000; loop();
  require(ready && Fake::initCalls == 3 && recoveries == 2, "next scheduled initialization succeeds");
}
void forcedRecovery() {
  start(); const auto before = Fake::us; Fake::recoveryRequested = true; loop();
  require(ready && Fake::initCalls == 2 && recoveries == 1, "forced recovery bypasses 1500 ms delay");
  require(Fake::us - before < 100000, "forced recovery must not wait 1500 ms");
  const auto& recovered = Fake::recoveryEvents.back();
  require(recovered.reason == SingleTag::RecoveryReason::BenchRequest &&
          recovered.nextCycleUs - recovered.atUs == 100000, "forced recovery retains reason and session quiet time");
}
void skipPeriods() {
  start(); Fake::us = nextStartUs + 3ULL * SingleTag::kPeriodUs + 100;
  const auto c = cycle();
  require(c.seq == 3 && c.skipped == 3 && sequence == 4, "missed periods advance sequence without replay");
  require(Fake::published.size() == 1, "skip must not replay old cycles");
}
void wrappingSequence() {
  start(); sequence = UINT32_MAX; transaction = UINT32_MAX - 1;
  const auto first = cycle(); const auto second = cycle();
  require(first.seq == UINT32_MAX && second.seq == 0, "cycle sequence wraps as uint32");
  require(Fake::sent[0].transaction == UINT32_MAX && Fake::sent[1].transaction == 0,
          "transaction wrap remains observable");
  require(first.mask == 15 && second.mask == 15, "wrapped identifiers still match exact frames");
}
void wrappingRadioTime() {
  // The DW high32 timebase wraps at approximately 17.2 seconds.
  Fake::us = (static_cast<uint64_t>(UINT32_MAX) * 5 / 1248) - 106000;
  start(); const auto c = cycle();
  require(c.mask == 15 && !c.deadlineMissed, "RF timestamp/deadline wrap must not reject valid exchange");
}
void wrappingMillisRecovery() {
  Fake::us = (static_cast<uint64_t>(UINT32_MAX) - 10) * 1000;
  start(); makeNotReady(); const uint64_t failureUs = Fake::us;
  Fake::us = failureUs + 1498000; loop();
  require(!ready && Fake::initCalls == 1, "natural delay across millis wrap is not early");
  Fake::us = failureUs + 1501000; loop();
  require(ready && Fake::initCalls == 2, "natural recovery survives millis wrap");
}
void invalidFrames() {
  start();
  require(one(Fault::ResponseInvalid).failure == SingleTag::ResponseInvalid, "mismatched RESP rejected");
  require(one(Fault::ReportInvalid, 1).failure == SingleTag::ReportInvalid, "mismatched REPORT rejected");
  require(ready, "invalid frames do not trigger full radio recovery");
  require(Fake::attempts[0].cause == SingleTag::DiagnosticCause::FrameIdentity &&
          Fake::attempts[1].cause == SingleTag::DiagnosticCause::FrameIdentity, "identity errors stay separate from RX flags");
}
void causeChecks() {
  start();
  const std::vector<std::pair<Fault,SingleTag::DiagnosticCause>> cases={
    {Fault::ResponseOversized,SingleTag::DiagnosticCause::FrameLength},
    {Fault::ReportRejected,SingleTag::DiagnosticCause::ReportRejected},
    {Fault::TofInvalid,SingleTag::DiagnosticCause::TofInvalid},
    {Fault::RangeZero,SingleTag::DiagnosticCause::RangeInvalid}};
  for(const auto& entry:cases) {
    const auto s=one(entry.first);
    require(s.failure!=SingleTag::Ok && std::isnan(s.distance),"invalid frame or math cannot produce distance");
    require(Fake::attempts.back().cause==entry.second,"validation failure has its own cause");
  }
}
void clockInvalidNoRetry() {
  start(); Fake::four(Fault::ClockMapWide); const auto c=cycle();
  require(c.samples[3].failure==SingleTag::ClockInvalid && c.samples[3].attempts==1,
          "clock mapping failure must stay excluded from retry");
  require(Fake::sent.size()==4 && c.samples[3].cause==(uint8_t)SingleTag::DiagnosticCause::ClockMapInvalid,
          "clock cause retained without another RF exchange");
}
void budgetSkipped() {
  start(); Fake::delayOnIdle=3; Fake::idleDelayUs=16000;
  const auto c=cycle();
  require(Fake::sent.size()==1 && !c.deadlineMissed,"cleanup delay leaves insufficient new-exchange budget");
  for(unsigned i=1;i<4;i++) {
    const auto& s=c.samples[i];
    require(s.attempts==0 && s.transaction==0 && s.firstTransaction==0,"unattempted slots have no transaction");
    require(s.failure==SingleTag::Deadline && s.firstFailure==SingleTag::Deadline &&
            s.firstCause==(uint8_t)SingleTag::DiagnosticCause::Budget,"budget reason is retained without a fake attempt");
  }
}
void notReadyAndPaused() {
  start(); Fake::pausedUntil=nextStartUs+100000; const auto paused=cycle();
  require(paused.mask==0 && Fake::sent.empty(),"pause does not start RF attempts");
  for(const auto& s:paused.samples)
    require(s.attempts==0 && s.firstFailure==SingleTag::NotReady &&
            s.firstCause==(uint8_t)SingleTag::DiagnosticCause::NotReady,"paused slots retain existing NotReady code");
  Fake::pausedUntil=0; makeNotReady(); const auto unready=cycle();
  require(unready.mask==0 && !unready.ready,"natural recovery wait still emits invalid RAW");
  for(const auto& s:unready.samples) require(s.attempts==0 && s.firstTransaction==0,"not-ready slots are not RF attempts");
}
void waitBoundary(bool expired) {
  start(); SingleTag::AttemptDiagnostic d={}; d.stage=SingleTag::DiagnosticStage::ResponseRx;
  Fake::status=SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_RXFCE_BIT_MASK;
  const auto deadline=Fake::highAt(Fake::us+(expired?5:20));
  const auto status=waitUntil(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR,deadline,d);
  require(d.wait.status==(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_RXFCE_BIT_MASK),"raw flags survive deadline classification");
  require(d.cause==(expired?SingleTag::DiagnosticCause::RadioDeadline:SingleTag::DiagnosticCause::Success),
          "original deadline-first and success-bit precedence retained");
  require((status!=0)==!expired && ready,"deadline alone does not disable radio");
}
void mcuLateWithRfProgress() {
  start(); SingleTag::AttemptDiagnostic d={}; d.stage=SingleTag::DiagnosticStage::ResponseRx;
  Fake::pauseJumpUs=2000;
  const auto status=waitUntil(SYS_STATUS_RXFCG_BIT_MASK,Fake::highAt(Fake::us+1400),d);
  require(status==0 && !ready && d.cause==SingleTag::DiagnosticCause::McuGuard,"MCU scheduling delay retains original guard policy");
  require(d.wait.rfProgress,"guard event must preserve observed RF progress, not claim RF clock stopped");
}
void serialization() {
  retry(true);
  char line[1024];
  SingleTag::DiagnosticEvent event = {};
  event.attempt = Fake::attempts[3];
  auto n = SingleTag::serializeDiagnostic(event, line, sizeof(line));
  require(n == std::strlen(line) && n > 0 && line[n-1] == '\n', "complete attempt JSON line");
  std::fputs(line, stdout);
  event.recovery = true; event.reset = Fake::recoveryEvents[1];
  n = SingleTag::serializeDiagnostic(event, line, sizeof(line));
  require(n > 0, "complete recovery JSON line"); std::fputs(line, stdout);
  const auto fields = SingleTag::serializeRawDiagnosticFields(Fake::published.back(), line, sizeof(line));
  require(fields > 0, "RAW diagnostic additions fit");
  std::printf("{\"type\":\"test_raw_fields\"%s}\n", line);
  char small[32]; std::memset(small, '#', sizeof(small));
  require(SingleTag::serializeDiagnostic(event, small, 16) == 0 && small[16] == '#', "truncated event is not sendable and cannot overrun");
  require(SingleTag::serializeRawDiagnosticFields(Fake::published.back(), small, 16) == 0 && small[16] == '#', "truncated RAW extension is not sendable");
  require(SingleTag::serializeDiagnostic(event, small, 0) == 0, "zero-capacity serializer is safe");
  event.recovery = false; event.attempt = Fake::attempts[3];
  event.attempt.boot = event.attempt.session = event.attempt.seq = event.attempt.transaction = UINT32_MAX;
  event.attempt.startUs = event.attempt.endUs = UINT64_MAX;
  event.attempt.wait.startUs = event.attempt.wait.endUs = UINT64_MAX;
  event.attempt.radioStart = event.attempt.radioEnd = UINT32_MAX;
  event.attempt.wait.radioStart = event.attempt.wait.radioEnd = event.attempt.wait.deadline = event.attempt.wait.status = UINT32_MAX;
  n = SingleTag::serializeDiagnostic(event, line, sizeof(line));
  require(n > 0 && n < sizeof(line), "maximum integer diagnostic fits bounded buffer");
  std::fputs(line, stdout);
}
}

int main(int argc, char** argv) {
  using Fake::Fault;
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
    {"normal_four_links", Test::normal},
    {"response_rx_timeout", [] { Test::responseFault(Fault::ResponseRxTimeout); }},
    {"response_rx_error", [] { Test::responseFault(Fault::ResponseRxError); }},
    {"response_deadline", [] { Test::responseFault(Fault::ResponseDeadline); }},
    {"response_mcu_guard", [] { Test::responseFault(Fault::ResponseClockStopped); }},
    {"report_rx_timeout", [] { Test::reportFault(Fault::ReportRxTimeout); }},
    {"report_rx_error", [] { Test::reportFault(Fault::ReportRxError); }},
    {"first_failure_retry_success", [] { Test::retry(true); }},
    {"first_failure_retry_failure", [] { Test::retry(false); }},
    {"only_one_retry", Test::oneRetryOnly},
    {"poll_start_failure_twice", [] { Test::consecutiveTx(Fault::PollStart, SingleTag::PollStart); }},
    {"poll_tx_timeout_twice", [] { Test::consecutiveTx(Fault::PollTxTimeout, SingleTag::PollTimeout); }},
    {"final_start_failure_twice", [] { Test::consecutiveTx(Fault::FinalStart, SingleTag::FinalStart); }},
    {"final_tx_timeout_twice", [] { Test::consecutiveTx(Fault::FinalTxTimeout, SingleTag::FinalTimeout); }},
    {"tx_streak_reset", Test::txStreakReset},
    {"natural_recovery_1500ms", Test::naturalRecovery},
    {"natural_initialization_failure", Test::failedInitialization},
    {"forced_recovery_bypass", Test::forcedRecovery},
    {"skip_periods", Test::skipPeriods},
    {"sequence_transaction_wrap", Test::wrappingSequence},
    {"radio_time_wrap", Test::wrappingRadioTime},
    {"millis_recovery_wrap", Test::wrappingMillisRecovery},
    {"invalid_transaction_frames", Test::invalidFrames},
    {"frame_and_math_causes", Test::causeChecks},
    {"clock_invalid_no_retry", Test::clockInvalidNoRetry},
    {"budget_skipped_slots", Test::budgetSkipped},
    {"not_ready_and_paused", Test::notReadyAndPaused},
    {"wait_success_mixed_flags", [] { Test::waitBoundary(false); }},
    {"wait_deadline_precedence", [] { Test::waitBoundary(true); }},
    {"mcu_late_with_rf_progress", Test::mcuLateWithRfProgress},
    {"diagnostic_serialization", Test::serialization},
  };
  if (argc == 2 && std::string(argv[1]) == "--list") {
    for (const auto& test : tests) std::puts(test.first.c_str());
    return 0;
  }
  if (argc != 2) { std::fputs("Usage: test_tag_main TEST_NAME | --list\n", stderr); return 2; }
  for (const auto& test : tests) {
    if (test.first != argv[1]) continue;
    try {
      test.second();
      Fake::require(Fake::plans.empty(), "unused fault plans");
      std::printf("PASS %s\n", argv[1]); return 0;
    } catch (const std::exception& error) {
      std::fprintf(stderr, "FAIL %s: %s\n", argv[1], error.what()); return 1;
    }
  }
  std::fprintf(stderr, "Unknown test: %s\n", argv[1]); return 2;
}
