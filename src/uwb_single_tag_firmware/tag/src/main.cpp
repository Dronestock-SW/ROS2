#include <Arduino.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <math.h>
#include "Br101Dw3000Port.h"
#include "UwbDsTwrProtocol.h"
#include "JetsonTagConfig.h"
#include "JetsonTagDisplay.h"
#include "SingleTagRuntime.h"

namespace {
using namespace SingleTag;
bool ready=false;
uint32_t bootId, session, transaction=0, sequence=0, skipped=0, recoveries=0;
uint32_t exchangeDeadline=0;
uint8_t txFaults=0;
uint64_t nextStartUs=0;
constexpr uint64_t kRecoveryBackoffUs=1500000;
uint64_t recoveryNotBeforeUs=0, lastInitEndUs=0;
bool hasRecoveryAttempt=false;
uint64_t retryNotBeforeUs[4]={};
uint8_t receiveBuffer[64];
uint32_t diagnosticCycle=0;
uint64_t recoveryDetectedUs=0;
RecoveryReason recoveryReason=RecoveryReason::Startup;
uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }
void recoveryEvent(RecoveryEvent event) {
  RecoveryDiagnostic d={};
  d.boot=bootId; d.session=session; d.seq=diagnosticCycle; d.recoveries=recoveries;
  d.atUs=nowUs(); d.detectedUs=recoveryDetectedUs; d.nextCycleUs=nextStartUs;
  d.eligibleUs=recoveryNotBeforeUs;
  d.event=event; d.reason=recoveryReason;
  recordRecoveryDiagnostic(d);
}
void scheduleRecovery(RecoveryReason reason) {
  if (!ready && reason!=RecoveryReason::BenchRequest) return;
  ready=false;
  recoveryReason=reason; recoveryDetectedUs=nowUs();
  recoveryNotBeforeUs=recoveryDetectedUs;
  // The first fault gets a prompt attempt. Repeated faults cannot create a
  // reset storm; the interval is measured from the previous initializer end.
  if (reason!=RecoveryReason::BenchRequest && hasRecoveryAttempt &&
      recoveryNotBeforeUs<lastInitEndUs+kRecoveryBackoffUs)
    recoveryNotBeforeUs=lastInitEndUs+kRecoveryBackoffUs;
  recoveryEvent(RecoveryEvent::Scheduled);
}
uint32_t radioNow() {
  // Preserve radiofix3: DW3000 SYS_TIME is latched until an SPI write.
  dwt_write32bitreg(SYS_STATUS_ID,0U);
  return dwt_readsystimestamphi32();
}
void idleRadio() {
  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_TO |
      SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
}
uint32_t waitUntil(uint32_t mask,uint32_t deadline,AttemptDiagnostic& diagnostic) {
  const uint32_t r=radioNow();
  WaitDiagnostic& w=diagnostic.wait;
  w={}; w.startUs=nowUs(); w.radioStart=w.radioEnd=r; w.deadline=deadline;
  w.stage=diagnostic.stage;
  const uint64_t watchdog=w.startUs+(st_before(r,deadline)?st_us(deadline-r):0)+300;
  while (nowUs()<watchdog) {
    const uint32_t status=dwt_read32bitreg(SYS_STATUS_ID);
    const uint32_t radio=radioNow();
    w.status=status; w.radioEnd=radio; w.rfProgress|=radio!=r;
    diagnostic.radioEnd=radio;
    if (!st_before(radio,deadline)) {
      w.endUs=nowUs(); diagnostic.cause=DiagnosticCause::RadioDeadline; return 0;
    }
    if (status&mask) {
      w.endUs=nowUs();
      // Preserve the original success-bit precedence, including mixed flags.
      if (status&mask&(SYS_STATUS_TXFRS_BIT_MASK|SYS_STATUS_RXFCG_BIT_MASK))
        diagnostic.cause=DiagnosticCause::Success;
      else if (status&SYS_STATUS_ALL_RX_ERR) diagnostic.cause=DiagnosticCause::RxError;
      else if (status&SYS_STATUS_ALL_RX_TO) diagnostic.cause=DiagnosticCause::RxTimeout;
      return status;
    }
    delayMicroseconds(ST_POLL_PAUSE_US);
  }
  // A descheduled MCU can resume after a healthy RF deadline. Re-read once:
  // reject the expired exchange without interpreting it as a radio failure.
  w.radioEnd=radioNow(); diagnostic.radioEnd=w.radioEnd;
  w.rfProgress|=w.radioEnd!=r;
  w.endUs=nowUs();
  if (!st_before(w.radioEnd,deadline)) {
    diagnostic.cause=DiagnosticCause::RadioDeadline; return 0;
  }
  diagnostic.cause=DiagnosticCause::McuGuard;
  scheduleRecovery(RecoveryReason::McuGuard);
  return 0;
}
bool readFrame(uint16_t& length) {
  const uint16_t total=dwt_read32bitreg(RX_FINFO_ID)&RX_FINFO_STD_RXFLEN_MASK;
  if (total<FCS_LEN || total>sizeof(receiveBuffer)) return false;
  length=total-FCS_LEN;
  dwt_readrxdata(receiveBuffer,length,0);
  return true;
}
bool initializeUwb() {
  // Dedicated SPI host retains the original reset and burst-transfer fixes.
  if (dw3000_hw_init()<0) return false;
  dw3000_hw_reset();
  delay(5);
  if (dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf)!=DWT_SUCCESS) return false;
  for (unsigned i=0;i<20 && !dwt_checkidlerc();i++) delay(2);
  if (!dwt_checkidlerc()) { dwt_softreset(0); delay(5); }
  if (!dwt_checkidlerc() || dwt_initialise(DWT_DW_INIT)!=DWT_SUCCESS) return false;
  dwt_config_t config=UwbDsTwr::defaultConfig();
  config.chan=5; config.txPreambLength=DWT_PLEN_128; config.rxPAC=DWT_PAC8;
  config.sfdType=DWT_SFD_DW_8; config.dataRate=DWT_BR_6M8;
  config.stsMode=DWT_STS_MODE_OFF; config.sfdTO=128+1+8-8;
  if (dwt_configure(&config)!=DWT_SUCCESS) return false;
  Br101Dw3000::spiFast();
  dwt_setrxaftertxdelay(0);
  dwt_txconfig_t txConfig=UwbDsTwr::defaultTxConfig();
  dwt_configuretxrf(&txConfig);
  dwt_settxantennadelay(JetsonTagConfig::kTagTxAntennaDelay);
  dwt_setrxantennadelay(JetsonTagConfig::kTagRxAntennaDelay);
  dwt_setlnapamode(DWT_LNA_ENABLE|DWT_PA_ENABLE);
  dwt_setleds(DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK);
  idleRadio();
  session=esp_random(); if (!session) session=1;
  transaction=0; txFaults=0;
  for (auto& eligible:retryNotBeforeUs) eligible=0;
  return true;
}
Sample emptySample(uint8_t failure) {
  Sample s={}; s.distance=NAN; s.failure=s.firstFailure=failure;
  s.cause=s.firstCause=(uint8_t)DiagnosticCause::NotReady; return s;
}
bool rangeOne(unsigned anchor,Sample& sample,AttemptDiagnostic& diagnostic) {
  sample=emptySample(NotReady);
  if (!ready) { diagnostic.cause=DiagnosticCause::NotReady; return false; }
  retryNotBeforeUs[anchor]=0;
  sample.transaction=++transaction; sample.attempts=1;
  diagnostic.transaction=sample.transaction;
  const uint16_t address=0xA1+anchor;
  uint8_t poll[ST_HEADER_LEN],finalFrame[ST_HEADER_LEN];
  st_header(poll,ST_POLL,address,ST_TAG,session,transaction);
  st_header(finalFrame,ST_FINAL,address,ST_TAG,session,transaction);
  const uint64_t mcuStart=nowUs();
  const uint32_t started=radioNow();
  diagnostic.radioStart=diagnostic.radioEnd=started;
  exchangeDeadline=started+st_ticks(ST_EXCHANGE_US);
  idleRadio();
  dwt_setrxtimeout(ST_RX_TIMEOUT_UUS);
  dwt_writetxdata(sizeof(poll),poll,0);
  dwt_writetxfctrl(sizeof(poll)+FCS_LEN,0,0);
  diagnostic.stage=DiagnosticStage::PollTx;
  diagnostic.radioEnd=radioNow();
  if (!st_fits(diagnostic.radioEnd,exchangeDeadline,2500)) {
    diagnostic.cause=DiagnosticCause::Budget; sample.failure=Deadline; return false;
  }
  if (dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED)!=DWT_SUCCESS) {
    diagnostic.cause=DiagnosticCause::TxStartRejected; sample.failure=PollStart; return false;
  }
  // Until TX completion is observed, allow its whole 650 us wait window.
  retryNotBeforeUs[anchor]=nowUs()+650U+ST_ANCHOR_US+ST_CLEANUP_US;
  uint32_t status=waitUntil(SYS_STATUS_TXFRS_BIT_MASK,started+st_ticks(650),diagnostic);
  if (!(status&SYS_STATUS_TXFRS_BIT_MASK)) { sample.failure=PollTimeout; return false; }
  diagnostic.pollTxConfirmed=true;
  // This MCU time is an upper bound on the observed POLL-TX event. Let the
  // anchor's previous transaction expire before sending it another POLL.
  retryNotBeforeUs[anchor]=nowUs()+ST_ANCHOR_US+ST_CLEANUP_US;
  const uint32_t pollTx=dwt_readtxtimestamplo32();
  const uint32_t pollRef=dwt_readtxtimestamphi32();
  UwbDsTwr::clearTxState();
  diagnostic.stage=DiagnosticStage::ResponseRx;
  status=waitUntil(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR,
                   started+st_ticks(ST_RESPONSE_WAIT_US),diagnostic);
  if (!(status&SYS_STATUS_RXFCG_BIT_MASK)) { sample.failure=ResponseTimeout; return false; }
  const uint32_t responseRx=dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
  uint16_t length;
  if (!readFrame(length)) {
    diagnostic.cause=DiagnosticCause::FrameLength; sample.failure=ResponseInvalid; return false;
  }
  if (!st_matches(receiveBuffer,length,ST_RESP,ST_TAG,address,session,transaction)) {
    diagnostic.cause=DiagnosticCause::FrameIdentity; sample.failure=ResponseInvalid; return false;
  }
  UwbDsTwr::clearRxState();
  // Real MCU turnaround margin; DS-TWR uses the measured RF timestamps.
  delayMicroseconds(200);
  UwbDsTwr::clearTxState();
  dwt_writetxdata(sizeof(finalFrame),finalFrame,0);
  dwt_writetxfctrl(sizeof(finalFrame)+FCS_LEN,0,0);
  diagnostic.stage=DiagnosticStage::FinalTx;
  diagnostic.radioEnd=radioNow();
  if (!st_fits(diagnostic.radioEnd,exchangeDeadline,ST_REPORT_RESERVE_US)) {
    diagnostic.cause=DiagnosticCause::Budget; sample.failure=Deadline; return false;
  }
  if (dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED)!=DWT_SUCCESS) {
    diagnostic.cause=DiagnosticCause::TxStartRejected; sample.failure=FinalStart; return false;
  }
  status=waitUntil(SYS_STATUS_TXFRS_BIT_MASK,st_earlier(exchangeDeadline,radioNow()+st_ticks(650)),diagnostic);
  if (!(status&SYS_STATUS_TXFRS_BIT_MASK)) { sample.failure=FinalTimeout; return false; }
  const uint32_t finalTx=dwt_readtxtimestamplo32();
  UwbDsTwr::clearTxState();
  diagnostic.stage=DiagnosticStage::ReportRx;
  status=waitUntil(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR,exchangeDeadline,diagnostic);
  if (!(status&SYS_STATUS_RXFCG_BIT_MASK)) { sample.failure=ReportTimeout; return false; }
  if (!readFrame(length)) {
    diagnostic.cause=DiagnosticCause::FrameLength; sample.failure=ReportInvalid; return false;
  }
  if (!st_matches(receiveBuffer,length,ST_REPORT,ST_TAG,address,session,transaction)) {
    diagnostic.cause=DiagnosticCause::FrameIdentity; sample.failure=ReportInvalid; return false;
  }
  if (receiveBuffer[ST_STATUS_IDX]!=1) {
    diagnostic.cause=DiagnosticCause::ReportRejected; sample.failure=ReportInvalid; return false;
  }
  diagnostic.stage=DiagnosticStage::Distance;
  const uint32_t pollRx=st_read32(receiveBuffer+ST_POLL_RX_IDX);
  const uint32_t responseTx=st_read32(receiveBuffer+ST_RESP_TX_IDX);
  const uint32_t finalRx=st_read32(receiveBuffer+ST_FINAL_RX_IDX);
  const uint32_t round1=responseRx-pollTx, reply1=responseTx-pollRx;
  const uint32_t round2=finalRx-responseTx, reply2=finalTx-responseRx;
  const double denominator=(double)round1+reply1+round2+reply2;
  const double maxDtu=(ST_EXCHANGE_US+500.0)*1e-6/DWT_TIME_UNITS;
  if (!(denominator>0) || round1>maxDtu || round2>maxDtu || reply1>maxDtu || reply2>maxDtu) {
    diagnostic.cause=DiagnosticCause::TofInvalid; sample.failure=TofInvalid; return false;
  }
  const double tof=((double)round1*round2-(double)reply1*reply2)/denominator;
  const double distance=tof*DWT_TIME_UNITS*299702547.0;
  if (!isfinite(distance) || !(distance>0 && distance<=80)) {
    diagnostic.cause=DiagnosticCause::RangeInvalid; sample.failure=RangeInvalid; return false;
  }
  // RF representative event = midpoint of this tag's POLL-TX and FINAL-TX.
  // Map it to esp_timer with one bracketed, refreshed DW SYS_TIME read.
  const uint32_t midpoint=pollRef+(uint32_t)(finalTx-pollTx)/512U;
  diagnostic.stage=DiagnosticStage::ClockMap;
  const uint64_t m0=nowUs();
  const uint32_t r=radioNow();
  const uint64_t m1=nowUs();
  diagnostic.radioEnd=r;
  const uint32_t delta=r-midpoint;
  if (m1-m0>500 || delta>st_ticks(5000)) {
    diagnostic.cause=DiagnosticCause::ClockMapInvalid; sample.failure=ClockInvalid; return false;
  }
  const uint64_t reference=m0+(m1-m0)/2;
  const uint32_t age=st_us(delta);
  if (reference<age || reference-age<mcuStart || reference-age>m1) {
    diagnostic.cause=DiagnosticCause::ClockMapInvalid; sample.failure=ClockInvalid; return false;
  }
  sample.timeUs=reference-age; sample.rfMapSpanUs=(uint16_t)(m1-m0);
  sample.distance=(float)distance; sample.failure=Ok;
  diagnostic.stage=DiagnosticStage::Done; diagnostic.cause=DiagnosticCause::Success;
  return true;
}
void attempt(unsigned index,Sample& sample,uint8_t ordinal=1) {
  const uint32_t firstTransaction=sample.firstTransaction;
  const uint8_t firstFailure=sample.firstFailure,firstCause=sample.firstCause;
  AttemptDiagnostic diagnostic={};
  diagnostic.boot=bootId; diagnostic.session=session; diagnostic.seq=diagnosticCycle;
  diagnostic.anchor=index+1; diagnostic.attempt=ordinal; diagnostic.startUs=nowUs();
  const bool ok=rangeOne(index,sample,diagnostic);
  diagnostic.endUs=nowUs(); diagnostic.failure=sample.failure;
  sample.cause=(uint8_t)diagnostic.cause;
  sample.firstTransaction=ordinal==1?sample.transaction:firstTransaction;
  sample.firstFailure=ordinal==1?sample.failure:firstFailure;
  sample.firstCause=ordinal==1?sample.cause:firstCause;
  idleRadio();
  // Without TX confirmation, a descheduled MCU cannot bound TX by the poll
  // wait alone. Forced idle is the latest possible end of that local TX.
  if (!diagnostic.pollTxConfirmed && retryNotBeforeUs[index])
    retryNotBeforeUs[index]=nowUs()+ST_ANCHOR_US+ST_CLEANUP_US;
  diagnostic.retryEligibleUs=retryNotBeforeUs[index];
  const bool txFailure=sample.failure==PollStart || sample.failure==PollTimeout ||
      sample.failure==FinalStart || sample.failure==FinalTimeout;
  txFaults=txFailure ? txFaults+1 : 0;
  if (txFaults>=2) scheduleRecovery(RecoveryReason::TxFaultStreak);
  if (!ok) sample.distance=NAN;
  recordAttemptDiagnostic(diagnostic);
}
}

void setup() {
  bootId=esp_random(); if (!bootId) bootId=1;
  beginRuntime(bootId);
  JetsonTagDisplay::begin();
  recoveryDetectedUs=nowUs();
  recoveryEvent(RecoveryEvent::InitStart);
  ready=initializeUwb();
  lastInitEndUs=nowUs();
  recoveryNotBeforeUs=ready?0:lastInitEndUs+kRecoveryBackoffUs;
  nextStartUs=nowUs()+ST_SESSION_QUIET_US;
  recoveryEvent(ready?RecoveryEvent::InitSuccess:RecoveryEvent::InitFailed);
}
void loop() {
  if (takeUwbRecovery()) {
    scheduleRecovery(RecoveryReason::BenchRequest);
  }
  if (!ready && nowUs()>=recoveryNotBeforeUs) {
    hasRecoveryAttempt=true; ++recoveries;
    recoveryEvent(RecoveryEvent::InitStart);
    ready=initializeUwb(); lastInitEndUs=nowUs();
    recoveryNotBeforeUs=ready?0:lastInitEndUs+kRecoveryBackoffUs;
    nextStartUs=lastInitEndUs+ST_SESSION_QUIET_US;
    recoveryEvent(ready?RecoveryEvent::InitSuccess:RecoveryEvent::InitFailed);
  }
  const uint64_t now=nowUs();
  if (now<nextStartUs) {
    if (nextStartUs-now>2000) delay(1); else delayMicroseconds(50);
    return;
  }
  // Drop missed periods. Never replay backlogged measurement cycles.
  if (now-nextStartUs>=kPeriodUs) {
    const uint32_t n=(uint32_t)((now-nextStartUs)/kPeriodUs);
    skipped+=n; sequence+=n; nextStartUs+=(uint64_t)n*kPeriodUs;
  }
  nextStartUs+=kPeriodUs;
  Cycle c={}; c.boot=bootId; c.session=session; c.seq=sequence++;
  diagnosticCycle=c.seq;
  c.startUs=nowUs(); c.ready=ready; c.skipped=skipped; c.recoveries=recoveries;
  const uint64_t deadline=c.startUs+ST_CYCLE_BUDGET_US;
  const bool paused=nowUs()<uwbPausedUntil();
  for (unsigned i=0;i<4;i++) {
    c.samples[i]=emptySample(NotReady);
    if (!ready || paused) continue;
    if (nowUs()+ST_EXCHANGE_US+ST_CLEANUP_US>deadline) {
      c.samples[i].failure=c.samples[i].firstFailure=Deadline;
      c.samples[i].cause=c.samples[i].firstCause=(uint8_t)DiagnosticCause::Budget;
      continue;
    }
    attempt(i,c.samples[i]);
  }
  // Every anchor's first attempt precedes the one optional retry.
  for (unsigned i=0;i<4 && ready && !paused;i++) {
    if (c.samples[i].attempts==1 && c.samples[i].failure!=Ok && c.samples[i].failure!=ClockInvalid) {
      const uint64_t cleanupDone=nowUs()+ST_CLEANUP_US;
      const uint64_t eligible=retryNotBeforeUs[i]>cleanupDone?retryNotBeforeUs[i]:cleanupDone;
      if (eligible+ST_EXCHANGE_US+ST_CLEANUP_US>deadline) continue;
      for (;;) {
        const uint64_t current=nowUs();
        if (current>=eligible) break;
        const uint64_t remaining=eligible-current;
        delayMicroseconds(remaining>50?50:(uint32_t)remaining);
      }
      // A task preemption during the cooldown must not start an over-budget TX.
      if (nowUs()+ST_EXCHANGE_US+ST_CLEANUP_US>deadline) continue;
      attempt(i,c.samples[i],2); c.samples[i].attempts=2;
      break;
    }
  }
  if (ready) idleRadio();
  c.endUs=nowUs(); c.deadlineMissed=c.endUs>deadline; c.ready=ready;
  for (unsigned i=0;i<4;i++) if(c.samples[i].failure==Ok) c.mask|=1U<<i;
  JetsonTagDisplay::render(ready,c.mask,false);
  publishCycle(c);
}
