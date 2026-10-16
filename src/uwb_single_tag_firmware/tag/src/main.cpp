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
uint32_t lastRecoveryMs=0, exchangeDeadline=0;
uint8_t txFaults=0;
uint64_t nextStartUs=0;
uint8_t receiveBuffer[64];
uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }
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
uint32_t waitUntil(uint32_t mask,uint32_t deadline) {
  const uint32_t r=radioNow();
  const uint64_t watchdog=nowUs()+(st_before(r,deadline)?st_us(deadline-r):0)+300;
  while (nowUs()<watchdog) {
    const uint32_t status=dwt_read32bitreg(SYS_STATUS_ID);
    if (!st_before(radioNow(),deadline)) return 0;
    if (status&mask) return status;
    delayMicroseconds(ST_POLL_PAUSE_US);
  }
  ready=false;
  lastRecoveryMs=millis();
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
  return true;
}
Sample emptySample(uint8_t failure) {
  Sample s={}; s.distance=NAN; s.failure=failure; return s;
}
bool rangeOne(unsigned anchor,Sample& sample) {
  sample=emptySample(NotReady);
  if (!ready) return false;
  sample.transaction=++transaction; sample.attempts=1;
  const uint16_t address=0xA1+anchor;
  uint8_t poll[ST_HEADER_LEN],finalFrame[ST_HEADER_LEN];
  st_header(poll,ST_POLL,address,ST_TAG,session,transaction);
  st_header(finalFrame,ST_FINAL,address,ST_TAG,session,transaction);
  const uint64_t mcuStart=nowUs();
  const uint32_t started=radioNow();
  exchangeDeadline=started+st_ticks(ST_EXCHANGE_US);
  idleRadio();
  dwt_setrxtimeout(ST_RX_TIMEOUT_UUS);
  dwt_writetxdata(sizeof(poll),poll,0);
  dwt_writetxfctrl(sizeof(poll)+FCS_LEN,0,0);
  if (!st_fits(radioNow(),exchangeDeadline,2500)) { sample.failure=Deadline; return false; }
  if (dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED)!=DWT_SUCCESS) {
    sample.failure=PollStart; return false;
  }
  uint32_t status=waitUntil(SYS_STATUS_TXFRS_BIT_MASK,started+st_ticks(650));
  if (!(status&SYS_STATUS_TXFRS_BIT_MASK)) { sample.failure=PollTimeout; return false; }
  const uint32_t pollTx=dwt_readtxtimestamplo32();
  const uint32_t pollRef=dwt_readtxtimestamphi32();
  UwbDsTwr::clearTxState();
  status=waitUntil(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR,
                   started+st_ticks(ST_RESPONSE_WAIT_US));
  if (!(status&SYS_STATUS_RXFCG_BIT_MASK)) { sample.failure=ResponseTimeout; return false; }
  const uint32_t responseRx=dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
  uint16_t length;
  if (!readFrame(length) || !st_matches(receiveBuffer,length,ST_RESP,ST_TAG,address,session,transaction)) {
    sample.failure=ResponseInvalid; return false;
  }
  UwbDsTwr::clearRxState();
  // Real MCU turnaround margin; DS-TWR uses the measured RF timestamps.
  delayMicroseconds(200);
  UwbDsTwr::clearTxState();
  dwt_writetxdata(sizeof(finalFrame),finalFrame,0);
  dwt_writetxfctrl(sizeof(finalFrame)+FCS_LEN,0,0);
  if (!st_fits(radioNow(),exchangeDeadline,ST_REPORT_RESERVE_US)) {
    sample.failure=Deadline; return false;
  }
  if (dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED)!=DWT_SUCCESS) {
    sample.failure=FinalStart; return false;
  }
  status=waitUntil(SYS_STATUS_TXFRS_BIT_MASK,st_earlier(exchangeDeadline,radioNow()+st_ticks(650)));
  if (!(status&SYS_STATUS_TXFRS_BIT_MASK)) { sample.failure=FinalTimeout; return false; }
  const uint32_t finalTx=dwt_readtxtimestamplo32();
  UwbDsTwr::clearTxState();
  status=waitUntil(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR,exchangeDeadline);
  if (!(status&SYS_STATUS_RXFCG_BIT_MASK)) { sample.failure=ReportTimeout; return false; }
  if (!readFrame(length) || !st_matches(receiveBuffer,length,ST_REPORT,ST_TAG,address,session,transaction)
      || receiveBuffer[ST_STATUS_IDX]!=1) { sample.failure=ReportInvalid; return false; }
  const uint32_t pollRx=st_read32(receiveBuffer+ST_POLL_RX_IDX);
  const uint32_t responseTx=st_read32(receiveBuffer+ST_RESP_TX_IDX);
  const uint32_t finalRx=st_read32(receiveBuffer+ST_FINAL_RX_IDX);
  const uint32_t round1=responseRx-pollTx, reply1=responseTx-pollRx;
  const uint32_t round2=finalRx-responseTx, reply2=finalTx-responseRx;
  const double denominator=(double)round1+reply1+round2+reply2;
  const double maxDtu=(ST_EXCHANGE_US+500.0)*1e-6/DWT_TIME_UNITS;
  if (!(denominator>0) || round1>maxDtu || round2>maxDtu || reply1>maxDtu || reply2>maxDtu) {
    sample.failure=TofInvalid; return false;
  }
  const double tof=((double)round1*round2-(double)reply1*reply2)/denominator;
  const double distance=tof*DWT_TIME_UNITS*299702547.0;
  if (!isfinite(distance) || !(distance>0 && distance<=80)) {
    sample.failure=RangeInvalid; return false;
  }
  // RF representative event = midpoint of this tag's POLL-TX and FINAL-TX.
  // Map it to esp_timer with one bracketed, refreshed DW SYS_TIME read.
  const uint32_t midpoint=pollRef+(uint32_t)(finalTx-pollTx)/512U;
  const uint64_t m0=nowUs();
  const uint32_t r=radioNow();
  const uint64_t m1=nowUs();
  const uint32_t delta=r-midpoint;
  if (m1-m0>500 || delta>st_ticks(5000)) { sample.failure=ClockInvalid; return false; }
  const uint64_t reference=m0+(m1-m0)/2;
  const uint32_t age=st_us(delta);
  if (reference<age || reference-age<mcuStart || reference-age>m1) {
    sample.failure=ClockInvalid; return false;
  }
  sample.timeUs=reference-age; sample.rfMapSpanUs=(uint16_t)(m1-m0);
  sample.distance=(float)distance; sample.failure=Ok;
  return true;
}
void attempt(unsigned index,Sample& sample) {
  const bool ok=rangeOne(index,sample);
  idleRadio();
  const bool txFailure=sample.failure==PollStart || sample.failure==PollTimeout ||
      sample.failure==FinalStart || sample.failure==FinalTimeout;
  txFaults=txFailure ? txFaults+1 : 0;
  if (txFaults>=2) { ready=false; lastRecoveryMs=millis(); }
  if (!ok) sample.distance=NAN;
}
}

void setup() {
  bootId=esp_random(); if (!bootId) bootId=1;
  beginRuntime(bootId);
  JetsonTagDisplay::begin();
  ready=initializeUwb();
  lastRecoveryMs=millis();
  nextStartUs=nowUs()+ST_SESSION_QUIET_US;
}
void loop() {
  if (takeUwbRecovery()) { ready=false; lastRecoveryMs=millis()-1500U; }
  if (!ready && (uint32_t)(millis()-lastRecoveryMs)>=1500U) {
    lastRecoveryMs=millis(); ++recoveries;
    ready=initializeUwb(); nextStartUs=nowUs()+ST_SESSION_QUIET_US;
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
  c.startUs=nowUs(); c.ready=ready; c.skipped=skipped; c.recoveries=recoveries;
  const uint64_t deadline=c.startUs+ST_CYCLE_BUDGET_US;
  const bool paused=nowUs()<uwbPausedUntil();
  for (unsigned i=0;i<4;i++) {
    c.samples[i]=emptySample(NotReady);
    if (!ready || paused) continue;
    if (nowUs()+ST_EXCHANGE_US+ST_CLEANUP_US>deadline) { c.samples[i].failure=Deadline; continue; }
    attempt(i,c.samples[i]);
  }
  // Every anchor's first attempt precedes the one optional retry.
  for (unsigned i=0;i<4 && ready && !paused;i++) {
    if (c.samples[i].failure!=Ok && c.samples[i].failure!=ClockInvalid &&
        nowUs()+ST_EXCHANGE_US+ST_CLEANUP_US<=deadline) {
      delayMicroseconds(ST_CLEANUP_US);
      attempt(i,c.samples[i]); c.samples[i].attempts=2;
      break;
    }
  }
  if (ready) idleRadio();
  c.endUs=nowUs(); c.deadlineMissed=c.endUs>deadline; c.ready=ready;
  for (unsigned i=0;i<4;i++) if(c.samples[i].failure==Ok) c.mask|=1U<<i;
  JetsonTagDisplay::render(ready,c.mask,false);
  publishCycle(c);
}
