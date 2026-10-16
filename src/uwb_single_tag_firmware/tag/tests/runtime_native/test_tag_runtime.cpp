#include <cstdio>
#include <limits>
#include "RuntimeFakes.h"
#include "../../src/SingleTagRuntime.cpp"
namespace SingleTag { void startLoRaTask() {} }

namespace Test {
using namespace SingleTag;
using RuntimeFake::check;
Cycle example() {
  Cycle c={}; c.boot=11; c.session=22; c.seq=33; c.ready=true; c.mask=15;
  c.startUs=RuntimeFake::us-10000; c.endUs=RuntimeFake::us;
  for(unsigned i=0;i<4;i++) {
    Sample& s=c.samples[i]; s.distance=3.f+i; s.timeUs=c.startUs+2000*i;
    s.transaction=s.firstTransaction=i+1; s.attempts=1; s.rfMapSpanUs=40;
    s.cause=s.firstCause=(uint8_t)DiagnosticCause::Success;
  }
  c.samples[3].firstFailure=ResponseTimeout; c.samples[3].firstCause=(uint8_t)DiagnosticCause::RxTimeout;
  c.samples[3].attempts=2; c.samples[3].transaction=5;
  return c;
}
AttemptDiagnostic failed(uint32_t seq=33) {
  AttemptDiagnostic d={}; d.boot=11; d.session=22; d.seq=seq; d.anchor=4; d.attempt=1;
  d.transaction=4; d.failure=ResponseTimeout; d.cause=DiagnosticCause::RxTimeout;
  d.stage=DiagnosticStage::ResponseRx; d.startUs=RuntimeFake::us-1000; d.endUs=RuntimeFake::us;
  d.wait.stage=d.stage; d.wait.startUs=d.startUs; d.wait.endUs=d.endUs;
  return d;
}
void runIO(unsigned count) {
  RuntimeFake::ticks=0; RuntimeFake::stopAfter=count;
  try { ioTask(nullptr); } catch(const RuntimeFake::Stop&) {}
}
void jsonSizes() {
  beginRuntime(11); Cycle c=example(); char line[2048];
  auto n=serializeRaw(c,line,sizeof(line));
  check(n>0 && n==std::strlen(line) && line[n-1]=='\n',"complete RAW JSON");
  std::fputs(line,stdout);
  c.mask=7; c.samples[3].failure=ResponseTimeout; c.samples[3].cause=(uint8_t)DiagnosticCause::RxError;
  c.samples[3].distance=NAN; c.samples[3].timeUs=0;
  n=serializeRaw(c,line,sizeof(line)); check(n>0,"failed RAW fits"); std::fputs(line,stdout);
  c.boot=c.session=c.seq=c.skipped=c.recoveries=UINT32_MAX;
  c.startUs=c.endUs=UINT64_MAX;
  for(auto& s:c.samples) {
    s.transaction=s.firstTransaction=UINT32_MAX; s.timeUs=UINT64_MAX; s.rfMapSpanUs=UINT16_MAX;
  }
  n=serializeRaw(c,line,sizeof(line)); check(n>0 && n<sizeof(line),"maximum-width RAW fits 2048");
  std::fputs(line,stdout);
  latest=c; RuntimeFake::us=UINT64_MAX; pose.received=true; pose.sampleUs=0; pose.seq=UINT32_MAX;
  generation=UINT32_MAX; rawDrops=auxDrops=UINT32_MAX;
  diagnosticAttempts=diagnosticDrops=diagnosticHighWater=UINT32_MAX;
  diagnosticStale=invalidInput=overflowInput=staleRaw=inputMaxUs=inputBytes=UINT32_MAX;
  radio.attempts=radio.done=radio.timeouts=radio.watchdog=radio.recoveries=UINT32_MAX;
  radio.rx=radio.rxErrors=radio.serviceMaxUs=UINT32_MAX;
  for(unsigned i=0;i<kDiagnosticCauses-1;i++) diagnosticFailures[i]=UINT32_MAX;
  n=heartbeat(line,sizeof(line)); check(n>0 && n<sizeof(line),"maximum-width heartbeat fits 2048");
  std::fputs(line,stdout);
  char small[64]; std::memset(small,'#',sizeof(small));
  check(serializeRaw(c,small,32)==0 && small[32]=='#',"truncated RAW is not sent or overrun");
  check(heartbeat(small,32)==0 && small[32]=='#',"truncated heartbeat is not sent or overrun");
}
void queueBounds() {
  beginRuntime(11); auto d=failed();
  d.failure=Ok; d.cause=DiagnosticCause::Success;
  recordAttemptDiagnostic(d);
  check(diagnosticAttempts==1 && uxQueueMessagesWaiting(diagnosticQueue)==0,"first success is counter-only");
  d.attempt=2; recordAttemptDiagnostic(d);
  check(diagnosticAttempts==2 && uxQueueMessagesWaiting(diagnosticQueue)==1,"successful retry is detailed");
  d=failed();
  for(unsigned i=0;i<20;i++) { d.seq=i; recordAttemptDiagnostic(d); }
  check(diagnosticAttempts==22 && diagnosticFailures[(unsigned)DiagnosticCause::RxTimeout]==20,"every failure is counted even on queue loss");
  check(uxQueueMessagesWaiting(diagnosticQueue)==16 && diagnosticDrops==5 && diagnosticHighWater==16,"bounded queue and exact overflow accounting");
  check(diagnosticFailures[(unsigned)DiagnosticCause::Success]==0,"success never enters failure counters");
  const Cycle c=example(); publishCycle(c); Cycle queued={};
  check(xQueueReceive(rawQueue,&queued,0)==pdTRUE && queued.samples[3].firstFailure==ResponseTimeout,
        "full diagnostic queue cannot erase first failure from RAW");
}
void rawPriority() {
  beginRuntime(11); auto c=example(); publishCycle(c); recordAttemptDiagnostic(failed());
  Serial.allowance=17; runIO(220);
  const auto status=Serial.output.find("uwb_raw_status"), raw=Serial.output.find("uwb_raw_cycle");
  const auto diagnostic=Serial.output.find("uwb_rf_attempt");
  check(status<raw && raw<diagnostic,"heartbeat then RAW precede pending diagnostics");
  check(!Serial.output.empty() && Serial.output.back()=='\n',"partial UART writes finish complete lines");
  check(std::count(Serial.output.begin(),Serial.output.end(),'\n')==3,"three complete lines only");
  std::fputs(Serial.output.c_str(),stdout);
}
void staleAndOldSession() {
  beginRuntime(11); auto c=example(); c.session=23; publishCycle(c);
  auto d=failed(); d.endUs=RuntimeFake::us-kDiagnosticMaxAgeUs-1;
  recordAttemptDiagnostic(d);
  d.endUs=RuntimeFake::us; recordAttemptDiagnostic(d);
  runIO(40);
  check(diagnosticStale==1,"expired diagnostic must be counted and dropped");
  check(Serial.output.find("uwb_rf_attempt")!=std::string::npos,"old session event is retained within TTL");
  check(Serial.output.find("\"session\":22,\"seq\":33,\"anchor\":4")!=std::string::npos,
        "old event retains its source identity");
  std::fputs(Serial.output.c_str(),stdout);
}
void stalledUSB() {
  beginRuntime(11); Serial.allowance=0;
  RuntimeFake::onTick=[] {
    if(RuntimeFake::ticks<500 && RuntimeFake::ticks%20==0) {
      auto c=example(); c.seq=RuntimeFake::ticks/20; publishCycle(c);
      recordAttemptDiagnostic(failed(c.seq));
    }
    if(RuntimeFake::ticks==500) {
      check(Serial.output.empty() && diagnosticDrops>0 && rawDrops>0,"stalled USB bounds queues without producer wait");
      const auto last=currentCycle();
      check(last.seq==24 && last.samples[3].firstFailure==ResponseTimeout,"newest RAW survives USB stall");
      Serial.allowance=256;
    }
  };
  runIO(580); RuntimeFake::onTick={};
  check(Serial.output.find("uwb_raw_cycle")!=std::string::npos,"RAW resumes after USB capacity returns");
  check(std::count(Serial.output.begin(),Serial.output.end(),'\n')>1,"output remains framed after stall");
  std::fputs(Serial.output.c_str(),stdout);
}
}

int main(int argc,char** argv) {
  const std::vector<std::pair<std::string,std::function<void()>>> cases={
    {"runtime_json_sizes",Test::jsonSizes},{"runtime_queue_bounds",Test::queueBounds},
    {"runtime_raw_priority",Test::rawPriority},{"runtime_stale_and_old_session",Test::staleAndOldSession},
    {"runtime_stalled_usb",Test::stalledUSB}};
  if(argc==2 && std::string(argv[1])=="--list") {
    for(const auto& c:cases) std::puts(c.first.c_str());
    return 0;
  }
  for(const auto& c:cases) if(argc==2 && c.first==argv[1]) {
    try { c.second(); std::printf("PASS %s\n",argv[1]); return 0; }
    catch(const std::exception& e) { std::fprintf(stderr,"FAIL %s: %s\n",argv[1],e.what()); return 1; }
  }
  return 2;
}
