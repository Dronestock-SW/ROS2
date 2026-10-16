#include "SingleTagRuntime.h"
#include "JetsonTagDisplay.h"
#include <Arduino.h>
#include "SingleTagDiagnosticJson.h"
#include <esp_timer.h>
#include <esp_system.h>
#include <driver/uart.h>
#include <cJSON.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace SingleTag {
namespace {
portMUX_TYPE sharedLock=portMUX_INITIALIZER_UNLOCKED;
Cycle latest={}; Pose pose={}; RadioStatus radio={};
uint64_t syncExpires=0,pauseUntil=0;
uint32_t generation=0;
std::atomic<uint32_t> rawDrops{0},auxDrops{0};
std::atomic<uint32_t> diagnosticAttempts{0},diagnosticDrops{0},diagnosticHighWater{0};
std::atomic<uint32_t> diagnosticFailures[kDiagnosticCauses]{};
uint32_t diagnosticStale=0;
bool recoverUwb=false,recoverLoRa=false,testLoRa=false;
StaticQueue_t rawQueueData,auxQueueData,diagnosticQueueData;
uint8_t rawStorage[sizeof(Cycle)];
struct Aux { char line[1536]; };
uint8_t auxStorage[4*sizeof(Aux)];
uint8_t diagnosticStorage[kDiagnosticQueueDepth*sizeof(DiagnosticEvent)];
QueueHandle_t rawQueue,auxQueue,diagnosticQueue;
uint32_t invalidInput=0,overflowInput=0,staleRaw=0;
uint32_t inputMaxUs=0,inputBytes=0;
uint32_t hostSession=0,requestSeq=0,repliedSeq=0,committedSeq=0;
bool haveRequest=false,haveCommit=false;
uint64_t requestHostTime=0,repliedTime=0;
uint64_t lastSyncRequestUs=0;
uint32_t retiredHosts[4]={},retiredNext=0;
struct Sync { bool pending; uint32_t host,seq,boot,session; uint64_t t1,t2; } sync={};

uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }
const char* jb(bool b) { return b?"true":"false"; }
bool add(char* dst,size_t cap,size_t& used,const char* fmt,...) {
  if(used>=cap) return false;
  va_list args; va_start(args,fmt); int n=vsnprintf(dst+used,cap-used,fmt,args); va_end(args);
  if(n<0 || (size_t)n>=cap-used) { used=cap; return false; }
  used+=n; return true;
}
bool integer(cJSON* root,const char* key,uint64_t& value,uint64_t max=9007199254740991ULL) {
  cJSON* v=cJSON_GetObjectItemCaseSensitive(root,key);
  if(!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || v->valuedouble<0 ||
      v->valuedouble>(double)max || floor(v->valuedouble)!=v->valuedouble) return false;
  value=(uint64_t)v->valuedouble; return true;
}
bool textEquals(cJSON* root,const char* key,const char* expected) {
  cJSON* v=cJSON_GetObjectItemCaseSensitive(root,key);
  return cJSON_IsString(v) && strcmp(v->valuestring,expected)==0;
}
bool uniqueFlatObject(cJSON* root) {
  if(!cJSON_IsObject(root)) return false;
  for(cJSON* a=root->child;a;a=a->next) {
    if(!a->string || cJSON_IsArray(a) || cJSON_IsObject(a)) return false;
    for(cJSON* b=a->next;b;b=b->next) if(!strcmp(a->string,b->string)) return false;
  }
  return true;
}
bool identity(cJSON* root,const Cycle& c) {
  uint64_t schema,boot,session;
  return integer(root,"schema",schema,2) && schema==2 && textEquals(root,"tag_id","6") &&
      integer(root,"boot_id",boot,UINT32_MAX) && boot==c.boot &&
      integer(root,"session",session,UINT32_MAX) && session==c.session && session!=0;
}
void ackPose(bool accepted,uint32_t seq,const char* reason) {
  char line[320];
  snprintf(line,sizeof(line),"{\"type\":\"jetson_pose_ack\",\"schema\":2,\"tag_id\":\"6\","
      "\"accepted\":%s,\"pose_seq\":%lu,\"lora_eligible\":%s,\"reason\":\"%s\"}\n",
      jb(accepted),(unsigned long)seq,jb(accepted && currentPose().fix),reason);
  queueAux(line);
}
bool acceptInput(cJSON* root,uint64_t received) {
  const Cycle c=currentCycle();
  if(!identity(root,c)) return false;
  uint64_t host;
  if(!integer(root,"host_session",host,UINT32_MAX) || host==0) return false;
  if(textEquals(root,"type","time_sync")) {
    uint64_t seq,t1;
    if(!integer(root,"seq",seq,UINT32_MAX) || !integer(root,"t1_host_us",t1) || !t1 || sync.pending ||
        (lastSyncRequestUs && received-lastSyncRequestUs<100000)) return false;
    lastSyncRequestUs=received;
    if(host!=hostSession) {
      for(unsigned i=0;i<4;i++) if(retiredHosts[i]==host) return false;
      if(hostSession) retiredHosts[retiredNext++ & 3U]=hostSession;
      hostSession=host; haveRequest=false; haveCommit=false;
      repliedTime=0; repliedSeq=0;
      portENTER_CRITICAL(&sharedLock); pose={}; syncExpires=0; generation=0; portEXIT_CRITICAL(&sharedLock);
    }
    if(haveRequest && ((int32_t)((uint32_t)seq-requestSeq)<=0 || t1<=requestHostTime)) return false;
    haveRequest=true; requestSeq=seq; requestHostTime=t1;
    sync={true,(uint32_t)host,(uint32_t)seq,c.boot,c.session,t1,received};
    return true;
  }
  if(host!=hostSession) return false;
  if(textEquals(root,"type","time_sync_commit")) {
    uint64_t seq,gen,error;
    if(!repliedTime || !integer(root,"seq",seq,UINT32_MAX) || seq!=repliedSeq ||
       !integer(root,"generation",gen,UINT32_MAX) || !gen ||
       !integer(root,"uncertainty_us",error,5000) || received<repliedTime ||
       received-repliedTime>500000 ||
       (haveCommit && (int32_t)((uint32_t)seq-committedSeq)<=0)) return false;
    haveCommit=true; committedSeq=seq;
    portENTER_CRITICAL(&sharedLock); syncExpires=repliedTime+2000000; generation=gen; portEXIT_CRITICAL(&sharedLock);
    return true;
  }
  if(textEquals(root,"type","jetson_pose")) {
    uint64_t seq,sample,source,quality;
    cJSON* fix=cJSON_GetObjectItemCaseSensitive(root,"fix");
    if(!integer(root,"seq",seq,UINT32_MAX) || !integer(root,"source_seq",source,UINT32_MAX) ||
       !integer(root,"sample_time_us",sample) || !integer(root,"quality",quality,100) ||
       !cJSON_IsBool(fix) || !textEquals(root,"clock_domain","esp32_monotonic_boot_us")) return false;
    const Pose previous=currentPose();
    const bool fresh=clockSynchronized() && sample<=received && received-sample<=500000 &&
      (int32_t)(c.seq-(uint32_t)source)>=0 && (uint32_t)(c.seq-source)<=25 &&
      sample<=c.endUs && (!previous.received ||
       ((int32_t)((uint32_t)seq-previous.seq)>0 && sample>previous.sampleUs));
    if(!fresh) { ackPose(false,seq,"stale_or_duplicate"); return false; }
    Pose next={}; next.received=true; next.fix=cJSON_IsTrue(fix); next.seq=seq;
    next.sourceSeq=source; next.boot=c.boot; next.session=c.session; next.hostSession=host;
    next.sampleUs=sample; next.quality=quality;
    if(next.fix) {
      cJSON* x=cJSON_GetObjectItemCaseSensitive(root,"x_m");
      cJSON* y=cJSON_GetObjectItemCaseSensitive(root,"y_m");
      if(!cJSON_IsNumber(x) || !cJSON_IsNumber(y) || !isfinite(x->valuedouble) ||
         !isfinite(y->valuedouble) || fabs(x->valuedouble)>10000 || fabs(y->valuedouble)>10000) return false;
      next.x=x->valuedouble; next.y=y->valuedouble;
    }
    cJSON* reason=cJSON_GetObjectItemCaseSensitive(root,"reason");
    const char* why=cJSON_IsString(reason)?reason->valuestring:(next.fix?"ok":"no_fix");
    if(strlen(why)>31) return false;
    for(const char* p=why;*p;p++) if(!((*p>='a'&&*p<='z') || (*p>='0'&&*p<='9') || *p=='_')) return false;
    snprintf(next.reason,sizeof(next.reason),"%s",why);
    portENTER_CRITICAL(&sharedLock); pose=next; portEXIT_CRITICAL(&sharedLock);
    ackPose(true,seq,why); return true;
  }
#if SINGLE_TAG_BENCH_CONTROLS
  if(textEquals(root,"type","bench") && clockSynchronized()) {
    uint64_t duration;
    if(textEquals(root,"action","uwb_pause") && integer(root,"duration_ms",duration,5000)) {
      portENTER_CRITICAL(&sharedLock); pauseUntil=received+duration*1000; portEXIT_CRITICAL(&sharedLock);
      return true;
    }
    portENTER_CRITICAL(&sharedLock);
    const bool uwb=textEquals(root,"action","uwb_recover");
    const bool lora=textEquals(root,"action","lora_recover");
    const bool packet=textEquals(root,"action","lora_255");
    recoverUwb|=uwb; recoverLoRa|=lora; testLoRa|=packet;
    portEXIT_CRITICAL(&sharedLock);
    return uwb || lora || packet;
  }
#endif
  return false;
}
void serviceInput() {
  static char line[768]; static size_t used=0; static bool discarding=false;
  // IDF's RX-ring-full recovery can wait one tick after every read. Read a
  // bounded block, not individual bytes, so a long invalid line cannot turn
  // hundreds of such waits into seconds of USB TX starvation.
  uint8_t incoming[512];
  const uint64_t started=nowUs();
  const size_t count=Serial.read(incoming,sizeof(incoming));
  inputBytes+=count;
  for(size_t i=0;i<count;i++) {
    const char ch=(char)incoming[i];
    if(ch=='\n') {
      if(!discarding && used) {
        line[used]=0; const char* end=nullptr;
        cJSON* root=cJSON_ParseWithOpts(line,&end,true);
        if(!root || !uniqueFlatObject(root) || !acceptInput(root,nowUs())) ++invalidInput;
        cJSON_Delete(root);
      }
      used=0; discarding=false;
    } else if(ch!='\r' && !discarding) {
      if((uint8_t)ch<32 || used+1>=sizeof(line)) { used=0; discarding=true; ++overflowInput; }
      else line[used++]=ch;
    }
  }
  const uint64_t duration=nowUs()-started;
  if(duration>inputMaxUs) inputMaxUs=duration;
}
size_t serializeRaw(const Cycle& c,char* line,size_t cap) {
  size_t n=0;
  add(line,cap,n,"{\"type\":\"uwb_raw_cycle\",\"schema\":2,\"tag_id\":\"6\",\"boot_id\":%lu,"
      "\"session\":%lu,\"seq\":%lu,\"period_us\":%lu,\"cycle_start_us\":%llu,\"cycle_end_us\":%llu,"
      "\"valid_mask\":%u,\"raw_slant_m\":[",(unsigned long)c.boot,(unsigned long)c.session,
      (unsigned long)c.seq,(unsigned long)kPeriodUs,(unsigned long long)c.startUs,
      (unsigned long long)c.endUs,c.mask);
  for(unsigned i=0;i<4;i++) {
    if(i) add(line,cap,n,",");
    if(c.samples[i].failure==Ok) add(line,cap,n,"%.4f",c.samples[i].distance);
    else add(line,cap,n,"null");
  }
  add(line,cap,n,"],\"sample_time_us\":[");
  for(unsigned i=0;i<4;i++) {
    if(i) add(line,cap,n,",");
    if(c.samples[i].failure==Ok) add(line,cap,n,"%llu",(unsigned long long)c.samples[i].timeUs);
    else add(line,cap,n,"null");
  }
  add(line,cap,n,"],\"failure\":[%u,%u,%u,%u],\"attempts\":[%u,%u,%u,%u],"
      "\"rf_map_span_us\":[%u,%u,%u,%u],\"clock_domain\":\"esp32_monotonic_boot_us\","
      "\"clock_sync_valid\":%s,\"clock_generation\":%lu,\"deadline_missed\":%s,"
      "\"skipped_cycles\":%lu,\"tx_dropped\":%lu,\"uwb_ready\":%s",
      c.samples[0].failure,c.samples[1].failure,c.samples[2].failure,c.samples[3].failure,
      c.samples[0].attempts,c.samples[1].attempts,c.samples[2].attempts,c.samples[3].attempts,
      c.samples[0].rfMapSpanUs,c.samples[1].rfMapSpanUs,c.samples[2].rfMapSpanUs,c.samples[3].rfMapSpanUs,
      jb(clockSynchronized()),(unsigned long)clockGeneration(),jb(c.deadlineMissed),
      (unsigned long)c.skipped,(unsigned long)rawDrops,jb(c.ready));
  if(n>=cap) return 0;
  const size_t diagnosticLength=serializeRawDiagnosticFields(c,line+n,cap-n);
  if(!diagnosticLength) return 0;
  n+=diagnosticLength;
  add(line,cap,n,"}\n");
  return n<cap?n:0;
}
size_t heartbeat(char* line,size_t cap) {
  const Cycle c=currentCycle(); const RadioStatus r=radioStatus();
  const Pose p=currentPose();
  const uint64_t now=nowUs();
  char poseAge[32];
  if(p.received && now>=p.sampleUs) snprintf(poseAge,sizeof(poseAge),"%llu",(unsigned long long)(now-p.sampleUs));
  else snprintf(poseAge,sizeof(poseAge),"null");
  const int n=snprintf(line,cap,"{\"type\":\"uwb_raw_status\",\"schema\":2,\"tag_id\":\"6\","
      "\"firmware\":\"%s\",\"boot_id\":%lu,\"session\":%lu,\"period_us\":%lu,\"uwb_ready\":%s,"
      "\"serial_baud\":921600,\"anchor_layout_id\":\"%s\",\"clock_domain\":\"esp32_monotonic_boot_us\","
      "\"anchor_count\":4,\"anchor_order\":[\"A1\",\"A2\",\"A3\",\"A4\"],"
      "\"range_bias_applied\":false,\"temporal_filter_applied\":false,"
      "\"timestamp_method\":\"rf_poll_final_midpoint\",\"rf_protocol\":2,\"clock_sync_valid\":%s,"
      "\"clock_generation\":%lu,\"lora_schedule\":\"independent_monotonic_1hz\","
      "\"lora_ready\":%s,\"lora_tx_busy\":%s,\"lora_tx_attempts\":%lu,\"lora_tx_done\":%lu,"
      "\"lora_tx_timeouts\":%lu,\"lora_watchdog_timeouts\":%lu,\"lora_recoveries\":%lu,"
      "\"lora_rx_packets\":%lu,\"lora_rx_errors\":%lu,\"lora_service_max_us\":%lu,"
      "\"uwb_recoveries\":%lu,\"input_invalid\":%lu,\"input_overflow\":%lu,\"aux_dropped\":%lu,"
      "\"raw_stale_dropped\":%lu,\"free_heap\":%u,\"min_free_heap\":%u,\"reset_reason\":%u,"
      "\"display_ready\":%s,\"bench_controls\":%s}\n",kVersion,(unsigned long)c.boot,
      (unsigned long)c.session,(unsigned long)kPeriodUs,jb(c.ready),kLayout,jb(clockSynchronized()),
      (unsigned long)clockGeneration(),jb(r.ready),jb(r.busy),(unsigned long)r.attempts,
      (unsigned long)r.done,(unsigned long)r.timeouts,(unsigned long)r.watchdog,
      (unsigned long)r.recoveries,(unsigned long)r.rx,(unsigned long)r.rxErrors,
      (unsigned long)r.serviceMaxUs,(unsigned long)c.recoveries,(unsigned long)invalidInput,
      (unsigned long)overflowInput,(unsigned long)auxDrops,(unsigned long)staleRaw,
      ESP.getFreeHeap(),ESP.getMinFreeHeap(),(unsigned)esp_reset_reason(),
      jb(JetsonTagDisplay::ready()),jb(SINGLE_TAG_BENCH_CONTROLS));
  if(n<=0 || (size_t)n>=cap) return 0;
  // Replace only the local terminator before this completed line is queued.
  const int extra=snprintf(line+n-2,cap-(n-2),",\"pose_received\":%s,\"pose_age_us\":%s,"
      "\"pose_seq\":%lu,\"lora_last_tx_fix\":%s,\"lora_last_tx_length\":%u,"
      "\"usb_input_max_us\":%lu,\"usb_input_bytes\":%lu",
      jb(p.received),poseAge,(unsigned long)p.seq,jb(r.lastTxFix),r.lastTxLength,
      (unsigned long)inputMaxUs,(unsigned long)inputBytes);
  if(extra<=0 || (size_t)extra>=cap-(n-2)) return 0;
  size_t used=n-2+extra;
  add(line,cap,used,",\"diag_version\":1,\"diag_attempts\":%lu,\"diag_dropped\":%lu,"
      "\"diag_stale\":%lu,\"diag_high_water\":%lu,\"diag_failures\":[",
      (unsigned long)diagnosticAttempts.load(),(unsigned long)diagnosticDrops.load(),
      (unsigned long)diagnosticStale,(unsigned long)diagnosticHighWater.load());
  for(unsigned i=0;i<kDiagnosticCauses;i++)
    add(line,cap,used,i?",%lu":"%lu",(unsigned long)diagnosticFailures[i].load());
  add(line,cap,used,"]}\n");
  return used<cap?used:0;
}
void ioTask(void*) {
  char active[2048]; size_t length=0,offset=0;
  char aux[sizeof(Aux)];
  uint64_t nextHeartbeat=nowUs();
  uint32_t seenSession=0;
  for(;;) {
    const Cycle c=currentCycle();
    if(c.session!=seenSession) {
      seenSession=c.session; sync.pending=false; hostSession=0; haveRequest=false; haveCommit=false;
      repliedTime=0; repliedSeq=0;
      portENTER_CRITICAL(&sharedLock); pose={}; syncExpires=0; generation=0; portEXIT_CRITICAL(&sharedLock);
    }
    serviceInput();
    if(!length && uart_wait_tx_done(UART_NUM_0,0)==ESP_OK) {
      const uint64_t now=nowUs();
      if(sync.pending) {
        sync.pending=false; repliedTime=now; repliedSeq=sync.seq;
        const int n=snprintf(active,sizeof(active),"{\"type\":\"time_sync_reply\",\"schema\":2,"
            "\"tag_id\":\"6\",\"boot_id\":%lu,\"session\":%lu,\"host_session\":%lu,\"seq\":%lu,"
            "\"t1_host_us\":%llu,\"t2_tag_us\":%llu,\"t3_tag_us\":%llu}\n",
            (unsigned long)sync.boot,(unsigned long)sync.session,(unsigned long)sync.host,
            (unsigned long)sync.seq,(unsigned long long)sync.t1,(unsigned long long)sync.t2,
            (unsigned long long)now);
        length=n>0 && n<(int)sizeof(active)?n:0;
      } else if(now>=nextHeartbeat) {
        nextHeartbeat=now+1000000; length=heartbeat(active,sizeof(active));
      } else {
        Cycle raw;
        if(xQueueReceive(rawQueue,&raw,0)==pdTRUE) {
          if(now>=raw.endUs && now-raw.endUs<=100000 && raw.session==c.session)
            length=serializeRaw(raw,active,sizeof(active));
          else ++staleRaw;
        }
        // Sustained auxiliary input must not starve new complete RAW frames.
        if(!length && xQueueReceive(auxQueue,aux,0)==pdTRUE) {
          length=strlen(aux); memcpy(active,aux,length);
        }
        // At most one diagnostic per service pass, only after RAW and auxiliary
        // output. Retain its original session even if RF recovered meanwhile.
        DiagnosticEvent diagnostic;
        if(!length && xQueueReceive(diagnosticQueue,&diagnostic,0)==pdTRUE) {
          const uint64_t at=diagnostic.recovery?diagnostic.reset.atUs:diagnostic.attempt.endUs;
          if(now>=at && now-at<=kDiagnosticMaxAgeUs) {
            length=serializeDiagnostic(diagnostic,active,sizeof(active));
            if(!length) ++diagnosticDrops;
          } else ++diagnosticStale;
        }
      }
      offset=0;
    }
    if(length) {
      const size_t available=Serial.availableForWrite();
      size_t count=length-offset;
      if(count>available) count=available;
      if(count>256) count=256;
      if(count) offset+=Serial.write((const uint8_t*)active+offset,count);
      if(offset==length) length=offset=0;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
}

void beginRuntime(uint32_t boot) {
  latest.boot=boot;
  rawQueue=xQueueCreateStatic(1,sizeof(Cycle),rawStorage,&rawQueueData);
  auxQueue=xQueueCreateStatic(4,sizeof(Aux),auxStorage,&auxQueueData);
  diagnosticQueue=xQueueCreateStatic(kDiagnosticQueueDepth,sizeof(DiagnosticEvent),
      diagnosticStorage,&diagnosticQueueData);
  Serial.setTxBufferSize(2048); Serial.setRxBufferSize(2048); Serial.begin(921600);
  xTaskCreatePinnedToCore(ioTask,"single_usb",8192,nullptr,3,nullptr,0);
  startLoRaTask();
}
void publishCycle(const Cycle& c) {
  portENTER_CRITICAL(&sharedLock);
  if(c.session!=latest.session) { syncExpires=0; pose={}; }
  latest=c;
  portEXIT_CRITICAL(&sharedLock);
  Cycle superseded;
  if(xQueueReceive(rawQueue,&superseded,0)==pdTRUE) ++rawDrops;
  xQueueOverwrite(rawQueue,&c);
}
Cycle currentCycle() { portENTER_CRITICAL(&sharedLock); Cycle c=latest; portEXIT_CRITICAL(&sharedLock); return c; }
Pose currentPose() { portENTER_CRITICAL(&sharedLock); Pose p=pose; portEXIT_CRITICAL(&sharedLock); return p; }
void setRadioStatus(const RadioStatus& r) { portENTER_CRITICAL(&sharedLock); radio=r; portEXIT_CRITICAL(&sharedLock); }
RadioStatus radioStatus() { portENTER_CRITICAL(&sharedLock); RadioStatus r=radio; portEXIT_CRITICAL(&sharedLock); return r; }
bool clockSynchronized() { portENTER_CRITICAL(&sharedLock); uint64_t e=syncExpires; portEXIT_CRITICAL(&sharedLock); return nowUs()<e; }
uint32_t clockGeneration() { portENTER_CRITICAL(&sharedLock); uint32_t g=generation; portEXIT_CRITICAL(&sharedLock); return g; }
uint64_t uwbPausedUntil() { portENTER_CRITICAL(&sharedLock); uint64_t t=pauseUntil; portEXIT_CRITICAL(&sharedLock); return t; }
bool takeUwbRecovery() { portENTER_CRITICAL(&sharedLock); bool v=recoverUwb; recoverUwb=false; portEXIT_CRITICAL(&sharedLock); return v; }
bool takeLoRaRecovery() { portENTER_CRITICAL(&sharedLock); bool v=recoverLoRa; recoverLoRa=false; portEXIT_CRITICAL(&sharedLock); return v; }
bool takeLoRaTest() { portENTER_CRITICAL(&sharedLock); bool v=testLoRa; testLoRa=false; portEXIT_CRITICAL(&sharedLock); return v; }
void queueAux(const char* line) {
  Aux a={}; const size_t n=strlen(line);
  if(n>=sizeof(a.line)) { ++auxDrops; return; }
  memcpy(a.line,line,n+1);
  if(xQueueSend(auxQueue,&a,0)!=pdTRUE) ++auxDrops;
}
namespace {
void queueDiagnostic(const DiagnosticEvent& event) {
  // RF task only copies a bounded POD record. It never formats or waits for USB.
  if(!diagnosticQueue || xQueueSend(diagnosticQueue,&event,0)!=pdTRUE) {
    ++diagnosticDrops; return;
  }
  const uint32_t count=uxQueueMessagesWaiting(diagnosticQueue);
  uint32_t high=diagnosticHighWater.load();
  while(count>high && !diagnosticHighWater.compare_exchange_weak(high,count)) {}
}
}
void recordAttemptDiagnostic(const AttemptDiagnostic& diagnostic) {
  ++diagnosticAttempts;
  const unsigned cause=(unsigned)diagnostic.cause;
  if(diagnostic.failure!=Ok && cause<kDiagnosticCauses) ++diagnosticFailures[cause];
  // Normal first successes already live in RAW. Bound diagnostic bandwidth.
  if(diagnostic.failure==Ok && diagnostic.attempt==1) return;
  DiagnosticEvent event={}; event.attempt=diagnostic;
  queueDiagnostic(event);
}
void recordRecoveryDiagnostic(const RecoveryDiagnostic& diagnostic) {
  DiagnosticEvent event={}; event.recovery=true; event.reset=diagnostic;
  queueDiagnostic(event);
}
}
