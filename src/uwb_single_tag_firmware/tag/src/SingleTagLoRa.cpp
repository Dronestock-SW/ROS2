#include "SingleTagRuntime.h"
#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <SPI.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

extern "C" {
extern TimerEvent_t TxTimeoutTimer,RxTimeoutTimer,CadTimeoutTimer;
}
namespace SingleTag {
namespace {
RadioEvents_t events={}; RadioStatus state={};
bool initialized=false,rxActive=false,recoveryRequested=false;
bool rxPending=false;
uint8_t rxPayload[255]; uint16_t rxSize=0; int16_t rxRssi; int8_t rxSnr;
uint32_t lastRecoveryMs=0,txStartedMs=0;
uint64_t nextSendUs=0;
void txDone() {
  if(!state.busy) return;
  ++state.done; state.lastDoneMs=millis(); state.busy=false; rxActive=false;
}
void txTimeout() {
  if(!state.busy) return;
  ++state.timeouts; state.busy=false; rxActive=false; recoveryRequested=true;
}
void rxDone(uint8_t* bytes,uint16_t count,int16_t rssi,int8_t snr) {
  ++state.rx; state.lastRxMs=millis();
  if(rxPending || count>255) ++state.rxErrors;
  else { memcpy(rxPayload,bytes,count); rxSize=count; rxRssi=rssi; rxSnr=snr; rxPending=true; }
  rxActive=false;
}
void rxError() { ++state.rxErrors; rxActive=false; }
void rxTimeout() { rxActive=false; }
void receive() {
  if(state.ready && !state.busy && !rxActive) { Radio.Rx(0); rxActive=true; }
}
bool initialize() {
  state.ready=false;
  if(initialized) {
    TimerStop(&TxTimeoutTimer); TimerStop(&RxTimeoutTimer); TimerStop(&CadTimeoutTimer);
    Radio.Standby();
  }
  state.busy=false; rxActive=false; recoveryRequested=false;
  lastRecoveryMs=millis();
  events.TxDone=txDone; events.TxTimeout=txTimeout;
  events.RxDone=rxDone; events.RxTimeout=rxTimeout; events.RxError=rxError;
  Radio.Init(&events); initialized=true;
  Radio.SetChannel(915000000);
  Radio.SetTxConfig(MODEM_LORA,10,0,0,7,1,8,false,true,0,0,false,3000);
  Radio.SetRxConfig(MODEM_LORA,0,7,1,0,8,0,false,0,true,0,0,false,true);
  Radio.SetPublicNetwork(false);
  state.ready=(((uint16_t)Radio.Read(0x0740)<<8)|Radio.Read(0x0741))==0x1424;
  if(state.ready) receive(); else Radio.Standby();
  return state.ready;
}
void emitDownlink() {
  char escaped[511],hex[511]; size_t n=0;
  constexpr char digits[]="0123456789abcdef";
  for(unsigned i=0;i<rxSize;i++) {
    uint8_t c=rxPayload[i];
    hex[i*2]=digits[c>>4]; hex[i*2+1]=digits[c&15];
    if(c=='"' || c=='\\') escaped[n++]='\\';
    escaped[n++]=(c>=32 && c<127)?(char)c:'?';
  }
  escaped[n]=0; hex[rxSize*2]=0;
  char line[1400];
  snprintf(line,sizeof(line),"{\"type\":\"lora_rx\",\"schema\":2,\"tag_id\":\"6\","
      "\"rssi_dbm\":%d,\"snr_db\":%d,\"length\":%u,\"payload\":\"%s\",\"payload_hex\":\"%s\"}\n",
      rxRssi,rxSnr,rxSize,escaped,hex);
  queueAux(line); rxPending=false;
}
void send(uint64_t now,bool maxLengthTest) {
  char payload[256]; int length;
  const Cycle c=currentCycle(); const Pose p=currentPose();
  const bool fresh=p.received && p.boot==c.boot && p.session==c.session &&
      now>=p.sampleUs && now-p.sampleUs<=500000 && clockSynchronized();
  const bool fix=fresh && p.fix && isfinite(p.x) && isfinite(p.y);
  const char* reason=fresh?p.reason:(c.ready?"jetson_pose_stale":"uwb_not_ready");
  if(maxLengthTest) {
    constexpr char prefix[]="SINGLE2_MAXLEN:";
    memcpy(payload,prefix,sizeof(prefix)-1);
    for(unsigned i=sizeof(prefix)-1;i<255;i++) payload[i]='0'+i%10;
    length=255;
  } else if(fix) {
    length=snprintf(payload,sizeof(payload),"{\"type\":\"drone.status\",\"id\":\"6\",\"seq\":%lu,"
        "\"t\":%lu,\"fix\":true,\"x\":%.3f,\"y\":%.3f,\"q\":%u,\"src\":\"jetson\","
        "\"uwb_seq\":%lu,\"reason\":\"%s\"}",(unsigned long)p.seq,(unsigned long)millis(),
        p.x,p.y,p.quality,(unsigned long)c.seq,reason);
  } else {
    length=snprintf(payload,sizeof(payload),"{\"type\":\"drone.status\",\"id\":\"6\",\"seq\":%lu,"
        "\"t\":%lu,\"fix\":false,\"x\":null,\"y\":null,\"q\":0,\"src\":\"none\","
        "\"uwb_seq\":%lu,\"reason\":\"%s\"}",(unsigned long)(p.received?p.seq:c.seq),
        (unsigned long)millis(),(unsigned long)c.seq,reason);
  }
  if(length<=0 || length>255) return;
  state.lastTxFix=fix && !maxLengthTest; state.lastTxLength=(uint8_t)length;
  rxActive=false; state.busy=true; txStartedMs=millis(); ++state.attempts;
  Radio.Send((uint8_t*)payload,(uint8_t)length);
}
void radioTask(void*) {
  // The bundled Heltec radio uses global Arduino SPI. Only this task owns it.
  pinMode(RADIO_NSS,OUTPUT); digitalWrite(RADIO_NSS,HIGH);
  SPI.begin(LORA_CLK,LORA_MISO,LORA_MOSI,RADIO_NSS);
  initialize();
  nextSendUs=esp_timer_get_time()+1000000;
  bool maxLengthPending=false;
  for(;;) {
    const uint64_t start=esp_timer_get_time();
    if(takeLoRaRecovery()) recoveryRequested=true;
    maxLengthPending|=takeLoRaTest();
    if(state.ready) {
      Radio.IrqProcess(); Mcu.timerhandler();
      if(rxPending) emitDownlink();
      if(state.busy && (uint32_t)(millis()-txStartedMs)>3500) {
        ++state.watchdog; txTimeout();
      }
    }
    if(recoveryRequested || (!state.ready && (uint32_t)(millis()-lastRecoveryMs)>=3000)) {
      ++state.recoveries; initialize();
    }
    const uint64_t now=esp_timer_get_time();
    if(now>=nextSendUs) {
      nextSendUs+=((now-nextSendUs)/1000000+1)*1000000;
      if(state.ready && !state.busy) { send(now,maxLengthPending); maxLengthPending=false; }
    }
    receive();
    const uint32_t elapsed=esp_timer_get_time()-start;
    if(elapsed>state.serviceMaxUs) state.serviceMaxUs=elapsed;
    setRadioStatus(state);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
}
void startLoRaTask() { xTaskCreatePinnedToCore(radioTask,"single_lora",6144,nullptr,2,nullptr,0); }
}
