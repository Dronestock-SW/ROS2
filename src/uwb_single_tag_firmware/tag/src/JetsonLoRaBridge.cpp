#include "JetsonLoRaBridge.h"

#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <SPI.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "JetsonTagConfig.h"
#include "LoRaRecoveryPolicy.h"

// Heltec 2.1.6 owns these timers. Unlink them before reinitializing the radio;
// Radio.Init calls TimerInit, which must not run on still-linked timer nodes.
extern "C" {
extern TimerEvent_t TxTimeoutTimer;
extern TimerEvent_t RxTimeoutTimer;
extern TimerEvent_t CadTimeoutTimer;
}

namespace JetsonLoRaBridge {
namespace {

enum class ActiveSpiBus : uint8_t { None, Uwb, LoRa };

struct JetsonPose {
  bool received;
  bool fix;
  uint32_t seq;
  uint64_t sampleTimeUs;
  float xM;
  float yM;
  uint8_t quality;
  uint32_t receivedAtMs;
  char reason[32];
};

RadioEvents_t gRadioEvents = {};
ActiveSpiBus gActiveSpiBus = ActiveSpiBus::None;
JetsonPose gPose = {};
bool gReady = false;
bool gTxBusy = false;
bool gRxActive = false;
bool gRxDone = false;
bool gRxTimeout = false;
bool gRxError = false;
uint8_t gRxPayload[256] = {0};
uint16_t gRxPayloadSize = 0;
int16_t gRxRssiDbm = 0;
int8_t gRxSnrDb = 0;
uint32_t gLastTxMs = 0;
uint32_t gLastServiceMs = 0;
uint32_t gLastRecoveryMs = 0;
bool gRadioInitialized = false;
bool gRecoveryRequested = false;
Diagnostics gDiagnostics;
char gSerialLine[256] = {0};
uint16_t gSerialLineSize = 0;

void selectLoRaSpi() {
  if (gActiveSpiBus == ActiveSpiBus::LoRa) {
    return;
  }
  digitalWrite(JetsonTagConfig::kUwbCs, HIGH);
  SPI.end();
  SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI, RADIO_NSS);
  gActiveSpiBus = ActiveSpiBus::LoRa;
}

const char* fieldValue(const char* json, const char* key) {
  if (json == nullptr || key == nullptr) {
    return nullptr;
  }
  char pattern[48] = {0};
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* value = strstr(json, pattern);
  if (value == nullptr) {
    return nullptr;
  }
  value = strchr(value + strlen(pattern), ':');
  if (value == nullptr) {
    return nullptr;
  }
  ++value;
  while (*value == ' ' || *value == '\t') {
    ++value;
  }
  return value;
}

bool readString(
    const char* json, const char* key, char* output, size_t outputSize) {
  const char* value = fieldValue(json, key);
  if (value == nullptr || *value != '"' || outputSize == 0) {
    return false;
  }
  ++value;
  size_t size = 0;
  while (*value != '\0' && *value != '"') {
    const char ch = *value++;
    if (ch == '\\' || static_cast<unsigned char>(ch) < 0x20) {
      return false;
    }
    if (size + 1 >= outputSize) {
      return false;
    }
    output[size++] = ch;
  }
  if (*value != '"') {
    return false;
  }
  output[size] = '\0';
  return true;
}

bool readUnsigned(const char* json, const char* key, uint64_t& output) {
  const char* value = fieldValue(json, key);
  if (value == nullptr || *value < '0' || *value > '9') {
    return false;
  }
  char* end = nullptr;
  const unsigned long long parsed = strtoull(value, &end, 10);
  if (end == value) {
    return false;
  }
  output = static_cast<uint64_t>(parsed);
  return true;
}

bool readBool(const char* json, const char* key, bool& output) {
  const char* value = fieldValue(json, key);
  if (value == nullptr) {
    return false;
  }
  if (strncmp(value, "true", 4) == 0) {
    output = true;
    return true;
  }
  if (strncmp(value, "false", 5) == 0) {
    output = false;
    return true;
  }
  return false;
}

bool readFloat(const char* json, const char* key, float& output) {
  const char* value = fieldValue(json, key);
  if (value == nullptr) {
    return false;
  }
  char* end = nullptr;
  const float parsed = strtof(value, &end);
  if (end == value || !isfinite(parsed)) {
    return false;
  }
  output = parsed;
  return true;
}

bool acceptJetsonPose(const char* line) {
  char type[24] = {0};
  char tagId[16] = {0};
  uint64_t schema = 0;
  uint64_t seq = 0;
  uint64_t sampleTimeUs = 0;
  uint64_t quality = 0;
  bool fix = false;
  if (!readString(line, "type", type, sizeof(type)) ||
      strcmp(type, "jetson_pose") != 0 ||
      !readUnsigned(line, "schema", schema) || schema != 1 ||
      !readString(line, "tag_id", tagId, sizeof(tagId)) ||
      strcmp(tagId, JetsonTagConfig::kTagId) != 0 ||
      !readUnsigned(line, "seq", seq) || seq > UINT32_MAX ||
      !readUnsigned(line, "sample_time_us", sampleTimeUs) ||
      !readBool(line, "fix", fix) ||
      !readUnsigned(line, "quality", quality) || quality > 100) {
    return false;
  }

  JetsonPose next = {};
  next.received = true;
  next.fix = fix;
  next.seq = static_cast<uint32_t>(seq);
  next.sampleTimeUs = sampleTimeUs;
  next.quality = static_cast<uint8_t>(quality);
  next.xM = NAN;
  next.yM = NAN;
  if (fix && (!readFloat(line, "x_m", next.xM) ||
              !readFloat(line, "y_m", next.yM))) {
    return false;
  }
  if (!readString(line, "reason", next.reason, sizeof(next.reason))) {
    snprintf(next.reason, sizeof(next.reason), "%s", fix ? "ok" : "no_fix");
  }
  next.receivedAtMs = millis();
  gPose = next;
  return true;
}

void emitPoseAck(bool accepted) {
  if (!accepted) {
    Serial.printf(
        "{\"type\":\"jetson_pose_ack\",\"schema\":1,"
        "\"tag_id\":\"%s\",\"accepted\":false,"
        "\"pose_seq\":null,\"fix\":false,"
        "\"lora_eligible\":false,\"reason\":\"invalid_packet\"}\n",
        JetsonTagConfig::kTagId);
    return;
  }
  const bool eligible =
      gPose.fix && isfinite(gPose.xM) && isfinite(gPose.yM);
  Serial.printf(
      "{\"type\":\"jetson_pose_ack\",\"schema\":1,"
      "\"tag_id\":\"%s\",\"accepted\":true,"
      "\"pose_seq\":%lu,\"fix\":%s,\"lora_eligible\":%s,"
      "\"reason\":\"%s\"}\n",
      JetsonTagConfig::kTagId,
      static_cast<unsigned long>(gPose.seq),
      gPose.fix ? "true" : "false",
      eligible ? "true" : "false",
      gPose.reason);
}

void printEscapedJsonString(const uint8_t* data, uint16_t size) {
  Serial.print('"');
  for (uint16_t index = 0; index < size; ++index) {
    const uint8_t ch = data[index];
    if (ch == '"' || ch == '\\') {
      Serial.print('\\');
      Serial.print(static_cast<char>(ch));
    } else if (ch >= 0x20 && ch <= 0x7E) {
      Serial.print(static_cast<char>(ch));
    } else {
      Serial.print('?');
    }
  }
  Serial.print('"');
}

void emitRxPacket() {
  Serial.printf(
      "{\"type\":\"lora_rx\",\"schema\":1,\"tag_id\":\"%s\"," 
      "\"rssi_dbm\":%d,\"snr_db\":%d,\"payload\":",
      JetsonTagConfig::kTagId,
      static_cast<int>(gRxRssiDbm),
      static_cast<int>(gRxSnrDb));
  printEscapedJsonString(gRxPayload, gRxPayloadSize);
  Serial.println('}');
}

void onTxDone() {
  if (!gTxBusy) {
    return;
  }
  ++gDiagnostics.txDone;
  gDiagnostics.lastTxDoneMs = millis();
  gTxBusy = false;
  gRxActive = false;
}

void onTxTimeout() {
  if (!gTxBusy) {
    return;
  }
  ++gDiagnostics.txTimeouts;
  gRecoveryRequested = true;
  gTxBusy = false;
  gRxActive = false;
}

void onRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  ++gDiagnostics.rxPackets;
  gDiagnostics.lastRxMs = millis();
  gRxPayloadSize = size < sizeof(gRxPayload) ? size : sizeof(gRxPayload);
  memcpy(gRxPayload, payload, gRxPayloadSize);
  gRxRssiDbm = rssi;
  gRxSnrDb = snr;
  gRxDone = true;
  gRxActive = false;
}

void onRxTimeout() {
  gRxTimeout = true;
  gRxActive = false;
}

void onRxError() {
  ++gDiagnostics.rxErrors;
  gRxError = true;
  gRxActive = false;
}

void startReceive() {
  if (!gReady || gTxBusy || gRxActive) {
    return;
  }
  selectLoRaSpi();
  Radio.Rx(0);
  gRxActive = true;
}

bool poseFresh(uint32_t nowMs) {
  return gPose.received &&
         nowMs - gPose.receivedAtMs <= JetsonTagConfig::kJetsonPoseFreshnessMs;
}

void sendStatus(uint32_t nowMs, uint32_t uwbCycleSequence) {
  if (!gReady || gTxBusy) {
    return;
  }
  const bool fresh = poseFresh(nowMs);
  const bool fix = fresh && gPose.fix && isfinite(gPose.xM) && isfinite(gPose.yM);
  const char* reason = fresh ? gPose.reason : "jetson_pose_stale";
  char payload[256] = {0};
  int length = 0;
  if (fix) {
    length = snprintf(
        payload, sizeof(payload),
        "{\"type\":\"drone.status\",\"id\":\"%s\",\"seq\":%lu,"
        "\"t\":%lu,\"fix\":true,\"x\":%.3f,\"y\":%.3f,"
        "\"q\":%u,\"src\":\"jetson\",\"uwb_seq\":%lu,\"reason\":\"%s\"}",
        JetsonTagConfig::kTagId,
        static_cast<unsigned long>(gPose.seq),
        static_cast<unsigned long>(nowMs),
        gPose.xM,
        gPose.yM,
        static_cast<unsigned int>(gPose.quality),
        static_cast<unsigned long>(uwbCycleSequence),
        reason);
  } else {
    length = snprintf(
        payload, sizeof(payload),
        "{\"type\":\"drone.status\",\"id\":\"%s\",\"seq\":%lu,"
        "\"t\":%lu,\"fix\":false,\"x\":null,\"y\":null,"
        "\"q\":0,\"src\":\"none\",\"uwb_seq\":%lu,\"reason\":\"%s\"}",
        JetsonTagConfig::kTagId,
        static_cast<unsigned long>(gPose.received ? gPose.seq : uwbCycleSequence),
        static_cast<unsigned long>(nowMs),
        static_cast<unsigned long>(uwbCycleSequence),
        reason);
  }
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(payload)) {
    return;
  }
  selectLoRaSpi();
  gRxActive = false;
  gTxBusy = true;
  ++gDiagnostics.txAttempts;
  Radio.Send(reinterpret_cast<uint8_t*>(payload), static_cast<uint8_t>(length));
}

}  // namespace

void selectUwbSpi() {
  if (gActiveSpiBus == ActiveSpiBus::Uwb) {
    return;
  }
  digitalWrite(RADIO_NSS, HIGH);
  SPI.end();
  SPI.begin(
      JetsonTagConfig::kUwbSck,
      JetsonTagConfig::kUwbMiso,
      JetsonTagConfig::kUwbMosi,
      JetsonTagConfig::kUwbCs);
  gActiveSpiBus = ActiveSpiBus::Uwb;
}

bool begin() {
  selectLoRaSpi();
  gReady = false;
  gDiagnostics.hwVerified = false;
  if (gRadioInitialized) {
    TimerStop(&TxTimeoutTimer);
    TimerStop(&RxTimeoutTimer);
    TimerStop(&CadTimeoutTimer);
    Radio.Standby();
  }
  gTxBusy = false;
  gRxActive = false;
  gRxDone = false;
  gRxTimeout = false;
  gRxError = false;
  gRecoveryRequested = false;
  gLastRecoveryMs = millis();
  gRadioEvents.TxDone = onTxDone;
  gRadioEvents.TxTimeout = onTxTimeout;
  gRadioEvents.RxDone = onRxDone;
  gRadioEvents.RxTimeout = onRxTimeout;
  gRadioEvents.RxError = onRxError;
  Radio.Init(&gRadioEvents);
  gRadioInitialized = true;
  Radio.SetChannel(JetsonTagConfig::kLoRaFrequencyHz);
  Radio.SetTxConfig(
      MODEM_LORA,
      JetsonTagConfig::kLoRaTxPowerDbm,
      0,
      JetsonTagConfig::kLoRaBandwidth,
      JetsonTagConfig::kLoRaSpreadingFactor,
      JetsonTagConfig::kLoRaCodingRate,
      JetsonTagConfig::kLoRaPreambleLength,
      false,
      true,
      0,
      0,
      false,
      JetsonTagConfig::kLoRaTxTimeoutMs);
  Radio.SetRxConfig(
      MODEM_LORA,
      JetsonTagConfig::kLoRaBandwidth,
      JetsonTagConfig::kLoRaSpreadingFactor,
      JetsonTagConfig::kLoRaCodingRate,
      0,
      JetsonTagConfig::kLoRaPreambleLength,
      0,
      false,
      0,
      true,
      0,
      0,
      false,
      true);
  Radio.SetPublicNetwork(false);
  const uint16_t syncWord =
      (static_cast<uint16_t>(Radio.Read(0x0740)) << 8) | Radio.Read(0x0741);
  gDiagnostics.hwVerified = syncWord == 0x1424;
  gReady = gDiagnostics.hwVerified;
  if (!gReady) {
    Radio.Standby();
    return false;
  }
  startReceive();
  return true;
}

bool ready() {
  return gReady;
}

Diagnostics diagnostics() {
  Diagnostics result = gDiagnostics;
  result.txBusy = gTxBusy;
  return result;
}

void serviceSerialInput() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (gSerialLineSize > 0) {
        gSerialLine[gSerialLineSize] = '\0';
        emitPoseAck(acceptJetsonPose(gSerialLine));
        gSerialLineSize = 0;
      }
      continue;
    }
    if (gSerialLineSize + 1 < sizeof(gSerialLine)) {
      gSerialLine[gSerialLineSize++] = ch;
    } else {
      gSerialLineSize = 0;
    }
  }
}

void service(uint32_t uwbCycleSequence) {
  if (!gReady) {
    if (millis() - gLastRecoveryMs >= JetsonTagConfig::kLoRaRecoveryIntervalMs) {
      ++gDiagnostics.recoveries;
      begin();
    }
    return;
  }
  const uint32_t nowMs = millis();
  const uint32_t periodMs = poseFresh(nowMs) && gPose.fix
                                ? JetsonTagConfig::kLoRaGoodFixPeriodMs
                                : JetsonTagConfig::kLoRaBadFixPeriodMs;
  const bool txDue = !gTxBusy && nowMs - gLastTxMs >= periodMs;
  if (!txDue && gLastServiceMs != 0 &&
      nowMs - gLastServiceMs < JetsonTagConfig::kLoRaServiceIntervalMs) {
    return;
  }
  gLastServiceMs = nowMs;
  selectLoRaSpi();
  // Consume hardware completion first, so a delayed main-loop iteration does
  // not count an already-completed transmission as a timeout.
  Radio.IrqProcess();
  // Heltec's software timer queue is cooperative, not a background task.
  Mcu.timerhandler();
  if (gRxDone) {
    gRxDone = false;
    emitRxPacket();
  }
  if (gRxTimeout || gRxError) {
    gRxTimeout = false;
    gRxError = false;
  }

  // Independent millis-based recovery still works if the library timer path
  // or TX-done notification fails. Never synthesize a successful TX/ACK.
  if (LoRaRecoveryPolicy::txDeadlineExpired(
          gTxBusy, millis(), gLastTxMs, JetsonTagConfig::kLoRaTxWatchdogMs)) {
    ++gDiagnostics.watchdogTimeouts;
    onTxTimeout();
  }
  if (gRecoveryRequested) {
    ++gDiagnostics.recoveries;
    begin();
    return;
  }

  if (!gTxBusy && nowMs - gLastTxMs >= periodMs) {
    gLastTxMs = nowMs;
    sendStatus(nowMs, uwbCycleSequence);
  }
  startReceive();
}

}  // namespace JetsonLoRaBridge
