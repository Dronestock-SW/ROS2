#pragma once
#include <stdint.h>
namespace JetsonTagConfig {
static constexpr char kTagDisplayName[] = "TAG B";
static constexpr uint32_t kSerialBaud=921600;
static constexpr uint8_t kUwbSck=41, kUwbMiso=42, kUwbMosi=39, kUwbCs=40, kUwbIrq=48, kUwbRst=47;
static constexpr uint16_t kTagTxAntennaDelay=16385, kTagRxAntennaDelay=16385;
}
