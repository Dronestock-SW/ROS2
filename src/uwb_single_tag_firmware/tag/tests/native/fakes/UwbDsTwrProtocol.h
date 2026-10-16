#pragma once
#include "Br101Dw3000Port.h"
namespace UwbDsTwr {
dwt_config_t defaultConfig();
dwt_txconfig_t defaultTxConfig();
void clearTxState();
void clearRxState();
}
