#pragma once
#include <stdint.h>
#include "../../../lib/heltec_dw3000_br101/vendor/br101/dwt_uwb_driver/dw3000/dw3000_deca_regs.h"

// Production register bits; aggregate masks follow deca_device_api.h.
constexpr uint32_t SYS_STATUS_ALL_RX_GOOD = SYS_STATUS_RXFR_BIT_MASK |
    SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_RXPRD_BIT_MASK |
    SYS_STATUS_RXSFDD_BIT_MASK | SYS_STATUS_RXPHD_BIT_MASK | SYS_STATUS_CIADONE_BIT_MASK;
constexpr uint32_t SYS_STATUS_ALL_RX_TO = SYS_STATUS_RXFTO_BIT_MASK |
    SYS_STATUS_RXPTO_BIT_MASK | SYS_STATUS_CPERR_BIT_MASK;
constexpr uint32_t SYS_STATUS_ALL_RX_ERR = SYS_STATUS_RXPHE_BIT_MASK |
    SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK |
    SYS_STATUS_ARFE_BIT_MASK | SYS_STATUS_CIAERR_BIT_MASK | SYS_STATUS_CPERR_BIT_MASK;
#undef SYS_STATUS_ALL_TX
constexpr uint32_t SYS_STATUS_ALL_TX = 0x000000F8;
constexpr unsigned FCS_LEN = 2;
constexpr int DWT_SUCCESS = 0;
constexpr int DWT_ERROR = -1;
constexpr int DWT_START_TX_IMMEDIATE = 0;
constexpr int DWT_RESPONSE_EXPECTED = 2;
constexpr int DWT_DW_INIT = 0;
constexpr int DWT_COMPAT_NONE = 0;
constexpr int DWT_PLEN_128 = 128;
constexpr int DWT_PAC8 = 8;
constexpr int DWT_SFD_DW_8 = 1;
constexpr int DWT_BR_6M8 = 2;
constexpr int DWT_STS_MODE_OFF = 0;
constexpr int DWT_LNA_ENABLE = 1;
constexpr int DWT_PA_ENABLE = 2;
constexpr int DWT_LEDS_ENABLE = 1;
constexpr int DWT_LEDS_INIT_BLINK = 2;
constexpr double DWT_TIME_UNITS = 1.0 / (499.2e6 * 128.0);

struct dwt_probe_s {};
extern dwt_probe_s dw3000_probe_interf;
struct dwt_config_t {
  int chan, txPreambLength, rxPAC, sfdType, dataRate, stsMode, sfdTO;
};
struct dwt_txconfig_t {};

int dw3000_hw_init();
void dw3000_hw_reset();
int dwt_probe(dwt_probe_s*);
bool dwt_checkidlerc();
void dwt_softreset(int);
int dwt_initialise(int);
int dwt_configure(dwt_config_t*);
void dwt_setrxaftertxdelay(uint32_t);
void dwt_configuretxrf(dwt_txconfig_t*);
void dwt_settxantennadelay(uint16_t);
void dwt_setrxantennadelay(uint16_t);
void dwt_setlnapamode(int);
void dwt_setleds(int);
void dwt_forcetrxoff();
void dwt_write32bitreg(uint32_t reg, uint32_t value);
uint32_t dwt_read32bitreg(uint32_t reg);
uint32_t dwt_readsystimestamphi32();
void dwt_readrxdata(uint8_t* bytes, uint16_t length, uint16_t offset);
void dwt_setrxtimeout(uint16_t timeout);
void dwt_writetxdata(uint16_t length, const uint8_t* bytes, uint16_t offset);
void dwt_writetxfctrl(uint16_t, uint16_t, int);
int dwt_starttx(int mode);
uint32_t dwt_readtxtimestamplo32();
uint32_t dwt_readtxtimestamphi32();
uint32_t dwt_readrxtimestamplo32(int);

namespace Br101Dw3000 { void spiFast(); }
