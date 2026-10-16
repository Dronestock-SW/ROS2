#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <random/rand32.h>
#include "../../common/UwbTdma40.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_vals.h"
#include "port.h"

#ifndef ANCHOR_PROFILE
#define ANCHOR_PROFILE 1
#endif

#ifndef UWB_FAST30_PROFILE
#define UWB_FAST30_PROFILE 0
#endif

#if ANCHOR_PROFILE == 1
#define ANCHOR_NAME "A1"
#define ANCHOR_SHORT_ADDR_VALUE 0x00A1U
#elif ANCHOR_PROFILE == 2
#define ANCHOR_NAME "A2"
#define ANCHOR_SHORT_ADDR_VALUE 0x00A2U
#elif ANCHOR_PROFILE == 3
#define ANCHOR_NAME "A3"
#define ANCHOR_SHORT_ADDR_VALUE 0x00A3U
#elif ANCHOR_PROFILE == 4
#define ANCHOR_NAME "A4"
#define ANCHOR_SHORT_ADDR_VALUE 0x00A4U
#else
#error "ANCHOR_PROFILE must be 1, 2, 3, or 4"
#endif

enum {
  PAN_ID = 0xDECA,
  ANCHOR_SHORT_ADDR = ANCHOR_SHORT_ADDR_VALUE,
  TX_ANT_DLY = 16385,
  RX_ANT_DLY = 16385,
  FUNC_POLL = 0xE0,
  FUNC_RESP = 0xE1,
  FUNC_FINAL = 0xE2,
  FUNC_REPORT = 0xE3,
  COMMON_FRAME_LEN = 10,
  SEQ_IDX = 2,
  REPORT_POLL_RX_TS_IDX = 10,
  REPORT_RESP_TX_TS_IDX = 14,
  REPORT_FINAL_RX_TS_IDX = 18,
  REPORT_STATUS_IDX = 22,
  REPORT_FRAME_LEN = 23,
  RX_BUFFER_LEN = 128,
};

static const uint32_t FINAL_RX_TIMEOUT_UUS = TDMA_RX_TIMEOUT_UUS;
static const uint32_t RESP_TX_TO_FINAL_RX_DELAY_UUS = 200U;

static dwt_config_t uwb_config = {
    5,
#if UWB_FAST30_PROFILE
    DWT_PLEN_128,
    DWT_PAC8,
    9,
    9,
    DWT_SFD_DW_8,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    128 + 1 + 8 - 8,
#else
    DWT_PLEN_1024,
    DWT_PAC16,
    9,
    9,
    DWT_SFD_DW_8,
    DWT_BR_850K,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    1024 + 1 + 8 - 16,
#endif
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0,
};

static dwt_txconfig_t tx_config = {
    0x34,
    0xFDFDFDFD,
    0,
};

static uint8_t rx_buffer[RX_BUFFER_LEN];
static uint8_t response_frame[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_RESP, 0, 0};
static uint8_t report_frame[REPORT_FRAME_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_REPORT};

static uint32_t poll_count;
static uint32_t response_count;
static uint32_t final_count;
static uint32_t success_count;
static uint32_t poll_error_count;
static uint32_t final_timeout_count;
static uint32_t final_mismatch_count;
static uint32_t response_fail_count;
static uint32_t report_fail_count;
static uint8_t last_sequence;

static uint16_t read_address(const uint8_t *frame, uint8_t low_index) {
  return (uint16_t)frame[low_index] |
         ((uint16_t)frame[low_index + 1U] << 8);
}

static void write_short_address(uint8_t *frame, uint16_t destination,
                                uint16_t source) {
  frame[3] = (uint8_t)(PAN_ID & 0xFFU);
  frame[4] = (uint8_t)(PAN_ID >> 8U);
  frame[5] = (uint8_t)(destination & 0xFFU);
  frame[6] = (uint8_t)(destination >> 8U);
  frame[7] = (uint8_t)(source & 0xFFU);
  frame[8] = (uint8_t)(source >> 8U);
}

static bool frame_matches(const uint8_t *frame, uint16_t frame_len,
                          uint8_t function_code, uint16_t destination,
                          uint16_t source) {
  return frame_len >= COMMON_FRAME_LEN &&
         frame[0] == 0x41U &&
         frame[1] == 0x88U &&
         frame[3] == (uint8_t)(PAN_ID & 0xFFU) &&
         frame[4] == (uint8_t)(PAN_ID >> 8U) &&
         read_address(frame, 5U) == destination &&
         read_address(frame, 7U) == source &&
         frame[9] == function_code;
}

static void write_timestamp32(uint8_t *field, uint32_t timestamp) {
  for (uint8_t i = 0U; i < 4U; ++i) {
    field[i] = (uint8_t)(timestamp >> (8U * i));
  }
}

static uint32_t read_rx_timestamp32(uint32_t *high) {
  uint8_t timestamp[5] = {0};
  dwt_readrxtimestamp(timestamp);
  if (high) *high = (uint32_t)timestamp[1] |
      ((uint32_t)timestamp[2] << 8U) |
      ((uint32_t)timestamp[3] << 16U) |
      ((uint32_t)timestamp[4] << 24U);
  return (uint32_t)timestamp[0] |
         ((uint32_t)timestamp[1] << 8U) |
         ((uint32_t)timestamp[2] << 16U) |
         ((uint32_t)timestamp[3] << 24U);
}

static void clear_rx_status(void) {
  dwt_write32bitreg(SYS_STATUS_ID,
                    SYS_STATUS_ALL_RX_GOOD |
                        SYS_STATUS_ALL_RX_TO |
                        SYS_STATUS_ALL_RX_ERR);
}

static void clear_tx_status(void) {
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
}

static uint32_t radio_now(void) {
  /* DW3000 SYS_TIME stays latched until an SPI write (User Manual 8.2.2.7).
   * SYS_STATUS is write-one-to-clear: zero refreshes the latch without
   * clearing a pending TX/RX event or changing the radio state. */
  dwt_write8bitoffsetreg(SYS_STATUS_ID, 0U, 0U);
  return dwt_readsystimestamphi32();
}

static uint32_t wait_until(uint32_t mask, uint32_t deadline) {
  const uint32_t now = radio_now();
  const uint32_t started_ms = k_uptime_get_32();
  const uint32_t remaining_us = tdma_before(now, deadline) ? tdma_us(deadline - now) : 0U;
  const uint32_t watchdog_ms = (remaining_us + 999U) / 1000U + 2U;
  while (true) {
    /* A stopped DW clock must not turn a bounded transaction into a hang. */
    if ((uint32_t)(k_uptime_get_32() - started_ms) >= watchdog_ms) return 0U;
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    if (!tdma_before(radio_now(), deadline)) return 0U;
    if ((status & mask) != 0U) return status;
    k_busy_wait(TDMA_POLL_PAUSE_US);
  }
  return 0U;
}

static void radio_idle(void) {
  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD |
      SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
}

static bool read_frame(uint16_t *payload_len) {
  const uint16_t frame_len =
      (uint16_t)(dwt_read32bitreg(RX_FINFO_ID) &
                 RX_FINFO_STD_RXFLEN_MASK);
  if (frame_len < FCS_LEN) {
    return false;
  }
  *payload_len = frame_len - FCS_LEN;
  if (*payload_len == 0U || *payload_len > sizeof(rx_buffer)) {
    return false;
  }
  dwt_readrxdata(rx_buffer, *payload_len, 0U);
  return true;
}

static bool send_report(uint16_t destination, uint8_t sequence, uint32_t deadline,
                        uint32_t poll_rx_timestamp,
                        uint32_t response_tx_timestamp,
                        uint32_t final_rx_timestamp) {
  write_short_address(report_frame, destination, ANCHOR_SHORT_ADDR);
  report_frame[SEQ_IDX] = sequence;
  write_timestamp32(&report_frame[REPORT_POLL_RX_TS_IDX],
                    poll_rx_timestamp);
  write_timestamp32(&report_frame[REPORT_RESP_TX_TS_IDX],
                    response_tx_timestamp);
  write_timestamp32(&report_frame[REPORT_FINAL_RX_TS_IDX],
                    final_rx_timestamp);
  report_frame[REPORT_STATUS_IDX] = 1U;

  dwt_forcetrxoff();
  clear_tx_status();
  dwt_writetxdata(sizeof(report_frame), report_frame, 0U);
  dwt_writetxfctrl(sizeof(report_frame) + FCS_LEN, 0U, 0);
  if (!tdma_fits(radio_now(), deadline, TDMA_TX_MARGIN_US) ||
      dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
    return false;
  }

  const uint32_t status =
      wait_until(SYS_STATUS_TXFRS_BIT_MASK, deadline);
  radio_idle();
  return (status & SYS_STATUS_TXFRS_BIT_MASK) != 0U;
}

static bool begin_uwb(void) {
  if (peripherals_init() != 0) {
    printk("[NRF52DK-4A][" ANCHOR_NAME "][FATAL] GPIO init failed\n");
    return false;
  }
  spi_peripheral_init();
  k_msleep(10);

  port_set_dw_ic_spi_fastrate();
  reset_DWIC();
  k_msleep(5);

  for (uint8_t attempt = 0U;
       attempt < 50U && !dwt_checkidlerc();
       ++attempt) {
    k_msleep(2);
  }
  if (!dwt_checkidlerc()) {
    printk("[NRF52DK-4A][" ANCHOR_NAME "][FATAL] IDLE_RC check failed\n");
    return false;
  }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    printk("[NRF52DK-4A][" ANCHOR_NAME "][FATAL] initialise failed\n");
    return false;
  }
  if (dwt_configure(&uwb_config) == DWT_ERROR) {
    printk("[NRF52DK-4A][" ANCHOR_NAME "][FATAL] configure failed\n");
    return false;
  }

  dwt_configuretxrf(&tx_config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  // Constant across every exchange, including coordinator SYNC transmissions.
  dwt_setpreambledetecttimeout(0U);
  dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DELAY_UUS);
  return true;
}

#if ANCHOR_PROFILE == 1
static uint32_t next_sync, sync_seq, sync_session, last_sync_ref;
static bool have_sync;
#endif

static void coordinator_service(void) {
#if ANCHOR_PROFILE == 1
  uint32_t now = radio_now();
  if (tdma_before(now, next_sync - tdma_ticks(TDMA_COORD_PREP_US))) return;
  const uint32_t target = next_sync;
  const uint32_t sequence = sync_seq++;
  next_sync += tdma_ticks(TDMA_PERIOD_US);
  /* A late control slot is lost. Never shift the entire schedule or burst. */
  if (!tdma_fits(now, target, 300U)) { have_sync = false; return; }
  uint8_t frame[TDMA_SYNC_LEN];
  tdma_encode_sync(frame, sync_session, sequence);
  radio_idle();
  dwt_writetxdata(sizeof(frame), frame, 0U);
  dwt_writetxfctrl(sizeof(frame) + FCS_LEN, 0U, 0);
  dwt_setdelayedtrxtime(target & ~1UL);
  if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
    radio_idle(); have_sync = false; return;
  }
  uint32_t status = wait_until(SYS_STATUS_TXFRS_BIT_MASK,
                              target + tdma_ticks(TDMA_TX_WAIT_US));
  have_sync = (status & SYS_STATUS_TXFRS_BIT_MASK) != 0U;
  if (have_sync) last_sync_ref = dwt_readtxtimestamphi32();
  radio_idle();
#endif
}

int main(void) {
  if (!begin_uwb()) return 0;
  printk("[TDMA40][" ANCHOR_NAME "] ch=5 tags=5,6 period=25000 schedule=0x4001\n");
#if ANCHOR_PROFILE == 1
  sync_session = sys_rand32_get();
  next_sync = radio_now() + tdma_ticks(TDMA_RESTART_QUIET_US);
#endif

  while (true) {
    coordinator_service();
    radio_idle();
    dwt_setrxtimeout(0U);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uint32_t poll_deadline = radio_now() + tdma_ticks(TDMA_PERIOD_US);
#if ANCHOR_PROFILE == 1
    poll_deadline = next_sync - tdma_ticks(TDMA_COORD_PREP_US);
#endif
    const uint32_t poll_status = wait_until(
        SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR,
        poll_deadline);
    if ((poll_status & SYS_STATUS_RXFCG_BIT_MASK) == 0U) continue;
    uint16_t payload_len = 0U;
    uint32_t poll_ref;
    const uint32_t poll_rx_timestamp = read_rx_timestamp32(&poll_ref);
    if (!read_frame(&payload_len) || payload_len < COMMON_FRAME_LEN) continue;
    const uint16_t peer = read_address(rx_buffer, 7U);
    if (!tdma_tag_allowed(peer) ||
        !frame_matches(rx_buffer, payload_len, FUNC_POLL, ANCHOR_SHORT_ADDR, peer)) {
      /* SYNC, other PAN/dst/type: no active peer or sequence is changed. */
      ++poll_error_count; continue;
    }
    uint32_t deadline = poll_ref + tdma_ticks(TDMA_ANCHOR_US);
    const uint32_t admission_now = radio_now();
#if ANCHOR_PROFILE == 1
    const uint32_t slot_start = last_sync_ref + tdma_ticks(tdma_slot_offset(peer));
    const uint32_t slot_end = slot_start + tdma_ticks(TDMA_SLOT_US);
    if (!have_sync || tdma_before(poll_ref, slot_start) ||
        !tdma_fits(admission_now, slot_end, TDMA_ANCHOR_US + TDMA_CLEANUP_US)) continue;
    deadline = tdma_earlier(deadline, slot_end - tdma_ticks(TDMA_CLEANUP_US));
#endif
    if (!tdma_fits(admission_now, deadline, TDMA_REPORT_RESERVE_US)) continue;
    const uint16_t active_tag_addr = peer;
    const uint8_t sequence = rx_buffer[SEQ_IDX];
    last_sequence = sequence; ++poll_count;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD |
        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
    write_short_address(response_frame, active_tag_addr, ANCHOR_SHORT_ADDR);
    response_frame[SEQ_IDX] = sequence;
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
    dwt_writetxdata(sizeof(response_frame), response_frame, 0U);
    dwt_writetxfctrl(sizeof(response_frame) + FCS_LEN, 0U, 0);
    if (!tdma_fits(radio_now(), deadline, TDMA_REPORT_RESERVE_US) ||
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
      ++response_fail_count; continue;
    }
    ++response_count;
    bool final_ok = false;
    uint32_t response_tx_timestamp = 0, final_rx_timestamp = 0;
    const uint32_t final_deadline = deadline - tdma_ticks(TDMA_REPORT_RESERVE_US);
    while (tdma_before(radio_now(), final_deadline)) {
      const uint32_t status = wait_until(
          SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR,
          final_deadline);
      if ((status & SYS_STATUS_RXFCG_BIT_MASK) == 0U) break;
      response_tx_timestamp = dwt_readtxtimestamplo32();
      final_rx_timestamp = read_rx_timestamp32(NULL);
      if (!read_frame(&payload_len)) break;
      uint32_t ignored_session, ignored_seq;
      if (tdma_decode_sync(rx_buffer, payload_len, &ignored_session, &ignored_seq)) {
        /* Control frames cannot renew the transaction's deadline. */
        clear_rx_status();
        uint32_t now = radio_now();
        if (!tdma_fits(now, final_deadline, 30U)) break;
        dwt_setrxtimeout((final_deadline - now) / 256U);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        continue;
      }
      final_ok = frame_matches(rx_buffer, payload_len, FUNC_FINAL,
                               ANCHOR_SHORT_ADDR, active_tag_addr) &&
                 rx_buffer[SEQ_IDX] == sequence;
      if (!final_ok) ++final_mismatch_count;
      break;
    }
    if (!final_ok) {
      ++final_timeout_count; radio_idle(); continue;
    }
    ++final_count;
    if (send_report(active_tag_addr, sequence, deadline, poll_rx_timestamp,
                    response_tx_timestamp, final_rx_timestamp)) ++success_count;
    else ++report_fail_count;
    radio_idle();
    /* No blocking UART output in the ranging/control path. */
  }
}
