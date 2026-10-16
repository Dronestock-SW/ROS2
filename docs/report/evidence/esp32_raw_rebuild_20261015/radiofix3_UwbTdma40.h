#ifndef UWB_TDMA40_H
#define UWB_TDMA40_H

/* Experimental two-tag schedule. Shared by both MCUs and host tests.
 * All schedule times are REAL microseconds, not DW UUS.
 * Radio clock: high 32 bits of the 40-bit DW timestamp, 249.6 ticks/us.
 * Signed comparisons require intervals below half the 17.207 s wrap.
 * Bump SCHEDULE_ID whenever timing or admission rules change.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define TDMA_SCHEDULE_ID 0x4001U
#define TDMA_VERSION 1U
#define TDMA_PAN 0xDECAU
#define TDMA_COORD_ADDR 0x00A1U
#define TDMA_SYNC_FUNC 0xE4U
#define TDMA_SYNC_LEN 21U /* excluding the hardware FCS */
#define TDMA_PERIOD_US 25000U
#define TDMA_A_START_US 1000U
#define TDMA_B_START_US 12500U
#define TDMA_SLOT_US 11000U
#define TDMA_CLEANUP_US 300U
#define TDMA_EXCHANGE_US 3000U
#define TDMA_ANCHOR_US 2300U
#define TDMA_POLL_LAUNCH_US 250U
#define TDMA_RESPONSE_WAIT_US 1200U
#define TDMA_FINAL_LAUNCH_US 2000U
#define TDMA_TX_WAIT_US 500U
#define TDMA_TX_MARGIN_US 300U
#define TDMA_REPORT_RESERVE_US 600U
#define TDMA_RX_TIMEOUT_UUS 1200U
#define TDMA_RETRY_BACKOFF_US 200U
#define TDMA_POLL_PAUSE_US 20U
#define TDMA_COORD_PREP_US 750U
#define TDMA_RESTART_QUIET_US 100000U
#define TDMA_SESSION_QUIET_US 75000U

static inline uint32_t tdma_ticks(uint32_t us) {
  return (uint32_t)(((uint64_t)us * 1248U) / 5U);
}
static inline uint32_t tdma_us(uint32_t ticks) {
  return (uint32_t)(((uint64_t)ticks * 5U) / 1248U);
}
static inline bool tdma_before(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) < 0;
}
static inline uint32_t tdma_earlier(uint32_t a, uint32_t b) {
  return tdma_before(a, b) ? a : b;
}
static inline bool tdma_fits(uint32_t now, uint32_t deadline, uint32_t cost_us) {
  return tdma_before(now, deadline) &&
         (uint32_t)(deadline - now) >= tdma_ticks(cost_us);
}
static inline bool tdma_tag_allowed(uint16_t peer) {
  return peer == 5U || peer == 6U;
}
static inline uint32_t tdma_slot_offset(uint16_t peer) {
  return peer == 5U ? TDMA_A_START_US : TDMA_B_START_US;
}
static inline uint16_t tdma_read16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t tdma_read32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void tdma_write16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline void tdma_write32(uint8_t *p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8U * i));
}
static inline bool tdma_header(const uint8_t *p, size_t len, uint8_t function,
                               uint16_t dest, uint16_t src) {
  return len >= 10U && p[0] == 0x41U && p[1] == 0x88U &&
         tdma_read16(p + 3) == TDMA_PAN && tdma_read16(p + 5) == dest &&
         tdma_read16(p + 7) == src && p[9] == function;
}
static inline void tdma_encode_sync(uint8_t *p, uint32_t session, uint32_t seq) {
  p[0] = 0x41; p[1] = 0x88; p[2] = (uint8_t)seq;
  tdma_write16(p + 3, TDMA_PAN); tdma_write16(p + 5, 0xFFFFU);
  tdma_write16(p + 7, TDMA_COORD_ADDR); p[9] = TDMA_SYNC_FUNC;
  p[10] = TDMA_VERSION; tdma_write16(p + 11, TDMA_SCHEDULE_ID);
  tdma_write32(p + 13, session); tdma_write32(p + 17, seq);
}
static inline bool tdma_decode_sync(const uint8_t *p, size_t len,
                                    uint32_t *session, uint32_t *seq) {
  if (len != TDMA_SYNC_LEN ||
      !tdma_header(p, len, TDMA_SYNC_FUNC, 0xFFFFU, TDMA_COORD_ADDR) ||
      p[10] != TDMA_VERSION || tdma_read16(p + 11) != TDMA_SCHEDULE_ID)
    return false;
  *session = tdma_read32(p + 13); *seq = tdma_read32(p + 17);
  return p[2] == (uint8_t)*seq;
}

typedef struct {
  bool seen;
  uint32_t session, seq, rx_ref;
} TdmaSyncHistory;

/* Every accepted frame grants ONE epoch, never a free-running lease. */
static inline bool tdma_accept_sync(TdmaSyncHistory *s, uint32_t session,
                                    uint32_t seq, uint32_t rx, uint32_t now,
                                    uint16_t tag) {
  if (!tdma_tag_allowed(tag) ||
      !tdma_fits(now, rx + tdma_ticks(tdma_slot_offset(tag)), 50U))
    return false;
  if (s->seen) {
    if (session == s->session) {
      if ((int32_t)(seq - s->seq) <= 0) return false;
    } else if ((uint32_t)(rx - s->rx_ref) < tdma_ticks(TDMA_SESSION_QUIET_US)) {
      return false;
    }
  }
  s->seen = true; s->session = session; s->seq = seq; s->rx_ref = rx;
  return true;
}

#endif
