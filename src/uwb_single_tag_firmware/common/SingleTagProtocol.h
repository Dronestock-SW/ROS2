#ifndef SINGLE_TAG_PROTOCOL_H
#define SINGLE_TAG_PROTOCOL_H
/* Single Tag B DS-TWR v2. No coordinator, TDMA slots, or tag arbitration.
 * Times with _US suffix use real microseconds; DW RX timeout uses UUS.
 * The old E0..E4 functions cannot be accepted by this protocol.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ST_VERSION 2U
#define ST_PAN 0xDECAU
#define ST_TAG 6U
#define ST_POLL 0xD0U
#define ST_RESP 0xD1U
#define ST_FINAL 0xD2U
#define ST_REPORT 0xD3U
#define ST_HEADER_LEN 19U
#define ST_REPORT_LEN 32U
#define ST_POLL_RX_IDX 19U
#define ST_RESP_TX_IDX 23U
#define ST_FINAL_RX_IDX 27U
#define ST_STATUS_IDX 31U
#define ST_EXCHANGE_US 3000U
#define ST_ANCHOR_US 2300U
#define ST_CLEANUP_US 200U
#define ST_RESPONSE_WAIT_US 1400U
#define ST_REPORT_RESERVE_US 600U
#define ST_RX_TIMEOUT_UUS 1200U
#define ST_POLL_PAUSE_US 10U
#define ST_SESSION_QUIET_US 100000U
#define ST_CYCLE_BUDGET_US 18000U

static inline uint32_t st_ticks(uint32_t us) {
  return (uint32_t)(((uint64_t)us * 1248U) / 5U);
}
static inline uint32_t st_us(uint32_t ticks) {
  return (uint32_t)(((uint64_t)ticks * 5U) / 1248U);
}
static inline bool st_before(uint32_t now, uint32_t deadline) {
  return (int32_t)(now-deadline) < 0;
}
static inline uint32_t st_earlier(uint32_t a, uint32_t b) {
  return st_before(a,b) ? a : b;
}
static inline bool st_fits(uint32_t now, uint32_t deadline, uint32_t us) {
  return st_before(now,deadline) && (deadline-now)>=st_ticks(us);
}
static inline uint16_t st_read16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1]<<8);
}
static inline uint32_t st_read32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1]<<8) |
    ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static inline void st_write16(uint8_t* p, uint16_t v) {
  p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8);
}
static inline void st_write32(uint8_t* p, uint32_t v) {
  for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i));
}
static inline void st_header(uint8_t* p, uint8_t function, uint16_t dest,
    uint16_t src, uint32_t session, uint32_t transaction) {
  p[0]=0x41; p[1]=0x88; p[2]=(uint8_t)transaction;
  st_write16(p+3,ST_PAN); st_write16(p+5,dest); st_write16(p+7,src);
  p[9]=function; p[10]=ST_VERSION;
  st_write32(p+11,session); st_write32(p+15,transaction);
}
static inline bool st_envelope(const uint8_t* p,size_t len,uint8_t function,
    uint16_t dest,uint16_t src) {
  const size_t expected=function==ST_REPORT?ST_REPORT_LEN:ST_HEADER_LEN;
  return len==expected && p[0]==0x41 && p[1]==0x88 &&
    st_read16(p+3)==ST_PAN && st_read16(p+5)==dest &&
    st_read16(p+7)==src && p[9]==function && p[10]==ST_VERSION &&
    st_read32(p+11)!=0 && p[2]==(uint8_t)st_read32(p+15);
}
static inline bool st_matches(const uint8_t* p,size_t len,uint8_t function,
    uint16_t dest,uint16_t src,uint32_t session,uint32_t transaction) {
  return st_envelope(p,len,function,dest,src) &&
    st_read32(p+11)==session && st_read32(p+15)==transaction;
}

/* Reject duplicate/reordered transactions and recently retired sessions.
 * This is freshness validation, not RF authentication. */
typedef struct {
  bool seen;
  uint32_t session, transaction, accepted_at_us;
  uint32_t retired[4];
  uint8_t retired_next;
} StHistory;
static inline bool st_accept(StHistory* h,uint32_t session,uint32_t tx,uint32_t now_us) {
  if (!session) return false;
  if (h->seen) {
    if (session==h->session) {
      if ((int32_t)(tx-h->transaction)<=0) return false;
    } else {
      if ((uint32_t)(now_us-h->accepted_at_us)<ST_SESSION_QUIET_US) return false;
      for(unsigned i=0;i<4;i++) if(h->retired[i]==session) return false;
      h->retired[h->retired_next++ & 3U]=h->session;
    }
  }
  h->seen=true; h->session=session; h->transaction=tx; h->accepted_at_us=now_us;
  return true;
}
#endif
