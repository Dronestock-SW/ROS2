#include "../common/SingleTagProtocol.h"
#include <cassert>
#include <cstdio>
#include <cstring>
int main() {
  uint8_t frame[ST_REPORT_LEN]={};
  st_header(frame,ST_POLL,0xA1,6,123,0x12345678);
  assert(st_matches(frame,ST_HEADER_LEN,ST_POLL,0xA1,6,123,0x12345678));
  for(size_t n=0;n<ST_HEADER_LEN;n++) assert(!st_envelope(frame,n,ST_POLL,0xA1,6));
  assert(!st_matches(frame,ST_HEADER_LEN,ST_POLL,0xA2,6,123,0x12345678));
  assert(!st_matches(frame,ST_HEADER_LEN,ST_POLL,0xA1,6,124,0x12345678));
  assert(!st_matches(frame,ST_HEADER_LEN,ST_POLL,0xA1,6,123,0x12345679));
  frame[10]=1; assert(!st_envelope(frame,ST_HEADER_LEN,ST_POLL,0xA1,6));
  frame[10]=2;frame[9]=0xE0;assert(!st_envelope(frame,ST_HEADER_LEN,ST_POLL,0xA1,6));
  StHistory h={};
  assert(st_accept(&h,1,0xffffffff,100000));
  assert(st_accept(&h,1,0,120000));
  assert(!st_accept(&h,1,0,140000));
  assert(!st_accept(&h,1,0xffffffff,140000));
  assert(!st_accept(&h,2,1,150000));
  assert(st_accept(&h,2,1,300000));
  assert(!st_accept(&h,1,2,500000));
  assert(st_before(0xfffffff0,0x20));
  assert(st_us(st_ticks(3000))==3000);
  assert(st_fits(0xffff0000,0xffff0000+st_ticks(3000),2900));
  assert(!st_fits(0xffff0000,0xffff0000+st_ticks(3000),3100));
  std::puts("SingleTag RF protocol: length/version/address/session/replay/wrap/deadline PASS");
}
