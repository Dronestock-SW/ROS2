#pragma once

#include <stdint.h>

namespace LoRaRecoveryPolicy {
constexpr bool txDeadlineExpired(
    bool busy, uint32_t now, uint32_t started, uint32_t deadline) {
  return busy && static_cast<uint32_t>(now - started) >= deadline;
}

static_assert(!txDeadlineExpired(false, 9000, 1, 3500), "idle is not a timeout");
static_assert(!txDeadlineExpired(true, 3599, 100, 3500), "before deadline");
static_assert(txDeadlineExpired(true, 3600, 100, 3500), "at deadline");
static_assert(!txDeadlineExpired(true, 5, UINT32_MAX - 10, 3500), "short wrap");
static_assert(txDeadlineExpired(true, 3500, UINT32_MAX - 10, 3500), "expired wrap");
}  // namespace LoRaRecoveryPolicy
