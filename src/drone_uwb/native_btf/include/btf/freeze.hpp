#pragma once
#include "btf/core.hpp"

namespace btf {
// FC health is assessed by the existing executor, not by the UWB solver.
enum class FcHealth { Healthy, Unknown, Invalid };
enum class FreezeState { Waiting, Running, Frozen, FcUnavailable };
struct FreezeDecision {
  FreezeState state = FreezeState::Waiting;
  bool pause_mission = true, stream_target = false, defer_to_fc_failsafe = false;
  std::optional<XY> target_xy;
  std::optional<XY> last_observation_xy;  // Diagnostics only. Never publish as EV.
  std::optional<Time> last_observation_monotonic_us;
  std::string reason;
};
class FreezePolicy {
 public:
  explicit FreezePolicy(Time observation_max_age_us = 150000);
  // source_host_us is mapped source time in the SAME monotonic clock as now_us.
  // Call for every decision, including rejection. Accepted observations can be
  // forwarded once with their original timestamp, independently of this policy.
  void observe(const Decision&, Time source_host_us, Time now_us);
  // Must tick even when no UWB packet arrives. The caller keeps its existing
  // setpoint sender running independently of solver / JSON / disk activity.
  FreezeDecision tick(Time now_us, FcHealth, std::optional<XY> approved_target);
  void reset();
 private:
  Time max_age_, last_now_ = -1;
  std::optional<Time> accepted_at_, accepted_source_;
  std::optional<XY> last_xy_, frozen_target_, last_streamed_target_;
  bool current_valid_ = false, frozen_ = false;
  std::string cause_ = "observation_not_received";
};
const char* state_name(FreezeState);
}  // namespace btf
