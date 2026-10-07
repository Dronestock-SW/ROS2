#pragma once
#include "sangwon_ai/mode_manager.hpp"

namespace sangwon {
class FlightGuard {
 public:
  FlightGuard(Config config, std::string execution, std::string boot,
              std::uint64_t frame, std::uint64_t revision);
  bool accept(const Intent& intent,double now);
  Output tick(const State& state,double now);
  bool landing_locked() const { return landing_; }
  bool released() const { return released_; }
  const std::string& fault_reason() const { return fault_reason_; }
  const ModeManager& modes() const { return modes_; }
  std::vector<ModeTransitionEvent> drain_mode_events() { return modes_.drain_events(); }
 private:
  Config c_; std::string execution_,boot_;
  std::uint64_t frame_,revision_,generation_{},sequence_{};
  std::optional<Intent> intent_;
  bool landing_{}, released_{}, active_{};
  ModeManager modes_;
  std::string fault_reason_;
  std::optional<Pose> native_target_;
  Pose land_hold_;
  std::optional<double> land_since_,prestream_since_,last_tick_;
  Output land(const State&,double now,const std::string& reason);
};
} // namespace sangwon
