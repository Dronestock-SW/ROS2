#pragma once
#include "sangwon_ai/types.hpp"

namespace sangwon {
enum class TransitionState { Idle, AwaitingState, Confirmed, Rejected, TimedOut, Cancelled };
const char* transition_state_name(TransitionState);
struct ModeTransitionEvent {
  ControlRequest request;TransitionState state;Mode expected_mode,observed_mode;
  unsigned attempts;bool ack_accepted,takeoff_completed;double observed_s;
};
// Transport-free command transactions. ACK never substitutes for actual FC
// state. The future hardware adapter must bind replies to the original request.
class ModeManager {
 public:
  ModeManager(Config, std::string execution, std::string boot,
              std::uint64_t frame, std::uint64_t revision);
  bool begin(ControlOperation, Pose, const State&, double now);
  void observe(const State&, double now);
  Output output(double now, Pose stream_target, bool stream);
  void cancel(double now);
  std::vector<ModeTransitionEvent> drain_events();
  void adopt_offboard();
  bool pending() const { return state_==TransitionState::AwaitingState; }
  bool failed() const { return state_==TransitionState::Rejected || state_==TransitionState::TimedOut; }
  bool unexpected_mode() const { return unexpected_; }
  bool takeoff_completed() const { return takeoff_completed_; }
  bool owned() const { return owned_; }
  Mode expected_mode() const { return expected_; }
  TransitionState state() const { return state_; }
  unsigned attempts() const { return attempts_; }
  bool ack_accepted() const { return ack_accepted_; }
  const std::optional<ControlRequest>& request() const { return request_; }
 private:
  Config c_; std::string execution_,boot_;
  std::uint64_t frame_,revision_,next_id_{};
  std::optional<ControlRequest> request_,takeoff_request_;
  TransitionState state_{TransitionState::Idle};
  Mode expected_{Mode::Unknown},from_{Mode::Unknown};
  bool owned_{},unexpected_{},ack_accepted_{},takeoff_completed_{};
  unsigned attempts_{};
  double last_send_s_{};
  Mode observed_{Mode::Unknown};
  std::vector<ModeTransitionEvent> events_;
  void record(double now);
};
} // namespace sangwon
