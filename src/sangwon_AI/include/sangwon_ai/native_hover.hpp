#pragma once
#include <cmath>
#include <string>

namespace sangwon::bench {
struct Sample {
  bool connected{}, armed{}, landed{}, fresh{}, estimator_ok{}, rc_ok{}, rc_stick{}, rc_switch{};
  std::string mode;
  double x{}, y{}, z{}, speed{};
};
enum class Action { None, Takeoff, Arm, Land, Position };
// One run per process, including rejected/aborted runs. PX4 owns every setpoint.
class NativeHover {
 public:
  std::string phase{"IDLE"}, reason;
  double height{1.3}, hover_s{2.0};
  bool enabled{}, used{};
  std::string readiness(const Sample& s) const {
    if (!enabled) return "OUTPUT_NOT_ENABLED";
    if (used) return "NEW_SESSION_REQUIRED";
    if (!s.connected || !s.fresh) return "TELEMETRY_NOT_FRESH";
    if (!s.estimator_ok) return "PX4_ESTIMATOR_NOT_READY";
    if (!s.rc_ok) return "RC_NOT_READY";
    if (s.armed || !s.landed) return "NOT_DISARMED_ON_GROUND";
    if (s.mode != "POSCTL") return "SELECT_RC_POSITION_MODE";
    if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.z) || !std::isfinite(s.speed)) return "INVALID_POSE";
    if (s.speed > 0.2) return "NOT_STATIONARY";
    return {};
  }
  bool start(const Sample& s, double now) {
    reason = readiness(s); if (!reason.empty()) return false;
    used = true; x_=s.x; y_=s.y; z_=s.z; start_=now;
    change("TAKEOFF_MODE", now); return true;
  }
  bool active() const { return phase=="TAKEOFF_MODE" || phase=="ARM" || phase=="CLIMB" || phase=="HOVER" || phase=="LANDING"; }
  void release(const std::string& why) { phase="RELEASED"; reason=why; }
  void rejected(const std::string& why) { failed_=true; reason=why; }
  void land(double now) { if (active()) change("LANDING",now); }
  Action tick(const Sample& s, double now) {
    if (phase=="HANDOFF") {
      if (s.fresh && s.mode=="POSCTL") release("RC_POSITION_CONFIRMED");
      else if (now-entered_ > 4) release("RC_HANDOFF_UNCONFIRMED");
      return Action::None;
    }
    if (!active()) return Action::None;
    if (!s.connected || !s.fresh) { release("TELEMETRY_LOST_PX4_FAILSAFE"); return Action::None; }
    if (s.rc_switch) { release("RC_SWITCH_CHANGED"); return Action::None; }
    // Never overwrite a pilot/failsafe mode with another automatic mode.
    bool expected = s.mode=="AUTO.TAKEOFF" || s.mode=="AUTO.LOITER" || s.mode=="AUTO.LAND";
    if (phase=="TAKEOFF_MODE") expected = expected || s.mode=="POSCTL";
    if (!expected) { release("PX4_MODE_CHANGED"); return Action::None; }
    if (s.rc_stick) {
      change("HANDOFF",now); reason="RC_STICK_TAKEOVER";
      return Action::Position;
    }
    if (!s.rc_ok) { release("RC_LOST_PX4_FAILSAFE"); return Action::None; }
    if (phase!="TAKEOFF_MODE" && phase!="ARM" && !s.armed) {
      if (s.landed && phase=="LANDING") { phase="COMPLETE"; reason="LANDED_AND_DISARMED"; }
      else release("UNEXPECTED_DISARM");
      return Action::None;
    }
    if (s.mode=="AUTO.LAND" && phase!="LANDING") change("LANDING",now);
    if ((failed_ || !s.estimator_ok || now-start_>35 || std::hypot(s.x-x_,s.y-y_)>0.6) && phase!="LANDING") {
      if (!s.armed) { release(reason.empty()?"PREFLIGHT_REJECTED":reason); return Action::None; }
      change("LANDING",now);
    }
    if (phase=="TAKEOFF_MODE") {
      if (s.mode=="AUTO.TAKEOFF") change("ARM",now);
      else if (now-entered_>5) release("TAKEOFF_MODE_TIMEOUT");
      else return request(Action::Takeoff,now);
    }
    if (phase=="ARM") {
      if (s.armed) change("CLIMB",now);
      else if (!s.landed || s.mode!="AUTO.TAKEOFF" || now-entered_>5) release("ARM_NOT_CONFIRMED");
      else return request(Action::Arm,now);
    }
    if (phase=="CLIMB" || phase=="HOVER") {
      bool settled = !s.landed && s.mode=="AUTO.LOITER" && std::abs(s.z-z_-height)<=0.15 && s.speed<=0.2 && std::hypot(s.x-x_,s.y-y_)<=0.3;
      if (!settled) stable_=-1;
      else if (stable_<0) stable_=now;
      if (phase=="CLIMB" && stable_>=0 && now-stable_>=0.5) change("HOVER",now);
      if (phase=="HOVER" && !settled) change("CLIMB",now);
      if (phase=="HOVER" && now-entered_>=hover_s) change("LANDING",now);
    }
    if (phase=="LANDING") {
      if (!s.armed && s.landed) { phase="COMPLETE"; reason="LANDED_AND_DISARMED"; }
      else if (s.mode!="AUTO.LAND" && now-entered_<6) return request(Action::Land,now);
      else if (s.mode!="AUTO.LAND" || now-entered_>35) release("LAND_NOT_CONFIRMED");
    }
    return Action::None;
  }
 private:
  double x_{},y_{},z_{},start_{},entered_{},last_request_{-1e9},stable_{-1};
  bool failed_{};
  void change(const std::string& phase_value,double now) { phase=phase_value; entered_=now; last_request_=-1e9; }
  Action request(Action a,double now) { if (now-last_request_<1) return Action::None; last_request_=now; return a; }
};
}
