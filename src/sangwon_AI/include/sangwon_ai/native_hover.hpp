#pragma once
#include <cmath>
#include <string>

namespace sangwon::bench {
struct Sample {
  bool connected{}, armed{}, landed{}, fresh{}, estimator_ok{}, rc_ok{}, rc_stick{}, rc_switch{};
  bool command_pending{}, completion_fresh{true};
  std::string mode;
  double x{}, y{}, z{}, speed{};
};
enum class Action { None, Takeoff, Arm, Land, Position, Disarm };
// One run per process. PX4 owns altitude, attitude and all setpoints.
class NativeHover {
 public:
  std::string phase{"IDLE"}, reason, outcome{"NOT_RUN"};
  double height{1.3}, hover_s{2.0};
  bool enabled{}, used{}, landing_verified{};
  std::string readiness(const Sample& s) const {
    if (!enabled) return "OUTPUT_NOT_ENABLED";
    if (used) return "NEW_SESSION_REQUIRED";
    if (!s.connected || !s.fresh) return "TELEMETRY_NOT_FRESH";
    if (!s.estimator_ok) return "PX4_ESTIMATOR_NOT_READY";
    if (!s.rc_ok) return "RC_NOT_READY";
    if (s.armed || !s.landed) return "NOT_DISARMED_ON_GROUND";
    if (s.mode != "POSCTL") return "SELECT_RC_POSITION_MODE";
    if (!finite(s)) return "INVALID_POSE";
    if (s.speed > 0.2) return "NOT_STATIONARY";
    return {};
  }
  bool start(const Sample& s, double now) {
    reason = readiness(s); if (!reason.empty()) return false;
    used = true; outcome="RUNNING"; x_=s.x; y_=s.y; z_=s.z; start_=now;
    change("TAKEOFF_MODE", now); return true;
  }
  bool active() const { return phase=="TAKEOFF_MODE" || phase=="ARM" || phase=="CLIMB" || phase=="HOVER" || phase=="LANDING" || phase=="ABORTING"; }
  void release(const std::string& why) { phase="RELEASED"; reason=why; outcome="UNCONFIRMED"; }
  void rejected(const std::string& why) { failed_=true; if(reason.empty())reason=why; }
  void land(double now) {
    if (active() && phase!="ABORTING") { cancelled_=true; reason="OPERATOR_CANCELLED"; change("ABORTING",now); }
  }
  Action tick(const Sample& s, double now) {
    if (phase=="HANDOFF") {
      if (s.fresh && s.mode=="POSCTL") release("RC_POSITION_CONFIRMED");
      else if (now-entered_ > 4) release("RC_HANDOFF_UNCONFIRMED");
      return Action::None;
    }
    if (!active()) return Action::None;
    if (!s.connected || !s.fresh || !finite(s)) { release("TELEMETRY_LOST_PX4_FAILSAFE"); return Action::None; }
    // An absent RC sample is never interpreted as a stick/switch change.
    if (!s.rc_ok) { release("RC_LOST_PX4_FAILSAFE"); return Action::None; }
    if (s.rc_switch) { release("RC_SWITCH_CHANGED"); return Action::None; }
    bool expected = s.mode=="AUTO.TAKEOFF" || s.mode=="AUTO.LOITER" || s.mode=="AUTO.LAND";
    if (phase=="TAKEOFF_MODE" || phase=="ABORTING") expected = expected || s.mode=="POSCTL";
    if (!expected) { release("PX4_MODE_CHANGED"); return Action::None; }
    if (s.rc_stick) { change("HANDOFF",now); reason="RC_STICK_TAKEOVER"; return Action::None; }
    if (s.mode=="AUTO.LAND" && phase!="LANDING" && phase!="ABORTING") {
      rejected("EXTERNAL_LAND"); change("ABORTING",now);
    }
    if (phase!="LANDING" && phase!="ABORTING") {
      if (!s.estimator_ok) rejected("ESTIMATOR_LOST");
      if (now-start_>35) rejected("RUN_TIMEOUT");
      if (std::hypot(s.x-x_,s.y-y_)>0.6) rejected("HORIZONTAL_DRIFT");
      if (failed_) change("ABORTING",now);
    }
    if (phase=="TAKEOFF_MODE") {
      if (s.mode=="AUTO.TAKEOFF") change("ARM",now);
      else if (now-entered_>5) { rejected("TAKEOFF_MODE_TIMEOUT"); change("ABORTING",now); }
      else return request(Action::Takeoff,now);
    }
    if (phase=="ARM") {
      if (s.armed) change("CLIMB",now);
      else if (!s.landed || s.mode!="AUTO.TAKEOFF" || now-entered_>5) { rejected("ARM_NOT_CONFIRMED"); change("ABORTING",now); }
      else return request(Action::Arm,now);
    }
    if ((phase=="CLIMB" || phase=="HOVER") && !s.armed) { release("UNEXPECTED_DISARM"); return Action::None; }
    if (phase=="CLIMB" || phase=="HOVER") {
      bool settled = !s.landed && s.mode=="AUTO.LOITER" && std::abs(s.z-z_-height)<=0.15 && s.speed<=0.2 && std::hypot(s.x-x_,s.y-y_)<=0.3;
      if (!settled) stable_=-1;
      else if (stable_<0) stable_=now;
      if (phase=="CLIMB" && stable_>=0 && now-stable_>=0.5) change("HOVER",now);
      if (phase=="HOVER" && !settled) change("CLIMB",now);
      if (phase=="HOVER" && now-entered_>=hover_s) change("LANDING",now);
    }
    if (phase=="LANDING" || phase=="ABORTING") {
      // In-flight requests must settle before a disarmed sample can end a run.
      if (s.command_pending) return Action::None;
      if (!s.armed && s.landed && s.completion_fresh) {
        if (phase=="ABORTING" && s.mode=="AUTO.TAKEOFF") return request(Action::Position,now);
        landing_verified=true;
        phase=failed_?"FAILED":cancelled_?"CANCELLED":"COMPLETE";
        outcome=failed_?"FAIL":cancelled_?"CANCELLED":"PASS";
        if(reason.empty())reason="LANDED_AND_DISARMED";
      } else if (phase=="ABORTING" && s.armed && s.landed) return request(Action::Disarm,now);
      else if (s.armed && s.mode!="AUTO.LAND" && now-entered_<6) return request(Action::Land,now);
      else if (now-entered_>35 || (s.armed && s.mode!="AUTO.LAND" && now-entered_>=6)) release("LAND_NOT_CONFIRMED");
    }
    return Action::None;
  }
 private:
  double x_{},y_{},z_{},start_{},entered_{},last_request_{-1e9},stable_{-1};
  bool failed_{},cancelled_{};
  static bool finite(const Sample& s) { return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.z) && std::isfinite(s.speed); }
  void change(const std::string& value,double now) { phase=value; entered_=now; last_request_=-1e9; }
  Action request(Action a,double now) { if (now-last_request_<1) return Action::None; last_request_=now; return a; }
};
}
