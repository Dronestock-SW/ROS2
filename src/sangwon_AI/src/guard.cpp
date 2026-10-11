#include "sangwon_ai/guard.hpp"

namespace sangwon {
FlightGuard::FlightGuard(Config c,std::string execution,std::string boot,
                         std::uint64_t frame,std::uint64_t revision)
  : c_(c),execution_(std::move(execution)),boot_(std::move(boot)),frame_(frame),revision_(revision),
    modes_(c_,execution_,boot_,frame_,revision_) {
  c_.validate();
  if(execution_.empty()||boot_.empty()) throw std::invalid_argument("Missing guard identity");
}
bool FlightGuard::accept(const Intent& i,double now) {
  switch(i.kind) {
    case IntentKind::Prestream: case IntentKind::Position: case IntentKind::Hold:
    case IntentKind::Offboard: case IntentKind::Arm: case IntentKind::Land: case IntentKind::Takeoff:
    case IntentKind::AbortGround: case IntentKind::Release: break;
    default: return false;
  }
  if(i.schema_version!=1||i.execution_id!=execution_||i.boot_session!=boot_
      ||i.frame_epoch!=frame_||i.transform_revision!=revision_||!finite(i.target)
      ||!std::isfinite(i.expires_s)||!fresh(now,i.issued_s,c_.lease_s)
      ||i.expires_s<=now||i.expires_s-i.issued_s>c_.lease_s+1e-9
      ||i.generation<generation_||i.sequence<=sequence_) return false;
  if(released_&&i.kind!=IntentKind::Release) return false;
  if(landing_&&i.kind!=IntentKind::Land&&i.kind!=IntentKind::Release) return false;
  if(i.kind==IntentKind::Takeoff&&c_.takeoff_policy!=TakeoffPolicy::NativePx4) return false;
  generation_=i.generation; sequence_=i.sequence; intent_=i; active_=true;
  if(i.kind==IntentKind::Land) landing_=true;
  return true;
}
Output FlightGuard::land(const State& s,double now,const std::string& why) {
  landing_=true;
  if(!land_since_) {land_since_=now;land_hold_=finite(s.pose)?s.pose:Pose{};fault_reason_=why;modes_.cancel(now);}
  if(s.mode==Mode::Land) return {OutputKind::None,{},why};
  if(now-*land_since_>=c_.transition_s || !s.connected) {
    released_=true;fault_reason_="LAND_UNCONFIRMED_STOP_STREAM"; return {OutputKind::Release,{},fault_reason_};
  }
  if(s.landed&&!s.armed) return {OutputKind::None,{},why};
  const auto op=s.landed?ControlOperation::Disarm:ControlOperation::Land;
  if(modes_.request()&&modes_.request()->operation!=op) modes_.cancel(now);
  if(modes_.failed()) {
    released_=true;fault_reason_="LAND_UNCONFIRMED_STOP_STREAM";
    return {OutputKind::Release,{},fault_reason_};
  }
  if(!modes_.pending()&&(!modes_.request()||modes_.request()->operation!=op))
    modes_.begin(op,land_hold_,s,now);
  const bool hold_valid=!s.landed&&s.mode==Mode::Offboard&&s.frame_epoch==frame_
    &&pose_valid(s,now,c_)&&yaw_valid(s,now)&&s.can_hold;
  auto out=modes_.output(now,land_hold_,hold_valid);
  out.reason=why;
  return out;
}
Output FlightGuard::tick(const State& s,double now) {
  if(!std::isfinite(now)|| (last_tick_&&now<*last_tick_)) {
    released_=true;fault_reason_="CLOCK_INVALID"; return {OutputKind::Release,{},fault_reason_};
  }
  const bool gap=last_tick_&&now-*last_tick_>0.100001;
  if(gap) prestream_since_.reset();
  last_tick_=now;
  if(released_) return {OutputKind::Release,{},"OWNERSHIP_RELEASED"};
  if(s.rc_override||s.px4_failsafe) {
    released_=true;fault_reason_="RC_OR_PX4_TAKEOVER";modes_.cancel(now); return {OutputKind::Release,{},fault_reason_};
  }
  if(!fresh(now,s.mode_time_s,1.0)) {
    released_=true;fault_reason_="MODE_STALE";modes_.cancel(now); return {OutputKind::Release,{},fault_reason_};
  }
  modes_.observe(s,now);
  if(s.mode==Mode::Land) return land(s,now,"PX4_LAND");
  if(modes_.unexpected_mode()&&!landing_) {
    released_=true;fault_reason_="UNEXPECTED_MODE_EXIT";modes_.cancel(now); return {OutputKind::Release,{},fault_reason_};
  }
  if(!active_) return {};
  if(landing_) return land(s,now,"LAND_LOCK");
  if(modes_.failed()) return land(s,now,modes_.state()==TransitionState::Rejected?"PX4_COMMAND_REJECTED":"PX4_MODE_TIMEOUT");
  if(s.frame_epoch!=frame_||!yaw_valid(s,now)||s.battery_sensor_failed)
    return land(s,now,"FRAME_OR_YAW_OR_BATTERY_INVALID");
  if(!intent_||now>=intent_->expires_s) return land(s,now,"INTENT_EXPIRED");
  const auto& i=*intent_;
  if(i.kind==IntentKind::Release) {
    released_=true; return {OutputKind::Release,{},"REQUESTED_RELEASE"};
  }
  if(i.kind==IntentKind::AbortGround) {
    if(!s.landed) return land(s,now,"ABORT_AIRBORNE");
    return {s.armed?OutputKind::GroundDisarm:OutputKind::None,{},"ABORT_GROUND"};
  }
  if(!pose_valid(s,now,c_)||!s.can_hold) return land(s,now,"PX4_CANNOT_HOLD");
  if(s.mode==Mode::Offboard) modes_.adopt_offboard();
  if(i.kind==IntentKind::Prestream) {
    if(!prestream_since_||gap) prestream_since_=now;
    Output out{OutputKind::Position,i.target,"PRESTREAM"};out.stream_position=true;return out;
  }
  if(i.kind==IntentKind::Offboard) {
    if(!prestream_since_||gap||now-*prestream_since_<c_.prestream_s)
    {
      if(!prestream_since_) prestream_since_=now;
      Output out{OutputKind::Position,i.target,"PRESTREAM_REQUIRED"};out.stream_position=true;return out;
    }
    if(s.mode==Mode::Offboard) {Output out{OutputKind::Position,i.target,"OFFBOARD_CONFIRMED"};out.stream_position=true;return out;}
    if(!modes_.begin(ControlOperation::Offboard,i.target,s,now)) return land(s,now,"CONTROL_TRANSACTION_CONFLICT");
    return modes_.output(now,i.target,true);
  }
  if(i.kind==IntentKind::Arm) {
    if(!s.landed||(s.mode!=Mode::Offboard&&!(c_.takeoff_policy==TakeoffPolicy::NativePx4&&s.mode==Mode::Takeoff)))
      return {OutputKind::None,{},"ARM_REJECTED"};
    if(s.armed) return {OutputKind::None,{},"ARM_CONFIRMED"};
    if(!modes_.begin(ControlOperation::Arm,i.target,s,now)) return land(s,now,"CONTROL_TRANSACTION_CONFLICT");
    return modes_.output(now,i.target,s.mode==Mode::Offboard);
  }
  if(i.kind==IntentKind::Takeoff) {
    if(!s.native_takeoff_target_validated) return land(s,now,"NATIVE_TAKEOFF_TARGET_UNVERIFIED");
    if(native_target_&&(norm(native_target_->p-i.target.p)>1e-9||std::abs(wrap(native_target_->yaw-i.target.yaw))>1e-9))
      return land(s,now,"NATIVE_TAKEOFF_TARGET_CHANGED");
    if(s.mode==Mode::Takeoff||(s.mode==Mode::Hold&&modes_.takeoff_completed())) return {OutputKind::None,{},"NATIVE_TAKEOFF_ACTIVE"};
    if(!s.landed||s.armed) return land(s,now,"NATIVE_TAKEOFF_GROUND_REQUIRED");
    native_target_=i.target;
    if(!modes_.begin(ControlOperation::Takeoff,i.target,s,now)) return land(s,now,"CONTROL_TRANSACTION_CONFLICT");
    return modes_.output(now,{},false);
  }
  if(s.mode!=Mode::Offboard||!s.armed) return {OutputKind::None,{},"NOT_ACQUIRED"};
  Output out{OutputKind::Position,i.target,"VALID"};out.stream_position=true;return out;
}
} // namespace sangwon
