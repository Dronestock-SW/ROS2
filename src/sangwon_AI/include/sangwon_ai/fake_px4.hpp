#pragma once
#include "sangwon_ai/guard.hpp"

namespace sangwon::testing {
// Synthetic second-order plant for policy tests. NOT a PX4 implementation or fitted aircraft model.
class FakePx4 {
 public:
  State state;
  bool freeze_motion{}, refuse_offboard{}, refuse_arm{}, refuse_land{}, refuse_takeoff{};
  bool takeoff_to_hold{}, omit_takeoff_completion{};
  double response_s{0.3};
  FakePx4() {
    state.connected=state.can_hold=state.logging_ok=state.observation_valid=state.route_revalidated=true;
    state.pose_quality=state.yaw_quality=Quality::Valid;
    state.battery=0.9;
    state.native_takeoff_target_validated=true;
  }
  void stamp(double now) {
    state.pose_time_s=state.mode_time_s=state.quality_time_s=state.log_time_s=state.battery_time_s=now;
    state.global_position_time_s=now;
  }
  void apply(const Output& out,double dt,double now) {
    if(out.request) {
      const bool reject=(out.kind==OutputKind::RequestOffboard&&refuse_offboard)
        ||(out.kind==OutputKind::RequestArm&&refuse_arm)||(out.kind==OutputKind::RequestLand&&refuse_land)
        ||(out.kind==OutputKind::RequestTakeoff&&refuse_takeoff);
      state.control_reply=ControlReply{*out.request,reject?ReplyResult::Rejected:ReplyResult::Accepted,now};
      if(out.kind==OutputKind::RequestTakeoff&&!reject) {native_request_=out.request;native_target_=out.request->target;state.mode=Mode::Takeoff;}
    }
    if(out.kind==OutputKind::RequestOffboard&&!refuse_offboard) state.mode=Mode::Offboard;
    if(out.kind==OutputKind::RequestArm&&!refuse_arm&&state.landed) state.armed=true;
    if(out.kind==OutputKind::RequestLand&&!refuse_land) state.mode=Mode::Land;
    if(out.kind==OutputKind::GroundDisarm&&state.landed) state.armed=false;
    if(state.mode==Mode::Land) {
      state.velocity={0,0,-0.3};
      state.pose.p.z=std::max(0.0,state.pose.p.z-0.3*dt);
      if(state.pose.p.z<=0) { state.landed=true; state.armed=false; state.velocity={}; }
    } else if(state.armed&&state.mode==Mode::Takeoff&&native_request_&&!freeze_motion) {
      const double dz=native_target_.p.z-state.pose.p.z;
      state.velocity={0,0,std::clamp(dz/dt,-0.3,0.3)};
      state.pose.p.z+=state.velocity.z*dt;
      if(state.pose.p.z>0.01) state.landed=false;
      if(std::abs(dz)<1e-6) {
        state.velocity={};
        if(!omit_takeoff_completion) state.takeoff_completion=ControlCompletion{*native_request_,now};
        if(takeoff_to_hold) state.mode=Mode::Hold;
      }
    } else if(state.armed&&state.mode==Mode::Offboard&&(out.kind==OutputKind::Position||out.stream_position)&&!freeze_motion) {
      auto desired=(out.target.p-state.pose.p)*2.0;
      if(norm(desired)>1.0) desired=desired*(1/norm(desired));
      auto dv=(desired-state.velocity)*(dt/response_s);
      if(norm(dv)>1.5*dt) dv=dv*((1.5*dt)/norm(dv));
      state.velocity=state.velocity+dv;
      state.pose.p=state.pose.p+state.velocity*dt;
      state.pose.p.z=std::max(0.0,state.pose.p.z);
      state.pose.yaw=wrap(state.pose.yaw+std::clamp(wrap(out.target.yaw-state.pose.yaw),-0.5*dt,0.5*dt));
      if(state.pose.p.z>0.01) state.landed=false;
    }
    stamp(now);
  }
 private:
  std::optional<ControlRequest> native_request_;
  Pose native_target_;
};
inline Mission example_mission() {
  Mission m;
  m.execution_id="replay-mission-1"; m.approved_fixture=true;
  m.points={{{2,0,1},pi/2,0.5},{{2,1,1.4},std::nullopt,0.25}};
  return m;
}
} // namespace sangwon::testing
