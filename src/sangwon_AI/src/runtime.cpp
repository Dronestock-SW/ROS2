#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/geometry.hpp"
#include "sangwon_ai/planner.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <functional>
#include <limits>

namespace sangwon {
namespace {
class TickAction : public BT::StatefulActionNode {
 public:
  TickAction(const std::string& n,const BT::NodeConfig& cfg,
             std::function<void()> step,std::function<void()> halt)
    : BT::StatefulActionNode(n,cfg),step_(std::move(step)),halt_(std::move(halt)) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { step_(); return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { step_(); return BT::NodeStatus::RUNNING; }
  void onHalted() override { halt_(); }
 private:
  std::function<void()> step_,halt_;
};
}
struct Runtime::TreeImpl {
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  explicit TreeImpl(Runtime& r) {
    for(const std::string name : {"Yield","Land","Recover","Blocked","Pause","Execute"}) {
      if(name!="Execute") {
        const std::string condition=name=="Blocked"?"ShouldBlock":"Should"+name;
        factory.registerSimpleCondition(condition,[&r,name](BT::TreeNode&) {
          return r.choose(name) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        });
      }
      factory.registerNodeType<TickAction>(name,[&r,name]{r.run(name);},[&r,name]{r.halt(name);});
    }
    tree=factory.createTreeFromFile(SANGWON_TREE_PATH);
  }
};
const char* phase_name(Phase p) {
  switch(p) {
    case Phase::Idle:return "IDLE"; case Phase::Acquire:return "ACQUIRE";
    case Phase::Takeoff:return "TAKEOFF"; case Phase::StartYaw:return "START_YAW";
    case Phase::Vertical:return "VERTICAL"; case Phase::Travel:return "TRAVEL";
    case Phase::Rotate:return "ROTATE"; case Phase::Dwell:return "DWELL";
    case Phase::Scan:return "SCAN";
    case Phase::Return:return "RETURN"; case Phase::HomeYaw:return "HOME_YAW";
    case Phase::Landing:return "LANDING"; case Phase::Complete:return "COMPLETE";
    case Phase::Manual:return "MANUAL"; case Phase::Aborted:return "ABORTED";
    case Phase::Handoff:return "OFFBOARD_HANDOFF";
  }
  return "UNKNOWN";
}
Runtime::Runtime(Config c):c_(c) { c_.validate(); bt_=std::make_unique<TreeImpl>(*this); }
Runtime::~Runtime()=default;
bool Runtime::start(const Mission& m,const State& s,double now) {
  // One execution per object in this prototype. Reconstruct only after confirmed landing.
  if(started_||!std::isfinite(now)||m.execution_id.empty()||!m.approved_fixture
      ||m.points.empty()||m.points.size()>1000||m.transform_revision==0
      ||!std::isfinite(m.takeoff_z_m)||m.takeoff_z_m<=0
      ||!std::isfinite(m.requested_speed_mps)||m.requested_speed_mps<=0
      ||m.requested_speed_mps>c_.xy_speed_mps
      ||(m.start_yaw&&!std::isfinite(*m.start_yaw))
      ||!s.connected||!s.landed||s.armed||s.mode==Mode::Land||s.rc_override||s.px4_failsafe
      ||!pose_valid(s,now,c_)||!yaw_valid(s,now)||!stopped(s,c_)||!s.can_hold
      ||!s.observation_valid||!s.route_revalidated||!s.return_route_valid
      ||s.blocked||!s.logging_ok||!fresh(now,s.log_time_s,c_.log_age_s)
      ||!fresh(now,s.mode_time_s,1.0)||s.battery_sensor_failed) return false;
  if(c_.takeoff_policy==TakeoffPolicy::NativePx4
      &&(!s.native_takeoff_target_validated||s.mode!=Mode::Manual||m.takeoff_z_m<=s.pose.p.z)) return false;
  if(s.battery&&fresh(now,s.battery_time_s,3.0)&&(!std::isfinite(*s.battery)||*s.battery<=c_.battery_return)) return false;
  if(m.global_detour_enabled) {
    if(!m.space||!m.space->boundary_provided) return false;
    try {m.planner.validate();} catch(const std::invalid_argument&) {return false;}
  }
  if(m.space) {
    try { validate_space(*m.space); } catch(const std::invalid_argument&) { return false; }
    if(!std::isfinite(m.clearance_xy_m)||m.clearance_xy_m<=0||!std::isfinite(m.clearance_z_m)||m.clearance_z_m<=0
        ||*space_segment_error(*m.space,s.pose.p,{s.pose.p.x,s.pose.p.y,m.takeoff_z_m},m.clearance_xy_m,m.clearance_z_m,true)) return false;
  }
  for(const auto& p:m.points) {
    if(!finite(p.p)||p.p.z<=0||(p.yaw&&!std::isfinite(*p.yaw))
        ||!std::isfinite(p.hover_s)||p.hover_s<0) return false;
    if(p.scan) {
      if(m.space&&p.scan->space!=m.space) return false;
      if(norm(p.scan->staging.p-p.p)>1e-6||!p.yaw||std::abs(wrap(*p.yaw-p.scan->staging.yaw))>1e-6) return false;
      try { ScanAction check(p.scan->config,c_,*p.scan,m.execution_id,now,0); }
      catch(const std::invalid_argument&) { return false; }
    }
  }
  mission_=m; state_=s; now_=last_tick_=now; home_=hold_=s.pose; frame_=s.frame_epoch;
  acquire_start_=now; started_=true; history_.push_back({{home_.p.x,home_.p.y,m.takeoff_z_m},home_.yaw});
  enter(Phase::Acquire); return true;
}
bool Runtime::command(Command c) {
  if(!started_||manual_lock_||phase_==Phase::Complete||phase_==Phase::Aborted) return false;
  if(c==Command::Land) { land_command_=true; return true; }
  if(c==Command::Cancel) { cancel_=true; return true; }
  if(return_lock_||land_lock_||phase_==Phase::Acquire||phase_==Phase::Handoff
      ||(c_.takeoff_policy==TakeoffPolicy::NativePx4&&phase_==Phase::Takeoff)) return false;
  if(c==Command::Pause) { paused_=true; return true; }
  if(c==Command::Resume&&paused_) { resume_=true; return true; }
  return false;
}
void Runtime::control_fault(const std::string& why,bool released) {
  if(why.empty()) return;
  control_fault_=why;control_released_=released;
}
void Runtime::enter(Phase p) {
  phase_=p; phase_start_active_=active_s_; segment_.reset(); stable_since_.reset();
  motion_initialized_=false; hold_=state_.pose; generation_++;
  navigation_active_=false; navigation_path_.clear();map_blocked_=false;
  navigation_deadline_active_.reset();
  return_vertical_goal_.reset();
  if(p==Phase::Takeoff) native_takeoff_since_=now_;
  if(p==Phase::Handoff) {handoff_since_=now_;prestream_since_.reset();last_prestream_.reset();}
  events_.push_back(std::to_string(now_)+" "+phase_name(p));
}
void Runtime::suspend_motion() {
  segment_.reset(); stable_since_.reset();
}
void Runtime::request_land(const std::string& why) {
  if(!land_lock_) {
    if(scan_) scan_->preempt(why);
    scan_decision_.reset();
    if(why!="RETURN_COMPLETE"||reason_.empty()) reason_=why;
    events_.push_back(std::to_string(now_)+" LAND_REASON "+why);
    land_lock_=true; enter(Phase::Landing);
  }
}
void Runtime::request_return(const std::string& why) {
  if(return_lock_||land_lock_) return;
  if(scan_) scan_->preempt(why);
  scan_decision_.reset();
  reason_=why; return_lock_=true; paused_=false;
  blocked_since_.reset();map_blocked_=false;
  // Reverse approved segment anchors; include the origin of a partly flown segment.
  return_path_.assign(history_.rbegin(),history_.rend());
  return_index_=0;
  enter(Phase::Return);
}
void Runtime::update_safety() {
  if(state_.rc_override||state_.px4_failsafe) { manual_lock_=true; reason_="RC_OR_PX4_TAKEOVER"; }
  if(manual_lock_) return;
  if(phase_==Phase::Complete||phase_==Phase::Aborted) return;
  if(!control_fault_.empty()) {
    if(control_released_) {manual_lock_=true;reason_=control_fault_;return;}
    request_land(control_fault_);return;
  }
  if(state_.mode==Mode::Land) { request_land("PX4_LAND"); return; }
  const bool native=c_.takeoff_policy==TakeoffPolicy::NativePx4;
  const bool native_phase=native&&(phase_==Phase::Takeoff||phase_==Phase::Handoff);
  const bool native_mode=native_phase&&(state_.mode==Mode::Takeoff||state_.mode==Mode::Hold);
  if(phase_!=Phase::Acquire&&phase_!=Phase::Idle&&state_.mode!=Mode::Offboard&&!native_mode&&!land_lock_) {
    manual_lock_=true; reason_="UNEXPECTED_MODE_EXIT"; return;
  }
  if(land_command_) request_land("WEB_LAND");
  if(state_.frame_epoch!=frame_||!yaw_valid(state_,now_)) request_land("YAW_OR_FRAME_INVALID");
  if(state_.battery_sensor_failed) request_land("PX4_BATTERY_FAILED");
  if(!fresh(now_,state_.mode_time_s,1.0)||!state_.connected) request_land("PX4_STATE_LOST");
  if(land_lock_) return;
  if(phase_==Phase::Acquire) {
    if(cancel_||!state_.logging_ok||!state_.observation_valid||!pose_valid(state_,now_,c_)) {
      if(!state_.landed) request_land("PREPARATION_ABORT_AIRBORNE");
      else {reason_="PREPARATION_ABORT"; enter(Phase::Aborted);}
    }
    return;
  }
  if(cancel_&&(phase_==Phase::Takeoff||phase_==Phase::Handoff)) { request_land("CANCEL_TAKEOFF"); return; }
  if(!pose_valid(state_,now_,c_)||!state_.can_hold) { request_land("PX4_CANNOT_HOLD"); return; }
  if(native_phase&&(!state_.observation_valid||!state_.route_revalidated)) {
    request_land("NATIVE_TAKEOFF_POSITION_UNAVAILABLE");return;
  }
  // Recovery refers to loss of external observations while PX4 can still hold.
  if(!state_.observation_valid&&!recovering_) {
    recovering_=true; recovery_since_=now_; recovery_stable_.reset(); hold_=state_.pose;
    recovery_interrupted_=true; suspend_motion();
  }
  if(recovering_) {
    if(now_-*recovery_since_>=c_.recovery_s) { request_land("POSITION_TIMEOUT"); return; }
    if(state_.observation_valid&&state_.route_revalidated) {
      if(!recovery_stable_) recovery_stable_=now_;
      if(now_-*recovery_stable_>=c_.recovery_stable_s) { recovering_=false; suspend_motion(); }
    } else recovery_stable_.reset();
  }
  const bool low=state_.battery&&std::isfinite(*state_.battery)
      &&fresh(now_,state_.battery_time_s,3)&&*state_.battery>=0&&*state_.battery<=c_.battery_return;
  if(low) {if(native_phase) request_land("BATTERY_LOW");else request_return("BATTERY_LOW");}
  if(!state_.logging_ok||!fresh(now_,state_.log_time_s,c_.log_age_s)) {
    if(native_phase) request_land("LOGGING_FAILED");else request_return("LOGGING_FAILED");
  }
  if(cancel_) request_return("CANCELLED");
  if(return_lock_&&!state_.return_route_valid&&!recovering_) { request_land("RETURN_UNAVAILABLE"); return; }
  if(!recovering_&&!return_lock_&&state_.blocked) {
    if(!blocked_since_) { blocked_since_=now_; hold_=state_.pose; suspend_motion(); }
  }
  if(blocked_since_) {
    if(now_-*blocked_since_>=c_.blocked_s) {
      request_return("BLOCKED_TIMEOUT"); blocked_since_.reset();
    } else if(!map_blocked_&&!state_.blocked&&state_.route_revalidated) { blocked_since_.reset(); suspend_motion(); }
  }
  if(resume_&&!recovering_&&state_.route_revalidated) { paused_=false; resume_=false; suspend_motion(); }
}
bool Runtime::choose(const std::string& name) const {
  if(name=="Yield") return manual_lock_;
  if(name=="Land") return land_lock_;
  if(name=="Recover") return recovering_;
  if(name=="Blocked") return blocked_since_.has_value()&&!return_lock_;
  if(name=="Pause") return paused_&&!return_lock_;
  return true;
}
void Runtime::halt(const std::string& name) {
  // ReactiveFallback may halt the old node after the new node ran this tick.
  // Never revoke the NEW node's generation in that case.
  if(branch_==name) { generation_++; output_.reset(); branch_.clear(); }
}
void Runtime::emit(IntentKind kind,Pose target) {
  output_=Intent{1,mission_.execution_id,boot_,generation_,++sequence_,frame_,
      mission_.transform_revision,kind,target,now_,now_+c_.lease_s};
}
bool Runtime::stable(Pose target,double seconds) {
  if(!at_pose(state_,target,c_)) { stable_since_.reset(); return false; }
  if(!stable_since_) stable_since_=now_;
  return now_-*stable_since_>=seconds;
}
void Runtime::block_route(Pose target,const std::string& code) {
  navigation_active_=false;navigation_path_.clear();suspend_motion();motion_initialized_=false;
  if(return_lock_) {request_land("RETURN_"+code);return;}
  map_blocked_=true;blocked_target_=target;
  if(!blocked_since_) {blocked_since_=now_;events_.push_back(std::to_string(now_)+" ROUTE_BLOCKED "+code);}
  plan_retry_s_=now_+0.5;hold_=state_.pose;
}
void Runtime::retry_route() {
  if(!map_blocked_||state_.blocked||!state_.route_revalidated||!stopped(state_,c_)||now_<plan_retry_s_) return;
  // Use the same original target. A successful retry does not visit a task;
  // a failed retry must not extend the original ten-second wait budget.
  plan_retry_s_=now_+0.5;
  const Vec3 start{state_.pose.p.x,state_.pose.p.y,blocked_target_.p.z};
  if(std::abs(state_.pose.p.z-start.z)>c_.z_error_m
      ||*space_segment_error(*mission_.space,state_.pose.p,start,mission_.clearance_xy_m,mission_.clearance_z_m)) return;
  const auto route=plan_horizontal_route(*mission_.space,start,blocked_target_.p,
    mission_.clearance_xy_m,mission_.clearance_z_m,mission_.planner);
  if(!route.ok()) return;
  map_blocked_=false;blocked_since_.reset();suspend_motion();motion_initialized_=false;
  navigation_path_=route.points;navigation_index_=1;navigation_active_=true;
  ++route_revision_;events_.push_back(std::to_string(now_)+" ROUTE_PLANNED "+std::to_string(route_revision_));
}
bool Runtime::move_route(Pose target,double speed,bool require_yaw,bool face_path) {
  if(!mission_.global_detour_enabled) return move_to(target,speed,require_yaw);
  if(navigation_deadline_active_&&active_s_>=*navigation_deadline_active_) {
    events_.push_back(std::to_string(now_)+" ROUTE_DEADLINE");
    if(return_lock_) request_land("GOAL_NOT_REACHED");else request_return("GOAL_NOT_REACHED");
    emit(IntentKind::Hold,state_.pose);return false;
  }
  if(!state_.route_revalidated||(!stopped(state_,c_)&&!navigation_active_)) {
    emit(IntentKind::Hold,hold_);
    if(!state_.route_revalidated) {
      if(return_lock_) request_land("ROUTE_NOT_VALIDATED");else request_return("ROUTE_NOT_VALIDATED");
    } else if(active_s_-phase_start_active_>c_.stall_s) {
      if(return_lock_) request_land("STOP_TIMEOUT");else request_return("STOP_TIMEOUT");
    }
    return false;
  }
  if(navigation_active_&&*space_segment_error(*mission_.space,state_.pose.p,navigation_path_.at(navigation_index_),
       mission_.clearance_xy_m,mission_.clearance_z_m)) {
    navigation_active_=false;navigation_path_.clear();suspend_motion();motion_initialized_=false;
    hold_=state_.pose;phase_start_active_=active_s_;emit(IntentKind::Hold,hold_);return false;
  }
  if(!navigation_active_) {
    const Vec3 start{state_.pose.p.x,state_.pose.p.y,target.p.z};
    if(std::abs(state_.pose.p.z-start.z)>c_.z_error_m
        ||*space_segment_error(*mission_.space,state_.pose.p,start,mission_.clearance_xy_m,mission_.clearance_z_m)) {
      if(return_lock_) request_land("RETURN_ALTITUDE_UNAVAILABLE");else request_return("ALTITUDE_UNAVAILABLE");
      emit(IntentKind::Hold,state_.pose);return false;
    }
    const auto route=plan_horizontal_route(*mission_.space,start,target.p,
      mission_.clearance_xy_m,mission_.clearance_z_m,mission_.planner);
    if(!route.ok()) {block_route(target,route.code);emit(IntentKind::Hold,state_.pose);return false;}
    navigation_path_=route.points;navigation_index_=1;navigation_active_=true;
    ++route_revision_;events_.push_back(std::to_string(now_)+" ROUTE_PLANNED "+std::to_string(route_revision_));
  }
  if(!navigation_deadline_active_) {
    double duration=0;Pose previous=state_.pose;
    for(std::size_t i=1;i<navigation_path_.size();++i) {
      const auto p=navigation_path_[i];const auto d=p-previous.p;
      const auto yaw=i+1==navigation_path_.size()&&!face_path?target.yaw:(xy(d)>1e-6?std::atan2(d.y,d.x):previous.yaw);
      const Pose next{p,yaw};duration+=Segment(previous,next,speed,c_).duration()+c_.stable_s;previous=next;
    }
    navigation_deadline_active_=active_s_+3*duration+5;
  }
  const auto p=navigation_path_.at(navigation_index_);
  const auto d=p-state_.pose.p;
  const bool final=navigation_index_+1==navigation_path_.size();
  const double yaw=final&&!face_path?target.yaw:(segment_?goal_.yaw:(xy(d)>1e-6?std::atan2(d.y,d.x):state_.pose.yaw));
  if(!move_to({p,yaw},speed,final?require_yaw:true)) return false;
  if(final) {navigation_active_=false;navigation_path_.clear();return true;}
  history_.push_back(goal_);++navigation_index_;suspend_motion();motion_initialized_=false;phase_start_active_=active_s_;
  return false;
}
bool Runtime::move_to(Pose target,double speed,bool require_yaw,const Config* trajectory_config) {
  if(!state_.route_revalidated) {
    if(return_lock_||phase_==Phase::Takeoff) request_land("ROUTE_NOT_VALIDATED");
    else request_return("ROUTE_NOT_VALIDATED");
    emit(IntentKind::Hold,state_.pose);return false;
  }
  if(mission_.space) {
    const bool micro=phase_==Phase::Scan&&scan_;
    const auto h=micro?scan_->plan().clearance_xy_m:mission_.clearance_xy_m;
    const auto v=micro?scan_->plan().clearance_z_m:mission_.clearance_z_m;
    const auto error=space_segment_error(*mission_.space,state_.pose.p,target.p,h,v,phase_==Phase::Takeoff);
    if(*error) {
      const auto reason=std::string("STATIC_MAP_")+error;
      if(return_lock_||phase_==Phase::Takeoff) request_land(reason);else request_return(reason);
      emit(IntentKind::Hold,state_.pose);return false;
    }
  }
  if(!segment_) {
    // Resume from rest; do not issue a new ramp from a moving vehicle.
    if(!stopped(state_,c_)) {
      emit(IntentKind::Hold,hold_);
      if(active_s_-phase_start_active_>c_.stall_s) {
        if(return_lock_||phase_==Phase::Takeoff) request_land("STOP_TIMEOUT");
        else request_return("STOP_TIMEOUT");
      }
      return false;
    }
    segment_.emplace(state_.pose,target,speed,trajectory_config?*trajectory_config:c_); goal_=target;
    stable_since_.reset();
    if(!motion_initialized_) {
      timeout_s_=phase_==Phase::Takeoff ? 20 : 3*segment_->duration()+5;
      // Explicit arrival rotations keep their angular progress metric even
      // when the actual position retains a small permitted tracking residual.
      motion_tracks_yaw_=require_yaw&&(phase_==Phase::Rotate||phase_==Phase::StartYaw||phase_==Phase::HomeYaw
        ||(xy(state_.pose.p-target.p)<=c_.xy_error_m&&std::abs(state_.pose.p.z-target.p.z)<=c_.z_error_m));
      best_error_=motion_tracks_yaw_?std::abs(wrap(state_.pose.yaw-target.yaw)):norm(state_.pose.p-target.p);
      progress_active_=active_s_; motion_initialized_=true;
    }
    // A resumed reference has its own time origin, but the phase timeout is not reset.
    segment_start_active_=active_s_;
  }
  const double elapsed=active_s_-segment_start_active_;
  emit(IntentKind::Position,segment_->sample(elapsed));
  max_overshoot_=std::max(max_overshoot_,segment_->overshoot(state_.pose.p));
  Pose arrival=target;
  if(!require_yaw) arrival.yaw=state_.pose.yaw;
  if(elapsed>=segment_->duration()&&stable(arrival,c_.stable_s)) return true;
  // Track one physical unit at a time. Rotation uses radians, translation metres.
  const bool rotation=motion_tracks_yaw_;
  const double error=rotation?std::abs(wrap(state_.pose.yaw-target.yaw)):norm(state_.pose.p-target.p);
  const double improvement=rotation?2*pi/180:0.05;
  if(best_error_-error>=improvement) { best_error_=error; progress_active_=active_s_; }
  const bool within=xy(state_.pose.p-target.p)<=c_.xy_error_m
      &&std::abs(state_.pose.p.z-target.p.z)<=c_.z_error_m
      &&(!require_yaw||std::abs(wrap(state_.pose.yaw-target.yaw))<=c_.yaw_error_rad);
  if(active_s_-phase_start_active_>timeout_s_||(!within&&active_s_-progress_active_>=c_.stall_s)) {
    events_.push_back(std::to_string(now_)+" GOAL_FAILURE "+phase_name(phase_)+
      (active_s_-phase_start_active_>timeout_s_?" DEADLINE":" STALL"));
    if(phase_==Phase::Takeoff||return_lock_) request_land("GOAL_NOT_REACHED");
    else request_return("GOAL_NOT_REACHED");
    emit(IntentKind::Hold,state_.pose);
  }
  return false;
}
void Runtime::next_waypoint() {
  if(visited_>=mission_.points.size()) { request_return("MISSION_FINISHED"); return; }
  hold_=state_.pose; enter(Phase::Vertical);
}
void Runtime::move_mission() {
  const bool native=c_.takeoff_policy==TakeoffPolicy::NativePx4;
  const Pose takeoff_target{{home_.p.x,home_.p.y,mission_.takeoff_z_m},home_.yaw};
  if(phase_==Phase::Acquire) {
    if(now_-acquire_start_>c_.prestream_s+2*c_.transition_s) {
      reason_="ACQUIRE_TIMEOUT"; enter(Phase::Aborted); emit(IntentKind::AbortGround,home_); return;
    }
    if(native) {
      if(state_.mode!=Mode::Takeoff) {emit(IntentKind::Takeoff,takeoff_target);return;}
      if(!state_.armed) {emit(IntentKind::Arm,home_);return;}
      enter(Phase::Takeoff);
    } else {
    if(!prestream_since_||!last_prestream_||now_-*last_prestream_>0.100001) prestream_since_=now_;
    if(now_-*prestream_since_<c_.prestream_s) {
      last_prestream_=now_; emit(IntentKind::Prestream,home_); return;
    }
    if(state_.mode!=Mode::Offboard) { last_prestream_=now_; emit(IntentKind::Offboard,home_); return; }
    if(!state_.armed) { last_prestream_=now_; emit(IntentKind::Arm,home_); return; }
    enter(Phase::Takeoff);
    }
  }
  if(phase_==Phase::Takeoff) {
    if(native) {
      emit(IntentKind::Takeoff,takeoff_target);
      const auto& complete=state_.takeoff_completion;
      if(complete&&complete->request.execution_id==mission_.execution_id&&complete->request.boot_session==boot_
          &&complete->request.operation==ControlOperation::Takeoff&&complete->request.frame_epoch==frame_
          &&complete->request.transform_revision==mission_.transform_revision
          &&complete->observed_s>=native_takeoff_since_&&fresh(now_,complete->observed_s,1)) native_completed_=true;
      if(now_-native_takeoff_since_>=c_.native_takeoff_timeout_s) request_land("NATIVE_TAKEOFF_TIMEOUT");
      else if(native_completed_&&stable(takeoff_target,c_.stable_s)) enter(Phase::Handoff);
      return;
    }
    if(move_to({{home_.p.x,home_.p.y,mission_.takeoff_z_m},home_.yaw},c_.z_speed_mps))
      enter(mission_.start_yaw?Phase::StartYaw:Phase::Vertical);
    return;
  }
  if(phase_==Phase::Handoff) {
    if(now_-handoff_since_>=c_.prestream_s+c_.transition_s) {request_land("OFFBOARD_HANDOFF_TIMEOUT");return;}
    if(state_.mode==Mode::Offboard) {
      enter(mission_.start_yaw?Phase::StartYaw:Phase::Vertical);return;
    }
    if(!prestream_since_||!last_prestream_||now_-*last_prestream_>0.100001) prestream_since_=now_;
    last_prestream_=now_;
    emit(now_-*prestream_since_<c_.prestream_s?IntentKind::Prestream:IntentKind::Offboard,hold_);
    return;
  }
  if(phase_==Phase::StartYaw) {
    if(move_to({history_.back().p,*mission_.start_yaw},c_.xy_speed_mps)) next_waypoint();
    return;
  }
  if(phase_==Phase::Vertical) {
    const auto& wp=mission_.points.at(visited_);
    if(!motion_initialized_) hold_=state_.pose;
    if(move_to({{hold_.p.x,hold_.p.y,wp.p.z},hold_.yaw},c_.z_speed_mps)) {
      history_.push_back(goal_); enter(Phase::Travel);
    }
    return;
  }
  if(phase_==Phase::Travel) {
    const auto& wp=mission_.points.at(visited_);
    const auto d=wp.p-history_.back().p;
    const double yaw=xy(d)>1e-6?std::atan2(d.y,d.x):history_.back().yaw;
    if(move_route({wp.p,yaw},mission_.requested_speed_mps,true,true)) {
      history_.push_back(goal_); enter(wp.yaw?Phase::Rotate:Phase::Dwell);
    }
    return;
  }
  if(phase_==Phase::Rotate) {
    const auto& wp=mission_.points.at(visited_);
    if(move_to({wp.p,*wp.yaw},c_.xy_speed_mps)) enter(Phase::Dwell);
    return;
  }
  if(phase_==Phase::Dwell) {
    const auto& wp=mission_.points.at(visited_);
    const Pose target={wp.p,wp.yaw.value_or(history_.back().yaw)};
    emit(IntentKind::Hold,target);
    if(stable(target,c_.stable_s+wp.hover_s)) {
      if(wp.scan) {
        scan_=std::make_unique<ScanAction>(wp.scan->config,c_,*wp.scan,mission_.execution_id,now_,active_s_);
        scan_movement_complete_=false; enter(Phase::Scan);
      } else { visited_++; next_waypoint(); }
    }
    else if(active_s_-phase_start_active_>wp.hover_s+10) request_return("HOVER_NOT_STABLE");
    return;
  }
  if(phase_==Phase::Scan) {
    const auto before=scan_->stage();
    const auto d=scan_->tick(state_,now_,active_s_,scan_movement_complete_);
    scan_decision_=d;
    scan_movement_complete_=false;
    if(d.flight_failed) { request_return("SCAN_FLIGHT_OR_SPACE_INVALID"); return; }
    if(d.record_failed) { request_return("SCAN_RECORD_FAILED"); return; }
    if(d.complete) {
      scan_results_.push_back(*scan_->result());
      history_.push_back(scan_->plan().staging); scan_.reset(); scan_decision_.reset(); visited_++; next_waypoint(); return;
    }
    if(before!=scan_->stage()) { segment_.reset(); motion_initialized_=false; stable_since_.reset(); phase_start_active_=active_s_; hold_=state_.pose; }
    if(d.move) {
      // Vertical correction precedes horizontal correction at the current safe XY.
      Pose goal=d.target;
      if(std::abs(state_.pose.p.z-goal.p.z)>scan_->config().position_error_m)
        goal={{state_.pose.p.x,state_.pose.p.y,goal.p.z},state_.pose.yaw};
      if(segment_&&(norm(goal.p-goal_.p)>0.005||std::abs(wrap(goal.yaw-goal_.yaw))>pi/180)) {
        segment_.reset(); stable_since_.reset(); motion_initialized_=false;
      }
      auto dynamics=c_;
      dynamics.accel_mps2=scan_->config().acceleration_mps2;
      dynamics.yaw_rate_rps=scan_->config().yaw_rate_rps;
      dynamics.yaw_accel_rps2=scan_->config().yaw_acceleration_rps2;
      if(move_to(goal,scan_->config().speed_mps,true,&dynamics)&&norm(goal.p-d.target.p)<1e-6&&std::abs(wrap(goal.yaw-d.target.yaw))<1e-6)
        scan_movement_complete_=true;
    } else emit(IntentKind::Hold,d.target);
    return;
  }
  if(phase_==Phase::Return) {
    if(return_index_>=return_path_.size()) { enter(Phase::HomeYaw); return; }
    const auto target=return_path_[return_index_];
    const auto speed=std::min(c_.z_speed_mps,mission_.requested_speed_mps);
    if(!return_vertical_goal_&&std::abs(state_.pose.p.z-target.p.z)>c_.z_error_m)
      return_vertical_goal_=Pose{{state_.pose.p.x,state_.pose.p.y,target.p.z},state_.pose.yaw};
    if(return_vertical_goal_) {
      if(move_to(*return_vertical_goal_,c_.z_speed_mps)) {
        suspend_motion();motion_initialized_=false;phase_start_active_=active_s_;
        return_vertical_goal_.reset();
      }
      return;
    }
    if(move_route(target,speed)) {
      return_index_++; enter(Phase::Return);
    }
    return;
  }
  if(phase_==Phase::HomeYaw) {
    if(move_to({{home_.p.x,home_.p.y,mission_.takeoff_z_m},home_.yaw},c_.z_speed_mps))
      request_land("RETURN_COMPLETE");
  }
}
void Runtime::run(const std::string& name) {
  if(branch_!=name) {
    generation_++; branch_=name;
    if(name=="Pause"||name=="Blocked") hold_=state_.pose;
    if(name!="Execute") suspend_motion();
  }
  if(name=="Yield") { if(scan_) scan_->preempt("RC_OR_PX4_TAKEOVER"); scan_decision_.reset(); phase_=Phase::Manual; emit(IntentKind::Release,{}); return; }
  if(name=="Land") {
    if(state_.landed&&!state_.armed) { phase_=Phase::Complete; emit(IntentKind::Release,{}); }
    else emit(IntentKind::Land,{});
    return;
  }
  if(name=="Recover"||name=="Pause"||name=="Blocked") {
    if(scan_) scan_->suspend();
    scan_decision_.reset();
    if(name=="Blocked") retry_route();
    emit(IntentKind::Hold,hold_);return;
  }
  if(phase_==Phase::Aborted) { emit(IntentKind::AbortGround,home_); return; }
  if(phase_==Phase::Complete||phase_==Phase::Idle) return;
  active_s_+=now_-last_tick_;
  if(recovery_interrupted_) { recovery_interrupted_=false; hold_=state_.pose; }
  move_mission();
}
std::optional<Intent> Runtime::tick(const State& s,double now) {
  output_.reset();
  if(!started_) return {};
  if(!std::isfinite(now)||now<last_tick_) throw std::invalid_argument("Clock moved backwards");
  now_=now; state_=s;
  update_safety();
  bt_->tree.tickExactlyOnce();
  last_tick_=now;
  return output_;
}
} // namespace sangwon
