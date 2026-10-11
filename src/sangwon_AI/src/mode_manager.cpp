#include "sangwon_ai/mode_manager.hpp"

namespace sangwon {
const char* mode_name(Mode m) {
  switch(m) {
    case Mode::Manual:return "MANUAL";case Mode::Offboard:return "OFFBOARD";
    case Mode::Land:return "AUTO.LAND";case Mode::Takeoff:return "AUTO.TAKEOFF";
    case Mode::Hold:return "AUTO.LOITER";case Mode::Unknown:return "UNKNOWN";
  }
  return "UNKNOWN";
}
const char* control_operation_name(ControlOperation op) {
  switch(op) {
    case ControlOperation::None:return "NONE";case ControlOperation::Offboard:return "OFFBOARD";
    case ControlOperation::Takeoff:return "TAKEOFF";case ControlOperation::Land:return "LAND";
    case ControlOperation::Arm:return "ARM";case ControlOperation::Disarm:return "GROUND_DISARM";
  }
  return "UNKNOWN";
}
const char* transition_state_name(TransitionState s) {
  switch(s) {
    case TransitionState::Idle:return "IDLE";case TransitionState::AwaitingState:return "AWAITING_STATE";
    case TransitionState::Confirmed:return "CONFIRMED";case TransitionState::Rejected:return "REJECTED";
    case TransitionState::TimedOut:return "TIMED_OUT";case TransitionState::Cancelled:return "CANCELLED";
  }
  return "UNKNOWN";
}
namespace {
Mode target_mode(ControlOperation op,Mode from) {
  switch(op) {
    case ControlOperation::Takeoff:return Mode::Takeoff;
    case ControlOperation::Offboard:return Mode::Offboard;
    case ControlOperation::Land:return Mode::Land;
    default:return from;
  }
}
bool matches(const ControlRequest& a,const ControlRequest& b) {
  return a.operation==b.operation&&a.execution_id==b.execution_id&&a.boot_session==b.boot_session
    &&a.request_id==b.request_id&&a.frame_epoch==b.frame_epoch&&a.transform_revision==b.transform_revision
    &&a.issued_s==b.issued_s&&a.deadline_s==b.deadline_s
    &&a.target.p.x==b.target.p.x&&a.target.p.y==b.target.p.y&&a.target.p.z==b.target.p.z&&a.target.yaw==b.target.yaw;
}
bool confirms(ControlOperation op,const State& s) {
  switch(op) {
    case ControlOperation::Takeoff:return s.mode==Mode::Takeoff;
    case ControlOperation::Offboard:return s.mode==Mode::Offboard;
    case ControlOperation::Land:return s.mode==Mode::Land;
    case ControlOperation::Arm:return s.armed;
    case ControlOperation::Disarm:return s.landed&&!s.armed;
    default:return false;
  }
}
OutputKind output_kind(ControlOperation op) {
  switch(op) {
    case ControlOperation::Takeoff:return OutputKind::RequestTakeoff;
    case ControlOperation::Offboard:return OutputKind::RequestOffboard;
    case ControlOperation::Land:return OutputKind::RequestLand;
    case ControlOperation::Arm:return OutputKind::RequestArm;
    case ControlOperation::Disarm:return OutputKind::GroundDisarm;
    default:return OutputKind::None;
  }
}
}
ModeManager::ModeManager(Config c,std::string execution,std::string boot,std::uint64_t frame,std::uint64_t revision)
 :c_(c),execution_(std::move(execution)),boot_(std::move(boot)),frame_(frame),revision_(revision) {
  c_.validate();
  if(execution_.empty()||boot_.empty()||!frame_||!revision_) throw std::invalid_argument("Missing mode identity");
}
bool ModeManager::begin(ControlOperation op,Pose target,const State& s,double now) {
  switch(op) {
    case ControlOperation::Takeoff:case ControlOperation::Offboard:case ControlOperation::Land:
    case ControlOperation::Arm:case ControlOperation::Disarm:break;
    default:return false;
  }
  if(pending()) return request_->operation==op;
  if(op==ControlOperation::None||!finite(target)||!s.connected||!fresh(now,s.mode_time_s,1)) return false;
  if((op==ControlOperation::Arm&&(!s.landed||s.armed))
      ||(op==ControlOperation::Disarm&&(!s.landed||!s.armed))
      ||(op==ControlOperation::Takeoff&&(!s.landed||s.armed||!s.native_takeoff_target_validated))) return false;
  from_=s.mode;expected_=from_;observed_=s.mode;owned_=true;unexpected_=false;
  request_=ControlRequest{op,execution_,boot_,++next_id_,frame_,revision_,now,now+c_.transition_s,target};
  state_=TransitionState::AwaitingState;attempts_=0;ack_accepted_=false;
  if(op==ControlOperation::Takeoff) {takeoff_request_=request_;takeoff_completed_=false;}
  record(now);
  return true;
}
void ModeManager::record(double now) {
  if(!request_) return;
  // The service drains each tick. Standalone fixtures retain at most 32 events.
  if(events_.size()==32) events_.erase(events_.begin());
  events_.push_back({*request_,state_,expected_,observed_,attempts_,ack_accepted_,takeoff_completed_,now});
}
std::vector<ModeTransitionEvent> ModeManager::drain_events() {
  auto out=std::move(events_);events_.clear();return out;
}
void ModeManager::cancel(double now) {if(pending()||failed()) {state_=TransitionState::Cancelled;record(now);}}
void ModeManager::adopt_offboard() {if(!owned_) {owned_=true;expected_=Mode::Offboard;}}
void ModeManager::observe(const State& s,double now) {
  if(!s.connected||!fresh(now,s.mode_time_s,1)||s.frame_epoch!=frame_) return;
  observed_=s.mode;
  const auto before=state_;const bool ack_before=ack_accepted_,complete_before=takeoff_completed_;
  const auto expected_before=expected_;
  if(pending()) {
    // A response received at/after the deadline cannot renew this transaction.
    if(now>=request_->deadline_s) state_=TransitionState::TimedOut;
    else {
      const auto& reply=s.control_reply;
      if(reply&&matches(reply->request,*request_)&&reply->observed_s>=request_->issued_s
          &&fresh(now,reply->observed_s,1)) {
        if(reply->result==ReplyResult::Rejected) state_=TransitionState::Rejected;
        else ack_accepted_=true;
      }
      if(pending()&&attempts_>0&&s.mode_time_s>=request_->issued_s&&confirms(request_->operation,s)) {
        expected_=target_mode(request_->operation,from_);state_=TransitionState::Confirmed;
      }
    }
  }
  if(takeoff_request_&&expected_==Mode::Takeoff&&s.takeoff_completion
      &&matches(s.takeoff_completion->request,*takeoff_request_)
      &&s.takeoff_completion->observed_s>=takeoff_request_->issued_s
      &&fresh(now,s.takeoff_completion->observed_s,1)) takeoff_completed_=true;
  // Only correlated automatic TAKEOFF completion may explain AUTO.LOITER.
  // Merely seeing HOLD is never permission to reclaim a pilot's control.
  if(owned_&&expected_==Mode::Takeoff&&s.mode==Mode::Hold&&takeoff_completed_
      &&s.global_position_quality==Quality::Valid&&fresh(now,s.global_position_time_s,1)) expected_=Mode::Hold;
  if(owned_&&s.mode!=expected_&&(!pending()||s.mode!=target_mode(request_->operation,from_))) unexpected_=true;
  if(before!=state_||ack_before!=ack_accepted_||complete_before!=takeoff_completed_||expected_before!=expected_) record(now);
}
Output ModeManager::output(double now,Pose target,bool stream) {
  Output out{OutputKind::None,target,"AWAITING_PX4_STATE"};out.stream_position=stream;
  if(!pending()||now>=request_->deadline_s) return out;
  if(attempts_>=c_.control_max_attempts||(attempts_&&now-last_send_s_<c_.control_retry_s)) return out;
  ++attempts_;last_send_s_=now;out.kind=output_kind(request_->operation);out.request=request_;
  record(now);
  out.reason=control_operation_name(request_->operation);
  if(request_->operation==ControlOperation::Takeoff) out.target=request_->target;
  return out;
}
} // namespace sangwon
