#include "btf/freeze.hpp"
#include <cmath>
#include <stdexcept>

namespace btf {
FreezePolicy::FreezePolicy(Time max_age):max_age_(max_age) {
  if(max_age<=0)throw std::invalid_argument("positive_observation_age_required");
}
void FreezePolicy::reset() {
  accepted_at_.reset();accepted_source_.reset();last_xy_.reset();frozen_target_.reset();last_streamed_target_.reset();
  current_valid_=false;frozen_=false;last_now_=-1;cause_="observation_not_received";
}
void FreezePolicy::observe(const Decision& d,Time source_host_us,Time now_us) {
  if(now_us<0||now_us<last_now_){
    current_valid_=false;accepted_at_.reset();
    if(!frozen_){frozen_=true;frozen_target_=last_streamed_target_;}
    last_now_=now_us;cause_="host_clock_reset";return;
  }
  last_now_=now_us;
  if(!d.ok||!d.xy) {current_valid_=false;cause_=d.reason;return;}
  if(!std::isfinite((*d.xy)[0])||!std::isfinite((*d.xy)[1])||d.source_stamp_us<=0||source_host_us<0||source_host_us>now_us||
     now_us-source_host_us>max_age_||(accepted_source_&&d.source_stamp_us<=*accepted_source_)||
     (accepted_at_&&source_host_us<=*accepted_at_)) {
    current_valid_=false;cause_="stale_or_repeated_observation";return;
  }
  current_valid_=true;accepted_source_=d.source_stamp_us;
  accepted_at_=source_host_us;last_xy_=d.xy;cause_="fresh_qualified_observation";
}
FreezeDecision FreezePolicy::tick(Time now_us,FcHealth fc,std::optional<XY> target) {
  if(now_us<0||now_us<last_now_){
    current_valid_=false;accepted_at_.reset();
    if(!frozen_){frozen_=true;frozen_target_=last_streamed_target_;}
    cause_="host_clock_reset";
  }
  last_now_=now_us;
  if(target&&(!std::isfinite((*target)[0])||!std::isfinite((*target)[1])))target.reset();
  FreezeDecision out;out.last_observation_xy=last_xy_;out.last_observation_monotonic_us=accepted_at_;
  const bool fresh=current_valid_&&accepted_at_&&now_us>=*accepted_at_&&now_us-*accepted_at_<=max_age_;
  if(!fresh&&!frozen_) {frozen_=true;frozen_target_=last_streamed_target_?last_streamed_target_:target;}
  // A valid measurement cannot release a freeze while FC health is unknown.
  if(fc!=FcHealth::Healthy) {
    if(!frozen_){frozen_=true;frozen_target_=last_streamed_target_?last_streamed_target_:target;}
    out.state=FreezeState::FcUnavailable;out.target_xy=frozen_target_;
    out.defer_to_fc_failsafe=fc==FcHealth::Invalid;
    // A delayed FC report is not an explicit estimator failure. Continue the
    // approved target stream while the executor resolves health, with progress paused.
    out.stream_target=fc==FcHealth::Unknown&&frozen_target_.has_value();
    out.reason=fc==FcHealth::Invalid?"fc_estimator_invalid":"fc_health_unknown";return out;
  }
  if(!fresh) {
    out.state=frozen_target_?FreezeState::Frozen:FreezeState::Waiting;
    out.target_xy=frozen_target_;out.stream_target=frozen_target_.has_value();
    out.reason=current_valid_?"observation_timeout":cause_;return out;
  }
  if(frozen_) {
    // One transition tick retains the latched target. The planner resumes from
    // this setpoint on the next tick; rejected/unapproved targets never leak.
    out.target_xy=frozen_target_;out.stream_target=frozen_target_.has_value();
    out.state=FreezeState::Frozen;out.reason="observation_recovered_resume_next_tick";
    frozen_=false;frozen_target_.reset();return out;
  }
  out.state=target?FreezeState::Running:FreezeState::Waiting;
  out.target_xy=target;out.stream_target=target.has_value();out.pause_mission=!target.has_value();
  if(target)last_streamed_target_=target;
  out.reason=target?"fresh_qualified_observation":"approved_target_required";return out;
}
const char* state_name(FreezeState s) {
  switch(s){case FreezeState::Waiting:return "WAITING";case FreezeState::Running:return "RUNNING";
    case FreezeState::Frozen:return "FREEZE";case FreezeState::FcUnavailable:return "FC_UNAVAILABLE";}
  return "WAITING";
}
}  // namespace btf
