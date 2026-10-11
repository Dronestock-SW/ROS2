#include "sangwon_ai/scan.hpp"
#include "sangwon_ai/geometry.hpp"
#include <behaviortree_cpp/contrib/json.hpp>
#include <set>

namespace sangwon {
void ScanConfig::validate() const {
  for(double value:{marker_age_s,qr_age_s,marker_acquire_s,align_s,settle_s,reader_s,camera_s,
      task_s,persist_s,speed_mps,position_error_m,yaw_error_rad,hover_speed_mps,max_adjustment_m,max_adjustment_yaw,reprojection_px,
      acceleration_mps2,yaw_rate_rps,yaw_acceleration_rps2})
    if(!std::isfinite(value)||value<=0) throw std::invalid_argument("INVALID_SCAN_LIMITS");
  if(!continuous_marker_samples || task_s<=settle_s || speed_mps>0.1)
    throw std::invalid_argument("INVALID_SCAN_LIMITS");
}
const char* scan_stage_name(ScanStage s) {
  switch(s) {
    case ScanStage::Approach:return "APPROACH"; case ScanStage::Acquire:return "ACQUIRE_MARKER";
    case ScanStage::Align:return "ALIGN"; case ScanStage::Settle:return "SETTLE";
    case ScanStage::Reader:return "READER_SCAN"; case ScanStage::Camera:return "CAMERA_SCAN";
    case ScanStage::Persist:return "PERSIST_RESULT"; case ScanStage::Egress:return "RETURN_STAGING";
    case ScanStage::Complete:return "NEXT"; case ScanStage::Preempted:return "PREEMPTED";
  }
  return "UNKNOWN";
}
ScanAction::ScanAction(ScanConfig c,Config flight,ScanPlan plan,std::string execution,double now,double active)
  :c_(c),flight_(flight),plan_(std::move(plan)),execution_(std::move(execution)),
   start_active_(active),phase_active_(active),last_active_(active),last_now_(now),target_(plan_.reader) {
  c_.validate(); flight_.validate();
  if(plan_.workspace.has_value()!=static_cast<bool>(plan_.space)) throw std::invalid_argument("SCAN_PLAN_NOT_VALIDATED");
  if(plan_.space) {
    validate_space(*plan_.space);validate_volume(*plan_.workspace);
    if(!std::isfinite(plan_.clearance_xy_m)||plan_.clearance_xy_m<=0
        ||!std::isfinite(plan_.clearance_z_m)||plan_.clearance_z_m<=0)
      throw std::invalid_argument("SCAN_PLAN_NOT_VALIDATED");
  }
  if(execution_.empty()||plan_.task_id.empty()||plan_.label_id.empty()||!plan_.space_validated
      ||plan_.dictionary.empty()||plan_.marker_id<0||!std::isfinite(plan_.marker_size_m)||plan_.marker_size_m<=0
      ||plan_.calibration_ref.empty()||plan_.mounting_ref.empty()||plan_.layout_ref.empty()
      ||!finite(plan_.centre_min)||!finite(plan_.centre_max)||plan_.centre_min.x>=plan_.centre_max.x
      ||plan_.centre_min.y>=plan_.centre_max.y||plan_.centre_min.z>=plan_.centre_max.z
      ||!inside(plan_.staging)||!inside(plan_.reader)||!inside(plan_.camera)||plan_.expected_qr_fields.empty()
      ||!path_clear(plan_.staging,plan_.reader)||!path_clear(plan_.reader,plan_.staging)
      ||!path_clear(plan_.staging,plan_.camera)||!path_clear(plan_.camera,plan_.staging)
      ||!std::isfinite(now)||!std::isfinite(active)) throw std::invalid_argument("SCAN_PLAN_NOT_VALIDATED");
}
bool ScanAction::inside(Pose p) const {
  return finite(p)&&segment_clear(p.p,p.p);
}
bool ScanAction::segment_clear(Vec3 a,Vec3 b) const {
  for(const auto p:{a,b}) if(!finite(p)||p.x<plan_.centre_min.x||p.x>plan_.centre_max.x
      ||p.y<plan_.centre_min.y||p.y>plan_.centre_max.y||p.z<plan_.centre_min.z||p.z>plan_.centre_max.z) return false;
  if(!plan_.space) return true; // Explicit map-free synthetic action fixture only.
  return volume_covers(*plan_.workspace,a,b,plan_.clearance_xy_m,plan_.clearance_z_m)
      &&!*space_segment_error(*plan_.space,a,b,plan_.clearance_xy_m,plan_.clearance_z_m);
}
bool ScanAction::path_clear(Pose a,Pose b) const {
  const Vec3 corner{a.p.x,a.p.y,b.p.z};
  return finite(a)&&finite(b)&&segment_clear(a.p,corner)&&segment_clear(corner,b.p);
}
void ScanAction::enter(ScanStage s,double active) {
  stage_=s; phase_active_=active; stable_active_.reset(); window_open_=false;
}
bool ScanAction::marker_valid(const State& state,double now) {
  const auto& input=state.scan.marker;
  if(input) {
    const auto& m=*input;
    if(!m.valid) { marker_.reset();marker_samples_=0;stable_active_.reset();return false; }
    // Observation sequence belongs to the image producer. Receiving the same
    // cached image never creates another stability sample or newer timestamp.
    const bool newer=m.producer_id!=marker_producer_ || m.sequence>marker_sequence_;
    if(newer) {
      const bool producer_changed=!marker_producer_.empty()&&marker_producer_!=m.producer_id;
      marker_producer_=m.producer_id; marker_sequence_=m.sequence;
      if(producer_changed) { marker_samples_=0; stable_active_.reset(); }
      const bool valid=m.valid&&!m.producer_id.empty()&&m.dictionary==plan_.dictionary&&m.marker_id==plan_.marker_id
        &&m.calibration_ref==plan_.calibration_ref&&m.mounting_ref==plan_.mounting_ref&&m.layout_ref==plan_.layout_ref
        &&m.frame_epoch==state.frame_epoch&&std::abs(m.marker_size_m-plan_.marker_size_m)<=1e-6
        &&std::isfinite(m.reprojection_px)&&m.reprojection_px>=0&&m.reprojection_px<=c_.reprojection_px
        &&fresh(now,m.observed_s,c_.marker_age_s)&&inside(m.desired_vehicle_pose_map)
        &&norm(m.desired_vehicle_pose_map.p-plan_.reader.p)<=c_.max_adjustment_m
        &&std::abs(wrap(m.desired_vehicle_pose_map.yaw-plan_.reader.yaw))<=c_.max_adjustment_yaw;
      if(valid) { marker_=m; marker_samples_++; }
      else { marker_.reset(); marker_samples_=0; stable_active_.reset(); }
    }
  }
  if(!state.scan.camera_healthy||!marker_||!fresh(now,marker_->observed_s,c_.marker_age_s)) {
    marker_samples_=0; stable_active_.reset(); return false;
  }
  return true;
}
bool ScanAction::aligned(const State& state,double now) {
  if(!marker_valid(state,now)||marker_samples_<c_.continuous_marker_samples) return false;
  const auto d=state.pose.p-target_.p;
  return xy(d)<=c_.position_error_m&&std::abs(d.z)<=c_.position_error_m
    &&std::abs(wrap(state.pose.yaw-target_.yaw))<=c_.yaw_error_rad
    &&xy(state.velocity)<=c_.hover_speed_mps&&std::abs(state.velocity.z)<=c_.hover_speed_mps
    &&norm(marker_target().p-state.pose.p)<=c_.position_error_m
    &&std::abs(wrap(marker_target().yaw-state.pose.yaw))<=c_.yaw_error_rad;
}
Pose ScanAction::marker_target() const {
  auto target=marker_->desired_vehicle_pose_map;
  if(camera_mode_) {
    const auto delta=plan_.camera.p-plan_.reader.p;
    const auto yaw=wrap(target.yaw-plan_.reader.yaw);
    target.p=target.p+Vec3{std::cos(yaw)*delta.x-std::sin(yaw)*delta.y,
      std::sin(yaw)*delta.x+std::cos(yaw)*delta.y,delta.z};
    target.yaw=wrap(target.yaw+plan_.camera.yaw-plan_.reader.yaw);
  }
  return target;
}
bool ScanAction::qr_valid(const QrObservation& q,double now) const {
  if(!window_open_||q.execution_id!=execution_||q.task_id!=plan_.task_id||q.window_id!=window_
      ||q.producer_id.empty()||q.raw.empty()||q.raw.size()>65536||q.observed_s<window_open_s_
      ||!fresh(now,q.observed_s,c_.qr_age_s)||(stage_==ScanStage::Reader&&q.source!=QrSource::Scanner)
      ||(stage_==ScanStage::Camera&&q.source!=QrSource::Camera)) return false;
  try {
    using Json=nlohmann::json;
    std::vector<std::set<std::string>> keys;
    const auto value=Json::parse(q.raw,[&keys](int depth,Json::parse_event_t event,Json& v) {
      if(depth>16) throw std::invalid_argument("QR_NESTING_LIMIT");
      if(event==Json::parse_event_t::object_start) keys.emplace_back();
      if(event==Json::parse_event_t::key&&!keys.back().insert(v.get<std::string>()).second)
        throw std::invalid_argument("DUPLICATE_QR_FIELD");
      if(event==Json::parse_event_t::object_end) keys.pop_back();
      return true;
    });
    if(!value.is_object()) return false;
    for(const auto& [key,want]:plan_.expected_qr_fields)
      if(!value.contains(key)||!value.at(key).is_string()||value.at(key).get<std::string>()!=want) return false;
    return true;
  } catch(const std::exception&) { return false; }
}
void ScanAction::open(QrSource source,double now,double active) {
  window_=execution_+"/"+plan_.task_id+"/"+std::to_string(++window_sequence_);
  window_open_=true; window_open_s_=now; window_active_=active;
  if(source==QrSource::Scanner) attempts_++; else camera_used_=true;
}
void ScanAction::finish(bool succeeded,const std::string& code,double active,const QrObservation* qr) {
  ScanResult result; result.id=execution_+"/"+plan_.task_id+"/result";
  result.task_id=plan_.task_id; result.label_id=plan_.label_id; result.code=code;
  result.scanner_attempts=attempts_; result.camera_used=camera_used_; result.succeeded=succeeded;
  if(qr) { result.raw=qr->raw; result.source=qr->source; result.decoded_s=qr->observed_s; }
  result_=std::move(result); enter(ScanStage::Persist,active);
}
void ScanAction::suspend() {
  if(suspended_||stage_==ScanStage::Complete||stage_==ScanStage::Preempted) return;
  suspended_=true; stable_active_.reset(); marker_samples_=0;
  if(stage_==ScanStage::Reader||stage_==ScanStage::Camera) {
    if(stage_==ScanStage::Camera) camera_budget_used_+=last_active_-window_active_;
    window_open_=false;
    enter(ScanStage::Settle,last_active_);
  }
}
void ScanAction::preempt(const std::string& reason) {
  if(stage_==ScanStage::Complete||stage_==ScanStage::Preempted) return;
  if(!result_) finish(false,reason,last_active_);
  result_->preempted=true;
  enter(ScanStage::Preempted,last_active_);
}
ScanDecision ScanAction::tick(const State& state,double now,double active,bool movement_complete) {
  if(!std::isfinite(now)||!std::isfinite(active)||now<last_now_||active<last_active_)
    throw std::invalid_argument("SCAN_CLOCK_REVERSED");
  last_now_=now; last_active_=active; suspended_=false;
  ScanDecision d; d.target=state.pose;
  if(stage_==ScanStage::Preempted) return d;
  if(stage_==ScanStage::Complete) { d.complete=true; return d; }
  if(state.rc_override||state.px4_failsafe||!pose_valid(state,now,flight_)||!yaw_valid(state,now)
      ||!fresh(now,state.mode_time_s,1.0)||!state.can_hold
      ||!state.connected||!state.armed||state.landed||state.mode!=Mode::Offboard||!inside(state.pose)) {
    preempt("SCAN_FLIGHT_OR_SPACE_INVALID"); d.flight_failed=true; return d;
  }
  if(stage_!=ScanStage::Persist&&stage_!=ScanStage::Egress&&active-start_active_>=c_.task_s)
    finish(false,"SCAN_TASK_TIMEOUT",active);
  if(!state.scan.camera_healthy&&stage_!=ScanStage::Persist&&stage_!=ScanStage::Egress)
    finish(false,"CAMERA_UNAVAILABLE",active);
  if(stage_==ScanStage::Approach) {
    const auto target=camera_mode_?plan_.camera:plan_.reader;
    if(!path_clear(state.pose,target)) finish(false,"SCAN_APPROACH_OBSTRUCTED",active);
    else {
      d.target=target;d.move=true;
      if(movement_complete) enter(ScanStage::Acquire,active);
      return d;
    }
  }
  if(stage_==ScanStage::Acquire||stage_==ScanStage::Align) {
    const bool valid=marker_valid(state,now);
    if(active-phase_active_>=(stage_==ScanStage::Acquire?c_.marker_acquire_s:c_.align_s)) {
      finish(false,stage_==ScanStage::Acquire?"MARKER_UNAVAILABLE":"ALIGNMENT_TIMEOUT",active);
    } else if(valid) {
      if(stage_==ScanStage::Acquire) enter(ScanStage::Align,active);
      target_=marker_target();
      if(!inside(target_)||!path_clear(state.pose,target_)) { finish(false,"ALIGNMENT_OUT_OF_BOUNDS",active); }
      else {
      d.target=target_; d.move=true;
      if(movement_complete && aligned(state,now)) enter(ScanStage::Settle,active);
      return d;
      }
    } else return d;
  }
  if(stage_==ScanStage::Settle) {
    d.target=target_;
    if(!aligned(state,now)) {
      stable_active_.reset();
      if(active-phase_active_>=c_.align_s) finish(false,"ALIGNMENT_TIMEOUT",active);
      else if(marker_&&fresh(now,marker_->observed_s,c_.marker_age_s)) enter(ScanStage::Align,active);
      return d;
    }
    if(!stable_active_) stable_active_=active;
    if(active-*stable_active_<c_.settle_s) return d;
    if(attempts_<3) { enter(ScanStage::Reader,active); open(QrSource::Scanner,now,active); }
    else if(!camera_mode_) { camera_mode_=true; enter(ScanStage::Approach,active); return d; }
    else if(camera_budget_used_<c_.camera_s) { enter(ScanStage::Camera,active); open(QrSource::Camera,now,active); }
    else { finish(false,"QR_UNREADABLE",active); }
  }
  if(stage_==ScanStage::Reader||stage_==ScanStage::Camera) {
    d.target=target_; d.window_id=window_;
    const bool camera=stage_==ScanStage::Camera;
    if(!aligned(state,now)) { suspend(); return d; }
    if(active-window_active_ >= (camera?c_.camera_s-camera_budget_used_:c_.reader_s)) {
      if(camera) finish(false,"QR_UNREADABLE",active); else enter(ScanStage::Settle,active);
    } else {
      d.open_reader=!camera&&state.scan.scanner_healthy;
      d.enable_camera=camera;
      const auto& q=state.scan.qr;
      const bool enabled=!camera||(state.scan.camera_decoder_enabled&&state.scan.camera_activation_id==window_);
      if(enabled&&q&&(q->producer_id!=qr_producer_||q->sequence>qr_sequence_)) {
        qr_producer_=q->producer_id; qr_sequence_=q->sequence;
        if(qr_valid(*q,now)) finish(true,"OK",active,&*q);
      }
      if(stage_==ScanStage::Reader||stage_==ScanStage::Camera) return d;
    }
  }
  if(stage_==ScanStage::Persist) {
    d.persist=true;
    if(state.scan.stored_result_id==result_->id) enter(ScanStage::Egress,active);
    else if(active-phase_active_>=c_.persist_s) d.record_failed=true;
    if(stage_==ScanStage::Persist) return d;
  }
  if(stage_==ScanStage::Egress) {
    if(!path_clear(state.pose,plan_.staging)) {
      preempt("SCAN_EGRESS_OBSTRUCTED");d.flight_failed=true;return d;
    }
    d.target=plan_.staging; d.move=true;
    if(movement_complete) { result_->egress_complete=true; enter(ScanStage::Complete,active); d.complete=true; }
  }
  return d;
}
} // namespace sangwon
