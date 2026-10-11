#include "sangwon_ai/service/engine.hpp"

namespace sangwon::service {
namespace {
Json null_string(const std::string& s) { return s.empty()?Json(nullptr):Json(s); }
Pose pose(const Json& p) {
  const auto& xyz=p.at("position_m");
  Pose v{{xyz.at("x").get<double>(),xyz.at("y").get<double>(),xyz.at("z").get<double>()},p.at("yaw_rad").get<double>()};
  require(finite(v),"INVALID_SCAN_POSE");return v;
}
Json pose_json(Pose p) { return {{"position_m",{{"x",p.p.x},{"y",p.p.y},{"z",p.p.z}}},{"yaw_rad",p.yaw}}; }
}
bool Engine::external_sensors() const {
  return replay_&&config_.value("replay_scan_source",std::string())=="EXTERNAL_SENSOR_REPLAY";
}
std::string Engine::scan_source() const { return config_.value("replay_scan_source",std::string()); }
Json Engine::scan_request(bool renew) {
  const auto now=steady_seconds();
  const auto* action=runtime_?runtime_->scan_action():nullptr;
  const auto& decision=runtime_?runtime_->scan_decision():std::optional<ScanDecision>{};
  const bool running=external_sensors()&&active_&&has_scan_&&!recovery_locked_&&runtime_
    &&!runtime_->paused()&&!runtime_->recovering();
  const bool scanning=running&&runtime_->phase()==Phase::Scan&&action
    &&action->stage()!=ScanStage::Persist&&action->stage()!=ScanStage::Egress
    &&action->stage()!=ScanStage::Complete&&action->stage()!=ScanStage::Preempted;
  const bool status=running&&(runtime_->phase()==Phase::Acquire||runtime_->phase()==Phase::Takeoff
    ||runtime_->phase()==Phase::StartYaw||runtime_->phase()==Phase::Vertical||runtime_->phase()==Phase::Travel
    ||runtime_->phase()==Phase::Rotate||runtime_->phase()==Phase::Dwell||scanning);
  const bool reader=scanning&&decision&&decision->open_reader&&action->stage()==ScanStage::Reader;
  const bool camera=scanning&&decision&&decision->enable_camera&&action->stage()==ScanStage::Camera;
  Json next={{"schema_version","sangwon-scan-request/2"},{"scope","SYNTHETIC_SENSOR_REPLAY"},
    {"flight_authority",false},{"physical_output_enabled",false},{"active",status},
    {"context",{{"drone_id",config_.at("drone_id")},{"boot_id",boot_},{"runtime_session_id",runtime_id_},
      {"execution_id",null_string(execution_)},{"task_id",scanning?Json(action->plan().task_id):Json(nullptr)},
      {"window_id",(reader||camera)?Json(decision->window_id):Json(nullptr)},{"frame_epoch",vehicle_.state.frame_epoch}}},
    {"stage",scanning?Json(scan_stage_name(action->stage())):Json(wire_phase())},
    {"observe_marker",scanning},{"open_reader",reader},{"enable_camera",camera},
    {"producer_id",external_sensors()?config_.at("approved_replay_sensor_producer"):Json(nullptr)}};
  if(scanning) {
    const auto& plan=action->plan();
    next["marker_contract"]={{"dictionary",plan.dictionary},{"marker_id",plan.marker_id},{"marker_size_m",plan.marker_size_m},
      {"calibration_ref",plan.calibration_ref},{"mounting_ref",plan.mounting_ref},{"layout_ref",plan.layout_ref},
      {"nominal_reader_pose",pose_json(plan.reader)}};
  } else next["marker_contract"]=nullptr;
  const auto limits=scanning?action->config():ScanConfig{};
  next["source_max_age_s"]={{"STATUS",0.5},{"MARKER",limits.marker_age_s},{"CAMERA_ACK",0.2},
    {"QR_SCANNER",limits.qr_age_s},{"QR_CAMERA",limits.qr_age_s}};
  // Changing task, phase, window, pause or runtime retires the previous token.
  if(sensor_request_.is_null()||sensor_request_.at("definition")!=next) {
    sensor_request_={{"definition",next},{"request_id",random_id("sensor-request")},
      {"issued_monotonic_s",now},{"expires_monotonic_s",now}};
    vehicle_.state.scan.qr.reset();vehicle_.state.scan.camera_decoder_enabled=false;
    vehicle_.state.scan.camera_activation_id.clear();sensor_ack_time_=-1e30;
  }
  if(renew) sensor_request_["expires_monotonic_s"]=now+0.2;
  next.update({{"request_id",sensor_request_.at("request_id")},
    {"issued_monotonic_s",sensor_request_.at("issued_monotonic_s")},
    {"expires_monotonic_s",sensor_request_.at("expires_monotonic_s")}});
  return next;
}
Json Engine::scan_observe(const Json& packet) {
  require(external_sensors(),"SCAN_INPUT_NOT_ENABLED");
  require(packet.at("schema_version")=="sangwon-scan-observation/2"&&packet.at("scope")=="SYNTHETIC_SENSOR_REPLAY"
    &&packet.at("flight_authority")==false,"SCAN_INPUT_SCOPE_MISMATCH");
  require(packet.dump().size()<=128*1024,"SCAN_INPUT_TOO_LARGE");
  const auto id=packet.at("observation_id").get<std::string>();
  require(!id.empty()&&id.size()<=128,"SCAN_OBSERVATION_ID_INVALID");
  const auto fingerprint=digest(packet.dump());
  const auto prior=sensor_receipts_.find(id);
  if(prior!=sensor_receipts_.end()) {
    require(prior->second.at("fingerprint")==fingerprint,"SCAN_OBSERVATION_ID_CONFLICT");
    // A lost response can be recovered after its window retires. This returns
    // the original ingress receipt without touching health, samples or action.
    auto receipt=prior->second.at("receipt");receipt["duplicate"]=true;return receipt;
  }
  const auto request=scan_request(false);const double now=steady_seconds();
  require(request.at("active")==true,"SCAN_INPUT_INACTIVE");
  require(packet.at("request_id")==request.at("request_id")&&packet.at("context")==request.at("context"),"STALE_SCAN_REQUEST");
  require(now<=request.at("expires_monotonic_s").get<double>(),"SCAN_REQUEST_EXPIRED");
  const auto producer=packet.at("producer_id").get<std::string>();
  require(producer==config_.at("approved_replay_sensor_producer"),"SCAN_PRODUCER_MISMATCH");
  const auto stamp=packet.at("observed_monotonic_s").get<double>();
  require(std::isfinite(stamp)&&stamp>=request.at("issued_monotonic_s").get<double>()&&stamp<=now,"SCAN_SOURCE_TIME_INVALID");
  const auto observed_utc=utc_parse(packet.at("observed_at").get<std::string>());
  require(std::abs(utc_seconds()-observed_utc-(now-stamp))<=0.05,"SCAN_SOURCE_CLOCK_MISMATCH");
  const auto kind=packet.at("kind").get<std::string>();const auto& data=packet.at("data");
  require(kind=="STATUS"||kind=="MARKER"||kind=="CAMERA_ACK"||kind=="QR_SCANNER"||kind=="QR_CAMERA","INVALID_SCAN_INPUT_KIND");
  require(packet.at("sequence").is_number_unsigned()&&packet.at("sequence").get<std::uint64_t>()>0,"SCAN_SEQUENCE_INVALID");
  const auto sequence=packet.at("sequence").get<std::uint64_t>();
  require(sequence>sensor_sequences_[kind],"SCAN_OBSERVATION_OUT_OF_ORDER");
  auto& input=vehicle_.state.scan;
  if(kind=="STATUS") {
    require(now-stamp<=0.5,"SCAN_OBSERVATION_STALE");
    const bool camera=data.at("camera_healthy").get<bool>(),scanner=data.at("scanner_healthy").get<bool>();
    input.camera_healthy=camera;input.scanner_healthy=scanner;
    sensor_camera_time_=sensor_scanner_time_=stamp;
  } else {
    require(runtime_->phase()==Phase::Scan&&runtime_->scan_action(),"SCAN_INPUT_INACTIVE");
    const auto& c=runtime_->scan_action()->config();const auto& plan=runtime_->scan_action()->plan();
    if(kind=="MARKER") {
      require(request.at("observe_marker")==true&&now-stamp<=c.marker_age_s,"SCAN_OBSERVATION_STALE");
      MarkerObservation marker{producer,data.at("dictionary"),data.at("calibration_ref"),data.at("mounting_ref"),data.at("layout_ref"),
        sequence,packet.at("context").at("frame_epoch"),data.at("marker_id"),simulation_time_+(stamp-last_tick_),
        data.at("marker_size_m"),data.at("reprojection_px"),pose(data.at("desired_vehicle_pose_map")),data.at("valid")};
      require(marker.dictionary==plan.dictionary&&marker.marker_id==plan.marker_id
        &&marker.calibration_ref==plan.calibration_ref&&marker.mounting_ref==plan.mounting_ref&&marker.layout_ref==plan.layout_ref
        &&std::isfinite(marker.marker_size_m)&&std::abs(marker.marker_size_m-plan.marker_size_m)<1e-6
        &&std::isfinite(marker.reprojection_px)&&marker.reprojection_px>=0&&marker.reprojection_px<=c.reprojection_px,"SCAN_MARKER_CONTRACT_MISMATCH");
      input.marker=std::move(marker);sensor_marker_time_=stamp;
    } else if(kind=="CAMERA_ACK") {
      require(request.at("enable_camera")==true&&data.at("enabled")==true,"CAMERA_ACTIVATION_MISMATCH");
      require(now-stamp<=0.2,"SCAN_OBSERVATION_STALE");
      input.camera_decoder_enabled=true;input.camera_activation_id=request.at("context").at("window_id");sensor_ack_time_=stamp;
    } else {
      const bool camera=kind=="QR_CAMERA";
      require(request.at(camera?"enable_camera":"open_reader")==true,"QR_SOURCE_NOT_REQUESTED");
      require(now-stamp<=c.qr_age_s,"SCAN_OBSERVATION_STALE");
      require(!camera||(input.camera_decoder_enabled&&input.camera_activation_id==request.at("context").at("window_id")
        &&now-sensor_ack_time_<=0.2),"CAMERA_ACTIVATION_MISMATCH");
      const auto raw=data.at("raw").get<std::string>();require(!raw.empty()&&raw.size()<=65536,"INVALID_QR_DATA");
      input.qr=QrObservation{producer+":"+kind,execution_,plan.task_id,request.at("context").at("window_id"),raw,sequence,
        simulation_time_+(stamp-last_tick_),camera?QrSource::Camera:QrSource::Scanner};sensor_qr_time_=stamp;sensor_qr_utc_=observed_utc;
    }
  }
  // Invalid packets cannot consume the sequence or replace a previous observation.
  sensor_sequences_[kind]=sequence;
  Json receipt={{"schema_version","sangwon-scan-receipt/1"},{"scope","SYNTHETIC_SENSOR_REPLAY"},
    {"flight_authority",false},{"acceptance_scope","INPUT_VALIDATION_ONLY"},{"accepted",true},{"duplicate",false},
    {"observation_id",id},{"request_id",packet.at("request_id")},{"context",packet.at("context")},
    {"producer_id",producer},{"kind",kind},{"sequence",sequence},{"accepted_monotonic_s",now},
    {"source_observed_monotonic_s",stamp},{"source_observed_at",packet.at("observed_at")}};
  sensor_receipts_[id]={{"fingerprint",fingerprint},{"receipt",receipt}};sensor_receipt_order_.push_back(id);
  if(sensor_receipt_order_.size()>64) {sensor_receipts_.erase(sensor_receipt_order_.front());sensor_receipt_order_.pop_front();}
  return receipt;
}
void Engine::external_scan_input(double now) {
  auto& input=vehicle_.state.scan;
  if(now-sensor_camera_time_>0.5) input.camera_healthy=false;
  if(now-sensor_scanner_time_>0.5) input.scanner_healthy=false;
  if(now-sensor_ack_time_>0.2) { input.camera_decoder_enabled=false;input.camera_activation_id.clear(); }
  if(runtime_&&runtime_->scan_action()) {
    const auto& c=runtime_->scan_action()->config();
    if(input.marker&&now-sensor_marker_time_>c.marker_age_s) input.marker->valid=false;
    if(now-sensor_qr_time_>c.qr_age_s) input.qr.reset();
  }
}
} // namespace sangwon::service
