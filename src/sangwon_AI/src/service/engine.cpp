#include "sangwon_ai/service/engine.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace sangwon::service {
double steady_seconds() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double utc_seconds() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
std::string utc_text(double seconds) {
  const auto whole = static_cast<std::time_t>(seconds);
  std::tm value{}; gmtime_r(&whole, &value);
  std::ostringstream out; out << std::put_time(&value, "%Y-%m-%dT%H:%M:%S") << '.'
    << std::setw(3) << std::setfill('0') << int((seconds-double(whole))*1000) << 'Z';
  return out.str();
}
std::string random_id(const std::string& prefix) {
  std::ifstream in("/proc/sys/kernel/random/uuid"); std::string id; in >> id;
  require(!id.empty(), "RANDOM_ID_UNAVAILABLE"); return prefix + '-' + id;
}
double utc_parse(std::string text) {
  if (text.size() >= 6 && text.substr(text.size()-6) == "+00:00") text.replace(text.size()-6,6,"Z");
  require(text.size() >= 20 && text.back() == 'Z', "INVALID_UTC");
  std::tm value{}; std::istringstream in(text.substr(0,19)); in >> std::get_time(&value,"%Y-%m-%dT%H:%M:%S");
  require(!in.fail(), "INVALID_UTC");
  const double base = double(timegm(&value));
  require(utc_text(base).substr(0,19) == text.substr(0,19), "INVALID_UTC");
  if (text.size() == 20) return base;
  require(text[19]=='.' && text.size() <= 30, "INVALID_UTC");
  const auto fraction = text.substr(20,text.size()-21);
  require(!fraction.empty() && fraction.find_first_not_of("0123456789")==std::string::npos,"INVALID_UTC");
  return base + std::stod("0."+fraction);
}
namespace {
Json nullable(const std::string& s) { return s.empty() ? Json(nullptr) : Json(s); }
Json vector_json(Vec3 p) { return {{"x",p.x},{"y",p.y},{"z",p.z}}; }
const std::vector<std::string> capabilities = {"mission_xyz","arrival_yaw","command_session","readiness_binding","replay_waypoints_v1",
  "map_volumes","scan_task_v1","label_pose_v1","scan_workspace_v1","replay_static_detour_v1","planner_plan_v1"};
}
Engine::Engine(Json config, const std::string& boot, const std::string& database)
  : config_(std::move(config)),ledger_(database),boot_(boot),runtime_id_(random_id("runtime")) {
  const auto mode=config_.at("profile").get<std::string>();
  require(mode=="HOST_OBSERVE" || mode=="REPLAY","PHYSICAL_OUTPUT_NOT_IMPLEMENTED");
  replay_=mode=="REPLAY";
  if(external_sensors()) {
    const auto producer=config_.at("approved_replay_sensor_producer").get<std::string>();
    require(!producer.empty()&&producer.size()<=128,"SCAN_PRODUCER_REQUIRED");
    require(config_.value("replay_steps_per_tick",1)==1,"EXTERNAL_SENSOR_REPLAY_REQUIRES_REALTIME");
  }
  require(config_.at("physical_output_enabled")==false,"PHYSICAL_OUTPUT_NOT_IMPLEMENTED");
  require(!config_.at("drone_id").get<std::string>().empty(),"DRONE_ID_REQUIRED");
  require(!replay_ || config_.at("drone_id").get<std::string>().rfind("TEST-",0)==0,"REPLAY_TEST_ID_REQUIRED");
  recovery_locked_=!ledger_.meta("active_execution").is_null();
  last_tick_=steady_seconds();
}
Json Engine::envelope(const std::string& type) const {
  return {{"contract_version",contract},{"type",type},{"drone_id",config_.at("drone_id")},
          {"profile",replay_?"REPLAY":"HOST_OBSERVE"}};
}
Json Engine::context() const {
  return {{"boot_id",boot_},{"runtime_session_id",runtime_id_},{"control_session_id",nullable(session_)},
    {"assignment_id",nullable(assignment_)},{"snapshot_id",nullable(snapshot_id_)},
    {"preparation_id",nullable(preparation_)},{"readiness_revision",readiness_revision_},
    {"execution_id",nullable(execution_)},{"flight_id",nullable(flight_)}};
}
bool Engine::link_fresh() const { return connected_ && steady_seconds()-link_time_ < 2.0; }
void Engine::invalidate_preparation() {
  if (!preparation_.empty()) {
    const auto previous=preparation_; preparation_.clear(); ++readiness_revision_;
    emit_event("READY_REVOKED",{{"previous_preparation_id",previous},{"readiness_revision",readiness_revision_}});
  }
}
std::string Engine::wire_phase() const {
  if(!active_ || !runtime_) return recovery_locked_?"RECOVERY_LOCK":(runtime_?"ENDED":"IDLE");
  if(runtime_->recovering()) return "RECOVERY_WAIT";
  if(runtime_->paused()) return "PAUSED";
  switch(runtime_->phase()) {
    case Phase::Acquire: case Phase::Handoff: return "PREPARING";
    case Phase::Takeoff: return "TAKING_OFF";
    case Phase::Vertical: case Phase::Travel: return "NAVIGATING";
    case Phase::Rotate: case Phase::StartYaw: return "ROTATING";
    case Phase::Dwell: return "HOVERING";
    case Phase::Return: case Phase::HomeYaw: return "RETURNING";
    case Phase::Landing: return "LANDING";
    case Phase::Manual: return "MANUAL";
    case Phase::Scan: return "SCANNING";
    case Phase::Complete: case Phase::Aborted: return "ENDED";
    default: return "IDLE";
  }
}
Json Engine::host_diagnostics() const {
  if(host_report_.is_null()) return {{"state","UNKNOWN"},{"checks",Json::array()}};
  const auto age=std::max(0.0,steady_seconds()-host_report_.at("generated_monotonic_s").get<double>());
  return {{"state",age<10?"LIVE":"STALE"},{"scope","HOST_DIAGNOSTICS_ONLY"},
    {"observed_at",host_report_.at("generated_at")},{"source_age_ms",int(age*1000)},
    {"max_age_ms",10000},{"monitor_session_id",host_report_.at("monitor_session_id")},
    {"monitor_seq",host_report_.at("monitor_seq")},
    {"checks",age<10?host_report_.at("checks"):Json::array()}};
}
Json Engine::readiness() const {
  Json r=envelope("readiness");
  const bool ready=replay_ && !active_ && !recovery_locked_ && link_fresh() && !preparation_.empty();
  const auto now=utc_text(utc_seconds());
  const auto report_sequence=++readiness_sequence_;
  const bool fixture=replay_ && !snapshot_id_.empty();
  r.update({{"generated_at",now},{"valid_until",utc_text(utc_seconds()+2)},
    {"boot_phase","RUNNING"},{"flight_authority",false},{"allowed_execution",replay_?"REPLAY_ONLY":"NONE"},
    {"can_start",ready},{"system_state",fixture&&link_fresh()&&!recovery_locked_?"READY":"NOT_READY"},
    {"mission_state",active_?"BUSY":(ready?"READY":(snapshot_id_.empty()?"NO_MISSION":"NOT_READY"))},
    {"readiness_revision",readiness_revision_},{"readiness_seq",report_sequence},{"context",context()},
    {"check_catalog_revision","BP-28-QS-05-draft4"},
    {"validation_scope",replay_?validation_scope_:"HOST_DIAGNOSTICS_ONLY"},
    {"host_diagnostics",host_diagnostics()}});
  // The service's five diagnostics are not substitutes for the BP/QS catalog.
  r["service_checks"]=Json::array({
    {{"check_id","SVC_CPP"},{"status","PASS"},{"blocking",false},{"code","OK"}},
    {{"check_id","SVC_WEB_CONTRACT"},{"status",link_fresh()?"PASS":"UNKNOWN"},{"blocking",!link_fresh()},{"code",link_fresh()?"OK":link_code_}},
    {{"check_id","SVC_PHYSICAL_OUTPUT"},{"status","UNKNOWN"},{"blocking",!replay_},{"code","PHYSICAL_OUTPUT_NOT_IMPLEMENTED"}},
    {{"check_id","SVC_MISSION"},{"status",!preparation_.empty()||active_?"PASS":"UNKNOWN"},{"blocking",preparation_.empty()&&!active_},{"code",preparation_code_}},
    {{"check_id","SVC_RESTART"},{"status",recovery_locked_?"FAIL":"PASS"},{"blocking",recovery_locked_},{"code",recovery_locked_?"RECOVERY_LOCK":"OK"}}
  });
  r["checks"]=Json::array();
  for(int i=1;i<=33;++i) {
    const bool scan=i>28;
    std::ostringstream id; id<<(scan?"QS-C":"BP-C")<<std::setw(2)<<std::setfill('0')<<(scan?i-28:i);
    const bool applicable=!(scan && fixture && !has_scan_);
    std::string status="UNKNOWN", code="CHECK_NOT_IMPLEMENTED";
    std::string reason="No current physical check evidence", source="UNAVAILABLE";
    // Explicit, hash-allowlisted synthetic environment only. Never used in HOST_OBSERVE.
    if(fixture) {
      status="PASS"; code="REPLAY_FIXTURE_ASSUMPTION";
      reason="Synthetic fixture environment only; no physical preflight evidence";
      source="APPROVED_REPLAY_FIXTURE";
    }
    if(!applicable) { status="UNKNOWN"; code="NOT_APPLICABLE"; reason="Waypoint-only fixture has no scan tasks"; }
    if(i==5) {
      status=recovery_locked_?"FAIL":"PASS"; code=recovery_locked_?"RECOVERY_LOCK":"OK";
      reason="SQLite command ledger loaded; prior execution recovery evaluated"; source="CPP_LEDGER";
    }
    if(i==20 || (i==28 && fixture)) {
      status=link_fresh() && !session_.empty()?"PASS":"UNKNOWN";
      code=status=="PASS"?"OK":"WEB_SESSION_UNAVAILABLE";
      reason="Current control session and fresh contract link required"; source="CPP_SESSION";
    }
    if(i==21 && !fixture) { code=preparation_code_; reason="No approved supported mission snapshot"; }
    const bool observed=status=="PASS" || status=="FAIL";
    r["checks"].push_back({{"check_id",id.str()},{"status",status},{"required",applicable},
      {"applicable",applicable},{"blocking",applicable && status!="PASS"},{"code",code},{"reason",reason},
      {"operator_action",observed?Json(nullptr):Json("Provide fresh evidence before flight readiness")},
      {"source",source},{"observed_at",observed?Json(now):Json(nullptr)},
      {"source_age_ms",observed?Json(0):Json(nullptr)},{"max_age_ms",2000},{"check_seq",report_sequence}});
  }
  r["allowed_commands"]=Json::array();
  if(ready) r["allowed_commands"].push_back("START");
  if(active_ && !recovery_locked_ && link_fresh()) {
    const auto phase=wire_phase();
    r["allowed_commands"].push_back("CANCEL");
    if(!flight_.empty()) r["allowed_commands"].push_back("LAND_NOW");
    if(phase!="PREPARING" && phase!="RETURNING" && phase!="LANDING" && phase!="MANUAL"
        &&!(motion_.takeoff_policy==TakeoffPolicy::NativePx4&&phase=="TAKING_OFF"))
      r["allowed_commands"].push_back(runtime_->paused()?"RESUME":"PAUSE");
  }
  return r;
}
Json Engine::telemetry() const {
  Json r=envelope("telemetry");
  const auto now=utc_text(utc_seconds());
  r.update({{"generated_at",now},{"telemetry_seq",sequence_},{"context",context()},
    {"phase",wire_phase()},{"flight_authority",false},{"control_owner",active_?"JETSON":"NONE"},
    {"manual_lock",recovery_locked_},{"source",replay_?"FAKE_PX4":"UNAVAILABLE"},
    {"px4",{{"connected",replay_},{"armed",replay_?Json(vehicle_.state.armed):Json(nullptr)},
            {"landed",replay_?Json(vehicle_.state.landed):Json(nullptr)}}},
    {"pose_fused",replay_?Json{{"frame","WAREHOUSE_MAP"},{"position_m",vector_json(vehicle_.state.pose.p)},
      {"yaw_deg",vehicle_.state.pose.yaw*180/pi},{"source","FAKE_PX4"},{"valid",true}}:Json(nullptr)},
    {"visited",runtime_?runtime_->visited():0},{"reason",runtime_?runtime_->reason():""},
    {"allowed_commands",readiness().at("allowed_commands")}});
  // LoRa is a ground-station liveness channel, never a coordinate authority.
  r["position_transport"]="JETSON_WIFI";
  r["validation_scope"]=replay_?validation_scope_:"HOST_DIAGNOSTICS_ONLY";
  r["scan"]=nullptr;
  if(replay_&&active_&&runtime_&&runtime_->scan_action()) {
    const auto* action=runtime_->scan_action();
    r["scan"]={{"task_id",action->plan().task_id},{"label_point_id",action->plan().label_id},
      {"stage",scan_stage_name(action->stage())},{"scanner_attempts_started",action->attempts()},
      {"source",scan_source()},{"window_id",nullable(action->window_id())}};
  }
  r["uwb_observation"]={{"source","UWB_ADAPTER_PENDING"},{"valid",false},{"is_new_observation",false},
    {"position_m",nullptr},{"observed_at",nullptr},{"source_age_ms",nullptr},{"reason_code","UWB_SPEC_PENDING"}};
  r["uwb_map_position"]={{"source","JETSON_UWB_MAP_TRANSFORM"},{"frame","WAREHOUSE_MAP"},
    {"valid",false},{"position_m",nullptr},{"observed_at",nullptr},{"source_age_ms",nullptr},
    {"transform_ref",nullptr},{"reason_code","UWB_SPEC_PENDING"}};
  if(replay_) {
    // One serialization instant for the current FakePx4 state; do not report
    // an observation newer than its own envelope. Real sensor time remains TBD.
    r["pose_fused"]["observed_at"]=now;
    r["pose_fused"]["source_age_ms"]=0;
  }
  return r;
}
Json Engine::mode_control() const {
  Json r={{"scope",replay_?"SYNTHETIC_CONTROL_ONLY":"UNAVAILABLE"},{"physical_output_enabled",false},
    {"hardware_transport_implemented",false},{"parameter_writer_implemented",false},
    {"takeoff_policy",motion_.takeoff_policy==TakeoffPolicy::NativePx4?"PX4_AUTO_TAKEOFF":"OFFBOARD"},
    {"observed_mode",replay_?Json(mode_name(vehicle_.state.mode)):Json(nullptr)},
    {"expected_mode",nullptr},{"state","IDLE"},{"request_id",nullptr},{"operation",nullptr},
    {"attempts",0},{"ack_accepted",false},{"takeoff_completed",false}};
  if(!guard_) return r;
  const auto& mode=guard_->modes();
  r.update({{"expected_mode",mode_name(mode.expected_mode())},{"state",transition_state_name(mode.state())},
    {"attempts",mode.attempts()},{"ack_accepted",mode.ack_accepted()},
    {"takeoff_completed",mode.takeoff_completed()},{"ownership_released",guard_->released()},
    {"landing_locked",guard_->landing_locked()},{"fault_reason",nullable(guard_->fault_reason())}});
  if(mode.request()) {
    const auto& request=*mode.request();
    r.update({{"request_id",request.request_id},{"operation",control_operation_name(request.operation)},
      {"issued_simulation_s",request.issued_s},{"deadline_simulation_s",request.deadline_s}});
  }
  return r;
}
Json Engine::status() const {
  Json r=envelope("service_status");
  r.update({{"ipc_version",1},{"context",context()},{"supported_capabilities",replay_?Json(capabilities):Json::array({"command_session","readiness_binding","planner_plan_v1"})},
    {"physical_output_enabled",false},{"web",{{"connected",connected_},{"fresh",link_fresh()},
      {"code",link_code_},{"observed_contract",server_version_}}},
    {"recovery_locked",recovery_locked_},{"readiness",readiness()},{"telemetry",telemetry()}});
  r["navigation"]={{"scope","SYNTHETIC_STATIC_MAP_ONLY"},{"global_detour_enabled",replay_&&mission_.global_detour_enabled},
    {"route_revision",runtime_?runtime_->route_revision():0},{"dynamic_obstacles_supported",false}};
  r["mode_control"]=mode_control();
  r["mission_plan"]=planner_plan_;
  r["navigation"]["recent_events"]=Json::array();
  if(runtime_) {
    const auto& events=runtime_->events();const auto first=events.size()>12?events.size()-12:0;
    for(std::size_t i=first;i<events.size();++i) r["navigation"]["recent_events"].push_back(events[i]);
  }
  return r;
}
void Engine::emit_event(const std::string& type, Json details) {
  Json e=envelope("event"); const auto id=runtime_id_+"-event-"+std::to_string(++event_sequence_);
  e.update({{"event_id",id},{"event_seq",event_sequence_},{"event_type",type},
    {"context",context()},{"generated_at",utc_text(utc_seconds())},{"details",std::move(details)}});
  ledger_.enqueue(id,"companion-phase",e);
}
Json Engine::result(const Json& command, const std::string& state, const std::string& code, int revision) const {
  Json r=envelope("command_result"); Json ctx=command.value("context",Json::object());
  if(state!="REJECTED") { ctx["execution_id"]=nullable(execution_); ctx["flight_id"]=nullable(flight_); }
  r.update({{"control_request_id",command.at("control_request_id")},{"status",state},{"code",code},
    {"result_revision",revision},{"context",ctx},{"recorded_at",utc_text(utc_seconds())},
    {"reason",code},{"operator_action",state=="REJECTED"?Json("Inspect current state and issue a new request if appropriate"):Json(nullptr)},
    {"blocking_check_ids",Json::array()},{"field_errors",Json::array()}});
  return r;
}
void Engine::save_result(const Json& command, const Json& response) {
  const auto id=command.at("control_request_id").get<std::string>();
  ledger_.put_command(id,command.dump(),response);
  ledger_.enqueue(id+":"+std::to_string(response.at("result_revision").get<int>()),"control-action/ack",response);
}
Json Engine::accept_command(const Json& command) {
  const auto id=command.at("control_request_id").get<std::string>();
  require(!id.empty() && id.size()<=128,"INVALID_REQUEST_ID");
  const auto prior=ledger_.command(id);
  if(!prior.is_null()) {
    if(prior.at("canonical")!=command.dump()) {
      emit_event("COMMAND_CONFLICT",{{"control_request_id",id},{"code","REQUEST_ID_CONFLICT"}});
      return result(command,"REJECTED","REQUEST_ID_CONFLICT",1);
    }
    return prior.at("result");
  }
  try {
    require(command.at("contract_version")==contract,"UNSUPPORTED_CONTRACT");
    require(command.at("drone_id")==config_.at("drone_id"),"CONTEXT_MISMATCH");
    require(replay_ && command.at("profile")=="REPLAY","PHYSICAL_OUTPUT_NOT_IMPLEMENTED");
    require(!recovery_locked_,"RECOVERY_LOCK");
    const auto& ctx=command.at("context");
    require(ctx.at("boot_id")==boot_ && ctx.at("runtime_session_id")==runtime_id_,"STALE_RUNTIME_SESSION");
    require(!session_.empty() && ctx.at("control_session_id")==session_,"STALE_CONTROL_SESSION");
    require(link_fresh(),"WEB_NOT_CONNECTED");
    const auto created=utc_parse(command.at("created_at").get<std::string>());
    const auto expires=utc_parse(command.at("expires_at").get<std::string>());
    const auto now=utc_seconds();
    require(std::abs(expires-created-10)<0.001,"INVALID_COMMAND_TTL");
    require(created<=now+0.25,"CLOCK_UNTRUSTED");
    require(now<expires,"COMMAND_EXPIRED");
    require(command.at("command_seq").is_number_unsigned() && command.at("command_seq").get<std::uint64_t>()>0,"INVALID_COMMAND_SEQUENCE");
    const auto seq=command.at("command_seq").get<std::uint64_t>();
    require(seq>ledger_.meta("seq:"+session_,0).get<std::uint64_t>(),"STALE_COMMAND_SEQUENCE");
    require(ctx.at("assignment_id")==assignment_ && ctx.at("snapshot_id")==snapshot_id_,"CONTEXT_MISMATCH");
    const auto action=command.at("action").get<std::string>();
    if(action=="START") {
      require(!active_,"BUSY");
      require(readiness().at("can_start")==true,"MISSION_NOT_READY");
      require(ctx.at("preparation_id")==preparation_ && ctx.at("readiness_revision")==readiness_revision_,"PREPARATION_STALE");
      require(ctx.at("execution_id").is_null() && ctx.at("flight_id").is_null(),"CONTEXT_MISMATCH");
      require(command.at("payload").at("start_mode")=="AUTO_TAKEOFF","UNSUPPORTED_CAPABILITY");
      execution_=random_id("execution"); flight_.clear();
      mission_.execution_id=execution_; vehicle_=testing::FakePx4();vehicle_.state.pose=launch_;simulation_time_=0;
      scan_revisions_.clear();scan_reports_.clear();perception_sequence_=0;
      sensor_request_=nullptr;sensor_sequences_.clear();
      sensor_receipts_.clear();sensor_receipt_order_.clear();
      sensor_camera_time_=sensor_scanner_time_=sensor_marker_time_=sensor_qr_time_=sensor_ack_time_=-1e30;
      sensor_qr_utc_=0;
      auto runtime=std::make_unique<Runtime>(motion_);
      require(runtime->start(mission_,vehicle_.state,0),"MISSION_NOT_READY");
      auto guard=std::make_unique<FlightGuard>(motion_,execution_,runtime->boot_session(),1,1);
      const auto response=result(command,"ACCEPTED","OK",1);
      ledger_.begin();
      try {
        save_result(command,response); ledger_.set_meta("seq:"+session_,seq);
        ledger_.set_meta("active_execution",{{"execution_id",execution_},{"control_request_id",id}});
        ledger_.commit();
      } catch(...) { ledger_.rollback(); throw; }
      runtime_=std::move(runtime); guard_=std::move(guard); active_=true;
      last_command_=command; cancel_requested_=land_requested_=false;
      pending_commands_=Json::array(); last_phase_.clear();last_route_revision_=0;last_mode_status_.clear(); preparation_.clear(); ++readiness_revision_;
      return response;
    }
    require(active_ && runtime_,"INVALID_PHASE");
    require(ctx.at("execution_id")==execution_ && ctx.at("flight_id")==nullable(flight_),"CONTEXT_MISMATCH");
    const auto allowed=readiness().at("allowed_commands");
    require(std::find(allowed.begin(),allowed.end(),Json(action))!=allowed.end(),"INVALID_PHASE");
    Command c=Command::Pause;
    if(action=="RESUME") c=Command::Resume;
    else if(action=="CANCEL") c=Command::Cancel;
    else if(action=="LAND_NOW") c=Command::Land;
    else require(action=="PAUSE","UNKNOWN_COMMAND");
    // Record before changing runtime; a restart never resumes this execution.
    const auto response=result(command,"ACCEPTED","OK",1);
    ledger_.begin();
    try { save_result(command,response); ledger_.set_meta("seq:"+session_,seq); ledger_.commit(); }
    catch(...) { ledger_.rollback(); throw; }
    if(!runtime_->command(c)) { auto fail=result(command,"FAILED","INVALID_PHASE",2); save_result(command,fail); return fail; }
    cancel_requested_=cancel_requested_ || c==Command::Cancel;
    land_requested_=land_requested_ || c==Command::Land;
    pending_commands_.push_back(command);
    return response;
  } catch(const std::invalid_argument& e) {
    const auto response=result(command,"REJECTED",e.what(),1); save_result(command,response); return response;
  } catch(const Json::exception&) {
    const auto response=result(command,"REJECTED","INVALID_COMMAND",1); save_result(command,response); return response;
  }
}
Json Engine::handle(const std::string& method, const Json& p) {
  if(method=="status") return status();
  if(method=="scan.request") return scan_request();
  if(method=="scan.observe") return scan_observe(p);
  if(method=="host.update") {
    require(p.at("schema_version")=="sangwon-host-health/1" && p.at("boot_id")==boot_,"HOST_BOOT_MISMATCH");
    require(p.at("scope")=="HOST_DIAGNOSTICS_ONLY" && p.at("can_start")==false && p.at("flight_authority")==false,"HOST_AUTHORITY_REFUSED");
    const auto stamp=p.at("generated_monotonic_s").get<double>();
    const auto age=steady_seconds()-stamp;
    require(std::isfinite(stamp) && age>=0 && age<10,"HOST_REPORT_STALE");
    require(std::abs(utc_seconds()-utc_parse(p.at("generated_at").get<std::string>())-age)<2,"HOST_CLOCK_UNTRUSTED");
    const auto producer=p.at("monitor_session_id").get<std::string>();
    require(!producer.empty() && producer.size()<=128 && p.at("monitor_seq").is_number_unsigned(),"HOST_SEQUENCE_INVALID");
    if(!host_report_.is_null()) {
      if(p==host_report_) return {{"updated",false}}; // A cached observation never refreshes its age.
      require(stamp>host_report_.at("generated_monotonic_s").get<double>(),"HOST_REPORT_OUT_OF_ORDER");
      if(producer==host_report_.at("monitor_session_id"))
        require(p.at("monitor_seq").get<std::uint64_t>()>host_report_.at("monitor_seq").get<std::uint64_t>(),"HOST_REPORT_OUT_OF_ORDER");
    }
    const auto checks=p.at("checks");
    require(checks.is_array() && checks.size()<=32,"HOST_CHECKS_INVALID");
    std::vector<std::string> identifiers;
    for(const auto& check:checks) {
      const auto id=check.at("id").get<std::string>();
      require(!id.empty() && id.size()<=64 && std::find(identifiers.begin(),identifiers.end(),id)==identifiers.end(),"HOST_CHECKS_INVALID");
      identifiers.push_back(id);
      const auto state=check.at("status").get<std::string>();
      require(state=="PASS" || state=="FAIL" || state=="WARN" || state=="UNKNOWN","HOST_CHECKS_INVALID");
      require(check.at("required_for_flight").is_boolean() && check.at("detail").get<std::string>().size()<=512
        && check.at("operator_action").get<std::string>().size()<=512,"HOST_CHECKS_INVALID");
    }
    host_report_=p; return {{"updated",true}};
  }
  if(method=="link.update") {
    connected_=p.at("connected").get<bool>(); link_time_=steady_seconds();
    link_code_=p.value("code",std::string("WEB_NOT_CONNECTED")); server_version_=p.value("observed_contract",std::string());
    if(!connected_) invalidate_preparation();
    return {{"updated",true}};
  }
  if(method=="session.set") {
    require(p.at("contract_version")==contract && p.at("drone_id")==config_.at("drone_id"),"UNSUPPORTED_CONTRACT");
    require(p.at("profile")==config_.at("profile") && p.at("boot_id")==boot_ && p.at("runtime_session_id")==runtime_id_,"CONTEXT_MISMATCH");
    require(p.value("flight_authority",true)==false && p.value("allowed_execution",std::string())==(replay_?"REPLAY_ONLY":"NONE"),"PHYSICAL_OUTPUT_FORBIDDEN");
    require(std::abs(utc_seconds()-utc_parse(p.at("server_time").get<std::string>()))<2,"CLOCK_UNTRUSTED");
    const auto next=p.at("control_session_id").get<std::string>(); require(!next.empty(),"INVALID_CONTROL_SESSION");
    if(session_!=next) { invalidate_preparation(); session_=next; planner_plan_=nullptr; }
    connected_=true; link_time_=steady_seconds(); link_code_="OK"; server_version_=contract;
    return context();
  }
  if(method=="plan.clear") { planner_plan_=nullptr; return {{"cleared",true}}; }
  if(method=="plan.receive") {
    require(!active_ && !session_.empty() && link_fresh(),"PLAN_CONTEXT_NOT_READY");
    const Json expected={{"boot_id",boot_},{"runtime_session_id",runtime_id_},{"control_session_id",session_}};
    require(p.at("context")==expected,"CONTEXT_MISMATCH");
    const auto text=p.at("plan_text").get<std::string>();require(text.size()<=2*1024*1024,"PLAN_TOO_LARGE");
    const auto sha=digest(text);const auto& ref=p.at("plan_ref");
    require(ref.at("sha256")==sha&&ref.at("byte_length")==text.size(),"PLAN_CONFLICT");
    const auto plan=parse(text);
    require(plan.at("contract_version")==contract&&plan.at("type")=="mission_plan"
      &&plan.at("scope")=="PLANNING_ONLY"&&plan.at("drone_id")==config_.at("drone_id"),"PLAN_CONTEXT_MISMATCH");
    require(plan.at("execution_eligible")==false&&plan.at("physical_flight_approval")==false,"PHYSICAL_OUTPUT_FORBIDDEN");
    require(plan.at("plan_id")==ref.at("plan_id"),"PLAN_CONFLICT");
    const auto id=plan.at("plan_id").get<std::string>();require(!id.empty()&&id.size()<=128,"INVALID_PLAN_ID");
    require(plan.at("start_mode")=="AUTO_TAKEOFF"&&plan.at("coordinate_frame").at("id")=="WAREHOUSE_MAP"
      &&plan.at("coordinate_frame").at("length_unit")=="m", "UNSUPPORTED_PLAN_FRAME");
    auto number=[](const Json& v,double lo,double hi) {require(v.is_number(),"INVALID_PLAN_NUMBER");const auto n=v.get<double>();
      require(std::isfinite(n)&&n>=lo&&n<=hi,"INVALID_PLAN_NUMBER");return n;};
    const double width=number(plan.at("map").at("width_m"),.01,10000);
    const double height=number(plan.at("map").at("height_m"),.01,10000);
    if(!plan.at("takeoff_z_m").is_null()) require(number(plan.at("takeoff_z_m"),0,30)>0,"INVALID_TAKEOFF_HEIGHT");
    if(plan.contains("start_yaw_deg")) require(number(plan.at("start_yaw_deg"),-180,180)<180,"INVALID_PLAN_YAW");
    const auto& tasks=plan.at("route_tasks");require(tasks.is_array()&&!tasks.empty()&&tasks.size()<=1000,"INVALID_PLAN_TASKS");
    for(const auto& task:tasks) {
      const auto type=task.value("type",std::string("waypoint"));require(type=="waypoint"||type=="hover"||type=="scan","INVALID_PLAN_TASK");
      number(task.at("x"),0,width);number(task.at("y"),0,height);number(task.at("z"),0,30);
      if(task.contains("yaw_deg")) require(number(task.at("yaw_deg"),-180,180)<180,"INVALID_PLAN_YAW");
      if(task.contains("hold_s")) number(task.at("hold_s"),0,300);
      if(type=="scan") {
        require(plan.at("labels").is_array()&&plan.at("labels").size()<=1000,"INVALID_PLAN_LABELS");
        require(std::any_of(plan.at("labels").begin(),plan.at("labels").end(),[&](const Json& label){
          return label.at("id")==task.at("label_point_id");}),"PLAN_LABEL_REFERENCE_MISMATCH");
      }
    }
    const auto& reasons=plan.at("blocking_reasons");require(reasons.is_array()&&reasons.size()<=32,"INVALID_PLAN_REASONS");
    require(std::find(reasons.begin(),reasons.end(),Json("EXECUTION_SNAPSHOT_REQUIRED"))!=reasons.end(),"PLAN_NOT_EXECUTABLE");
    for(const auto& reason:reasons) require(reason.is_string()&&reason.get<std::string>().size()<=128,"INVALID_PLAN_REASONS");
    const auto key="plan:"+id+":"+session_+":"+runtime_id_;
    const auto previous=ledger_.meta("plan-content:"+id);
    require(previous.is_null()||previous==sha,"PLAN_ID_CONFLICT");
    Json summary={{"plan_id",id},{"sha256",sha},{"state","RECEIVED"},{"scope","PLANNING_ONLY"},
      {"mission_db_id",plan.at("mission_db_id")},{"task_count",tasks.size()},{"blocking_reasons",reasons},
      {"execution_eligible",false},{"physical_flight_approval",false},{"context",expected}};
    if(ledger_.meta(key).is_null()) {
      Json receipt=envelope("mission_plan_receipt");receipt.update({{"scope","PLANNING_ONLY"},{"context",expected},
        {"plan_id",id},{"plan_sha256",sha},{"state","RECEIVED"},{"blocking_reasons",reasons},
        {"execution_eligible",false},{"physical_flight_approval",false}});
      ledger_.begin();
      try {ledger_.set_meta("plan-content:"+id,sha);ledger_.set_meta("planner-plan:"+id,text);
        ledger_.set_meta(key,summary);ledger_.enqueue(key,"mission-plan-receipts",receipt);ledger_.commit();}
      catch(...) {ledger_.rollback();throw;}
    }
    planner_plan_=summary;return summary;
  }
  if(method=="prepare") {
    try {
    require(replay_ && !recovery_locked_,"PROFILE_NOT_READY");
    require(!session_.empty() && link_fresh(),"WEB_NOT_CONNECTED");
    const auto assignment=p.at("assignment_id").get<std::string>();
    require(!assignment.empty() && assignment.size()<=128,"INVALID_ASSIGNMENT");
    const auto text=p.at("snapshot_text").get<std::string>(); const auto sha=digest(text);
    if(active_) { require(assignment==assignment_ && sha==prepared_context_.value("raw_sha256",std::string()),"BUSY"); return {{"state","EXECUTING"}}; }
    if(assignment==assignment_ && !preparation_.empty() && sha==prepared_context_.value("raw_sha256",std::string())) return readiness();
    require(ledger_.meta("closed_assignment:"+assignment).is_null(),"ASSIGNMENT_CLOSED");
    invalidate_preparation();
    require(p.at("snapshot_ref").at("sha256")==sha && p.at("snapshot_ref").at("byte_length")==text.size(),"SNAPSHOT_CONFLICT");
    auto s=parse(text);
    require(s.at("contract_version")==contract && s.at("profile")=="REPLAY","UNSUPPORTED_CONTRACT");
    require(s.at("drone_id")==config_.at("drone_id"),"CONTEXT_MISMATCH");
    if(!s.contains("map")) for(const auto& task:s.at("route_tasks"))
      require(task.at("type")=="waypoint","UNSUPPORTED_CAPABILITY:scan_task_v1");
    for(const auto& capability:s.at("required_capabilities"))
      require(std::find(capabilities.begin(),capabilities.end(),capability.get<std::string>())!=capabilities.end(),"UNSUPPORTED_CAPABILITY");
    const auto allow=config_.at("approved_replay_snapshot_sha256");
    require(std::find(allow.begin(),allow.end(),Json(sha))!=allow.end(),"UNAPPROVED_REPLAY_FIXTURE");
    require(s.at("snapshot_id")==p.at("snapshot_ref").at("snapshot_id"),"SNAPSHOT_CONFLICT");
    const auto snapshot_key="snapshot:"+s.at("snapshot_id").get<std::string>();
    const auto old_sha=ledger_.meta(snapshot_key);
    require(old_sha.is_null() || old_sha==sha,"SNAPSHOT_CONFLICT");
    require(s.at("start_mode")=="AUTO_TAKEOFF","UNSUPPORTED_CAPABILITY");
    const auto compiled=compile_replay_snapshot(s,config_,utc_seconds());
    require(!compiled.has_scan||scan_source()=="SYNTHETIC_SCAN_FIXTURE"||external_sensors(),"SCAN_ADAPTER_NOT_CONFIGURED");
    const auto scan_modes=config_.value("replay_scan_outcomes",Json::object());
    if(compiled.has_scan) for(const auto& [id,mode]:scan_modes.items()) {
      const auto supported=Json::array({"SCANNER_SUCCESS","CAMERA_SUCCESS","QR_UNREADABLE","MARKER_UNAVAILABLE","CAMERA_UNAVAILABLE"});
      require(std::find(supported.begin(),supported.end(),mode)!=supported.end(),"INVALID_SCAN_FIXTURE_MODE");
      require(std::any_of(compiled.mission.points.begin(),compiled.mission.points.end(),[&](const Waypoint& w){return w.scan&&w.scan->task_id==id;}),"SCAN_FIXTURE_TASK_MISMATCH");
    }
    ledger_.set_meta(snapshot_key,sha);
    snapshot_=std::move(s);mission_=compiled.mission;motion_=compiled.motion;launch_=compiled.launch;
    has_scan_=compiled.has_scan;validation_scope_=compiled.validation_scope;assignment_=assignment;
    snapshot_id_=snapshot_.at("snapshot_id").get<std::string>(); preparation_=random_id("preparation");
    execution_.clear(); flight_.clear(); ++readiness_revision_; prepared_context_={{"raw_sha256",sha}};
    preparation_code_="OK";
    auto report=envelope("preparation_report"); const auto report_id=random_id("preparation-report");
    report.update({{"report_id",report_id},{"report_revision",1},{"context",context()},
      {"assignment_state","READY"},{"readiness",readiness()},
      {"validation_scope",validation_scope_},{"field_errors",Json::array()}});
    ledger_.enqueue(report_id,"preparation-reports",report);
    return readiness();
    } catch(const std::invalid_argument& e) {
      preparation_code_=e.what();
      emit_event("PREPARATION_REJECTED",{{"code",preparation_code_},{"assignment_id",p.value("assignment_id",Json(nullptr))}});
      throw;
    } catch(const Json::exception&) {
      preparation_code_="INVALID_SNAPSHOT";
      emit_event("PREPARATION_REJECTED",{{"code",preparation_code_}});
      throw std::invalid_argument(preparation_code_);
    }
  }
  if(method=="prepare.clear") {
    if(!active_) { invalidate_preparation(); snapshot_id_.clear(); assignment_.clear(); preparation_code_="NO_MISSION"; }
    return {{"cleared",!active_}};
  }
  if(method=="command") return accept_command(p);
  if(method=="command.get") { const auto found=ledger_.command(p.at("control_request_id")); return found.is_null()?Json(nullptr):found.at("result"); }
  if(method=="outbox.list") {
    const auto after=p.value("after_key",std::string());require(after.size()<=256,"INVALID_OUTBOX_CURSOR");
    return ledger_.pending(after);
  }
  if(method=="outbox.ack") { ledger_.acknowledge(p.at("key"),p.at("sha256")); return {{"stored",true}}; }
  throw std::invalid_argument("UNKNOWN_IPC_METHOD");
}
void Engine::replay_scan_input() {
  if(!replay_||external_sensors()||!runtime_||!runtime_->scan_action()) return;
  auto& input=vehicle_.state.scan;
  input.qr.reset();input.marker.reset();input.camera_decoder_enabled=false;input.camera_activation_id.clear();
  const auto* action=runtime_->scan_action();const auto& plan=action->plan();
  const auto mode=config_.value("replay_scan_outcomes",Json::object()).value(plan.task_id,std::string("SCANNER_SUCCESS"));
  input.camera_healthy=mode!="CAMERA_UNAVAILABLE";input.scanner_healthy=true;
  const auto seq=++perception_sequence_;
  if(input.camera_healthy&&mode!="MARKER_UNAVAILABLE") {
    input.marker=MarkerObservation{"SYNTHETIC_SCAN_FIXTURE",plan.dictionary,plan.calibration_ref,plan.mounting_ref,plan.layout_ref,
      seq,vehicle_.state.frame_epoch,plan.marker_id,simulation_time_,plan.marker_size_m,0,plan.reader,true};
  }
  const auto& d=runtime_->scan_decision();
  if(!d||runtime_->paused()||runtime_->recovering()||runtime_->phase()!=Phase::Scan) return;
  if(d->enable_camera) {input.camera_decoder_enabled=true;input.camera_activation_id=d->window_id;}
  if((d->open_reader&&mode=="SCANNER_SUCCESS")||(d->enable_camera&&mode=="CAMERA_SUCCESS")) {
    Json qr=plan.expected_qr_fields;
    input.qr=QrObservation{"SYNTHETIC_SCAN_FIXTURE",execution_,plan.task_id,d->window_id,qr.dump(),seq,simulation_time_,
      d->open_reader?QrSource::Scanner:QrSource::Camera};
  }
}
void Engine::persist_scan_results() {
  if(!runtime_) return;
  std::map<std::string,ScanResult> results;
  for(const auto& r:runtime_->scan_results()) results[r.id]=r;
  if(runtime_->scan_action()&&runtime_->scan_action()->result()) {
    const auto& r=*runtime_->scan_action()->result();results[r.id]=r;
  }
  for(const auto& [id,r]:results) {
    const auto public_id="scan-"+digest(id).substr(0,32);
    auto ctx=last_command_.at("context");ctx["preparation_id"]=nullptr;ctx["readiness_revision"]=nullptr;
    ctx["execution_id"]=execution_;ctx["flight_id"]=nullable(flight_);
    auto report=envelope("scan_task_result");
    report.update({{"task_result_id",public_id},{"client_scan_id",public_id},{"context",ctx},
      {"task_id",r.task_id},{"label_point_id",r.label_id},
      {"task_status",r.preempted?"FAILED":(r.egress_complete?"COMPLETED":"RUNNING")},
      {"task_outcome",r.preempted?"FAILED":(r.egress_complete?(r.succeeded?"SUCCEEDED":"FAILED"):"PENDING")},
      {"scan_outcome",r.succeeded?"SUCCEEDED":"FAILED"},{"failure_code",r.succeeded?Json(nullptr):Json(r.code)},
      {"scanner_attempts_started",r.scanner_attempts},{"camera_qr_used",r.camera_used},
      {"reader_source",r.source?Json(*r.source==QrSource::Scanner?"SCANNER":"CAMERA"):Json(nullptr)},
      {"raw_qr_data",r.raw.empty()?Json(nullptr):Json(r.raw)},
      {"egress_status",r.preempted?"PREEMPTED":(r.egress_complete?"COMPLETED":"PENDING")},
      {"source",scan_source()},{"validation_scope",validation_scope_},
      {"decoded_at",r.decoded_s?Json(utc_text(external_sensors()?sensor_qr_utc_:utc_seconds())):Json(nullptr)}});
    if(scan_reports_.count(id)) report["decoded_at"]=scan_reports_.at(id).at("decoded_at");
    if(scan_reports_.count(id)&&report==scan_reports_.at(id)) {
      vehicle_.state.scan.stored_result_id=id;continue;
    }
    const int revision=scan_revisions_[id]+1;
    auto stored=report;stored["result_revision"]=revision;stored["recorded_at"]=utc_text(utc_seconds());
    // Only SQLite FULL-sync commit grants the action permission to egress.
    // Receipt acknowledgement is separate and can arrive after network recovery.
    ledger_.begin();
    try {ledger_.set_meta("scan:"+public_id,stored);ledger_.enqueue(public_id+":"+std::to_string(revision),"scan-task-results",stored);ledger_.commit();}
    catch(...) {ledger_.rollback();vehicle_.state.logging_ok=false;return;}
    scan_reports_[id]=std::move(report);scan_revisions_[id]=revision;vehicle_.state.scan.stored_result_id=id;
  }
}
void Engine::tick(double steady_now) {
  ++sequence_;
  if(!link_fresh()) invalidate_preparation();
  if(!active_ || !runtime_) { last_tick_=steady_now; return; }
  const int steps=std::clamp(config_.value("replay_steps_per_tick",1),1,20);
  for(int step=0;step<steps && active_;++step) {
    // External producers use the same host's monotonic time. Do not accelerate
    // their capture lifetime or silently substitute internally generated data.
    simulation_time_+=external_sensors()?std::max(motion_.tick_s,steady_now-last_tick_):motion_.tick_s;
    vehicle_.stamp(simulation_time_);
    if(external_sensors()) external_scan_input(steady_now);
    replay_scan_input();
    if(guard_->released()||guard_->landing_locked()) runtime_->control_fault(guard_->fault_reason(),guard_->released());
    const auto intent=runtime_->tick(vehicle_.state,simulation_time_);
    if(intent) require(guard_->accept(*intent,simulation_time_),"REPLAY_GUARD_REJECTED");
    vehicle_.apply(guard_->tick(vehicle_.state,simulation_time_),motion_.tick_s,simulation_time_);
    const auto control=mode_control();const auto current_control=control.dump();
    for(const auto& e:guard_->drain_mode_events()) {
      emit_event("PX4_CONTROL_TRANSITION",{{"scope","SYNTHETIC_CONTROL_ONLY"},
        {"request_id",e.request.request_id},{"operation",control_operation_name(e.request.operation)},
        {"state",transition_state_name(e.state)},{"expected_mode",mode_name(e.expected_mode)},
        {"observed_mode",mode_name(e.observed_mode)},{"attempts",e.attempts},{"ack_accepted",e.ack_accepted},
        {"observed_simulation_s",e.observed_s},{"deadline_simulation_s",e.request.deadline_s},
        {"takeoff_completed",e.takeoff_completed},{"physical_output_enabled",false}});
    }
    if(current_control!=last_mode_status_&&guard_->released()) emit_event("PX4_CONTROL_RELEASED",control);
    last_mode_status_=current_control;
    persist_scan_results();
    if(flight_.empty() && !vehicle_.state.landed) {
      flight_=random_id("flight"); emit_event("TAKEOFF_OBSERVED",{{"source","FAKE_PX4"}});
    }
    if(wire_phase()!=last_phase_) { last_phase_=wire_phase(); emit_event("PHASE_CHANGED",{{"phase",last_phase_}}); }
    if(runtime_->route_revision()!=last_route_revision_) {
      last_route_revision_=runtime_->route_revision();emit_event("ROUTE_PLANNED",{{"route_revision",last_route_revision_},
        {"scope","SYNTHETIC_STATIC_MAP_ONLY"},{"visited_tasks",runtime_->visited()}});
    }
    const bool terminal=runtime_->phase()==Phase::Complete || runtime_->phase()==Phase::Aborted || runtime_->phase()==Phase::Manual;
    Json still_pending=Json::array();
    for(const auto& command:pending_commands_) {
      const auto action=command.at("action").get<std::string>();
      const bool done=(action=="PAUSE" && runtime_->paused() && stopped(vehicle_.state,motion_)) ||
        (action=="RESUME" && !runtime_->paused() && !runtime_->recovering()) ||
        ((action=="CANCEL" || action=="LAND_NOW") && terminal && vehicle_.state.landed && !vehicle_.state.armed);
      if(done || terminal) save_result(command,result(command,done?"COMPLETED":"PREEMPTED",done?"OK":"EXECUTION_ENDED",2));
      else still_pending.push_back(command);
    }
    pending_commands_=std::move(still_pending);
    if(terminal) {
      const bool manual=runtime_->phase()==Phase::Manual;
      const bool landed=vehicle_.state.landed && !vehicle_.state.armed;
      const bool all=runtime_->visited()==mission_.points.size();
      const std::string outcome=manual?"MANUAL_HANDOVER":(!landed?"FAILED":(cancel_requested_?"CANCELED":(all&&!land_requested_?"SUCCEEDED":"FAILED")));
      auto response=result(last_command_,manual?"PREEMPTED":(landed?"COMPLETED":"FAILED"),outcome,2);
      std::size_t scan_total=0,scan_succeeded=0,scan_failed=0;
      for(const auto& point:mission_.points) if(point.scan) ++scan_total;
      for(const auto& r:runtime_->scan_results()) {if(r.succeeded) ++scan_succeeded;else ++scan_failed;}
      const bool scan_preempted=runtime_->scan_action()&&runtime_->scan_action()->stage()==ScanStage::Preempted&&runtime_->scan_action()->result();
      if(scan_preempted) {if(runtime_->scan_action()->result()->succeeded) ++scan_succeeded;else ++scan_failed;}
      const std::string work_outcome=all?(scan_failed?"PARTIAL_FAILED":"ALL_SUCCEEDED"):"INCOMPLETE";
      response["flight_outcome"]=outcome;response["work_outcome"]=work_outcome;
      response["visited"]=runtime_->visited();
      auto execution_result=envelope("execution_result");
      const auto total=mission_.points.size();
      const auto visits=std::min(runtime_->visited(),total);
      const auto completed_scan_failures=std::count_if(runtime_->scan_results().begin(),runtime_->scan_results().end(),[](const ScanResult& r){return !r.succeeded||r.preempted;});
      const auto succeeded=visits-std::size_t(completed_scan_failures);
      const std::size_t failed=std::size_t(completed_scan_failures)+((scan_preempted||(!manual&&!cancel_requested_&&!land_requested_&&!all))?1:0);
      auto execution_context=last_command_.at("context");
      execution_context["preparation_id"]=nullptr; execution_context["readiness_revision"]=nullptr;
      execution_context["execution_id"]=execution_; execution_context["flight_id"]=nullable(flight_);
      execution_result.update({{"context",execution_context},{"result_revision",1},
        {"command_status",response.at("status")},{"flight_outcome",outcome},
        {"work_outcome",work_outcome},
        {"landed",vehicle_.state.landed},{"armed",vehicle_.state.armed},
        {"ended_at",utc_text(utc_seconds())},{"source","FAKE_PX4"},
        {"validation_scope",validation_scope_},
        {"task_summary",{{"total",total},{"succeeded",succeeded},{"failed",failed},
                         {"not_attempted",total-succeeded-failed}}},
        {"scan_summary",{{"total",scan_total},{"succeeded",scan_succeeded},{"failed",scan_failed},{"not_attempted",scan_total-scan_succeeded-scan_failed}}},
        {"ulog",{{"collection_status","NOT_APPLICABLE_REPLAY"},{"verified",false}}}});
      ledger_.begin();
      try { save_result(last_command_,response); ledger_.set_meta("active_execution",manual||!landed?Json{{"execution_id",execution_}}:Json(nullptr)); ledger_.set_meta("closed_assignment:"+assignment_,execution_); emit_event("EXECUTION_ENDED",response); ledger_.enqueue(execution_+":result:1","companion-phase",execution_result); ledger_.commit(); }
      catch(...) { ledger_.rollback(); throw; }
      active_=false; recovery_locked_=manual||!landed; ++readiness_revision_;
    }
  }
  last_tick_=steady_now;
}
}  // namespace sangwon::service
