#pragma once
// PX4 requests only. The fixed height reference is controlled by PX4.
#include "sangwon_ai/field_mission.hpp"

namespace sangwon::field {
inline bool field_mode_allowed(const std::string& phase,const std::string& mode) {
  if(phase=="PREPARING"||phase=="GROUND_RECOVERY")return mode=="POSCTL"||mode=="AUTO.LOITER"||mode=="OFFBOARD";
  if(phase=="POSITION_HANDOFF"||phase=="RESUME_PREPARING")return mode=="OFFBOARD"||mode=="POSCTL";
  if(phase=="HOLD_READY")return mode=="POSCTL";
  if(phase=="LANDING")return mode=="OFFBOARD"||mode=="POSCTL"||mode=="AUTO.LAND";
  return mode=="OFFBOARD";
}
class FieldExecutor {
 public:
  explicit FieldExecutor(Json settings):cfg_(std::move(settings)) {}
  std::string phase{"IDLE"}, reason{"waiting_for_web_start"}, failure;
  Json decision=Json::object(), actions=Json::array();
  bool streaming() const { return phase=="PREPARING" || phase=="ARMING" || phase=="TAKING_OFF" || phase=="POSITION_HANDOFF" || phase=="RESUME_PREPARING" || phase=="HANDOFF" || phase=="MOVING" || phase=="DWELL" || phase=="RETURNING" || phase=="LANDING"; }
  bool terminal() const { return phase=="END" || phase=="FAILED" || phase=="PILOT_OVERRIDE" || phase=="UNCONFIRMED"; }
  bool pending() const { return pending_token_!=0; }
  unsigned pending_token() const { return pending_token_; }
  void command_queued(unsigned token) { if(token==pending_token_)waiting_journal_=true; }
  void command_dispatched(unsigned token,double now) {
    if(token==pending_token_){waiting_journal_=false;pending_at_=now;entered_=now;}
  }
  double height() const { return height_; }
  double yaw() const { return yaw_; }
  Vec3 target() const { return target_; }
  Json status() const {
    Json out={{"phase",phase},{"state",phase},{"reason",reason},{"failure",failure.empty()?Json():Json(failure)},
      {"executor_backend","sangwon_cpp_executor"},{"executor_fc_transport",true},{"mission_ai",{{"backend","sangwon_cpp_bt"},{"connected",true},{"prepared",bool(ai_)},{"fc_transport",false}}},
      {"home_xy_m",ai_?xy_json(home_):Json()}, {"goal_xy_m",streaming()?xy_json(target_):Json()},
      {"command_record_pending",pending()&&waiting_journal_},{"height_reference_m",ai_?Json(height_):Json()}, {"control_ack",ack_},
      {"flight_control_enabled",flag(cfg_,"execute")&&bool(ai_)&&!terminal()},
      {"bridge_observation_timeout_s",cfg_.value("bridge_observation_timeout_s",.35)},{"landing_verified",landing_verified_},{"completed_waypoints",decision.value("completed_waypoints",0)},
      {"mission_complete",phase=="END"&&landing_verified_},{"route_complete",flag(decision,"home_verified")},
      {"home_verified",flag(decision,"home_verified")},{"active_waypoint_index",decision.value("waypoint_index",0)},
      {"flight_outcome",phase=="END"?"SUCCEEDED":terminal()?phase:"IN_PROGRESS"},
      {"work_outcome",phase=="END"?"SUCCEEDED":"INCOMPLETE"},
      {"horizontal_observation",decision.value("lidar_support",Json::object())},{"cpp_decision",decision}};
    if(separate())out["native_takeoff"]={{"can_takeoff",can_takeoff_&&!terminal()},{"can_start",can_start_&&!terminal()},
      {"can_land",bool(ai_)&&!terminal()&&last_armed_},{"blocker",hold_blocker_},{"home_xy_m",ai_?xy_json(home_):Json()},
      {"mission_started",mission_started_},{"takeoff_request_id",takeoff_request_id_}};
    for(const auto* key:{"mission_db_id","mission_code","route_revision"})if(assignment_.contains(key))out[key]=assignment_.at(key);
    return out;
  }
  void command_result(unsigned token,bool accepted,double now) {
    if(token!=pending_token_ || terminal()) return;
    pending_token_=0; waiting_journal_=false; accepted_=accepted; confirmed_=accepted?now:-1;
    if(!accepted) failure="px4_rejected_command";
  }
  void interrupt(const std::string& why,bool pilot,double now,const Json& s) {
    actions=Json::array();
    if(terminal())return;
    last_armed_=flag(s,"armed");can_takeoff_=false;can_start_=false;
    if(pilot) { finish("PILOT_OVERRIDE",why,now); return; }
    abort(why,s,now);
  }
  void tick(Json s,double now,const Json& request=Json(),double wall=0) {
    actions=Json::array();
    if(terminal())return;
    last_armed_=flag(s,"armed");can_takeoff_=false;can_start_=false;
    const double dt=last_>=0?now-last_:.05; last_=now;
    if(ai_) {
      // Pilot selections win over every decision, timeout and recovery action.
      if(flag(s,"connected") && age(s,"state_age_s",cfg_.value("state_timeout_s",2.5))) {
        const auto mode=s.value("mode",std::string());
        const bool allowed=field_mode_allowed(phase,mode);
        if(!allowed || flag(s,"rc_switch_changed")) { finish("PILOT_OVERRIDE","manual_or_px4_failsafe_has_priority",now); return; }
        if((phase=="HOLD_READY"||phase=="POSITION_HANDOFF"||phase=="RESUME_PREPARING")&&flag(s,"rc_required")&&age(s,"rc_age_s",1)&&flag(s,"rc_neutral_valid")&&!flag(s,"rc_sticks_centered")) { finish("PILOT_OVERRIDE","position_sticks_moved",now);return; }
      }
      if(pending() && waiting_journal_ && now-pending_at_>cfg_.value("request_ttl_s",30.)) { abort("command_record_wait_expired_unsent",s,now); return; }
      if(pending() && !waiting_journal_ && now-pending_at_>cfg_.value("command_timeout_s",3.)) { finish("UNCONFIRMED","command_response_timeout_effect_unknown",now); return; }
      if(dt<=0) { abort("cpp_executor_clock_invalid",s,now); return; }
    }
    if(!ai_) {
      bool stationary=flag(s,"connected") && !flag(s,"armed") && s.value("landed",0)==1 && age(s,"pose_age_s",.2) && s.contains("xy") && s.at("xy").size()==2 && s.contains("velocity_xy") && s.at("velocity_xy").size()==2 && xy(point(s.at("velocity_xy")))<=.1;
      if(!stationary || (ground_since_>=0 && xy(point(s.at("xy"))-ground_)>.1)) ground_since_=-1;
      else if(ground_since_<0) { ground_since_=now; ground_=point(s.at("xy")); }
      hold_blocker_=ground_gate(s,false);
      can_takeoff_=separate()&&hold_blocker_=="ready"&&ground_since_>=0&&now-ground_since_>=3&&flag(cfg_,"execute");
      if(request.is_null() || request.value("action",std::string())!=(separate()?"takeoff":"start")) return;
      reason=ground_gate(s);
      if(reason!="ready")return;
      if(ground_since_<0 || now-ground_since_<3) { reason="wait_stationary_three_seconds"; return; }
      if(!flag(cfg_,"execute")) { reason="execution_disabled"; return; }
      require(wall>=request.at("requested_s").get<double>() && wall-request.at("requested_s").get<double>()<=cfg_.value("request_ttl_s",30.),"CONTROL_REQUEST_EXPIRED");
      require(flag(s,"request_persisted"),"REQUEST_PERSISTENCE_REQUIRED");
      home_=target_=point(s.at("xy")); ground_height_=height_=number(s.at("altitude_enu_m"),-100,100); yaw_=number(s.at("yaw_deg"),-360,360);
      execution_=request.at("request_id").get<std::string>();
      assignment_=request;mission_started_=!separate();
      if(separate()){takeoff_request_id_=execution_;assignment_["waypoints"]=Json::array({Json{{"xy",xy_json(home_)},{"dwell_s",10.},{"type","hover"}}});}
      prepare_ai(assignment_);
      ack_={{"action",request.at("action")},{"request_id",execution_},{"requested_at",request.at("requested_at")}};
      started_=now; enter("PREPARING","local_setpoint_warmup",now); return;
    }
    if(!request.is_null() && request.value("action",std::string())=="land" && phase!="LANDING" && phase!="GROUND_RECOVERY") { abort("operator_cancelled",s,now); return; }
    if((phase=="PREPARING"||phase=="ARMING")&&!flag(s,"armed")&&!age(s,"uwb_age_s",.25)) { abort("prearm_uwb_unavailable",s,now);return; }
    if((phase=="PREPARING"||phase=="ARMING")&&!flag(s,"armed")&&!age(s,"pose_age_s",cfg_.value("pose_timeout_s",.2))) { abort("prearm_position_feedback_missing",s,now);return; }
    if(!failure.empty() && !pending() && phase!="LANDING" && phase!="GROUND_RECOVERY") { abort(failure,s,now); return; }
    const bool landed=accepted_ && !pending() && confirmed_>=0 && flag(s,"connected") && !flag(s,"armed") && s.value("landed",0)==1 && age(s,"state_age_s",1) && age(s,"landed_age_s",1) && now-s.at("state_age_s").get<double>()>=confirmed_ && now-s.at("landed_age_s").get<double>()>=confirmed_;
    if(phase=="GROUND_RECOVERY") {
      if(landed)finish("FAILED",failure,now);
      else if(now-entered_>cfg_.value("landing_timeout_s",30.))finish("UNCONFIRMED","ground_recovery_not_confirmed",now);
      return;
    }
    Json in={{"execution_id",execution_},{"plan_ref",plan_},{"sample_time_s",now},
      {"executor_phase",phase},{"executor_failure",failure},{"landing_confirmed",landed},{"state",s},{"mission_authorized",!separate()||mission_started_}};
    if(!request.is_null() && request.value("action",std::string())=="return_to_home")in["command"]="return_to_home";
    decision=ai_->step(in);
    if(phase=="LANDING") {
      if(landed && decision.at("phase")=="END") { landing_verified_=true; finish("END","home_landed_disarmed",now); }
      else if(landed && !failure.empty()) { landing_verified_=true;finish("FAILED",failure,now); }
      else if(now-entered_>cfg_.value("landing_timeout_s",30.))finish("UNCONFIRMED","landing_completion_unverified",now);
      return;
    }
    const auto ai_phase=decision.at("phase").get<std::string>();
    if(ai_phase=="PILOT_OVERRIDE") { finish("PILOT_OVERRIDE","cpp_ai_released_control",now); return; }
    if(ai_phase=="LANDING" || ai_phase=="FAILED") {
      if(flag(decision,"home_verified") && ai_phase=="LANDING" && decision.at("completed_waypoints")==assignment_.at("waypoints").size()) { enter("LANDING","cpp_bt_home_arrival_verified",now); issue("land",now); }
      else abort("cpp_ai:"+decision.at("reason").get<std::string>(),s,now);
      return;
    }
    const bool pose_current=age(s,"pose_age_s",cfg_.value("pose_timeout_s",.2))&&s.at("xy").is_array()&&s.at("xy").size()==2;
    if(pose_current){const auto position=point(s.at("xy"));const auto& bounds=cfg_.at("bounds_xy_m");
    if(position.x<bounds[0].get<double>()||position.x>bounds[1].get<double>()||position.y<bounds[2].get<double>()||position.y>bounds[3].get<double>()) { abort("px4_pose_outside_bounds",s,now); return; }
    const double pose_source=now-s.at("pose_age_s").get<double>(),pose_dt=previous_pose_?std::max(dt,pose_source-previous_pose_source_):dt;
    if(previous_pose_ && (xy(point(s.at("xy"))-*previous_pose_)>.3+pose_dt || std::abs(s.at("altitude_enu_m").get<double>()-previous_height_)>.3+pose_dt)) { abort("px4_local_frame_or_pose_jump",s,now); return; }
    previous_pose_=point(s.at("xy"));previous_height_=s.at("altitude_enu_m").get<double>();previous_pose_source_=pose_source;
    }
    if(now-started_>300) { abort("mission_time_limit",s,now); return; }
    if(separate()&&!mission_started_&&pose_current&&xy(point(s.at("xy"))-home_)>.6) { abort("takeoff_hold_horizontal_drift",s,now);return; }
    if(separate()&&(phase=="POSITION_HANDOFF"||phase=="HOLD_READY"||phase=="RESUME_PREPARING")&&flag(s,"rc_required")&&(!flag(s,"rc_valid")||!age(s,"rc_age_s",1))) { abort("fresh_rc_required",s,now);return; }
    if(phase=="PREPARING") {
      if(!pending() && !accepted_ && now-entered_>=1.2)issue("offboard_mode",now);
      else if(!pending() && accepted_ && s.at("mode")=="OFFBOARD") { enter("ARMING","local_mode_confirmed",now); issue("arm",now); }
      else if(!waiting_journal_&&now-entered_>6)abort("local_mode_not_confirmed",s,now);
    } else if(phase=="ARMING") {
      if(!pending() && accepted_ && flag(s,"armed")) { height_=ground_height_+cfg_.at("expected_mis_takeoff_alt_m").get<double>();enter("TAKING_OFF","px4_local_height_goal",now); }
      else if(!waiting_journal_&&now-entered_>cfg_.value("command_timeout_s",3.))abort("arm_state_not_confirmed",s,now);
    } else if(separate()&&!mission_started_) {
      tick_takeoff_hold(s,now,request,wall);
    } else if(phase=="RESUME_PREPARING") {
      // Warm up the ORIGINAL launch reference. Never capture drift as home.
      if(!pending()&&!accepted_&&now-entered_>=1.2&&resume_gate(s)=="ready")issue("offboard_mode",now);
      else if(!pending()&&accepted_&&mode_confirmed(s,"OFFBOARD",now))enter("HANDOFF","airborne_offboard_confirmed_no_rearm",now);
      else if(!pending()&&!accepted_&&resume_gate(s)!="ready")reason="offboard_reentry_wait_for_fresh_inputs";
      else if(!waiting_journal_&&now-entered_>6)abort("offboard_reentry_not_confirmed",s,now);
    } else if(ai_phase=="MOVING" || ai_phase=="DWELL" || ai_phase=="RETURNING") {
      target_=point(decision.at("stream_xy"));
      if(phase!=ai_phase)enter(ai_phase,"cpp_bt_mission_decision",now);
    }
  }
 private:
  Json cfg_,ack_,assignment_; std::unique_ptr<MissionAI> ai_; std::string execution_,plan_;
  Vec3 home_{},target_{},ground_{}; std::optional<Vec3> previous_pose_;
  double height_{},ground_height_{},yaw_{},ground_since_{-1},previous_height_{},previous_pose_source_{},last_{-1},started_{},entered_{},pending_at_{},confirmed_{-1};
  unsigned counter_{},pending_token_{}; bool accepted_{},landing_verified_{},waiting_journal_{};
  bool mission_started_{},can_takeoff_{},can_start_{},last_armed_{};
  double hold_since_{-1};std::string takeoff_request_id_,hold_blocker_="waiting_for_ground_inputs";
  bool separate()const{return cfg_.value("start_mode",std::string("ground_takeoff"))=="takeoff_hold";}
  void prepare_ai(const Json& request) {
    Json prep={{"execution_id",execution_},{"settings",cfg_},{"home_xy",xy_json(home_)},
      {"height_reference_m",ground_height_+cfg_.at("expected_mis_takeoff_alt_m").get<double>()},
      {"ceiling_height_m",request.at("ceiling_height_m")},{"waypoints",request.at("waypoints")}};
    ai_=std::make_unique<MissionAI>(prep);plan_=ai_->prepared().at("plan_ref").get<std::string>();
  }
  bool mode_confirmed(const Json& s,const std::string& mode,double now)const {
    return confirmed_>=0&&age(s,"state_age_s",cfg_.value("state_timeout_s",2.5))&&s.at("mode")==mode&&now-s.at("state_age_s").get<double>()>=confirmed_;
  }
  bool position_stable(const Json& s)const {
    return !flag(observation_status(s,cfg_.value("bridge_observation_timeout_s",.35)),"progress_held")&&flag(s,"armed")&&s.value("landed",0)==2
      &&age(s,"landed_age_s",cfg_.value("state_timeout_s",2.5))&&xy(point(s.at("xy"))-home_)<=.15&&xy(point(s.at("uwb_xy"))-home_)<=.25
      &&xy(point(s.at("velocity_xy")))<=.1&&std::abs(s.at("vertical_speed_m_s").get<double>())<=.1&&std::abs(s.at("altitude_enu_m").get<double>()-height_)<=.15;
  }
  std::string stability_gate(const Json& s)const {
    if(!position_stable(s))return "hold_position_or_observation_not_stable";
    if(flag(s,"rc_required")&&(!flag(s,"rc_valid")||!age(s,"rc_age_s",1)||!flag(s,"rc_neutral_valid")||!flag(s,"rc_sticks_centered")))return "position_requires_fresh_centered_sticks";
    return "ready";
  }
  std::string resume_gate(const Json& s)const {
    const auto stable=stability_gate(s);if(stable!="ready")return stable;
    if(!flag(s,"estimator_valid")||!flag(s,"vertical_estimator_valid"))return "px4_estimator_invalid";
    if(!flag(s,"command_services_ready")||!flag(s,"local_setpoint_ready"))return "local_transport_unavailable";
    if(!age(s,"battery_age_s",3)||number(s.at("battery"),0,1)<.3)return "battery_start_reserve_required";
    return "ready";
  }
  void tick_takeoff_hold(const Json& s,double now,const Json& request,double wall) {
    if(!flag(s,"armed")){finish("UNCONFIRMED","unexpected_disarm",now);return;}
    hold_blocker_=stability_gate(s);
    if(hold_blocker_=="ready"){if(hold_since_<0)hold_since_=now;}else hold_since_=-1;
    const bool stable=hold_since_>=0&&now-hold_since_>=3;
    if(phase=="TAKING_OFF") {
      if(stable){enter("POSITION_HANDOFF","height_and_xy_stable_request_position",now);issue("position_mode",now);hold_since_=-1;}
      else if(flag(observation_status(s,cfg_.value("bridge_observation_timeout_s",.35)),"progress_held")){entered_=now;reason="takeoff_wait_for_observation";}
      else if(now-entered_>cfg_.value("takeoff_timeout_s",30.))abort("takeoff_stability_timeout",s,now);
    } else if(phase=="POSITION_HANDOFF") {
      if(!pending()&&accepted_&&mode_confirmed(s,"POSCTL",now)){enter("HOLD_READY","position_confirmed_wait_for_web_start",now);hold_since_=-1;}
      else if(!waiting_journal_&&now-entered_>6)abort("position_mode_not_confirmed",s,now);
    } else if(phase=="HOLD_READY") {
      hold_blocker_=resume_gate(s);can_start_=stable&&hold_blocker_=="ready";
      reason=can_start_?"position_stable_wait_for_web_start":"position_wait_for_stability_or_observation";
      if(request.is_null()||request.value("action",std::string())!="start"||!can_start_)return;
      require(flag(s,"request_persisted")&&wall>=request.at("requested_s").get<double>()&&wall-request.at("requested_s").get<double>()<=cfg_.value("request_ttl_s",30.),"CONTROL_REQUEST_EXPIRED_OR_UNRECORDED");
      require(request.at("request_id")!=takeoff_request_id_,"NEW_MISSION_REQUEST_REQUIRED");
      execution_=request.at("request_id").get<std::string>();assignment_=request;prepare_ai(request);mission_started_=true;
      ack_={{"action","start"},{"request_id",execution_},{"requested_at",request.at("requested_at")}};
      can_start_=false;enter("RESUME_PREPARING","web_start_warmup_original_launch_reference",now);
    }
  }
  void enter(std::string p,std::string why,double now) { phase=std::move(p);reason=std::move(why);entered_=now;accepted_=false;confirmed_=-1; }
  void finish(const std::string& p,const std::string& why,double now) { actions=Json::array();pending_token_=0;waiting_journal_=false;enter(p,why,now); }
  void issue(const std::string& kind,double now) { pending_token_=++counter_;pending_at_=now;waiting_journal_=false;accepted_=false;confirmed_=-1;actions.push_back({{"kind",kind},{"token",pending_token_},{"sent_s",now}}); }
  void abort(const std::string& why,const Json& s,double now) {
    failure=why;
    if(pending()) {
      // An unsent command has no unknown FC effect. Cancel its token before
      // recovery; an explicit airborne LAND also supersedes an in-flight ACK.
      if(waiting_journal_||(why=="operator_cancelled"&&flag(s,"armed"))) {pending_token_=0;waiting_journal_=false;}
      else { finish("UNCONFIRMED",why,now); return; }
    }
    if(phase=="LANDING" || phase=="GROUND_RECOVERY")return;
    if(flag(s,"connected") && !flag(s,"armed") && s.value("landed",0)==1 && age(s,"state_age_s",1)) { enter("GROUND_RECOVERY",why,now);issue("position_mode",now); }
    else { enter("LANDING",why,now); issue("land",now); }
  }
  std::string ground_gate(const Json& s,bool check_web=true) const {
    for(const auto* key:{"layout_confirmed","sensor_mount_confirmed","takeoff_settings_confirmed"})if(!flag(cfg_,key))return std::string(key)+"_required";
    for(const auto* key:{"command_services_ready","local_setpoint_ready","estimator_valid","vertical_estimator_valid","bridge_ready","alignment_matches","ground_gate_ready"})if(!flag(s,key))return std::string(key)+"_required";
    if(!flag(s,"connected") || flag(s,"armed") || s.value("landed",0)!=1 || !age(s,"state_age_s",cfg_.value("state_timeout_s",2.5)) || !age(s,"landed_age_s",cfg_.value("state_timeout_s",2.5)) || !age(s,"estimator_age_s",.5) || !age(s,"uwb_age_s",.25))return "fresh_disarmed_ground_inputs_required";
    if(!age(s,"bridge_age_s",2.5) || !age(s,"bridge_observation_age_s",cfg_.value("bridge_observation_timeout_s",.35)) || s.value("bridge_published",0)<=0)return "observation_bridge_unavailable";
    if(s.at("mode")!="POSCTL" && s.at("mode")!="AUTO.LOITER")return "px4_position_mode_required";
    if(check_web&&!age(s,"web_age_s",cfg_.value("web_timeout_s",2.)))return "web_assignment_stale";
    if(separate()&&(s.at("mode")!="POSCTL"||(flag(s,"rc_required")&&(!flag(s,"rc_neutral_valid")||!flag(s,"rc_sticks_centered")))))return "position_with_centered_sticks_required";
    if(!age(s,"max_up_age_s",5) || !s.at("max_up_m_s").is_number() || number(s.at("max_up_m_s"),0,20)<=0 || s.at("max_up_m_s").get<double>()>.5)return "px4_up_limit_0p5_required";
    if(!s.at("rc_override").is_number_integer() || !(s.at("rc_override").get<int>()&2))return "px4_offboard_rc_override_required";
    if(flag(s,"rc_required") && (!flag(s,"rc_valid") || !flag(s,"rc_mapping_valid") || !age(s,"rc_age_s",1)))return "physical_rc_required";
    if(!age(s,"battery_age_s",3) || number(s.at("battery"),0,1)<.3)return "battery_start_reserve_required";
    return "ready";
  }
};
}
