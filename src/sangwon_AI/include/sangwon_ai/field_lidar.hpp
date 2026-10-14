#pragma once
#include "sangwon_ai/field_mission.hpp"

namespace sangwon::field {
// Sender evidence only. FC fusion qualification is a separate, explicit record.
inline Json lidar_sender_state(const Json& report,double ros_s,double receipt_age_s,
                               size_t publishers,const std::string& tag) {
  Json out={{"lidar_ready",false},{"lidar_gate","lidar_status_unavailable"},
    {"lidar_speed_m_s",Json()},{"lidar_fusion_verified",false}};
  try {
    require(publishers==1,"lidar_source_ambiguous_or_missing");
    require(receipt_age_s>=0&&receipt_age_s<=.5,"lidar_status_stale");
    const auto& cfg=report.at("settings");const auto& obs=report.at("last_published_observation");
    const double status_age=ros_s-number(report.at("status_stamp_ns"),1,1e20)/1e9;
    const double observation_age=ros_s-number(report.at("last_sent_stamp_ns"),1,1e20)/1e9;
    require(status_age>=0&&status_age<=.5&&observation_age>=0&&observation_age<=.35,"lidar_observation_stale_or_future");
    require(report.at("gate")=="ready"&&report.at("geometry_gate")=="ready","lidar_sender_not_ready");
    require(flag(cfg,"enabled")&&cfg.at("tag_id")==tag&&cfg.at("floor_policy")=="metric_scan"
      &&!cfg.at("vertical_structure_ref").get<std::string>().empty(),"lidar_takeoff_geometry_required");
    require(flag(report,"mode_independent")&&report.at("z_observed")==false&&report.at("flight_commands_enabled")==false,"lidar_horizontal_contract_required");
    require(flag(obs,"accepted")&&flag(obs,"published")&&obs.at("z_observed")==false
      &&obs.at("observed_axes")==Json::array({"x","y"})&&obs.at("reference_point")=="fc_imu"
      &&obs.at("frame_id")=="local_ned"&&obs.at("representative_stamp_ns")==report.at("last_sent_stamp_ns"),"lidar_latest_observation_rejected");
    const auto& params=report.at("parameters");
    require(params.at("EKF2_LID2_VER")==1&&params.at("EKF2_LID2_EN")==1,"lidar2d_firmware_required");
    const auto velocity=point(obs.at("velocity_ne_m_s"));const auto& window=report.at("stationary_window");
    const bool stationary_ready=flag(window,"ready")&&window.at("last_stamp_ns")==report.at("last_sent_stamp_ns")
      &&window.at("reset_counter")==obs.at("reset_counter")&&number(window.at("samples"),0,100)>=8
      &&number(window.at("span_s"),0,2)>=.9&&number(window.at("observed_s"),0,2)>=.75
      &&window.at("purpose")=="stationary_readiness_only"&&window.at("fc_input_filtered")==false;
    out.update({{"lidar_ready",true},{"lidar_gate","ready"},{"lidar_stationary_window_ready",stationary_ready},
      {"lidar_speed_m_s",stationary_ready?Json(number(window.at("speed_m_s"),0,100)):Json()},
      {"lidar_instant_speed_m_s",xy(velocity)},
      {"lidar_observation_age_s",observation_age},{"lidar_status_age_s",status_age},
      {"lidar_stamp_ns",report.at("last_sent_stamp_ns")}});
  }catch(const std::exception& e){out["lidar_gate"]=e.what();}
  return out;
}
}
