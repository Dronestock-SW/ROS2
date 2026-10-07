#include "sangwon_ai/service/snapshot.hpp"
#include "sangwon_ai/geometry.hpp"
#include "sangwon_ai/planner.hpp"

namespace sangwon::service {
namespace {
Vec3 xyz(const Json& value) {
  Vec3 p{value.at("x").get<double>(),value.at("y").get<double>(),value.at("z").get<double>()};
  require(finite(p),"INVALID_COORDINATE"); return p;
}
double number(const Json& value,const char* key,double lo,double hi) {
  const auto v=value.at(key).get<double>();
  require(std::isfinite(v)&&v>=lo&&v<=hi,"INVALID_PROFILE_VALUE"); return v;
}
std::string ref_key(const Json& ref) {
  return ref.at("id").get<std::string>()+"@"+ref.at("revision").get<std::string>();
}
const Json& resolve(const Json& ref,const Json& values,const char* id="id") {
  const Json* found=nullptr;
  for(const auto& value:values) if(value.at(id)==ref.at("id")&&value.at("revision")==ref.at("revision")) {
    require(!found,"REFERENCE_AMBIGUOUS"); found=&value;
  }
  require(found,"REFERENCE_MISMATCH"); return *found;
}
void approval(const Json& s,const Json& item,const std::string& kind,double now) {
  const auto& a=resolve(item.at("approval_ref"),s.at("approvals"));
  require(a.at("kind")==kind&&a.at("status")=="SYNTHETIC_ONLY"
    &&a.at("valid_for_profiles")==Json::array({"REPLAY"}),"REPLAY_APPROVAL_REQUIRED");
  require(utc_parse(a.at("approved_at"))<=now+0.25&&utc_parse(a.at("valid_until"))>now,"APPROVAL_EXPIRED");
  const auto id=item.contains("id")?"id":"label_point_id";
  require(std::find(a.at("targets").begin(),a.at("targets").end(),Json{{"id",item.at(id)},{"revision",item.at("revision")}})
    !=a.at("targets").end(),"APPROVAL_TARGET_MISMATCH");
}
Json artifact(const Json& s,const Json& config,const Json& ref) {
  require(config.contains("replay_profile_artifacts"),"PROFILE_ARTIFACTS_REQUIRED");
  const auto& meta=resolve(ref,s.at("profile_artifacts"));
  const auto& text=config.at("replay_profile_artifacts").at(ref_key(ref));
  require(text.is_string()&&digest(text.get<std::string>())==meta.at("sha256"),"PROFILE_HASH_MISMATCH");
  auto value=parse(text);
  require(value.at("id")==ref.at("id")&&value.at("revision")==ref.at("revision")
    &&value.at("approval_status")=="SYNTHETIC_ONLY"&&value.at("valid_for_profiles")==Json::array({"REPLAY"}),"PROFILE_BINDING_MISMATCH");
  return value;
}
PolygonVolume volume(const Json& value) {
  PolygonVolume out;
  const auto& p=value.at("polygon_xy_m");require(p.is_array()&&p.size()>=3&&p.size()<=256,"INVALID_MAP_GEOMETRY");
  for(const auto& v:p) {
    require(v.is_array()&&v.size()==2,"INVALID_MAP_GEOMETRY");
    out.polygon.push_back({v.at(0).get<double>(),v.at(1).get<double>(),0});
  }
  out.z_min=value.at("z_min_m");out.z_max=value.at("z_max_m");validate_volume(out);return out;
}
struct Map : FlightSpace {
  double body_radius{},body_z{};
  void segment(Vec3 a,Vec3 b,double radius,double vertical,bool ground=false) const {
    const auto error=space_segment_error(*this,a,b,radius,vertical,ground);
    require(!*error,error);
  }
};
}
CompiledSnapshot compile_replay_snapshot(const Json& s,const Json& config,double now) {
  require(s.at("profile")=="REPLAY"&&config.at("profile")=="REPLAY","PHYSICAL_OUTPUT_NOT_IMPLEMENTED");
  require(s.at("start_mode")=="AUTO_TAKEOFF","UNSUPPORTED_START_MODE");
  CompiledSnapshot out;auto& m=out.mission;m.approved_fixture=true;
  const auto takeoff=config.value("replay_takeoff_policy",std::string("OFFBOARD"));
  require(takeoff=="OFFBOARD"||takeoff=="PX4_AUTO_TAKEOFF","UNSUPPORTED_TAKEOFF_POLICY");
  if(takeoff=="PX4_AUTO_TAKEOFF") {
    require(config.value("approved_replay_takeoff_reference",std::string())=="WAREHOUSE_MAP_SYNTHETIC_ONLY",
      "NATIVE_TAKEOFF_REFERENCE_REQUIRED");
    out.motion.takeoff_policy=TakeoffPolicy::NativePx4;
  }
  m.takeoff_z_m=number(s,"takeoff_z_m",0.01,100);
  if(s.contains("start_yaw_deg")) m.start_yaw=number(s,"start_yaw_deg",-180,179.999999)*pi/180;
  const bool full=s.contains("map");
  if(!full) require(s.at("validation_scope")=="ALLOWLISTED_SYNTHETIC_ROUTE_ONLY"
    &&std::find(s.at("required_capabilities").begin(),s.at("required_capabilities").end(),Json("replay_waypoints_v1"))!=s.at("required_capabilities").end(),"UNSUPPORTED_SNAPSHOT_DIALECT");
  Map map;
  if(full) {
    out.validation_scope="ALLOWLISTED_SYNTHETIC_MAP_SCAN_ONLY";
    const auto& f=s.at("coordinate_frame");
    require(f.at("id")=="WAREHOUSE_MAP"&&f.at("length_unit")=="m"&&f.at("axes")=="RIGHT_HANDED_XY_Z_UP"
      &&f.at("z_reference")=="COMMON_WAREHOUSE_FLOOR"&&f.at("yaw_unit")=="deg"&&f.at("yaw_zero_axis")=="+X"
      &&f.at("yaw_positive")=="CCW_VIEWED_FROM_ABOVE"&&f.at("yaw_range")=="[-180,180)","UNSUPPORTED_COORDINATE_FRAME");
    require(s.at("policy").at("global_detour_enabled").is_boolean(),"UNSUPPORTED_POLICY");
    m.global_detour_enabled=s.at("policy").at("global_detour_enabled").get<bool>();
    require(s.at("policy").at("waypoints_required_in_order")==true
      &&s.at("policy").at("scan_reader_attempts")==3
      &&s.at("policy").at("scan_final_failure")=="RECORD_RETURN_STAGING_NEXT_IF_FLIGHT_HEALTHY"
      &&s.at("policy").at("mission_end")=="RETURN_LAUNCH_AND_LAND"
      &&s.at("policy").at("wifi_loss")=="CONTINUE_ACCEPTED_MISSION"
      &&s.at("policy").at("cancel")=="GROUND_ABORT_ASCENT_LAND_OTHER_RETURN"
      &&s.at("policy").at("manual_handover")=="LOCK_UNTIL_LANDED_NEW_MISSION","UNSUPPORTED_POLICY");
    const auto motion=artifact(s,config,s.at("motion_profile_ref"));auto& c=out.motion;
    c.xy_speed_mps=number(motion,"max_speed_xy_mps",0.01,1);c.z_speed_mps=number(motion,"max_speed_z_mps",0.01,1);
    c.accel_mps2=number(motion,"max_acceleration_mps2",0.01,1);c.xy_error_m=number(motion,"arrival_xy_m",0.001,0.5);
    c.z_error_m=number(motion,"arrival_z_m",0.001,0.5);c.yaw_error_rad=number(motion,"arrival_yaw_deg",0.1,30)*pi/180;
    c.stable_s=number(motion,"arrival_stable_s",0.1,10);c.lease_s=number(motion,"intent_lease_ms",100,1000)/1000;
    c.pose_age_s=number(motion,"sensor_max_age_ms",10,1000)/1000;c.validate();
    m.requested_speed_mps=c.xy_speed_mps;
    const auto body=artifact(s,config,s.at("body_profile_ref"));
    require(body.at("reference")=="PX4_BODY_ORIGIN"&&s.at("device_config").at("body_profile_ref")==s.at("body_profile_ref"),"BODY_REFERENCE_MISMATCH");
    map.body_radius=std::hypot(number(body,"length_m",0.01,2),number(body,"width_m",0.01,2))/2+number(body,"clearance_m",0.01,1);
    map.body_z=number(body,"height_m",0.01,2)/2+number(body,"clearance_m",0.01,1);
    const auto& transform=resolve(s.at("map_transform_ref"),s.at("transforms"));approval(s,transform,"CALIBRATION",now);
    require(transform.at("from_frame")=="WAREHOUSE_MAP"&&transform.at("to_frame")=="PX4_LOCAL_ENU"
      &&transform.at("binding_policy")=="REVALIDATE_PX4_LOCAL_ORIGIN_EACH_BOOT"
      &&transform.at("quaternion_xyzw")==Json::array({0,0,0,1})&&norm(xyz(transform.at("translation_m")))<1e-9,"UNSUPPORTED_REPLAY_TRANSFORM");
    const auto& data=s.at("map");approval(s,data,"MAP",now);
    for(const char* name:{"obstacles","ceilings"}) for(const auto& v:data.at(name)) map.forbidden.push_back(volume(v));
    for(const auto& v:data.at("altitude_zones")) map.altitude.push_back(volume(v));
    for(const auto& v:data.at("yaw_validation_zones")) {approval(s,v,"YAW_ZONE",now);map.yaw.push_back(volume(v));}
    for(const char* name:{"flight_boundary","no_fly_zones"}) {
      const auto& field=data.at(name);const bool provided=field.at("status")=="PROVIDED";
      require(provided?field.at("volumes").is_array():field.at("status")=="NOT_PROVIDED"&&field.at("volumes").is_null(),"INVALID_VOLUME_STATUS");
      if(provided) for(const auto& v:field.at("volumes")) (std::string(name)=="flight_boundary"?map.boundary:map.forbidden).push_back(volume(v));
      if(std::string(name)=="flight_boundary") map.boundary_provided=provided;
    }
    const auto& launch=config.at("approved_replay_launch_pose");out.launch={xyz(launch.at("position_m")),number(launch,"yaw_deg",-180,179.999999)*pi/180};
    require(std::abs(out.launch.p.z)<1e-9,"INVALID_REPLAY_LAUNCH");
    validate_space(map);m.space=std::make_shared<const FlightSpace>(map);
    if(m.global_detour_enabled) {
      out.validation_scope="ALLOWLISTED_SYNTHETIC_MAP_STATIC_DETOUR_ONLY";
      require(map.boundary_provided&&s.at("map").at("no_fly_zones").at("status")=="PROVIDED","DETOUR_BOUNDARY_REQUIRED");
      require(std::find(s.at("required_capabilities").begin(),s.at("required_capabilities").end(),Json("replay_static_detour_v1"))
          !=s.at("required_capabilities").end(),"DETOUR_CAPABILITY_REQUIRED");
      require(config.contains("approved_replay_planner"),"PLANNER_LIMITS_REQUIRED");
      const auto& planner=config.at("approved_replay_planner");auto& limits=m.planner;
      limits.resolution_m=number(planner,"resolution_m",0.02,1);
      limits.max_length_m=number(planner,"max_length_m",0.01,1000);
      limits.budget_s=number(planner,"budget_ms",1,100)/1000;
      require(planner.at("max_nodes").is_number_integer()&&planner.at("max_expansions").is_number_integer(),"INVALID_PLANNER_LIMITS");
      limits.max_nodes=static_cast<unsigned>(number(planner,"max_nodes",4,65536));
      limits.max_expansions=static_cast<unsigned>(number(planner,"max_expansions",1,65536));limits.validate();
    }
    m.clearance_xy_m=map.body_radius+out.motion.xy_error_m+out.motion.xy_speed_mps*out.motion.xy_speed_mps/(2*out.motion.accel_mps2);
    m.clearance_z_m=map.body_z+out.motion.z_error_m;
    map.segment(out.launch.p,{out.launch.p.x,out.launch.p.y,m.takeoff_z_m},m.clearance_xy_m,m.clearance_z_m,true);
  }
  Pose exit{{out.launch.p.x,out.launch.p.y,m.takeoff_z_m},m.start_yaw.value_or(out.launch.yaw)};
  std::set<std::string> ids;const auto& tasks=s.at("route_tasks");require(tasks.is_array()&&!tasks.empty()&&tasks.size()<=1000,"INVALID_TASKS");
  for(std::size_t index=0;index<tasks.size();++index) {
    const auto& t=tasks.at(index);const auto id=t.at("task_id").get<std::string>();require(!id.empty()&&id.size()<=64&&ids.insert(id).second,"INVALID_TASK_ID");
    Waypoint w;
    if(t.at("type")=="waypoint") {
      w.p=xyz(t.at("position_m"));w.hover_s=number(t,"hold_s",0,3600);
      if(t.contains("yaw_deg")) w.yaw=number(t,"yaw_deg",-180,179.999999)*pi/180;
      require(w.p.z>0,"INVALID_WAYPOINT");
      if(full) {const auto corner=Vec3{exit.p.x,exit.p.y,w.p.z};
        map.segment(exit.p,corner,m.clearance_xy_m,m.clearance_z_m);
        if(m.global_detour_enabled) {
          const auto route=plan_horizontal_route(map,corner,w.p,m.clearance_xy_m,m.clearance_z_m,m.planner);
          require(route.ok(),route.code);
        } else map.segment(corner,w.p,m.clearance_xy_m,m.clearance_z_m);}
      exit={w.p,w.yaw.value_or(std::atan2(w.p.y-exit.p.y,w.p.x-exit.p.x))};
    } else {
      require(full&&t.at("type")=="scan","UNSUPPORTED_TASK_TYPE");out.has_scan=true;
      const Json expected=index==0?Json{{"kind","LAUNCH"}}:Json{{"kind","PREVIOUS_TASK_EXIT"},{"task_id",tasks.at(index-1).at("task_id")}};
      require(t.at("approach_anchor")==expected,"SCAN_ANCHOR_MISMATCH");
      const Json* label=nullptr;const Json* workspace=nullptr;
      for(const auto& v:s.at("labels")) if(v.at("label_point_id")==t.at("label_point_id")) {require(!label,"REFERENCE_AMBIGUOUS");label=&v;}
      for(const auto& v:s.at("scan_workspaces")) if(v.at("id")==t.at("workspace_id")) {require(!workspace,"REFERENCE_AMBIGUOUS");workspace=&v;}
      require(label&&workspace,"REFERENCE_MISMATCH");approval(s,*workspace,"SCAN_SPACE",now);
      require(label->at("position_reference")=="QR_CENTER"&&workspace->at("geometry_semantics")=="PHYSICAL_ALLOWED_VOLUME"
        &&std::find(workspace->at("label_point_ids").begin(),workspace->at("label_point_ids").end(),t.at("label_point_id"))!=workspace->at("label_point_ids").end(),"SCAN_WORKSPACE_MISMATCH");
      ScanPlan plan;plan.task_id=id;plan.label_id=t.at("label_point_id");auto& c=plan.config;
      const auto sp=artifact(s,config,t.at("scan_profile_ref"));
      require(sp.at("distance_reference")=="SCANNER_OPTICAL_ORIGIN_TO_LABEL_PLANE"&&sp.at("scanner_attempts_max")==3,"UNSUPPORTED_SCAN_POLICY");
      const auto dmin=number(sp,"distance_min_m",0.4,0.6),d=number(sp,"distance_target_m",0.4,0.6),dmax=number(sp,"distance_max_m",0.4,0.6);
      require(dmin<d&&d<dmax,"INVALID_SCAN_DISTANCE");
      c.max_adjustment_m=std::min(c.max_adjustment_m,std::min(d-dmin,dmax-d));
      c.reader_s=number(sp,"scanner_attempt_timeout_s",0.1,30);c.camera_s=number(sp,"camera_qr_timeout_s",0.1,30);
      c.marker_acquire_s=number(sp,"marker_acquire_timeout_s",0.1,30);c.align_s=number(sp,"align_timeout_s",0.1,60);
      c.settle_s=number(sp,"settle_s",0.1,10);c.speed_mps=number(sp,"max_adjustment_speed_mps",0.001,0.1);
      c.marker_age_s=number(sp,"marker_max_age_ms",10,1000)/1000;c.reprojection_px=number(sp,"marker_reprojection_max_px",0.1,5);
      c.position_error_m=number(sp,"alignment_position_tolerance_m",0.001,0.05);c.yaw_error_rad=number(sp,"alignment_angle_tolerance_deg",0.1,10)*pi/180;
      const auto& n=label->at("outward_normal_map");require(n.is_array()&&n.size()==3,"INVALID_LABEL_NORMAL");
      Vec3 normal{n.at(0),n.at(1),n.at(2)};require(finite(normal)&&std::abs(norm(normal)-1)<1e-6&&std::abs(normal.z)<1e-6,"UNSUPPORTED_LABEL_NORMAL");
      const auto& dev=s.at("device_config");const auto& mount=resolve(dev.at("scanner_mount_ref"),s.at("transforms"));approval(s,mount,"CALIBRATION",now);
      require(mount.at("from_frame")=="SCANNER_OPTICAL_ORIGIN"&&mount.at("to_frame")=="PX4_BODY_ORIGIN"
        &&mount.at("quaternion_xyzw")==Json::array({0,0,0,1}),"UNSUPPORTED_SCANNER_MOUNT");
      const auto offset=xyz(mount.at("translation_m"));const double yaw=std::atan2(-normal.y,-normal.x);
      plan.reader={xyz(label->at("position_m"))+normal*d-Vec3{std::cos(yaw)*offset.x-std::sin(yaw)*offset.y,std::sin(yaw)*offset.x+std::cos(yaw)*offset.y,offset.z},yaw};
      // This contract has no camera-to-body extrinsic. Its explicit MOCK scope
      // permits coincident optical origins only in synthetic REPLAY observations.
      const auto& calibration=resolve(dev.at("camera_calibration_ref"),s.at("camera_calibrations"));
      require(calibration.at("extrinsics_scope")=="MOCK_OBSERVATIONS_ONLY"&&calibration.at("approval_status")=="SYNTHETIC_ONLY"
        &&calibration.at("valid_for_profiles")==Json::array({"REPLAY"}),"CAMERA_EXTRINSICS_REQUIRED");
      plan.camera=plan.reader;plan.staging={exit.p,yaw};
      const auto& marker=label->at("aruco");plan.dictionary=marker.at("dictionary");plan.marker_id=marker.at("marker_id");plan.marker_size_m=number(marker,"marker_size_m",0.001,1);
      const auto& layout=resolve(marker.at("qr_marker_transform_ref"),s.at("transforms"));approval(s,layout,"CALIBRATION",now);
      require(layout.at("from_frame")=="ARUCO_MARKER"&&layout.at("to_frame")=="QR_CENTER_FRAME","MARKER_LAYOUT_MISMATCH");
      plan.layout_ref=ref_key(marker.at("qr_marker_transform_ref"));plan.mounting_ref=ref_key(dev.at("scanner_mount_ref"));plan.calibration_ref=ref_key(dev.at("camera_calibration_ref"));
      const auto& rule=resolve(label->at("expected_qr_ref"),s.at("qr_match_rules"));
      require(rule.at("mode")=="JSON_FIELD_EQUALS"&&rule.at("label_point_id")==plan.label_id,"QR_RULE_MISMATCH");
      plan.expected_qr_fields=rule.at("expected_fields").get<std::map<std::string,std::string>>();
      const double radius=map.body_radius+c.position_error_m+c.speed_mps*c.speed_mps/(2*c.acceleration_mps2);
      plan.workspace=volume(*workspace);plan.space=m.space;
      plan.clearance_xy_m=radius+1e-5;plan.clearance_z_m=map.body_z+c.position_error_m+1e-5;
      auto allowed=volume_bounds(*plan.workspace);
      allowed.lo=allowed.lo+Vec3{plan.clearance_xy_m,plan.clearance_xy_m,plan.clearance_z_m};
      allowed.hi=allowed.hi-Vec3{plan.clearance_xy_m,plan.clearance_xy_m,plan.clearance_z_m};
      require(volume_covers(*plan.workspace,plan.reader.p,plan.reader.p,plan.clearance_xy_m,plan.clearance_z_m)
        &&volume_covers(*plan.workspace,plan.staging.p,plan.staging.p,plan.clearance_xy_m,plan.clearance_z_m),"SCAN_WORKSPACE_TOO_SMALL");
      // Check the actual vertical-first approach and egress, not their diagonal
      // or the workspace's bounding rectangle. Every correction is checked again.
      for(const auto& pair:{std::pair{plan.staging.p,plan.reader.p},std::pair{plan.reader.p,plan.staging.p}}) {
        const Vec3 corner{pair.first.x,pair.first.y,pair.second.z};
        require(volume_covers(*plan.workspace,pair.first,corner,plan.clearance_xy_m,plan.clearance_z_m)
          &&volume_covers(*plan.workspace,corner,pair.second,plan.clearance_xy_m,plan.clearance_z_m),"SCAN_WORKSPACE_OBSTRUCTED");
        map.segment(pair.first,corner,plan.clearance_xy_m,plan.clearance_z_m);
        map.segment(corner,pair.second,plan.clearance_xy_m,plan.clearance_z_m);
      }
      plan.centre_min=allowed.lo;plan.centre_max=allowed.hi;plan.space_validated=true;
      c.task_s=2*norm(plan.reader.p-plan.staging.p)/c.speed_mps+3*c.reader_s+c.camera_s+2*c.marker_acquire_s+2*c.align_s+5*c.settle_s+10;
      c.validate();w.p=plan.staging.p;w.yaw=yaw;w.scan=std::move(plan);exit={w.p,yaw};
    }
    m.points.push_back(std::move(w));
  }
  return out;
}
} // namespace sangwon::service
