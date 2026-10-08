// Static map validation for the sole native PX4 mission executor. No FC I/O.
#include "sangwon_ai/geometry.hpp"
#include "sangwon_ai/service/json.hpp"
#include <iostream>
#include <set>
using namespace sangwon;
using namespace sangwon::service;
namespace {
double number(const Json& x, double lo, double hi) {
  require(x.is_number()&&!x.is_boolean(),"INVALID_NUMBER");
  const double v=x.get<double>(); require(std::isfinite(v)&&v>=lo&&v<=hi,"NUMBER_OUTSIDE_LIMIT");return v;
}
PolygonVolume volume(const Json& x) {
  PolygonVolume v;
  const auto& p=x.at("polygon_xy_m");require(p.is_array()&&p.size()>=3&&p.size()<=256,"INVALID_POLYGON");
  for(const auto& q:p) {require(q.is_array()&&q.size()==2,"INVALID_VERTEX");v.polygon.push_back({number(q[0],-1000,1000),number(q[1],-1000,1000),0});}
  v.z_min=number(x.at("z_min_m"),-10,100);v.z_max=number(x.at("z_max_m"),-10,100);validate_volume(v);return v;
}
Vec3 xy_point(const Json& x,double z) {
  require(x.is_array()&&x.size()==2,"XY_REQUIRED");return {number(x[0],-1000,1000),number(x[1],-1000,1000),z};
}
}
int main() {
  try {
    std::string text,line;while(std::getline(std::cin,line)) {text+=line;require(text.size()<=2*1024*1024,"PLAN_INPUT_TOO_LARGE");}
    const auto input=parse(text);require(input.at("schema")=="sangwon-native-plan/1","INVALID_SCHEMA");
    const auto& map=input.at("map");const auto& request=input.at("assignment");
    require(map.at("frame")=="UWB_ANCHOR_LOCAL"&&map.at("unit")=="meter","FRAME_MISMATCH");
    require(map.at("layout_id")==request.at("anchor_layout_id"),"LAYOUT_MISMATCH");
    require(map.at("altitude_policy")=="px4_MIS_TAKEOFF_ALT","ALTITUDE_POLICY_MISMATCH");
    require(map.at("survey_status")=="SURVEYED"||map.at("survey_status")=="VIRTUAL","MAP_SURVEY_REQUIRED");
    const double climb=number(map.at("native_height_m"),.1,10);
    require(std::abs(climb-number(input.at("native_height_m"),.1,10))<.01,"NATIVE_HEIGHT_MISMATCH");
    const double launch_z=number(map.at("launch_body_floor_height_m"),0,1);
    // Native takeoff reference depends on FC height/home configuration. A
    // measured floor reference is mandatory; never infer it by adding climb.
    const double height=number(map.at("native_body_floor_height_m"),.1,11);
    require(height>launch_z,"NATIVE_HEIGHT_BELOW_LAUNCH");
    const int mag=input.at("mag_type").get<int>();
    require(mag==0||mag==1||mag==6,"UNSUPPORTED_HEADING_POLICY");
    require(mag==6||height>=1.6,"HEADING_HEIGHT_POLICY_INCOMPATIBLE");
    const double hm=number(map.at("clearance_xy_m"),.1,2),vm=number(map.at("clearance_z_m"),.05,1);
    const double maxleg=number(input.at("max_leg_m"),.01,2);
    FlightSpace space;space.boundary_provided=true;
    for(const auto& v:map.at("boundary")) space.boundary.push_back(volume(v));
    for(const auto& v:map.at("altitude_zones")) space.altitude.push_back(volume(v));
    for(const auto& v:map.at("yaw_zones")) space.yaw.push_back(volume(v));
    for(const auto& v:map.at("forbidden")) space.forbidden.push_back(volume(v));
    require(!space.boundary.empty()&&!space.altitude.empty()&&!space.yaw.empty(),"COMPLETE_MAP_REQUIRED");validate_space(space);
    Vec3 home=xy_point(input.at("launch_xy_m"),launch_z),prev={home.x,home.y,height};
    const auto check=[&](Vec3 a,Vec3 b,bool ground=false,double extra=0.) {
      const char* error=space_segment_error(space,a,b,hm+extra,vm,ground);require(!*error,error);
    };
    check(home,prev,true);
    auto tasks=request.at("route_tasks");require(tasks.is_array()&&!tasks.empty()&&tasks.size()<=20,"INVALID_TASKS");
    std::set<std::string> ids;
    const auto proof="native-map-sha256:"+digest(map.dump())+":"+digest(tasks.dump());
    for(auto& t:tasks) {
      const auto id=t.at("id").get<std::string>();require(!id.empty()&&ids.insert(id).second,"TASK_ID_REQUIRED");
      require(!t.contains("z")&&!t.contains("altitude_m"),"COMPANION_Z_FORBIDDEN");
      const auto kind=t.value("type",std::string("waypoint"));require(kind=="waypoint"||kind=="hover"||kind=="scan","TASK_TYPE_INVALID");
      Vec3 goal{number(t.at("x"),-1000,1000),number(t.at("y"),-1000,1000),height};
      if(t.contains("yaw_deg")) number(t.at("yaw_deg"),-180,180);
      if(t.contains("dwell_s")) number(t.at("dwell_s"),0,30);
      if(kind=="scan") {
        goal=xy_point(t.at("staging_xy_m"),height);
        require(t.at("marker_id").is_number_integer()&&t.at("marker_id").get<int>()>=0,"MARKER_REQUIRED");
        number(t.at("yaw_deg"),-180,180);
        for(const char* k:{"label_id","calibration_ref","mounting_ref"}) require(t.at(k).is_string()&&!t.at(k).get<std::string>().empty(),"SCAN_REFERENCE_REQUIRED");
        // Cover every bounded marker adjustment, not only the nominal staging point.
        check(goal,goal,false,.1);t["path_validation_ref"]=proof;
      }
      require(xy(goal-prev)<=maxleg,"NATIVE_LEG_TOO_LONG");check(prev,goal);prev=goal;
    }
    // Return retraces these same immutable segments. It does not shortcut obstacles.
    std::cout<<Json{{"schema","sangwon-native-plan/1"},{"validated",true},{"flight_authority",false},
      {"scope","SINGLE_ALTITUDE_STATIC_MAP"},{"plan_ref",proof},{"route_tasks",tasks}}.dump()<<'\n';return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
