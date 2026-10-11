#include "sangwon_ai/geometry.hpp"
#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include <functional>
#include <iostream>
#include <limits>

using namespace sangwon;
#define CHECK(x) do { if(!(x)) throw std::runtime_error(#x); } while(false)
template<class F> void rejects(F f) { bool rejected=false;try{f();}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected); }
PolygonVolume rectangle(double x1,double y1,double x2,double y2,double z1=0,double z2=3) {
  return {{{x1,y1,0},{x2,y1,0},{x2,y2,0},{x1,y2,0}},z1,z2};
}
FlightSpace open_space() { FlightSpace s;s.altitude=s.yaw=s.boundary={rectangle(-1,-1,5,5)};s.boundary_provided=true;return s; }
void polygon_validation() {
  const PolygonVolume triangle{{{0,0,0},{3,0,0},{0,3,0}},0,3};validate_volume(triangle);
  auto reverse=triangle;std::reverse(reverse.polygon.begin(),reverse.polygon.end());validate_volume(reverse);
  CHECK(volume_covers(triangle,{.5,.5,1},{.6,.7,1},.1,.1));
  CHECK(volume_covers(reverse,{.5,.5,1},{.6,.7,1},.1,.1));
  auto invalid=rectangle(0,0,2,2);std::swap(invalid.polygon[1],invalid.polygon[2]);rejects([&]{validate_volume(invalid);});
  invalid=rectangle(0,0,2,2);invalid.polygon[1]=invalid.polygon[0];rejects([&]{validate_volume(invalid);});
  invalid={{{0,0,0},{2,0,0},{-1,0,0},{-1,2,0},{0,2,0}},0,3};rejects([&]{validate_volume(invalid);});
  invalid=triangle;invalid.polygon[0].x=std::numeric_limits<double>::quiet_NaN();rejects([&]{validate_volume(invalid);});
  invalid=triangle;invalid.z_max=invalid.z_min;rejects([&]{validate_volume(invalid);});
}
void concave_segment_and_clearance() {
  PolygonVolume u{{{0,0,0},{4,0,0},{4,4,0},{3,4,0},{3,1,0},{1,1,0},{1,4,0},{0,4,0}},0,3};validate_volume(u);
  CHECK(volume_covers(u,{.5,2,1},{.5,3,1},.2,.1));
  CHECK(volume_covers(u,{.5,2,1},{.5,2,1},.2,.1));
  CHECK(volume_covers(u,{3.5,2,1},{3.5,2,1},.2,.1));
  CHECK(!volume_covers(u,{.5,2,1},{3.5,2,1},.2,.1));
  CHECK(!volume_covers(u,{2,2,1},{2,2,1},0,.1));
  CHECK(!volume_covers(u,{.2,2,1},{.2,3,1},.2,.1)); // Tangency is insufficient clearance.
}
void slanted_obstacle_and_height_clipping() {
  PolygonVolume triangle{{{0,0,0},{2,0,0},{0,2,0}},0,1};validate_volume(triangle);
  CHECK(!volume_intersects(triangle,{1.6,1.6,.5},{1.7,1.7,.5},.1,.1)); // Inside AABB, outside obstacle.
  CHECK(volume_intersects(triangle,{-1,.5,.5},{2,.5,.5},.1,.1));
  CHECK(!volume_intersects(triangle,{-.5,.5,2},{1,.5,2},.1,.1));
  const auto pillar=rectangle(.5,-.2,1,.2,0,1);
  CHECK(!volume_intersects(pillar,{0,0,0},{2,0,6},0,.1));
  CHECK(volume_intersects(pillar,{0,0,0},{2,0,2},0,.1));
  CHECK(volume_intersects(pillar,{.4,-1,.5},{.4,1,.5},.1,.1));
  CHECK(!volume_intersects(pillar,{.399,-1,.5},{.399,1,.5},.1,.1));
}
void zones_and_ground() {
  auto s=open_space();validate_space(s);
  CHECK(!*space_segment_error(s,{0,0,1},{2,0,1},.1,.1));
  CHECK(!*space_segment_error(s,{0,0,0},{0,0,1},.1,.1,true));
  CHECK(std::string(space_segment_error(s,{0,0,0},{0,0,1},.1,.1))=="ALTITUDE_ZONE_UNCOVERED");
  s.altitude[0].z_min=.5;
  CHECK(std::string(space_segment_error(s,{0,0,0},{0,0,1},.1,.1,true))=="ALTITUDE_ZONE_UNCOVERED");
  s=open_space();s.yaw={rectangle(-1,-1,.4,2),rectangle(.6,-1,5,2)};
  CHECK(std::string(space_segment_error(s,{0,0,1},{2,0,1},.1,.1))=="YAW_ZONE_UNCOVERED");
  s=open_space();s.boundary={rectangle(-1,-1,.5,2)};
  CHECK(std::string(space_segment_error(s,{0,0,1},{2,0,1},.1,.1))=="FLIGHT_BOUNDARY_EXCEEDED");
  s.boundary.clear();rejects([&]{validate_space(s);});
}
void runtime_rechecks_map_and_return() {
  Config c;testing::FakePx4 px4;Runtime runtime(c);auto s=open_space();
  s.forbidden={rectangle(.8,-.3,1.2,.3)};
  auto m=testing::example_mission();m.points={{{2,0,1},0,0,{}}};
  m.space=std::make_shared<const FlightSpace>(s);m.clearance_xy_m=m.clearance_z_m=.1;
  CHECK(runtime.start(m,px4.state,0));FlightGuard guard(c,m.execution_id,runtime.boot_session(),1,1);
  double now=0;
  for(int i=0;i<1000&&runtime.phase()!=Phase::Complete;++i) {
    now+=c.tick_s;px4.stamp(now);const auto intent=runtime.tick(px4.state,now);
    if(intent) { CHECK(guard.accept(*intent,now));if(intent->kind==IntentKind::Position) CHECK(intent->target.p.x<.5); }
    px4.apply(guard.tick(px4.state,now),c.tick_s,now);
  }
  CHECK(runtime.phase()==Phase::Complete&&runtime.visited()==0);
  CHECK(runtime.reason()=="STATIC_MAP_ROUTE_OBSTRUCTED");
}
int main() {
  const std::vector<std::pair<const char*,void(*)()>> tests={{"polygon_validation",polygon_validation},
    {"concave_segment_clearance",concave_segment_and_clearance},{"obstacle_height_clipping",slanted_obstacle_and_height_clipping},
    {"zones_ground",zones_and_ground},{"runtime_map_recheck",runtime_rechecks_map_and_return}};
  unsigned failures=0;
  for(const auto& [name,fn]:tests) {try {fn();std::cout<<"PASS "<<name<<'\n';}
    catch(const std::exception& e) {++failures;std::cerr<<"FAIL "<<name<<' '<<e.what()<<'\n';}}
  return failures?1:0;
}
