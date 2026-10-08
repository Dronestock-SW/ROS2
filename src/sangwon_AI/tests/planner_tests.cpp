#include "sangwon_ai/planner.hpp"
#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include <functional>
#include <iostream>

using namespace sangwon;
#define CHECK(x) do {if(!(x)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #x);} while(false)
PolygonVolume box(double x1,double y1,double x2,double y2,double z1=0,double z2=3) {
  return {{{x1,y1,0},{x2,y1,0},{x2,y2,0},{x1,y2,0}},z1,z2};
}
FlightSpace warehouse() {
  FlightSpace s;s.altitude=s.yaw=s.boundary={box(-2,-2,6,4)};s.boundary_provided=true;return s;
}
void verify(const FlightSpace& s,const PlannedRoute& route,Vec3 start,Vec3 goal,double h=.15,double v=.1) {
  CHECK(route.ok()&&route.points.size()>=2);CHECK(norm(route.points.front()-start)<1e-8&&norm(route.points.back()-goal)<1e-8);
  double length=0;
  for(std::size_t i=1;i<route.points.size();++i) {
    CHECK(route.points[i].z==goal.z);CHECK(!*space_segment_error(s,route.points[i-1],route.points[i],h,v));
    length+=xy(route.points[i]-route.points[i-1]);
  }
  CHECK(std::abs(length-route.length_m)<1e-8);
}
void bounded_obstacle_detour() {
  auto s=warehouse();s.forbidden={box(1,-.6,2,.6)};
  const auto route=plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1);verify(s,route,{0,0,1},{4,0,1});
  CHECK(route.points.size()>2&&route.length_m>4&&route.expanded>0);
  auto reverse=plan_horizontal_route(s,{4,0,1},{0,0,1},.15,.1);verify(s,reverse,{4,0,1},{0,0,1});
}
void concave_and_corner_segments() {
  auto s=warehouse();
  const PolygonVolume u{{{-1,-1,0},{5,-1,0},{5,3,0},{3,3,0},{3,0,0},{1,0,0},{1,3,0},{-1,3,0}},0,3};
  s.altitude=s.yaw=s.boundary={u};
  const auto route=plan_horizontal_route(s,{0,2,1},{4,2,1},.15,.1);verify(s,route,{0,2,1},{4,2,1});
  CHECK(route.points.size()>2);
  s=warehouse();s.forbidden={box(.15,.15,.65,4),box(.15,-2,2,.05)};
  const auto blocked=plan_horizontal_route(s,{0,0,1},{1,1,1},.1,.1);
  CHECK(!blocked.ok()); // A diagonal cannot cut across the inflated corner.
}
void height_boundary_and_goal() {
  auto s=warehouse();s.forbidden={box(1,-.6,2,.6,0,.5)};
  const auto direct=plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1);verify(s,direct,{0,0,1},{4,0,1});CHECK(direct.points.size()==2);
  CHECK(plan_horizontal_route(s,{0,0,1},{4,0,1.1},.15,.1).code=="PLAN_INVALID_INPUT");
  s.boundary_provided=false;CHECK(plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1).code=="PLAN_BOUNDARY_REQUIRED");
  s=warehouse();s.forbidden={box(3,-.5,5,.5)};
  CHECK(plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1).code=="PLAN_GOAL_INVALID");
}
void limits_and_no_route() {
  auto s=warehouse();s.forbidden={box(1,-2,2,4)};
  PlannerLimits limits;limits.max_expansions=1;
  CHECK(plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1,limits).code=="PLAN_EXPANSION_LIMIT");
  limits.max_nodes=4;limits.max_expansions=4;
  CHECK(plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1,limits).code=="PLAN_NODE_LIMIT");
  limits=PlannerLimits{};limits.max_length_m=3;
  CHECK(plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1,limits).code=="PLAN_LENGTH_LIMIT");
  limits=PlannerLimits{};limits.resolution_m=0;bool failed=false;
  try {plan_horizontal_route(s,{0,0,1},{4,0,1},.15,.1,limits);} catch(const std::invalid_argument&) {failed=true;}CHECK(failed);
}
struct Rig {
  Config c;testing::FakePx4 px4;Runtime runtime;Mission m;FlightGuard guard;double now{};
  Rig(bool wall=false):runtime(c),guard(c,"planner-mission",runtime.boot_session(),1,1) {
    auto s=warehouse();s.forbidden={wall?box(1,-2,2,4):box(1,-.6,2,.6)};
    m=testing::example_mission();m.execution_id="planner-mission";m.points={{{4,0,1},-pi/2,0},{{4,2,1.4},0,0}};
    m.space=std::make_shared<const FlightSpace>(s);m.clearance_xy_m=.4;m.clearance_z_m=.2;m.global_detour_enabled=true;
    CHECK(runtime.start(m,px4.state,0));
  }
  void step() {
    now+=c.tick_s;px4.stamp(now);const auto intent=runtime.tick(px4.state,now);
    if(intent) {
      CHECK(guard.accept(*intent,now));
      if(intent->kind==IntentKind::Position) CHECK(!*space_segment_error(*m.space,px4.state.pose.p,intent->target.p,
        m.clearance_xy_m,m.clearance_z_m,runtime.phase()==Phase::Takeoff));
    }
    px4.apply(guard.tick(px4.state,now),c.tick_s,now);
  }
  void until(std::function<bool()> p,int maximum=10000) {for(int i=0;i<maximum&&!p();++i) step();CHECK(p());}
};
void runtime_required_points_and_return() {
  Rig r;std::size_t before=0;bool went_around=false;
  for(int i=0;i<10000&&r.runtime.phase()!=Phase::Complete;++i) {
    r.step();went_around|=std::abs(r.px4.state.pose.p.y)>.7;
    if(r.runtime.visited()!=before) {
      CHECK(r.runtime.visited()==before+1);CHECK(xy(r.px4.state.pose.p-r.m.points[before].p)<=r.c.xy_error_m);
      before=r.runtime.visited();
    }
  }
  CHECK(went_around&&before==2&&r.runtime.phase()==Phase::Complete&&r.px4.state.landed);
  CHECK(xy(r.px4.state.pose.p)<r.c.xy_error_m&&r.runtime.route_revision()>1);
}
void runtime_wait_budget_and_return_failure() {
  Rig r(true);r.until([&]{return r.runtime.phase()==Phase::Travel;});const auto began=r.now;
  r.until([&]{return r.runtime.returning();});CHECK(r.now-began>=9.9&&r.now-began<10.5);
  CHECK(r.runtime.visited()==0&&r.runtime.reason()=="BLOCKED_TIMEOUT");
  r.px4.state.return_route_valid=false;r.step();CHECK(r.runtime.phase()==Phase::Landing);
}
void runtime_cancel_and_manual_preemption() {
  Rig r;r.until([&]{return r.px4.state.pose.p.x>2.6&&r.runtime.phase()==Phase::Travel;});
  CHECK(r.runtime.visited()==0);CHECK(r.runtime.command(Command::Cancel));r.step();CHECK(r.runtime.returning());
  r.until([&]{return r.runtime.phase()==Phase::Complete;});CHECK(r.runtime.visited()==0&&xy(r.px4.state.pose.p)<r.c.xy_error_m);
  Rig manual;manual.until([&]{return manual.runtime.phase()==Phase::Travel;});
  manual.px4.state.rc_override=true;manual.step();CHECK(manual.runtime.phase()==Phase::Manual);
  manual.px4.state.rc_override=false;manual.step();CHECK(manual.runtime.phase()==Phase::Manual&&!manual.runtime.command(Command::Resume));
}
int main() {
  const std::vector<std::pair<const char*,void(*)()>> cases={{"obstacle_detour",bounded_obstacle_detour},
    {"concave_and_corners",concave_and_corner_segments},{"height_boundary_goal",height_boundary_and_goal},
    {"limits",limits_and_no_route},{"required_points_return",runtime_required_points_and_return},
    {"wait_return_failure",runtime_wait_budget_and_return_failure},{"cancel_manual",runtime_cancel_and_manual_preemption}};
  unsigned failures=0;
  for(const auto& [name,fn]:cases) {try {fn();std::cout<<"PASS "<<name<<'\n';}catch(const std::exception& e) {++failures;std::cerr<<"FAIL "<<name<<' '<<e.what()<<'\n';}}
  return failures?1:0;
}
