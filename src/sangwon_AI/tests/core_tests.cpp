#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/guard.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include "sangwon_ai/frames.hpp"
#include <functional>
#include <iostream>
#include <limits>

using namespace sangwon;
#define CHECK(condition) do { if(!(condition)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #condition); } while(false)
template<class F> void throws(F f) { bool caught=false; try {f();} catch(const std::exception&) {caught=true;} CHECK(caught); }
struct Rig {
  Config c; Runtime runtime; testing::FakePx4 px4; Mission m;
  FlightGuard guard; double now{};
  Rig():runtime(c),m(testing::example_mission()),guard(c,m.execution_id,runtime.boot_session(),1,1) {
    CHECK(runtime.start(m,px4.state,now));
  }
  void step(bool runtime_alive=true) {
    now+=c.tick_s; px4.stamp(now);
    if(runtime_alive) {
      auto i=runtime.tick(px4.state,now); if(i) CHECK(guard.accept(*i,now));
    }
    px4.apply(guard.tick(px4.state,now),c.tick_s,now);
  }
  void until(std::function<bool()> pred,int max=5000) { for(int n=0;n<max&&!pred();n++) step(); CHECK(pred()); }
  void travel() { until([this]{return runtime.phase()==Phase::Travel&&px4.state.pose.p.x>0.3;}); }
};
Intent intent(double now=0) {
  return {1,"e","b",1,1,1,1,IntentKind::Hold,{{0,0,1},0},now,now+0.5};
}
State airborne(double t=0) {
  testing::FakePx4 fake; fake.stamp(t);
  auto s=fake.state; s.mode=Mode::Offboard; s.armed=true; s.landed=false; s.pose.p.z=1; return s;
}
void trajectory_limits() {
  for(double distance:{0.0,0.01,0.4,2.0,10.0}) for(double speed:{0.1,0.5,1.0}) {
    ScalarProfile p(distance,speed,0.4); double last=0,last_v=0; const double dt=0.001;
    for(double t=0;t<p.duration()+dt;t+=dt) {
      auto s=p.sample(t); CHECK(s.position>=last-1e-8); CHECK(s.position<=distance+1e-8);
      CHECK(s.speed<=speed+1e-8); CHECK(std::abs(s.speed-last_v)<=0.4*dt+1e-7);
      last=s.position; last_v=s.speed;
    }
    CHECK(p.sample(p.duration()).position==distance); CHECK(p.sample(p.duration()).speed==0);
  }
  throws([]{ScalarProfile p(1,0,1);});
  CHECK(std::abs(stopping_distance(1,0.5,0.2,0.1)-1.3)<1e-8);
}
void yaw_short_path() {
  Config c; Segment s({{},179*pi/180},{ {},-179*pi/180},0.5,c);
  CHECK(s.duration()<1); CHECK(std::abs(wrap(s.sample(100).yaw+179*pi/180))<1e-8);
}
void coordinate_transform() {
  const auto p=MapToEnu{{10,20,-1},pi/2,1}.apply({{2,3,4},0});
  CHECK(std::abs(p.p.x-7)<1e-9); CHECK(std::abs(p.p.y-22)<1e-9);
  CHECK(p.p.z==3); CHECK(std::abs(p.yaw-pi/2)<1e-9);
  throws([]{MapToEnu{}.apply({});});
}
void profile_isolation() {
  Config c; c.profile=Profile::Flight; throws([&]{Runtime r(c);});
  c.profile=Profile::Sitl; throws([&]{Runtime r(c);});
  c.profile=Profile::Replay; c.accel_mps2=std::numeric_limits<double>::quiet_NaN();
  throws([&]{Runtime r(c);});
}
void input_validation() {
  testing::FakePx4 p; auto m=testing::example_mission();
  m.points[0].p.x=std::numeric_limits<double>::infinity(); Runtime r; CHECK(!r.start(m,p.state,0));
  m=testing::example_mission(); m.requested_speed_mps=1; CHECK(!r.start(m,p.state,0));
  m=testing::example_mission(); m.approved_fixture=false; CHECK(!r.start(m,p.state,0));
  m=testing::example_mission(); p.state.pose_quality=Quality::Unknown; CHECK(!r.start(m,p.state,0));
}
void guard_metadata() {
  FlightGuard g({ },"e","b",1,1); auto i=intent(); CHECK(g.accept(i,0)); CHECK(!g.accept(i,0));
  i.sequence++; i.frame_epoch=2; CHECK(!g.accept(i,0)); i.frame_epoch=1;
  i.boot_session="old"; CHECK(!g.accept(i,0)); i.boot_session="b";
  i.generation=0; CHECK(!g.accept(i,0)); i.generation=1;
  i.target.p.x=std::numeric_limits<double>::quiet_NaN(); CHECK(!g.accept(i,0));
}
void guard_expiry() {
  FlightGuard g({},"e","b",1,1); CHECK(g.accept(intent(),0));
  CHECK(g.tick(airborne(),0).kind==OutputKind::Position);
  CHECK(g.tick(airborne(0.5),0.5).kind==OutputKind::RequestLand);
  auto newer=intent(0.6); newer.sequence=2; CHECK(!g.accept(newer,0.6));
  CHECK(g.tick(airborne(7),7).kind==OutputKind::Release);
}
void guard_precedence() {
  FlightGuard g({},"e","b",1,1); CHECK(g.accept(intent(),0));
  auto s=airborne(); s.rc_override=true;
  CHECK(g.tick(s,0).kind==OutputKind::Release);
  auto newer=intent(0.1); newer.sequence=2; CHECK(!g.accept(newer,0.1));
}
void guard_quality() {
  FlightGuard g({},"e","b",1,1); CHECK(g.accept(intent(),0));
  auto s=airborne(); s.pose_quality=Quality::Invalid;
  CHECK(g.tick(s,0).kind==OutputKind::RequestLand);
}
void guard_no_airborne_disarm() {
  FlightGuard g({},"e","b",1,1); auto i=intent(); i.kind=IntentKind::AbortGround;
  CHECK(g.accept(i,0)); CHECK(g.tick(airborne(),0).kind==OutputKind::RequestLand);
}
void nominal_mission() {
  Rig r; r.until([&]{return r.runtime.phase()==Phase::Complete;});
  CHECK(r.runtime.visited()==2); CHECK(r.px4.state.landed&&!r.px4.state.armed);
  CHECK(xy(r.px4.state.pose.p)<0.2); CHECK(std::abs(wrap(r.px4.state.pose.yaw))<10*pi/180);
}
void no_visit_on_overshoot() {
  Rig r; r.travel(); r.px4.state.pose.p.x=2.5; r.px4.state.velocity.x=0.4;
  r.step(); CHECK(r.runtime.visited()==0); CHECK(r.runtime.max_overshoot_m()>=0.49);
  r.until([&]{return r.runtime.phase()==Phase::Complete;}); CHECK(r.runtime.visited()==2);
}
void rc_latched() {
  Rig r; r.travel(); r.px4.state.rc_override=true; r.step(); CHECK(r.runtime.phase()==Phase::Manual);
  CHECK(!r.runtime.command(Command::Resume)); r.px4.state.rc_override=false;
  r.step(); CHECK(r.runtime.phase()==Phase::Manual);
}
void battery_return() {
  Rig r; r.travel(); r.px4.state.battery=0.3; r.step(); CHECK(r.runtime.returning());
  CHECK(!r.runtime.command(Command::Pause)); r.px4.state.battery=0.9;
  r.until([&]{return r.runtime.phase()==Phase::Complete;}); CHECK(r.runtime.visited()==0);
}
void logging_return() {
  Rig r; r.travel(); r.px4.state.logging_ok=false; r.step(); CHECK(r.runtime.returning());
}
void position_recovery() {
  Rig r; r.travel(); r.px4.state.observation_valid=false;
  for(int i=0;i<50;i++) r.step(); CHECK(r.runtime.recovering()); CHECK(r.runtime.visited()==0);
  r.px4.state.observation_valid=true;
  for(int i=0;i<22;i++) r.step(); CHECK(!r.runtime.recovering());
  r.until([&]{return r.runtime.phase()==Phase::Complete;}); CHECK(r.runtime.visited()==2);
}
void position_timeout() {
  Rig r; r.travel(); r.px4.state.observation_valid=false;
  for(int i=0;i<162;i++) r.step(); CHECK(r.runtime.phase()==Phase::Landing);
  CHECK(r.px4.state.mode==Mode::Land);
}
void px4_cannot_hold() {
  Rig r; r.travel(); r.px4.state.pose_quality=Quality::Invalid; r.step();
  CHECK(r.px4.state.mode==Mode::Land);
}
void pause_recovery_context() {
  Rig r; r.travel(); CHECK(r.runtime.command(Command::Pause)); r.step();
  r.px4.state.observation_valid=false; for(int i=0;i<20;i++) r.step();
  r.px4.state.observation_valid=true; for(int i=0;i<22;i++) r.step();
  CHECK(r.runtime.paused()); CHECK(!r.runtime.recovering());
  CHECK(r.runtime.command(Command::Resume)); r.step(); CHECK(!r.runtime.paused());
}
void cancel_ascent() {
  Rig r; r.until([&]{return r.runtime.phase()==Phase::Takeoff&&r.px4.state.pose.p.z>0.2;});
  CHECK(r.runtime.command(Command::Cancel)); r.step(); CHECK(r.px4.state.mode==Mode::Land);
}
void blocked_return_land() {
  Rig r; r.travel(); r.px4.state.blocked=true;
  for(int i=0;i<202;i++) r.step(); CHECK(r.runtime.returning());
  CHECK(r.px4.state.mode==Mode::Offboard);
  r.px4.state.return_route_valid=false; r.step(); CHECK(r.px4.state.mode==Mode::Land);
}
void runtime_loss_guard_alive() {
  Rig r; r.travel(); for(int i=0;i<12;i++) r.step(false);
  CHECK(r.px4.state.mode==Mode::Land);
}
void goal_stall() {
  Rig r; r.travel(); r.px4.freeze_motion=true; r.px4.state.velocity={};
  for(int i=0;i<110;i++) r.step(); CHECK(r.runtime.returning());
}
void land_sticky() {
  Rig r; r.travel(); CHECK(r.runtime.command(Command::Land)); r.step();
  CHECK(!r.runtime.command(Command::Resume)); CHECK(r.runtime.phase()==Phase::Landing);
  r.px4.state.observation_valid=true; r.step(); CHECK(r.px4.state.mode==Mode::Land);
}
void yaw_reset_land() {
  Rig r; r.travel(); r.px4.state.frame_epoch=2; r.step(); CHECK(r.px4.state.mode==Mode::Land);
}
void utc_not_used() {
  Rig r; r.step(); throws([&]{r.runtime.tick(r.px4.state,-1);});
}
void arrival_requires_stop() {
  Config c; auto s=airborne(); s.pose={{2,3,1},pi/2}; s.velocity={0.3,0,0};
  CHECK(!at_pose(s,s.pose,c)); s.velocity={}; CHECK(at_pose(s,s.pose,c));
}
void recovery_budget_not_reset() {
  Rig r; r.travel(); r.px4.state.observation_valid=false;
  for(int n=0;n<160;n++) { r.px4.state.observation_valid=(n%15<3); r.step(); }
  for(int n=0;n<6;n++) { r.px4.state.observation_valid=false; r.step(); }
  CHECK(r.px4.state.mode==Mode::Land);
}
void low_battery_during_recovery() {
  Rig r; r.travel(); r.px4.state.observation_valid=false; r.step();
  r.px4.state.battery=0.2; r.step(); CHECK(r.runtime.recovering()&&r.runtime.returning());
  CHECK(r.px4.state.mode==Mode::Offboard);
  r.px4.state.observation_valid=true; for(int i=0;i<23;i++) r.step();
  CHECK(!r.runtime.recovering()&&r.runtime.returning());
}
void land_without_confirmation() {
  FlightGuard g({},"e","b",1,1); auto i=intent(); i.kind=IntentKind::Land;
  CHECK(g.accept(i,0)); CHECK(g.tick(airborne(),0).kind==OutputKind::RequestLand);
  CHECK(g.tick(airborne(6.1),6.1).kind==OutputKind::Release);
}
void pause_resets_hover() {
  Rig r; r.until([&]{return r.runtime.phase()==Phase::Dwell;});
  for(int i=0;i<20;i++) r.step(); CHECK(r.runtime.visited()==0);
  CHECK(r.runtime.command(Command::Pause)); for(int i=0;i<40;i++) r.step();
  CHECK(r.runtime.command(Command::Resume)); for(int i=0;i<20;i++) r.step();
  CHECK(r.runtime.visited()==0);
  r.until([&]{return r.runtime.visited()==1;});
}
void reject_unowned_start() {
  testing::FakePx4 p; p.state.armed=true; p.state.landed=false;
  Runtime r; CHECK(!r.start(testing::example_mission(),p.state,0));
}
int main() {
  const std::vector<std::pair<const char*,void(*)()>> tests={
    {"trajectory_limits",trajectory_limits},{"yaw_short_path",yaw_short_path},
    {"coordinate_transform",coordinate_transform},
    {"profile_isolation",profile_isolation},{"input_validation",input_validation},
    {"guard_metadata",guard_metadata},{"guard_expiry",guard_expiry},
    {"guard_precedence",guard_precedence},{"guard_quality",guard_quality},
    {"guard_no_airborne_disarm",guard_no_airborne_disarm},{"nominal_mission",nominal_mission},
    {"no_visit_on_overshoot",no_visit_on_overshoot},{"rc_latched",rc_latched},
    {"battery_return",battery_return},{"logging_return",logging_return},
    {"position_recovery",position_recovery},{"position_timeout",position_timeout},
    {"px4_cannot_hold",px4_cannot_hold},{"pause_recovery_context",pause_recovery_context},
    {"cancel_ascent",cancel_ascent},{"blocked_return_land",blocked_return_land},
    {"runtime_loss_guard_alive",runtime_loss_guard_alive},{"goal_stall",goal_stall},
    {"land_sticky",land_sticky},{"yaw_reset_land",yaw_reset_land},{"clock_backwards",utc_not_used},
    {"arrival_requires_stop",arrival_requires_stop},{"recovery_budget_not_reset",recovery_budget_not_reset},
    {"low_battery_during_recovery",low_battery_during_recovery},{"land_without_confirmation",land_without_confirmation},
    {"pause_resets_hover",pause_resets_hover},{"reject_unowned_start",reject_unowned_start}};
  unsigned failed=0;
  for(const auto& [name,fn]:tests) {
    try {fn(); std::cout<<"PASS "<<name<<'\n';}
    catch(const std::exception& e) {failed++; std::cerr<<"FAIL "<<name<<" "<<e.what()<<'\n';}
  }
  std::cout<<tests.size()-failed<<'/'<<tests.size()<<" passed\n"; return failed?1:0;
}
