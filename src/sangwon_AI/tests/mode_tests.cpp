#include "sangwon_ai/mode_manager.hpp"
#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include <functional>
#include <iostream>
#include <limits>

using namespace sangwon;
#define CHECK(c) do {if(!(c)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #c);} while(false)
const Pose target{{0,0,1},0};
struct Transaction {
  Config c; testing::FakePx4 px4; ModeManager manager{c,"e","b",1,1};
  void start(ControlOperation op=ControlOperation::Offboard) {
    CHECK(manager.begin(op,target,px4.state,0));CHECK(manager.output(0,target,true).request.has_value());
  }
  void observe(double now) {px4.stamp(now);manager.observe(px4.state,now);}
};
void ack_is_not_mode() {
  Transaction t;t.start();
  t.px4.state.control_reply=ControlReply{*t.manager.request(),ReplyResult::Accepted,.1};t.observe(.1);
  CHECK(t.manager.pending()&&t.manager.ack_accepted());
  t.px4.state.mode=Mode::Offboard;t.observe(.2);CHECK(t.manager.state()==TransitionState::Confirmed);
}
void state_confirms_without_ack() {
  Transaction t;t.start();t.px4.state.mode=Mode::Offboard;t.observe(.1);
  CHECK(t.manager.state()==TransitionState::Confirmed&&!t.manager.ack_accepted());
}
void reply_binding() {
  for(int mutation=0;mutation<8;++mutation) {
    Transaction t;t.start();auto request=*t.manager.request();
    if(mutation==0) ++request.request_id;
    if(mutation==1) request.execution_id="other";
    if(mutation==2) request.boot_session="previous";
    if(mutation==3) ++request.frame_epoch;
    if(mutation==4) ++request.transform_revision;
    if(mutation==5) request.operation=ControlOperation::Arm;
    if(mutation==6) request.target.p.z+=1;
    if(mutation==7) request.deadline_s+=1;
    t.px4.state.control_reply=ControlReply{request,ReplyResult::Rejected,.1};t.observe(.1);
    CHECK(t.manager.pending());
  }
  Transaction t;t.start();t.px4.state.control_reply=ControlReply{*t.manager.request(),ReplyResult::Rejected,.1};
  t.observe(.1);CHECK(t.manager.state()==TransitionState::Rejected);
}
void stale_reply_and_original_deadline() {
  Transaction t;t.start();const auto request=*t.manager.request();
  t.px4.state.control_reply=ControlReply{request,ReplyResult::Rejected,-.01};t.observe(.1);CHECK(t.manager.pending());
  t.px4.state.control_reply=ControlReply{request,ReplyResult::Accepted,5.99};t.observe(5.99);
  CHECK(t.manager.pending()&&t.manager.request()->deadline_s==6);
  t.px4.state.mode=Mode::Offboard;t.observe(6);CHECK(t.manager.state()==TransitionState::TimedOut);
}
void bounded_retry_and_continuous_stream() {
  Transaction t;t.start();const auto request=*t.manager.request();
  for(int n=1;n<120;++n) {
    const double now=n*.05;t.observe(now);auto out=t.manager.output(now,target,true);
    CHECK(out.stream_position);CHECK(out.target.p.z==1);
    if(out.request) CHECK(out.request->request_id==request.request_id&&out.request->deadline_s==request.deadline_s);
  }
  CHECK(t.manager.attempts()==3);t.observe(6.01);CHECK(t.manager.failed());
}
void arm_requires_observation() {
  Transaction t;t.px4.state.mode=Mode::Takeoff;t.start(ControlOperation::Arm);
  t.px4.state.control_reply=ControlReply{*t.manager.request(),ReplyResult::Accepted,.1};t.observe(.1);CHECK(t.manager.pending());
  t.px4.state.armed=true;t.observe(.2);CHECK(t.manager.state()==TransitionState::Confirmed);
}
void unexpected_mode_and_pending_conflict() {
  Transaction t;t.start();CHECK(!t.manager.begin(ControlOperation::Takeoff,target,t.px4.state,.1));
  t.px4.state.mode=Mode::Offboard;t.observe(.1);t.px4.state.mode=Mode::Manual;t.observe(.2);
  CHECK(t.manager.unexpected_mode());
}
void hold_requires_correlated_completion() {
  for(int mutation=0;mutation<4;++mutation) {
    Transaction t;t.start(ControlOperation::Takeoff);auto request=*t.manager.request();
    t.px4.state.mode=Mode::Takeoff;t.observe(.1);CHECK(t.manager.state()==TransitionState::Confirmed);
    t.px4.state.mode=Mode::Hold;t.px4.state.global_position_quality=Quality::Valid;
    auto completion=ControlCompletion{request,.2};
    if(mutation==0) ++completion.request.request_id;
    if(mutation==1) completion.observed_s=-1;
    if(mutation==2) t.px4.state.global_position_quality=Quality::Unknown;
    t.px4.state.takeoff_completion=completion;t.observe(.2);
    CHECK(t.manager.unexpected_mode()==(mutation!=3));
    if(mutation==3) CHECK(t.manager.expected_mode()==Mode::Hold&&t.manager.takeoff_completed());
  }
}
Config native_config() {Config c;c.takeoff_policy=TakeoffPolicy::NativePx4;return c;}
struct NativeRig {
  Config c{native_config()};Runtime runtime{c};testing::FakePx4 px4;
  Mission mission{testing::example_mission()};
  FlightGuard guard{c,mission.execution_id,runtime.boot_session(),1,1};
  double now{};Output out;unsigned takeoff_commands{},offboard_commands{},land_commands{};
  NativeRig() {CHECK(runtime.start(mission,px4.state,now));}
  void step(bool alive=true) {
    now+=c.tick_s;px4.stamp(now);
    if(alive) {
      if(guard.released()||guard.landing_locked()) runtime.control_fault(guard.fault_reason(),guard.released());
      const auto i=runtime.tick(px4.state,now);if(i) CHECK(guard.accept(*i,now));
    }
    out=guard.tick(px4.state,now);
    takeoff_commands+=out.kind==OutputKind::RequestTakeoff;
    offboard_commands+=out.kind==OutputKind::RequestOffboard;
    land_commands+=out.kind==OutputKind::RequestLand;
    CHECK(!(out.kind==OutputKind::GroundDisarm&&!px4.state.landed));
    if(runtime.phase()==Phase::Takeoff) CHECK(out.kind!=OutputKind::Position);
    px4.apply(out,c.tick_s,now);
  }
  void until(std::function<bool()> predicate,int max=6000) {
    for(int n=0;n<max&&!predicate();++n) step();CHECK(predicate());
  }
};
void native_nominal() {
  NativeRig r;r.until([&]{return r.runtime.phase()==Phase::Complete;});
  CHECK(r.runtime.visited()==2&&r.px4.state.landed&&!r.px4.state.armed);
  CHECK(r.takeoff_commands==1&&r.offboard_commands==1&&r.land_commands==1);
}
void native_automatic_hold() {
  NativeRig r;r.px4.takeoff_to_hold=true;r.px4.state.global_position_quality=Quality::Valid;
  r.until([&]{return r.runtime.phase()==Phase::Complete;});CHECK(r.runtime.visited()==2);
}
void native_completion_missing() {
  NativeRig r;r.px4.omit_takeoff_completion=true;r.until([&]{return r.runtime.phase()==Phase::Complete;});
  CHECK(r.runtime.visited()==0&&r.offboard_commands==0&&r.runtime.reason()=="NATIVE_TAKEOFF_TIMEOUT");
}
void native_denied_commands() {
  for(int operation=0;operation<3;++operation) {
    NativeRig r;
    r.px4.refuse_takeoff=operation==0;r.px4.refuse_arm=operation==1;r.px4.refuse_offboard=operation==2;
    r.until([&]{return r.runtime.phase()==Phase::Complete;});
    CHECK(r.runtime.visited()==0&&!r.px4.state.armed&&r.runtime.reason()=="PX4_COMMAND_REJECTED");
  }
}
void native_rc_never_reclaimed() {
  for(const auto phase:{Phase::Takeoff,Phase::Handoff}) {
    NativeRig r;r.until([&]{return r.runtime.phase()==phase;});
    r.px4.state.rc_override=true;r.px4.state.mode=Mode::Manual;r.step();
    CHECK(r.runtime.phase()==Phase::Manual&&r.out.kind==OutputKind::Release);
    const auto count=r.offboard_commands;r.px4.state.rc_override=false;
    for(int n=0;n<50;++n) r.step();
    CHECK(r.offboard_commands==count&&!r.runtime.command(Command::Resume)&&r.guard.released());
  }
}
void handoff_fixed_target_and_prestream() {
  NativeRig r;r.until([&]{return r.runtime.phase()==Phase::Handoff;});
  r.step();const auto fixed=r.out.target;CHECK(r.out.stream_position);
  r.px4.state.pose.p.x+=.05;
  for(int n=0;n<20;++n) {r.step();CHECK(r.out.stream_position&&r.out.target.p.x==fixed.p.x&&r.offboard_commands==0);}
  CHECK(!r.runtime.command(Command::Pause));
  r.until([&]{return r.px4.state.mode==Mode::Offboard;});CHECK(r.offboard_commands==1);
}
void native_cancel_and_watchdog() {
  for(bool watchdog:{false,true}) {
    NativeRig r;r.until([&]{return r.runtime.phase()==Phase::Takeoff&&r.px4.state.pose.p.z>.2;});
    if(watchdog) {for(int n=0;n<12;++n) r.step(false);}
    else {CHECK(r.runtime.command(Command::Cancel));r.step();}
    CHECK(r.px4.state.mode==Mode::Land&&r.offboard_commands==0);
  }
}
void native_start_requires_validated_target() {
  testing::FakePx4 px4;px4.state.native_takeoff_target_validated=false;Runtime r(native_config());
  CHECK(!r.start(testing::example_mission(),px4.state,0));
}
void land_refused_releases_control() {
  NativeRig r;r.px4.refuse_land=true;
  r.until([&]{return r.runtime.phase()==Phase::Manual;});
  CHECK(r.guard.released()&&r.land_commands==1&&r.runtime.reason()=="LAND_UNCONFIRMED_STOP_STREAM");
  CHECK(!r.runtime.command(Command::Resume));
}
void ground_disarm_must_not_continue_airborne() {
  const auto c=native_config();FlightGuard g(c,"e","b",1,1);testing::FakePx4 px4;
  px4.state.mode=Mode::Takeoff;px4.state.armed=true;
  Intent i{1,"e","b",1,1,1,1,IntentKind::Land,{},0,.5};CHECK(g.accept(i,0));
  CHECK(g.tick(px4.state,0).kind==OutputKind::GroundDisarm);
  px4.state.landed=false;px4.state.pose.p.z=.2;px4.stamp(.05);
  CHECK(g.tick(px4.state,.05).kind==OutputKind::RequestLand);
}
void land_invalid_pose_never_streams() {
  for(int invalid=0;invalid<3;++invalid) {
    FlightGuard g({},"e","b",1,1);testing::FakePx4 px4;px4.state.mode=Mode::Offboard;
    px4.state.armed=true;px4.state.landed=false;px4.state.pose.p.z=1;
    if(invalid==0) px4.state.pose.p.x=std::numeric_limits<double>::quiet_NaN();
    if(invalid==1) px4.state.frame_epoch=2;
    if(invalid==2) px4.state.yaw_quality=Quality::Invalid;
    Intent i{1,"e","b",1,1,1,1,IntentKind::Land,{},0,.5};CHECK(g.accept(i,0));
    const auto out=g.tick(px4.state,0);CHECK(out.kind==OutputKind::RequestLand&&!out.stream_position);
    CHECK(out.request&&finite(out.request->target));
  }
}
int main() {
  const std::vector<std::pair<const char*,void(*)()>> tests={
    {"ack_is_not_mode",ack_is_not_mode},{"state_confirms_without_ack",state_confirms_without_ack},
    {"reply_binding",reply_binding},{"stale_reply_and_original_deadline",stale_reply_and_original_deadline},
    {"bounded_retry_and_continuous_stream",bounded_retry_and_continuous_stream},{"arm_requires_observation",arm_requires_observation},
    {"unexpected_mode_and_pending_conflict",unexpected_mode_and_pending_conflict},{"hold_requires_correlated_completion",hold_requires_correlated_completion},
    {"native_nominal",native_nominal},{"native_automatic_hold",native_automatic_hold},{"native_completion_missing",native_completion_missing},
    {"native_denied_commands",native_denied_commands},{"native_rc_never_reclaimed",native_rc_never_reclaimed},
    {"handoff_fixed_target_and_prestream",handoff_fixed_target_and_prestream},{"native_cancel_and_watchdog",native_cancel_and_watchdog},
    {"native_start_requires_validated_target",native_start_requires_validated_target},
    {"land_refused_releases_control",land_refused_releases_control},
    {"ground_disarm_must_not_continue_airborne",ground_disarm_must_not_continue_airborne},
    {"land_invalid_pose_never_streams",land_invalid_pose_never_streams}};
  unsigned passed=0;for(const auto& [name,test]:tests) {try {test();++passed;std::cout<<"PASS "<<name<<'\n';}
    catch(const std::exception& e) {std::cerr<<"FAIL "<<name<<" "<<e.what()<<'\n';}}
  std::cout<<passed<<'/'<<tests.size()<<" synthetic mode tests\n";return passed==tests.size()?0:1;
}
