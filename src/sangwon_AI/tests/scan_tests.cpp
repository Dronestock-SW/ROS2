#include "sangwon_ai/scan.hpp"
#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include <functional>
#include <iostream>

using namespace sangwon;
#define CHECK(x) do { if(!(x)) throw std::runtime_error(#x); } while(false)
ScanPlan plan() {
  ScanPlan p;
  p.task_id="S01"; p.label_id="L01"; p.dictionary="4X4_50"; p.marker_id=7; p.marker_size_m=0.159;
  p.calibration_ref="cal/r1"; p.mounting_ref="mount/r1"; p.layout_ref="label/r1";
  p.staging={{0.4,0,1},0}; p.reader={{0.5,0,1},0}; p.camera={{0.45,0,1.05},0};
  p.centre_min={0,-0.3,0.5}; p.centre_max={1.2,0.3,1.3}; p.space_validated=true;
  p.expected_qr_fields={{"code","BOX-A"}}; return p;
}
MarkerObservation marker(const ScanPlan& p,double now,std::uint64_t sequence) {
  MarkerObservation m; m.producer_id="synthetic-image"; m.sequence=sequence;
  m.dictionary=p.dictionary; m.marker_id=p.marker_id; m.marker_size_m=p.marker_size_m;
  m.calibration_ref=p.calibration_ref; m.mounting_ref=p.mounting_ref; m.layout_ref=p.layout_ref;
  m.observed_s=now; m.frame_epoch=1; m.valid=true; m.reprojection_px=0.1;
  m.desired_vehicle_pose_map=p.reader; return m;
}
struct Rig {
  Config flight; ScanConfig c; ScanPlan p{plan()}; State state;
  ScanAction action{c,flight,p,"execution",0,0};
  double now{},active{};
  std::uint64_t sequence{};
  bool movement{}, images{true}, store{true}, follow{true}, decoder_ack{true};
  ScanDecision d;
  Rig() {
    state.pose=p.staging; state.pose_quality=state.yaw_quality=Quality::Valid;
    state.connected=state.armed=state.can_hold=true; state.landed=false; state.mode=Mode::Offboard;
    state.scan.camera_healthy=state.scan.scanner_healthy=true;
  }
  void step() {
    now+=0.05; active+=0.05;
    state.pose_time_s=state.quality_time_s=state.mode_time_s=now;
    if(images) state.scan.marker=marker(p,now,++sequence);
    d=action.tick(state,now,active,movement);
    movement=false;
    if(d.move&&follow) { state.pose=d.target; state.velocity={}; movement=true; }
    if(d.persist&&store) state.scan.stored_result_id=action.result()->id;
    if(d.enable_camera&&decoder_ack) { state.scan.camera_decoder_enabled=true; state.scan.camera_activation_id=d.window_id; }
  }
  void until(const std::function<bool()>& condition,int max=1000) {
    for(int i=0;i<max&&!condition();i++) step();
    CHECK(condition());
  }
  void qr(QrSource source,const std::string& raw="{\"code\":\"BOX-A\"}",const std::string& window="") {
    QrObservation q; q.producer_id="synthetic-reader"; q.sequence=++sequence; q.execution_id="execution";
    q.task_id=p.task_id; q.window_id=window.empty()?action.window_id():window; q.source=source;
    q.raw=raw; q.observed_s=now+0.01; state.scan.qr=q;
  }
};
void scanner_success() {
  Rig r; r.until([&]{return r.action.stage()==ScanStage::Reader;});
  CHECK(r.action.attempts()==1); r.qr(QrSource::Scanner); r.step();
  CHECK(r.action.result()&&r.action.result()->succeeded);
  CHECK(!r.action.result()->egress_complete);
  r.until([&]{return r.action.stage()==ScanStage::Complete;});
  CHECK(r.action.result()->egress_complete); CHECK(norm(r.state.pose.p-r.p.staging.p)<1e-6);
}
void reader_three_then_camera_and_mount_offset() {
  Rig r; r.until([&]{return r.action.stage()==ScanStage::Camera;});
  CHECK(r.action.attempts()==3); CHECK(norm(r.state.pose.p-r.p.camera.p)<1e-6);
  r.qr(QrSource::Camera); r.step();
  CHECK(r.action.result()&&r.action.result()->succeeded&&r.action.result()->camera_used);
  r.until([&]{return r.action.stage()==ScanStage::Complete;});
  CHECK(r.action.result()->scanner_attempts==3);
}
void unreadable_persists_before_egress() {
  Rig r; r.store=false;
  r.until([&]{return r.action.stage()==ScanStage::Persist;});
  CHECK(r.action.result()->code=="QR_UNREADABLE"&&!r.action.result()->succeeded);
  CHECK(r.action.result()->scanner_attempts==3&&r.action.result()->camera_used);
  for(int i=0;i<10;i++) r.step();
  CHECK(r.action.stage()==ScanStage::Persist&&!r.action.result()->egress_complete);
  r.state.scan.stored_result_id=r.action.result()->id;
  r.until([&]{return r.action.stage()==ScanStage::Complete;});
}
void stale_wrong_marker_and_cached_sequence() {
  Rig r; r.images=false; r.state.scan.marker=marker(r.p,0,1);
  r.until([&]{return r.action.stage()==ScanStage::Persist;});
  CHECK(!r.action.result()->succeeded&&r.action.attempts()==0);
  Rig wrong; wrong.images=false;
  for(int i=0;i<180&&!wrong.action.result();i++) {
    auto m=marker(wrong.p,wrong.now+0.05,++wrong.sequence); m.marker_id=8;
    wrong.state.scan.marker=m; wrong.step();
  }
  CHECK(wrong.action.result()&&wrong.action.attempts()==0);
}
void qr_context_and_identity_are_enforced() {
  Rig r; r.until([&]{return r.action.stage()==ScanStage::Reader;});
  r.qr(QrSource::Scanner,"{\"code\":\"WRONG\"}"); r.step(); CHECK(!r.action.result());
  r.qr(QrSource::Scanner,"{\"code\":\"BOX-A\",\"code\":\"BOX-A\"}"); r.step(); CHECK(!r.action.result());
  r.qr(QrSource::Scanner,"{\"code\":\"BOX-A\"}","earlier-window"); r.step(); CHECK(!r.action.result());
  r.qr(QrSource::Scanner); r.state.scan.qr->task_id="other"; r.step(); CHECK(!r.action.result());
  r.qr(QrSource::Scanner); r.step(); CHECK(r.action.result()->succeeded);
}
void invalid_cached_marker_cancels_reader_window() {
  Rig r;r.until([&]{return r.action.stage()==ScanStage::Reader;});
  r.images=false;r.state.scan.marker->valid=false;
  r.qr(QrSource::Scanner);r.step();
  CHECK(!r.action.result()&&r.action.stage()==ScanStage::Settle&&!r.d.open_reader);
}
void hover_loss_discards_reader_window_and_preserves_attempts() {
  Rig r; r.until([&]{return r.action.stage()==ScanStage::Reader;}); const auto old=r.action.window_id();
  r.state.velocity={0.3,0,0}; r.qr(QrSource::Scanner); r.step(); CHECK(!r.action.result());
  CHECK(r.action.attempts()==1); r.state.velocity={}; r.action.suspend();
  r.until([&]{return r.action.stage()==ScanStage::Reader&&r.action.attempts()==2;});
  r.qr(QrSource::Scanner,"{\"code\":\"BOX-A\"}",old); r.step(); CHECK(!r.action.result());
}
void camera_failure_and_record_failure() {
  Rig camera; camera.state.scan.camera_healthy=false;
  camera.until([&]{return camera.action.stage()==ScanStage::Complete;});
  CHECK(camera.action.result()->code=="CAMERA_UNAVAILABLE"&&camera.action.attempts()==0);
  Rig storage; storage.store=false; storage.state.scan.camera_healthy=false;
  storage.until([&]{return storage.d.record_failed;});
  CHECK(storage.action.stage()==ScanStage::Persist&&!storage.action.result()->egress_complete);
}
void camera_activation_and_pause_budget() {
  Rig r; r.decoder_ack=false;
  r.until([&]{return r.action.stage()==ScanStage::Camera;});
  r.qr(QrSource::Camera); r.step(); CHECK(!r.action.result());
  r.state.scan.camera_decoder_enabled=true; r.state.scan.camera_activation_id="old-activation";
  r.qr(QrSource::Camera); r.step(); CHECK(!r.action.result());
  for(int i=0;i<40;i++) r.step();
  r.action.suspend();
  // PAUSE advances the sensor clock while retaining the active-time budget.
  r.now+=30;
  r.until([&]{return r.action.stage()==ScanStage::Camera;});
  const auto resumed=r.now;
  r.until([&]{return r.action.stage()==ScanStage::Persist;});
  CHECK(r.now-resumed<1.1); CHECK(r.action.result()->scanner_attempts==3);
}
void unsafe_adjustment_is_not_output_and_rc_preempts() {
  Rig r; r.images=false; r.until([&]{return r.action.stage()==ScanStage::Acquire;});
  auto m=marker(r.p,r.now+0.05,1); m.desired_vehicle_pose_map.p.x=2; r.state.scan.marker=m; r.step();
  CHECK(!r.d.move); CHECK(r.action.attempts()==0);
  r.state.rc_override=true; r.step();
  CHECK(r.action.stage()==ScanStage::Preempted&&r.action.result()->preempted&&!r.d.move);
}
void correction_checks_polygon_and_whole_path() {
  auto p=plan();p.camera=p.reader;
  p.workspace=PolygonVolume{{{0,-.3,0},{1.2,-.3,0},{1.2,.3,0},{.6,.3,0},{.6,.02,0},{.52,.02,0},{.52,.3,0},{0,.3,0}},.4,1.5};
  FlightSpace space;const PolygonVolume area{{{-1,-1,0},{2,-1,0},{2,1,0},{-1,1,0}},0,2};
  space.altitude=space.yaw={area};p.space=std::make_shared<const FlightSpace>(space);
  p.clearance_xy_m=p.clearance_z_m=.01;
  Config c;Rig r;
  ScanAction action(p.config,c,p,"execution",0,0);
  r.state.pose=p.reader;r.state.pose_time_s=r.state.quality_time_s=r.state.mode_time_s=.05;
  action.tick(r.state,.05,.05,true);CHECK(action.stage()==ScanStage::Acquire);
  auto observation=marker(p,.1,1);observation.desired_vehicle_pose_map.p={.56,.07,1};r.state.scan.marker=observation;
  r.state.pose_time_s=r.state.quality_time_s=r.state.mode_time_s=.1;
  CHECK(!action.tick(r.state,.1,.1).move);CHECK(action.attempts()==0); // Inside bounds, outside polygon.
  p.workspace=area;
  space.forbidden={PolygonVolume{{{.54,-.01,0},{.55,-.01,0},{.55,.01,0},{.54,.01,0}},.5,1.3}};
  p.space=std::make_shared<const FlightSpace>(space);
  ScanAction blocked(p.config,c,p,"execution",0,0);
  r.state.pose_time_s=r.state.quality_time_s=r.state.mode_time_s=.05;
  blocked.tick(r.state,.05,.05,true);
  observation=marker(p,.1,2);observation.desired_vehicle_pose_map.p.x=.59;r.state.scan.marker=observation;
  r.state.pose_time_s=r.state.quality_time_s=r.state.mode_time_s=.1;
  const auto decision=blocked.tick(r.state,.1,.1);
  CHECK(!decision.move&&decision.persist&&blocked.result()->code=="ALIGNMENT_OUT_OF_BOUNDS");
  CHECK(blocked.attempts()==0); // Both endpoints safe; connecting segment hits the obstacle.
  r.state.scan.stored_result_id=blocked.result()->id;r.state.pose.p.x=.59;
  r.state.pose_time_s=r.state.quality_time_s=r.state.mode_time_s=.15;
  const auto egress=blocked.tick(r.state,.15,.15);
  CHECK(egress.flight_failed&&!egress.move&&blocked.result()->preempted&&!blocked.result()->egress_complete);
}
void runtime_scan_failure_returns_staging_then_next_waypoint() {
  Config c; Runtime runtime(c); testing::FakePx4 px4;
  Mission m; m.execution_id="runtime-scan"; m.approved_fixture=true;
  auto p=plan(); p.camera=p.reader;
  m.points={Waypoint{p.staging.p,p.staging.yaw,0,p},Waypoint{{0.7,0,1},0,0,{}}};
  CHECK(runtime.start(m,px4.state,0));
  FlightGuard guard(c,m.execution_id,runtime.boot_session(),1,1);
  double now=0; unsigned sequence=0; bool exited=false;
  for(int i=0;i<6000&&runtime.phase()!=Phase::Complete;i++) {
    now+=c.tick_s; px4.stamp(now);
    px4.state.scan.camera_healthy=px4.state.scan.scanner_healthy=true;
    px4.state.scan.marker=marker(p,now,++sequence);
    if(const auto scan=runtime.scan_action()) {
      if(scan->result()) px4.state.scan.stored_result_id=scan->result()->id;
      if(scan->stage()==ScanStage::Camera) {
        px4.state.scan.camera_decoder_enabled=true; px4.state.scan.camera_activation_id=scan->window_id();
      }
    }
    const auto intent=runtime.tick(px4.state,now);
    if(intent) CHECK(guard.accept(*intent,now));
    if(runtime.visited()==1&&!exited) { CHECK(norm(px4.state.pose.p-p.staging.p)<0.2); exited=true; }
    px4.apply(guard.tick(px4.state,now),c.tick_s,now);
  }
  CHECK(runtime.phase()==Phase::Complete&&runtime.visited()==2&&exited);
  CHECK(runtime.scan_results().size()==1&&!runtime.scan_results()[0].succeeded&&runtime.scan_results()[0].egress_complete);
}
int main() {
  const std::vector<std::pair<const char*,void(*)()>> tests={
    {"scanner_success",scanner_success},{"three_reader_then_camera",reader_three_then_camera_and_mount_offset},
    {"persist_before_egress",unreadable_persists_before_egress},{"stale_and_wrong_marker",stale_wrong_marker_and_cached_sequence},
    {"qr_context_and_identity",qr_context_and_identity_are_enforced},
    {"invalid_cached_marker",invalid_cached_marker_cancels_reader_window},
    {"hover_loss_invalidates_attempt",hover_loss_discards_reader_window_and_preserves_attempts},
    {"camera_and_record_failure",camera_failure_and_record_failure},
    {"camera_activation_pause_budget",camera_activation_and_pause_budget},
    {"unsafe_adjustment_and_rc",unsafe_adjustment_is_not_output_and_rc_preempts},
    {"polygon_and_path_correction",correction_checks_polygon_and_whole_path},
    {"runtime_failure_egress_next",runtime_scan_failure_returns_staging_then_next_waypoint}};
  unsigned failures=0;
  for(const auto& [name,fn]:tests) {
    try { fn(); std::cout<<"PASS "<<name<<'\n'; }
    catch(const std::exception& e) { failures++; std::cerr<<"FAIL "<<name<<" "<<e.what()<<'\n'; }
  }
  return failures?1:0;
}
