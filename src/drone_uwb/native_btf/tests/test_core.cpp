#include "btf/core.hpp"
#include "btf/freeze.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok,const char* what){if(!ok)throw std::runtime_error(what);}
btf::Config config(){btf::Config c;c.anchors={{{0,0,2.2},{6.3,0,2.2},{0,4.6,2.2},{6.3,4.6,2.2}}};return c;}
btf::Frame frame(int k,bool moving=true) {
  btf::Frame f;f.start_us=1000000+k*25000;f.end_us=f.start_us+16000;
  f.height_source=btf::HeightSource::MeasuredTofImu;
  const auto c=config();
  for(int i=0;i<4;++i) {
    f.sample_us[i]=f.start_us+4000*(i+1);const auto t=f.sample_us[i]/1e6;
    const auto x=moving?.7+.3*t:2.,y=moving?1.8-.1*t:2.,z=1.1+.02*t;
    f.height_m[i]=z;const auto a=c.anchors[i];
    f.raw_range_m[i]=std::hypot(std::hypot(x-a[0],y-a[1]),z-a[2]);
  }
  return f;
}
void numerical_contract() {
  btf::Processor p(config());
  for(int k=0;k<100;++k) {
    const auto f=frame(k);const auto d=p.process(f);
    if(k<2){check(!d.ok,"warmup must reject");continue;}
    check(d.ok&&d.source=="unchanged_B4","nominal B4 missing");
    check(std::hypot((*d.xy)[0]-(.7+.3*f.end_us/1e6),(*d.xy)[1]-(1.8-.1*f.end_us/1e6))<1e-8,"moving XY incorrect");
    check(d.b4.rank==7&&d.source_stamp_us==f.end_us,"rank or original time lost");
    check(d.history_samples<=132,"unbounded 0.8s history");
  }
  auto f=frame(100);f.sample_us[0]=f.end_us+1;
  check(!p.process(f).ok,"future sample accepted");
  f=frame(100);f.raw_range_m[0]=-1;check(!p.process(f).ok,"negative distance accepted");
  p.reset();check(!p.process(frame(101)).ok,"reset retained warmup");
  btf::Processor no_height(config());
  for(int k=0;k<10;++k){f=frame(k);f.height_m={};f.height_source=btf::HeightSource::Missing;
    if(k>=2)check(no_height.process(f).ok,"height-free B4 blocked");else no_height.process(f);}
  auto bad=config();bad.anchors[3][2]+=.1;
  bool threw=false;try{btf::Processor invalid(bad);}catch(const std::invalid_argument&){threw=true;}
  check(threw,"unequal-height geometry accepted");
}
btf::Decision accepted(btf::Time stamp){btf::Decision d;d.ok=true;d.source_stamp_us=stamp;d.xy=btf::XY{2.,3.};return d;}
void fresh_recovery_contract() {
  auto cfg=config();cfg.fresh_b4_recovery_enabled=true;
  btf::Processor baseline(config()),recovery(cfg);int old_rejected=0,new_rejected=0,rescued=0;
  for(int k=0;k<120;++k) {
    auto f=frame(k,false);f.height_source=btf::HeightSource::Missing;f.height_m={};
    if(k==50)f.raw_range_m[0]+=.8;
    const auto a=baseline.process(f),b=recovery.process(f);
    if(k>50){old_rejected+=!a.ok;new_rejected+=!b.ok;}
    if(b.source=="fresh_B4_recovery") {
      ++rescued;check(b.recovery_frames>=3,"recovery lacks actual measured history");
      check(b.source_stamp_us==f.end_us,"recovery rewrote source timestamp");
      check(std::hypot((*b.xy)[0]-2.,(*b.xy)[1]-2.)<.04,"recovered XY inaccurate");
      check(b.recovery_raw_residual_m&&*b.recovery_raw_residual_m<cfg.rms_limit_m,"fresh ranges not qualified");
    }
  }
  check(rescued>0&&new_rejected<old_rejected,"fresh measured recovery did not shorten contamination");
  btf::Processor bad(cfg);
  for(int k=0;k<100;++k) {
    auto f=frame(k,false);f.height_source=btf::HeightSource::Missing;f.height_m={};
    if(k>=40){f.raw_range_m[0]+=.8;f.raw_range_m[3]+=.7;}
    const auto d=bad.process(f);
    if(k>=40)check(d.source!="fresh_B4_recovery","incoherent ranges released FREEZE");
  }
}
void freeze_contract() {
  btf::FreezePolicy p;const btf::XY goal{1.,1.},changed{9.,9.};
  p.observe(accepted(1000),1000,1000);
  check(!p.tick(1000,btf::FcHealth::Healthy,goal).pause_mission,"fresh observation paused");
  auto d=p.tick(151001,btf::FcHealth::Healthy,changed);
  check(d.state==btf::FreezeState::Frozen&&d.stream_target&&d.target_xy==goal,"silence not frozen");
  d=p.tick(300000,btf::FcHealth::Healthy,changed);
  check(d.target_xy==goal&&d.pause_mission,"freeze target followed drift/new waypoint");
  check(d.last_observation_monotonic_us==1000,"old measurement timestamp refreshed");
  p.observe(accepted(1000),1000,300000);
  check(p.tick(300000,btf::FcHealth::Healthy,changed).state==btf::FreezeState::Frozen,"stale pose released freeze");
  p.observe(accepted(310000),310000,310000);
  d=p.tick(310000,btf::FcHealth::Healthy,changed);
  check(d.target_xy==goal&&d.pause_mission,"recovery skipped latched setpoint");
  d=p.tick(320000,btf::FcHealth::Healthy,goal);check(!d.pause_mission,"qualified recovery never resumed");
  btf::Decision reject;reject.reason="no_height_consistent_subset";
  p.observe(reject,321000,321000);
  d=p.tick(321000,btf::FcHealth::Healthy,goal);
  check(d.state==btf::FreezeState::Frozen&&d.reason==reject.reason,"rejection did not freeze immediately");
  d=p.tick(322000,btf::FcHealth::Invalid,goal);
  check(!d.stream_target&&d.defer_to_fc_failsafe&&d.pause_mission,"invalid FC falsely promised hold");
  d=p.tick(323000,btf::FcHealth::Unknown,goal);
  check(!d.defer_to_fc_failsafe&&d.pause_mission&&d.stream_target&&d.target_xy==goal,"report unknown interrupted target stream");
  btf::FreezePolicy uninitialized;
  d=uninitialized.tick(1,btf::FcHealth::Healthy,std::nullopt);
  check(!d.target_xy&&!d.stream_target&&d.pause_mission,"fabricated startup target");
}
}
int main(){try{numerical_contract();fresh_recovery_contract();freeze_contract();std::cout<<"numerical and FREEZE contracts passed\n";return 0;}
catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
