#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc,char** argv) {
  using namespace sangwon;
  try {
    std::string scenario="nominal",csv;bool native_takeoff=false;
    for(int i=1;i<argc;i++) {
      const std::string arg=argv[i];
      if(arg=="--scenario"&&i+1<argc) scenario=argv[++i];
      else if(arg=="--csv"&&i+1<argc) csv=argv[++i];
      else if(arg=="--native-takeoff") native_takeoff=true;
      else throw std::invalid_argument("Usage: sangwon_replay [--scenario nominal|overshoot] [--native-takeoff] [--csv path]");
    }
    if(scenario!="nominal"&&scenario!="overshoot") throw std::invalid_argument("Unknown scenario");
    Config c;if(native_takeoff) c.takeoff_policy=TakeoffPolicy::NativePx4;
    Runtime runtime(c); testing::FakePx4 vehicle;
    auto mission=testing::example_mission();
    FlightGuard guard(c,mission.execution_id,runtime.boot_session(),1,1);
    if(!runtime.start(mission,vehicle.state,0)) throw std::runtime_error("Start rejected");
    std::ofstream log;
    if(!csv.empty()) {
      log.open(csv); if(!log) throw std::runtime_error("Cannot open CSV");
      log<<"time_s,phase,x,y,z,vx,vy,vz,target_x,target_y,target_z,visited,overshoot_m\n";
      log<<std::setprecision(10);
    }
    bool injected=false;
    double max_xy_speed=0;
    for(int step=1;step<=6000;step++) {
      const double now=step*c.tick_s;
      vehicle.stamp(now);
      if(scenario=="overshoot"&&!injected&&runtime.phase()==Phase::Travel&&vehicle.state.pose.p.x>1.8) {
        vehicle.state.pose.p.x=2.45; vehicle.state.velocity.x=0.35; injected=true;
      }
      if(guard.released()||guard.landing_locked()) runtime.control_fault(guard.fault_reason(),guard.released());
      const auto intent=runtime.tick(vehicle.state,now);
      if(intent&&!guard.accept(*intent,now)) throw std::runtime_error("Guard rejected runtime intent");
      const auto out=guard.tick(vehicle.state,now);
      max_xy_speed=std::max(max_xy_speed,xy(vehicle.state.velocity));
      if(log) log<<now<<','<<phase_name(runtime.phase())<<','<<vehicle.state.pose.p.x<<','
        <<vehicle.state.pose.p.y<<','<<vehicle.state.pose.p.z<<','<<vehicle.state.velocity.x<<','
        <<vehicle.state.velocity.y<<','<<vehicle.state.velocity.z<<','<<out.target.p.x<<','
        <<out.target.p.y<<','<<out.target.p.z<<','<<runtime.visited()<<','<<runtime.max_overshoot_m()<<'\n';
      vehicle.apply(out,c.tick_s,now);
      if(runtime.phase()==Phase::Complete) {
        const bool ok=runtime.visited()==mission.points.size()&&vehicle.state.landed&&!vehicle.state.armed
            &&(!injected||runtime.max_overshoot_m()>=0.4);
        std::cout<<"REPLAY "<<scenario<<" takeoff="<<(native_takeoff?"AUTO.TAKEOFF":"OFFBOARD")<<" result="<<(ok?"PASS":"FAIL")<<" visited="<<runtime.visited()
          <<" time_s="<<now<<" observed_max_xy_mps="<<max_xy_speed
          <<" injected_overshoot_m="<<runtime.max_overshoot_m()<<'\n';
        return ok?0:1;
      }
    }
    std::cerr<<"Replay did not complete; phase="<<phase_name(runtime.phase())<<" reason="<<runtime.reason()<<'\n';
    for(const auto& e:runtime.events()) std::cerr<<e<<'\n';
    return 1;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
}
