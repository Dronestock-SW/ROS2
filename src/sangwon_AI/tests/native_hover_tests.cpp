#include "sangwon_ai/native_hover.hpp"
#include <iostream>
#include <stdexcept>
using namespace sangwon::bench;
void check(bool v,const char* what) { if(!v)throw std::runtime_error(what); }
Sample ground() { Sample s; s.connected=s.landed=s.fresh=s.estimator_ok=s.rc_ok=true; s.mode="POSCTL"; return s; }
NativeHover climb(Sample& s) {
  NativeHover c; c.enabled=true; check(c.start(s,0),"start");
  check(c.tick(s,.05)==Action::Takeoff,"takeoff request");
  check(c.tick(s,.1)==Action::None,"no arm on sent mode only");
  s.mode="AUTO.TAKEOFF"; check(c.tick(s,.2)==Action::Arm,"arm only actual takeoff");
  s.armed=true; s.landed=false; c.tick(s,.3); check(c.phase=="CLIMB","armed confirmed"); return c;
}
int main() {
  try {
    auto s=ground(); NativeHover disabled; check(!disabled.start(s,0),"disabled gate");
    NativeHover c; c.enabled=true; s.estimator_ok=false; check(!c.start(s,0),"estimator gate");
    s=ground(); s.mode="OFFBOARD"; check(!c.start(s,0),"pilot position required");
    s=ground(); c=climb(s); s.z=1.3; s.mode="AUTO.LOITER";
    c.tick(s,1); c.tick(s,1.51); check(c.phase=="HOVER","settle before hover");
    check(c.tick(s,3.50)==Action::None,"two full seconds");
    check(c.tick(s,3.52)==Action::Land,"land after hover");
    s.mode="AUTO.LAND"; c.tick(s,4); check(c.phase=="LANDING","mode alone not completion");
    s.landed=true; c.tick(s,5); check(c.phase=="LANDING","landed armed not completion");
    s.armed=false; c.tick(s,6); check(c.phase=="COMPLETE","landed disarmed complete");
    check(!c.start(ground(),7),"no duplicate/restart");
    s=ground(); c=climb(s); s.rc_stick=true; check(c.tick(s,1)==Action::None && c.phase=="HANDOFF","PX4 owns native stick handoff");
    check(c.tick(s,1.1)==Action::None,"no repeated handoff"); s.mode="POSCTL"; c.tick(s,1.2); check(c.phase=="RELEASED","permanent handoff");
    s=ground(); c=climb(s); s.rc_switch=true; check(c.tick(s,1)==Action::None && c.phase=="RELEASED","do not override RC mode switch");
    s=ground(); c=climb(s); s.mode="ALTCTL"; check(c.tick(s,1)==Action::None && c.phase=="RELEASED","do not reclaim failsafe/pilot mode");
    s=ground(); c=climb(s); s.fresh=false; check(c.tick(s,1)==Action::None && c.phase=="RELEASED","stale no commands");
    s=ground(); c=climb(s); s.x=.7; check(c.tick(s,1)==Action::Land,"horizontal drift land");
    s=ground(); c=climb(s); check(c.tick(s,36)==Action::Land,"climb timeout land");
    s=ground(); c=climb(s); s.z=1.3; s.mode="AUTO.LOITER"; c.tick(s,1); c.tick(s,1.6); s.z=.8; c.tick(s,2); s.z=1.3; c.tick(s,2.1); c.tick(s,2.7); check(c.tick(s,4)==Action::None,"hover timer resets on departure");
    s=ground(); c=climb(s); s.rc_ok=false; s.rc_stick=true; s.rc_switch=true;
    check(c.tick(s,1)==Action::None && c.reason=="RC_LOST_PX4_FAILSAFE","lost RC is not stick takeover");
    s=ground(); c=climb(s); c.rejected("ARM_REJECTED"); c.tick(s,1);
    s.mode="AUTO.LAND"; s.armed=false; s.landed=true; c.tick(s,2);
    check(c.phase=="FAILED" && c.outcome=="FAIL" && c.landing_verified,"failure landing is not success");
    s=ground(); c.enabled=true; c=NativeHover{}; c.enabled=true; check(c.start(s,0),"cancel start");
    s.mode="AUTO.TAKEOFF"; s.command_pending=true; c.land(.1); c.tick(s,.2);
    check(c.phase=="ABORTING" && !c.landing_verified,"cancel cannot finish pending arm");
    s.command_pending=false; s.armed=true; check(c.tick(s,.3)==Action::Disarm,"ground abort normal disarm");
    s.armed=false; s.completion_fresh=false; c.tick(s,.4);
    check(c.phase=="ABORTING","completion requires telemetry after command reply");
    s.completion_fresh=true; check(c.tick(s,1.4)==Action::Position,"cancel restores position before finish");
    s.mode="POSCTL"; c.tick(s,1.5); check(c.phase=="CANCELLED" && c.outcome=="CANCELLED","cancel is separate outcome");
    s=ground(); c=climb(s); s.z=std::nan(""); check(c.tick(s,1)==Action::None && c.phase=="RELEASED","nonfinite active telemetry rejected");
    std::cout<<"native hover sequence and failure tests passed\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
