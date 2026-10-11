#include "sangwon_ai/trajectory.hpp"

namespace sangwon {
void Config::validate() const {
  if(profile!=Profile::Replay) throw std::invalid_argument("This executable supports REPLAY only");
  if(takeoff_policy!=TakeoffPolicy::Offboard&&takeoff_policy!=TakeoffPolicy::NativePx4)
    throw std::invalid_argument("Invalid takeoff policy");
  for(double value : {tick_s,lease_s,pose_age_s,xy_error_m,z_error_m,yaw_error_rad,
      arrival_speed_mps,stable_s,recovery_s,recovery_stable_s,prestream_s,transition_s,
      blocked_s,stall_s,xy_speed_mps,z_speed_mps,accel_mps2,yaw_rate_rps,yaw_accel_rps2,
      battery_return,log_age_s,control_retry_s,native_takeoff_timeout_s}) {
    if(!std::isfinite(value)||value<=0) throw std::invalid_argument("Invalid profile value");
  }
  if(tick_s>0.1||lease_s<=tick_s||battery_return>=1||recovery_stable_s>=recovery_s
      ||control_retry_s<tick_s||control_retry_s>=transition_s||!control_max_attempts||control_max_attempts>10)
    throw std::invalid_argument("Inconsistent profile");
}
ScalarProfile::ScalarProfile(double d, double v, double a) : distance_(d),accel_(a) {
  if(!std::isfinite(d)||!std::isfinite(v)||!std::isfinite(a)||d<0||v<=0||a<=0)
    throw std::invalid_argument("Invalid trajectory limits");
  peak_=std::min(v,std::sqrt(d*a));
  ramp_s_=peak_/a;
  cruise_s_=peak_>0 ? std::max(0.0,(d-peak_*ramp_s_)/peak_) : 0;
}
ScalarProfile::Sample ScalarProfile::sample(double t) const {
  if(!std::isfinite(t)) throw std::invalid_argument("Invalid trajectory time");
  if(t<=0) return {0,0};
  if(t>=duration()) return {distance_,0};
  if(t<ramp_s_) return {0.5*accel_*t*t,accel_*t};
  if(t<ramp_s_+cruise_s_) return {0.5*peak_*ramp_s_+peak_*(t-ramp_s_),peak_};
  const double remaining=duration()-t;
  return {distance_-0.5*accel_*remaining*remaining,accel_*remaining};
}
Segment::Segment(Pose a, Pose b, double speed, const Config& c)
    : from_(a),to_(b),yaw_delta_(wrap(b.yaw-a.yaw)),
      travel_(norm(b.p-a.p),speed,c.accel_mps2),
      yaw_(std::abs(yaw_delta_),c.yaw_rate_rps,c.yaw_accel_rps2) {
  if(!finite(a)||!finite(b)) throw std::invalid_argument("Nonfinite pose");
  const double d=norm(b.p-a.p);
  direction_=d>0 ? (b.p-a.p)*(1/d) : Vec3{};
}
Pose Segment::sample(double t) const {
  return {from_.p+direction_*travel_.sample(t).position,
          wrap(from_.yaw+std::copysign(yaw_.sample(t).position,yaw_delta_))};
}
double Segment::duration() const { return std::max(travel_.duration(),yaw_.duration()); }
double Segment::overshoot(Vec3 p) const { return std::max(0.0,dot(p-to_.p,direction_)); }
double stopping_distance(double speed,double a,double latency,double margin) {
  if(!std::isfinite(speed)||!std::isfinite(a)||!std::isfinite(latency)||!std::isfinite(margin)
      ||speed<0||a<=0||latency<0||margin<0) throw std::invalid_argument("Invalid braking input");
  return speed*latency+speed*speed/(2*a)+margin;
}
} // namespace sangwon
