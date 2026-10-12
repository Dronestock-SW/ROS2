#include "sangwon_ai/lidar_relative.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace sangwon::observe {
using service::Json;
namespace {
constexpr double pi = 3.14159265358979323846;
bool numeric(const Json& x) { return x.is_number() && std::isfinite(x.get<double>()); }
double value(const Json& j, const char* k, double fallback=1e9) {
  return j.contains(k) && numeric(j.at(k)) ? j.at(k).get<double>() : fallback;
}
bool age(const Json& j, const char* k, double limit) { const double a=value(j,k); return a>=0 && a<=limit; }
double angle(double x) { return std::remainder(x,2*pi); }
double duration(const Json& s) {
  const auto ranges=s.value("ranges",Json::array());
  if(!ranges.is_array() || ranges.size()<40 || ranges.size()>2048) return 1e9;
  if(s.value("input_source",std::string())=="sdk_pointcloud") {
    const auto times=s.value("point_times_s",Json::array());
    if(!times.is_array() || times.size()!=ranges.size()) return 1e9;
    double last=-1;
    for(const auto& t:times) { if(!numeric(t) || t.get<double>()<0 || t.get<double>()<=last || t.get<double>()>.15) return 1e9;last=t.get<double>(); }
    return last;
  }
  const double increment=value(s,"time_increment",0);
  return increment>=0?increment*(ranges.size()-1):1e9;
}
bool solve(double h[3][3], double b[3], double x[3]) {
  double a[3][4];
  for(int i=0;i<3;++i) { for(int j=0;j<3;++j) a[i][j]=h[i][j]; a[i][3]=b[i]; }
  for(int i=0;i<3;++i) {
    int pivot=i; for(int j=i+1;j<3;++j) if(std::abs(a[j][i])>std::abs(a[pivot][i])) pivot=j;
    if(std::abs(a[pivot][i])<1e-8) return false;
    for(int j=i;j<4;++j) std::swap(a[i][j],a[pivot][j]);
    const double v=a[i][i]; for(int j=i;j<4;++j) a[i][j]/=v;
    for(int k=0;k<3;++k) if(k!=i) { const double w=a[k][i]; for(int j=i;j<4;++j) a[k][j]-=w*a[i][j]; }
  }
  for(int i=0;i<3;++i) x[i]=a[i][3];
  return true;
}
std::array<double,2> lever(const std::array<double,3>& p,double r,double t,double y,int sign) {
  const double a=p[0]*std::cos(t)+(p[1]*std::sin(r)+p[2]*std::cos(r))*std::sin(t);
  const double b=p[1]*std::cos(r)-p[2]*std::sin(r);
  return {std::cos(y)*a-sign*std::sin(y)*b,std::sin(y)*a+sign*std::cos(y)*b};
}
} // namespace
std::vector<ScanPoint> scan_points(const Json& s,const std::array<double,3>& mount,double mount_yaw) {
  std::vector<ScanPoint> raw, out;
  const auto& ranges=s.at("ranges");
  const bool cloud=s.value("input_source",std::string())=="sdk_pointcloud";
  const auto angles=s.value("point_angles_rad",Json::array());
  const double first=value(s,"angle_min"), step=value(s,"angle_increment"), lo=value(s,"range_min"), hi=value(s,"range_max");
  if(!ranges.is_array() || ranges.size()<40 || ranges.size()>2048 || lo<0 || hi<=lo || hi>100) return out;
  if(cloud) { if(!angles.is_array() || angles.size()!=ranges.size()) return out;
    for(const auto& a:angles) if(!numeric(a) || std::abs(a.get<double>())>pi+1e-6) return out;
  } else if(std::abs(step)<1e-6 || std::abs(step)>pi || std::abs(first)>2*pi) return out;
  raw.resize(ranges.size());
  std::vector<bool> valid(ranges.size());
  const auto quaternions=s.value("beam_quaternions",Json::array()),reference=s.value("reference_quaternion",Json::array());
  const bool deskew=quaternions.is_array() && quaternions.size()==ranges.size() && reference.size()==4;
  double reference_yaw=0, reference_roll=0, reference_pitch=0;
  if(deskew) {
    for(const auto& v:reference) if(!numeric(v)) return out;
    const double w=reference[0],x=reference[1],y=reference[2],z=reference[3];
    if(std::abs(w*w+x*x+y*y+z*z-1)>.002) return out;
    reference_yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    reference_roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    reference_pitch=std::asin(std::clamp(2*(w*y-z*x),-1.,1.));
  }
  const auto reference_lever=lever(mount,reference_roll,reference_pitch,reference_yaw,1);
  for(std::size_t i=0;i<ranges.size();++i) if(numeric(ranges[i])) {
    const double r=ranges[i].get<double>(), a=cloud?angles[i].get<double>():first+i*step;
    if(r>=std::max(lo,.15) && r<=std::min(hi,10.)) {
      const double px=r*std::cos(a+mount_yaw),py=r*std::sin(a+mount_yaw);
      if(deskew) {
        const auto& q=quaternions[i]; if(q.size()!=4) return out;
        for(const auto& v:q) if(!numeric(v)) return out;
        const double w=q[0],x=q[1],y=q[2],z=q[3];
        const double roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y)),pitch=std::asin(std::clamp(2*(w*y-z*x),-1.,1.));
        const double dot=w*reference[0].get<double>()+x*reference[1].get<double>()+y*reference[2].get<double>()+z*reference[3].get<double>();
        if(std::abs(w*w+x*x+y*y+z*z-1)>.002 || std::abs(roll)>.05 || std::abs(pitch)>.05 || std::abs(dot)<std::cos(.03/2)) return out;
        const double bx=px+mount[0],by=py+mount[1],bz=mount[2];
        const double wx=(1-2*(y*y+z*z))*bx+2*(x*y-w*z)*by+2*(x*z+w*y)*bz-reference_lever[0];
        const double wy=2*(x*y+w*z)*bx+(1-2*(x*x+z*z))*by+2*(y*z-w*x)*bz-reference_lever[1];
        raw[i]={std::cos(reference_yaw)*wx+std::sin(reference_yaw)*wy,-std::sin(reference_yaw)*wx+std::cos(reference_yaw)*wy,0,0};
      } else {
        const double roll=value(s,"roll_rad",0),pitch=value(s,"pitch_rad",0);
        raw[i]={std::cos(pitch)*px+std::sin(pitch)*std::sin(roll)*py,std::cos(roll)*py,0,0};
      }
      valid[i]=true;
    }
  }
  if(cloud) {
    std::vector<ScanPoint> selected;
    for(std::size_t i=0;i<raw.size();++i) if(valid[i]) selected.push_back(raw[i]);
    std::sort(selected.begin(),selected.end(),[](const ScanPoint& a,const ScanPoint& b){return std::atan2(a.y,a.x)<std::atan2(b.y,b.x);});
    raw=std::move(selected);valid.assign(raw.size(),true);
  }
  const std::size_t stride=std::max<std::size_t>(1,(raw.size()+239)/240);
  for(std::size_t i=1;i+1<raw.size();i+=stride) if(valid[i-1] && valid[i] && valid[i+1]) {
    const double a=std::hypot(raw[i].x-raw[i-1].x,raw[i].y-raw[i-1].y);
    const double b=std::hypot(raw[i+1].x-raw[i].x,raw[i+1].y-raw[i].y);
    const double x=raw[i+1].x-raw[i-1].x,y=raw[i+1].y-raw[i-1].y,n=std::hypot(x,y);
    if(a<=.4 && b<=.4 && n>.001) { auto p=raw[i]; p.nx=-y/n; p.ny=x/n; out.push_back(p); }
  }
  return out;
}
ScanMatch match_scans(const std::vector<ScanPoint>& prev,const std::vector<ScanPoint>& cur) {
  ScanMatch result;
  if(prev.size()<40 || cur.size()<40) return result;
  double tx=0,ty=0,yaw=0;
  struct Row { double j[3],r; };
  for(int iteration=0;iteration<15;++iteration) {
    std::vector<Row> rows; const double c=std::cos(yaw),s=std::sin(yaw);
    for(const auto& p:cur) {
      const double px=c*p.x-s*p.y,py=s*p.x+c*p.y,x=px+tx,y=py+ty;
      const ScanPoint* q=nullptr; double best=.3*.3;
      for(const auto& v:prev) { const double d=(x-v.x)*(x-v.x)+(y-v.y)*(y-v.y); if(d<best) { best=d; q=&v; } }
      if(q) rows.push_back({{q->nx,q->ny,-q->nx*py+q->ny*px},q->nx*(x-q->x)+q->ny*(y-q->y)});
    }
    result.overlap=static_cast<double>(rows.size())/cur.size();
    if(rows.size()<40 || result.overlap<.6) { result.reason="LOW_OVERLAP"; return result; }
    std::sort(rows.begin(),rows.end(),[](const Row& a,const Row& b){return std::abs(a.r)<std::abs(b.r);});
    rows.resize(std::max<std::size_t>(40,rows.size()*4/5));
    double h[3][3]{}, b[3]{}, residual=0;
    for(const auto& row:rows) {
      const double w=std::min(1.,.03/std::max(std::abs(row.r),1e-9));
      for(int i=0;i<3;++i) { b[i]-=w*row.j[i]*row.r; for(int j=0;j<3;++j) h[i][j]+=w*row.j[i]*row.j[j]; }
      residual+=row.r*row.r;
    }
    const double tr=h[0][0]+h[1][1];
    const double eig=(tr-std::hypot(h[0][0]-h[1][1],2*h[0][1]))/2;
    result.diversity=eig/std::max(tr,1e-9);
    result.correspondences=static_cast<unsigned>(rows.size()); result.rms=std::sqrt(residual/rows.size());
    double delta[3]{};
    if(result.diversity<.08 || !solve(h,b,delta)) { result.reason="DEGENERATE_GEOMETRY"; return result; }
    if(std::hypot(delta[0],delta[1])>.3 || std::abs(delta[2])>.15) { result.reason="UNSTABLE_SOLUTION"; return result; }
    tx+=delta[0];ty+=delta[1];yaw+=delta[2];
    // Stop below 0.5mm / 0.1mrad, well below the 10mm uncertainty floor.
    // A 10-micrometre stop oscillates between correspondences with 4mm noise.
    if(std::hypot(delta[0],delta[1])<.0005 && std::abs(delta[2])<.0001) {
      // Schur complement includes translation/yaw correlation. Never zero covariance.
      const double a=h[0][0]-h[0][2]*h[0][2]/h[2][2], d=h[1][1]-h[1][2]*h[1][2]/h[2][2];
      const double z=h[0][1]-h[0][2]*h[1][2]/h[2][2];
      const double lambda=(a+d-std::hypot(a-d,2*z))/2;
      if(lambda<1 || result.rms>.035) { result.reason="RESIDUAL_OR_INFORMATION_LIMIT"; return result; }
      result.dx=tx;result.dy=ty;result.dyaw=yaw;
      // Residuals alone miss scan-time translation and correlated surface error.
      result.sigma=std::max(.025,std::max(.01,result.rms)/std::sqrt(lambda));
      result.valid=true;result.reason="MATCHED";return result;
    }
  }
  result.reason="NOT_CONVERGED";return result;
}
LidarGapSupport::LidarGapSupport(const Json& c) {
  map_sign_=c.value("map_y_axis_sign",1);
  input_source_=c.value("input_source",std::string("sdk_pointcloud"));
  if(input_source_!="sdk_pointcloud" && input_source_!="uniform_laserscan") throw std::invalid_argument("LIDAR_INPUT_SOURCE_INVALID");
  if(map_sign_!=1 && map_sign_!=-1) throw std::invalid_argument("LIDAR_MAP_SIGN_INVALID");
  enabled_=c.value("enabled",false);
  if(!enabled_) return;
  qualified_=c.value("mount_confirmed",false) && c.value("timing_confirmed",false)
      && c.value("vertical_structure_confirmed",false);
  mount_ref_=c.value("mount_ref",std::string());timing_ref_=c.value("timing_ref",std::string());scene_ref_=c.value("scene_ref",std::string());
  frame_=c.at("frame_id").get<std::string>(); const auto& p=c.at("sensor_to_body_flu_m");
  if(!qualified_ || frame_.empty() || mount_ref_.empty() || timing_ref_.empty() || scene_ref_.empty()
      || !p.is_array() || p.size()!=3) throw std::invalid_argument("LIDAR_SURVEY_AND_TIMING_EVIDENCE_REQUIRED");
  for(int i=0;i<3;++i) { if(!numeric(p[i]) || std::abs(p[i].get<double>())>1) throw std::invalid_argument("LIDAR_MOUNT_INVALID"); mount_[i]=p[i].get<double>(); }
  mount_yaw_=value(c,"sensor_to_body_yaw_rad");
  if(std::abs(mount_yaw_)>pi || !c.value("scan_plane_parallel_to_body_xy",false)) throw std::invalid_argument("LIDAR_SCAN_PLANE_REQUIRED");
}
Json LidarGapSupport::update(const Json& state,double now) {
  const auto raw=state.value("lidar_scan",Json::object());
  const auto s=raw.is_object()?raw:Json::object();
  const auto stamp=s.value("stamp_ns",std::int64_t(0));
  const double roll=value(s,"roll_rad"),pitch=value(s,"pitch_rad"),yaw=value(s,"map_yaw_rad");
  const bool fresh=stamp>0 && age(s,"age_s",.15) && age(s,"attitude_age_s",.15)
      && std::abs(value(s,"attitude_time_error_s"))<=.05 && std::abs(roll)<=.05 && std::abs(pitch)<=.05
      && std::abs(yaw)<=2*pi && s.value("frame_id",std::string())==frame_
      && s.value("input_source",std::string())==input_source_
      && duration(s)>0 && duration(s)<=.15 && duration(s)<=value(s,"first_beam_age_s",value(s,"age_s"))
      && std::llabs(s.value("first_beam_stamp_ns",stamp)+static_cast<std::int64_t>(std::llround(duration(s)*1e9))-stamp)<=1;
  if(stamp>scan_stamp_ && fresh) {
    auto current=scan_points(s,mount_,mount_yaw_); const double measurement=now-value(s,"age_s");
    const double dt=scan_stamp_>0?static_cast<double>(stamp-scan_stamp_)/1e9:1e9;
    match_=match_scans(previous_,current); match_valid_=false;
    if(!previous_.empty() && dt>=.04 && dt<=.2 && std::abs(roll-last_roll_)<=.02 && std::abs(pitch-last_pitch_)<=.02
        && match_.valid && std::hypot(match_.dx,match_.dy)<=.03+.5*dt
        && std::abs(match_.dyaw)<=.08 && std::abs(angle(match_.dyaw-map_sign_*angle(yaw-last_yaw_)))<=.03) {
      const double a=last_yaw_;
      const auto before=lever(mount_,last_roll_,last_pitch_,last_yaw_,map_sign_),after=lever(mount_,roll,pitch,yaw,map_sign_);
      displacement_[0]+=std::cos(a)*match_.dx-map_sign_*std::sin(a)*match_.dy+before[0]-after[0];
      displacement_[1]+=std::sin(a)*match_.dx+map_sign_*std::cos(a)*match_.dy+before[1]-after[1];
      variance_+=match_.sigma*match_.sigma; ++consecutive_;match_valid_=true;reason_="MATCHED";
    } else { reason_=previous_.empty()?"BASELINE":match_.reason=="MATCHED"?"MOTION_OR_TIMING_LIMIT":match_.reason;consecutive_=0;anchored_=false; }
    previous_=std::move(current);scan_stamp_=stamp;last_scan_s_=measurement;last_scan_age_=value(s,"age_s");
    last_roll_=roll;last_pitch_=pitch;last_yaw_=yaw;
    const auto u=s.value("anchor_uwb_xy",state.value("uwb_xy",Json::array()));
    const auto ustamp=s.value("anchor_uwb_stamp_ns",state.value("uwb_stamp_ns",std::int64_t(0)));
    const double uage=value(s,"anchor_uwb_age_s",value(state,"uwb_age_s"));
    if(uage>=0 && uage<=.15 && std::abs(uage-value(s,"age_s"))<=.05 && std::llabs(ustamp-stamp)<=30'000'000
        && ustamp>uwb_stamp_ && u.size()==2 && numeric(u[0]) && numeric(u[1])) {
      if(anchored_ && std::hypot(u[0].get<double>()-anchor_xy_[0]-displacement_[0],u[1].get<double>()-anchor_xy_[1]-displacement_[1])>.2) {
        failed_=true; reason_="UWB_LIDAR_RECOVERY_DISAGREEMENT";
      } else if(consecutive_>=3 && !failed_) {
        anchor_xy_={u[0].get<double>(),u[1].get<double>()};displacement_={0,0};variance_=0;
        anchor_s_=now-uage;uwb_stamp_=ustamp;anchored_=true;
      }
    }
  } else if(!fresh || stamp<scan_stamp_) {
    if(stamp<=0) reason_="LIDAR_STAMP_INVALID_OR_MISSING";
    else if(stamp<scan_stamp_) reason_="LIDAR_BACKWARD_STAMP";
    else if(s.value("frame_id",std::string())!=frame_) reason_="LIDAR_FRAME_MISMATCH";
    else if(s.value("input_source",std::string())!=input_source_) reason_="LIDAR_INPUT_SOURCE_MISMATCH";
    else if(!age(s,"age_s",.15)) reason_="LIDAR_STALE_OR_FUTURE";
    else if(!age(s,"attitude_age_s",.15)) reason_="LIDAR_ATTITUDE_STALE_OR_MISSING";
    else if(std::abs(value(s,"attitude_time_error_s"))>.05) reason_="LIDAR_ATTITUDE_TIME_MISMATCH";
    else if(std::abs(roll)>.05 || std::abs(pitch)>.05 || std::abs(yaw)>2*pi) reason_="LIDAR_TILT_OR_HEADING_LIMIT";
    else reason_="LIDAR_BEAM_TIMING_INVALID";
    match_valid_=false;
  }
  const double scan_age=scan_stamp_?std::max(last_scan_age_,now-last_scan_s_):1e9;
  const double anchor_age=anchored_?now-anchor_s_:1e9;
  const double sigma=std::sqrt(variance_+.02*.02);
  const std::array<double,2> observed={anchor_xy_[0]+displacement_[0],anchor_xy_[1]+displacement_[1]};
  const auto fc=state.value("xy",Json::array()),velocity=state.value("velocity_xy",Json::array());
  const bool fc_ok=fc.size()==2 && numeric(fc[0]) && numeric(fc[1])
      && velocity.size()==2 && numeric(velocity[0]) && numeric(velocity[1])
      && std::hypot(velocity[0].get<double>(),velocity[1].get<double>())<=.15
      && std::hypot(fc[0].get<double>()-observed[0],fc[1].get<double>()-observed[1])<=.2;
  const bool approved=enabled_ && qualified_ && anchored_ && !failed_ && fresh && match_valid_ && consecutive_>=3
      && scan_age<=.15 && anchor_age>=0 && anchor_age<=.8 && sigma<=.08 && fc_ok
      && s.value("beam_quaternions",Json::array()).size()==s.value("ranges",Json::array()).size()
      && s.value("reference_quaternion",Json::array()).size()==4
      && s.value("input_source",std::string())==input_source_ && duration(s)>0;
  return {{"approved",approved},{"enabled",enabled_},{"mode",approved?"LIDAR_GAP_SUPPORT":"SHADOW_OR_UNAVAILABLE"},
      {"reason",failed_?"UWB_LIDAR_RECOVERY_DISAGREEMENT":reason_},{"measurement_stamp_ns",scan_stamp_},
      {"uwb_anchor_stamp_ns",uwb_stamp_},{"scan_age_s",scan_age},{"anchor_age_s",anchor_age},{"sigma_m",sigma},
      {"xy",observed},{"delta_xy",displacement_},{"residual_m",match_.rms},{"overlap",match_.overlap},
      {"normal_diversity",match_.diversity},{"consecutive_matches",consecutive_},{"fc_aiding",false},
      {"input_frame",s.value("frame_id",std::string())},{"input_scan_age_s",value(s,"age_s")},
      {"attitude_age_s",value(s,"attitude_age_s")},{"attitude_time_error_s",value(s,"attitude_time_error_s")},
      {"beam_timing_provided",duration(s)>0 && duration(s)<=.15},
      {"first_beam_stamp_ns",s.value("first_beam_stamp_ns",std::int64_t(0))},
      {"input_source",s.value("input_source",std::string())},
      {"input_error",state.value("lidar_input_error",std::string())},
      {"lineage","fresh_UWB_anchor_plus_independent_LIDAR_displacement"}};
}
} // namespace sangwon::observe
