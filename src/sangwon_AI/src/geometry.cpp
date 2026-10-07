#include "sangwon_ai/geometry.hpp"
#include <limits>

namespace sangwon {
namespace {
constexpr double eps=1e-8;
double cross(Vec3 a,Vec3 b,Vec3 c) { return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x); }
double point_distance(Vec3 p,Vec3 a,Vec3 b) {
  const auto d=b-a; const double square=d.x*d.x+d.y*d.y;
  const double t=square>0?std::clamp(((p.x-a.x)*d.x+(p.y-a.y)*d.y)/square,0.0,1.0):0;
  return xy(p-(a+d*t));
}
bool on(Vec3 p,Vec3 a,Vec3 b) { return point_distance(p,a,b)<=eps; }
bool intersects_xy(Vec3 a,Vec3 b,Vec3 c,Vec3 d) {
  const auto aa=cross(a,b,c),bb=cross(a,b,d),cc=cross(c,d,a),dd=cross(c,d,b);
  return (((aa>0&&bb<0)||(aa<0&&bb>0))&&((cc>0&&dd<0)||(cc<0&&dd>0)))
      ||on(c,a,b)||on(d,a,b)||on(a,c,d)||on(b,c,d);
}
double segment_distance(Vec3 a,Vec3 b,Vec3 c,Vec3 d) {
  if(intersects_xy(a,b,c,d)) return 0;
  return std::min({point_distance(a,c,d),point_distance(b,c,d),point_distance(c,a,b),point_distance(d,a,b)});
}
bool inside_xy(Vec3 p,const PolygonVolume& v) {
  bool inside=false;
  for(std::size_t i=0;i<v.polygon.size();++i) {
    const auto a=v.polygon[i],b=v.polygon[(i+1)%v.polygon.size()];
    if(on(p,a,b)) return true;
    if((a.y>p.y)!=(b.y>p.y)) {
      const auto edge_x=a.x+(p.y-a.y)*(b.x-a.x)/(b.y-a.y);
      if(p.x<edge_x) inside=!inside;
    }
  }
  return inside;
}
bool margins(double horizontal,double vertical) {
  return std::isfinite(horizontal)&&std::isfinite(vertical)&&horizontal>=0&&vertical>=0;
}
bool covered(const std::vector<PolygonVolume>& volumes,Vec3 a,Vec3 b,double h,double v,bool ground) {
  for(const auto& volume:volumes) if(volume_covers(volume,a,b,h,v,ground)) return true;
  return false;
}
}
void validate_volume(const PolygonVolume& v) {
  const auto n=v.polygon.size();
  if(n<3||n>256||!std::isfinite(v.z_min)||!std::isfinite(v.z_max)||v.z_min>=v.z_max
      ||std::abs(v.z_min)>1e6||std::abs(v.z_max)>1e6) throw std::invalid_argument("INVALID_MAP_GEOMETRY");
  double area=0;
  const auto origin=v.polygon[0];
  for(const auto a:v.polygon) if(!finite(a)||std::abs(a.x)>1e6||std::abs(a.y)>1e6||a.z!=0)
    throw std::invalid_argument("INVALID_MAP_GEOMETRY");
  for(std::size_t i=0;i<n;++i) {
    const auto a=v.polygon[i],b=v.polygon[(i+1)%n],c=v.polygon[(i+2)%n];
    if(xy(b-a)<=eps)
      throw std::invalid_argument("INVALID_MAP_GEOMETRY");
    // Consecutive collinear vertices may extend an edge, but never reverse it.
    const auto next=c-b,edge=b-a;
    if(std::abs(cross(a,b,c))<=eps&&next.x*edge.x+next.y*edge.y<=0)
      throw std::invalid_argument("INVALID_MAP_GEOMETRY");
    area+=cross(origin,a,b);
    for(std::size_t j=i+1;j<n;++j) {
      if(j==i+1||(i==0&&j==n-1)) continue;
      if(intersects_xy(a,b,v.polygon[j],v.polygon[(j+1)%n])) throw std::invalid_argument("INVALID_MAP_GEOMETRY");
    }
  }
  if(!std::isfinite(area)||std::abs(area)<=eps) throw std::invalid_argument("INVALID_MAP_GEOMETRY");
}
void validate_space(const FlightSpace& s) {
  if(s.altitude.empty()||s.yaw.empty()||(s.boundary_provided&&s.boundary.empty())
      ||s.forbidden.size()+s.altitude.size()+s.yaw.size()+s.boundary.size()>2048)
    throw std::invalid_argument("INVALID_FLIGHT_SPACE");
  for(const auto* group:{&s.forbidden,&s.altitude,&s.yaw,&s.boundary})
    for(const auto& v:*group) validate_volume(v);
}
VolumeBounds volume_bounds(const PolygonVolume& v) {
  VolumeBounds bounds{{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),v.z_min},
    {-std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),v.z_max}};
  for(const auto p:v.polygon) {
    bounds.lo.x=std::min(bounds.lo.x,p.x); bounds.hi.x=std::max(bounds.hi.x,p.x);
    bounds.lo.y=std::min(bounds.lo.y,p.y); bounds.hi.y=std::max(bounds.hi.y,p.y);
  }
  return bounds;
}
bool volume_covers(const PolygonVolume& v,Vec3 a,Vec3 b,double h,double vertical,bool ground) {
  if(v.polygon.size()<3||!finite(a)||!finite(b)||!margins(h,vertical)) return false;
  // Ground exemption applies only to a zone that actually reaches the floor.
  const auto lower=ground&&v.z_min<=0?std::min(v.z_min+vertical,0.0):v.z_min+vertical;
  const auto upper=v.z_max-vertical;
  if(a.z<lower-eps||b.z<lower-eps||a.z>upper+eps||b.z>upper+eps||lower>upper
      ||!inside_xy(a,v)||!inside_xy(b,v)) return false;
  for(std::size_t i=0;i<v.polygon.size();++i)
    if(segment_distance(a,b,v.polygon[i],v.polygon[(i+1)%v.polygon.size()])<=h+eps) return false;
  return true;
}
bool volume_intersects(const PolygonVolume& v,Vec3 a,Vec3 b,double h,double vertical) {
  if(v.polygon.size()<3||!finite(a)||!finite(b)||!margins(h,vertical)) return true;
  // Only the portion crossing the inflated height range can hit its footprint.
  double low=0,high=1;
  const auto lower=v.z_min-vertical-eps,upper=v.z_max+vertical+eps,dz=b.z-a.z;
  if(std::abs(dz)<=eps) { if(a.z<lower||a.z>upper) return false; }
  else {
    auto first=(lower-a.z)/dz,last=(upper-a.z)/dz;
    if(first>last) std::swap(first,last);
    low=std::max(low,first);high=std::min(high,last);if(low>high) return false;
  }
  const auto delta=b-a; b=a+delta*high; a=a+delta*low;
  if(inside_xy(a,v)||inside_xy(b,v)) return true;
  for(std::size_t i=0;i<v.polygon.size();++i)
    if(segment_distance(a,b,v.polygon[i],v.polygon[(i+1)%v.polygon.size()])<=h+eps) return true;
  return false;
}
const char* space_segment_error(const FlightSpace& s,Vec3 a,Vec3 b,double h,double v,bool ground) {
  if(!finite(a)||!finite(b)||!margins(h,v)) return "INVALID_COORDINATE";
  for(const auto& volume:s.forbidden) if(volume_intersects(volume,a,b,h,v)) return "ROUTE_OBSTRUCTED";
  if(!covered(s.altitude,a,b,h,v,ground)) return "ALTITUDE_ZONE_UNCOVERED";
  if(!covered(s.yaw,a,b,h,v,ground)) return "YAW_ZONE_UNCOVERED";
  if(s.boundary_provided&&!covered(s.boundary,a,b,h,v,ground)) return "FLIGHT_BOUNDARY_EXCEEDED";
  return "";
}
} // namespace sangwon
