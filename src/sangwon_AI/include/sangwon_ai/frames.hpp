#pragma once
#include "sangwon_ai/types.hpp"

namespace sangwon {
// Surveyed rigid map -> ROS local ENU transform. MAVROS performs ENU -> NED later.
struct MapToEnu {
  Vec3 translation;
  double rotation_rad{};
  std::uint64_t revision{};
  Pose apply(Pose map) const {
    if(!finite(map)||!finite(translation)||!std::isfinite(rotation_rad)||revision==0)
      throw std::invalid_argument("Unverified map transform");
    const double c=std::cos(rotation_rad),s=std::sin(rotation_rad);
    return {{c*map.p.x-s*map.p.y+translation.x,s*map.p.x+c*map.p.y+translation.y,
             map.p.z+translation.z},wrap(map.yaw+rotation_rad)};
  }
};
} // namespace sangwon
