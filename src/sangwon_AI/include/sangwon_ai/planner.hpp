#pragma once
#include "sangwon_ai/geometry.hpp"

namespace sangwon {
struct PlannedRoute {
  std::vector<Vec3> points; // Original start and exact goal, never task replacements.
  std::string code;
  unsigned expanded{};
  double length_m{};
  bool ok() const { return code=="OK"; }
};
// Call only with a validated immutable space. Horizontal, bounded REPLAY A*;
// each returned continuous segment is checked with all original margins.
PlannedRoute plan_horizontal_route(const FlightSpace&,Vec3 start,Vec3 goal,
                                  double horizontal_margin,double vertical_margin,
                                  const PlannerLimits& = {});
} // namespace sangwon
