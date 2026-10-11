#pragma once
#include "sangwon_ai/types.hpp"

namespace sangwon {
struct VolumeBounds { Vec3 lo, hi; };
void validate_volume(const PolygonVolume&);
void validate_space(const FlightSpace&);
VolumeBounds volume_bounds(const PolygonVolume&);
bool volume_covers(const PolygonVolume&, Vec3 a, Vec3 b,
                   double horizontal_margin, double vertical_margin, bool ground=false);
bool volume_intersects(const PolygonVolume&, Vec3 a, Vec3 b,
                      double horizontal_margin, double vertical_margin);
// Empty means safe. Every allowed group must contain the whole segment within
// one of its volumes. Different proven segments may use different volumes;
// this conservative test never bridges an uncovered gap or assumes a union.
const char* space_segment_error(const FlightSpace&, Vec3 a, Vec3 b,
                               double horizontal_margin, double vertical_margin, bool ground=false);
} // namespace sangwon
