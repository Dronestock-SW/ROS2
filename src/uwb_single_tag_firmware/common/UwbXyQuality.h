#pragma once
#include <math.h>
#include <stdint.h>

// A consistency check, not an NLOS detector or accuracy guarantee. Correlated
// range biases can describe another physically plausible position and pass.
struct UwbXyQuality {
  bool valid;
  const char* reason;
  double maxPairResidualM;
};

inline UwbXyQuality uwbCheckXyQuality(
    const double ranges[4], const double anchors[4][3], uint8_t mask,
    bool solverValid, double x, double y, double limitM) {
  UwbXyQuality result = {false, "insufficient_anchors", NAN};
  if (mask != 15) return result;
  result.reason = "invalid_geometry";
  if (!solverValid || !isfinite(x) || !isfinite(y) ||
      !isfinite(limitM) || limitM <= 0) return result;
  for (unsigned i = 0; i < 4; ++i) {
    if (!isfinite(ranges[i]) || ranges[i] <= 0) return result;
    for (unsigned k = 0; k < 3; ++k)
      if (!isfinite(anchors[i][k])) return result;
    if (fabs(anchors[i][2] - anchors[0][2]) > 0.001) return result;
  }
  double maximum = 0;
  // All six pairs avoid making the gate depend on which anchor is the LS base.
  for (unsigned i = 0; i < 4; ++i) {
    for (unsigned j = i + 1; j < 4; ++j) {
      const double dx = anchors[j][0] - anchors[i][0];
      const double dy = anchors[j][1] - anchors[i][1];
      const double separation = sqrt(dx * dx + dy * dy);
      if (separation < 1e-6) return result;
      const double rhs = ranges[i] * ranges[i] - ranges[j] * ranges[j] +
          anchors[j][0] * anchors[j][0] + anchors[j][1] * anchors[j][1] -
          anchors[i][0] * anchors[i][0] - anchors[i][1] * anchors[i][1];
      const double residual = fabs(rhs - 2 * (dx * x + dy * y)) / (2 * separation);
      if (!isfinite(residual)) return result;
      if (residual > maximum) maximum = residual;
    }
  }
  result.maxPairResidualM = maximum;
  result.valid = maximum <= limitM;
  result.reason = result.valid ? "consistent" : "inconsistent_ranges";
  return result;
}
