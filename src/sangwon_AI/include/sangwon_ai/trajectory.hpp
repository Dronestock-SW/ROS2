#pragma once
#include "sangwon_ai/types.hpp"

namespace sangwon {
// Rest-to-rest, time-parameterized reference. PX4 still owns all feedback loops.
class ScalarProfile {
 public:
  ScalarProfile(double distance, double max_speed, double acceleration);
  struct Sample { double position, speed; };
  Sample sample(double time_s) const;
  double duration() const { return 2*ramp_s_+cruise_s_; }
 private:
  double distance_{}, accel_{}, peak_{}, ramp_s_{}, cruise_s_{};
};
class Segment {
 public:
  Segment(Pose from, Pose to, double speed, const Config& config);
  Pose sample(double elapsed_s) const;
  double duration() const;
  double overshoot(Vec3 observed) const;
 private:
  Pose from_, to_; Vec3 direction_;
  double yaw_delta_{};
  ScalarProfile travel_, yaw_;
};
double stopping_distance(double speed, double deceleration, double latency_s, double margin_m);
} // namespace sangwon
