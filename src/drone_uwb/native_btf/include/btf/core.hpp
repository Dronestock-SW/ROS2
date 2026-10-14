#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace btf {
using Time = std::int64_t;
using XY = std::array<double, 2>;
using Anchors = std::array<std::array<double, 3>, 4>;
struct Settings {
  double window_s = .8, sigma_r_m = .08, huber_m = .12;
  double max_scaled_condition = 1e6, reset_gap_s = .15;
  double max_extrapolation_s = .04, max_range_m = 80.;
  int min_samples_per_anchor = 3, min_samples_total = 12;
  int required_anchor_count = 4, h80_irls_iterations = 8;
  void validate() const;
};
struct Config {
  Anchors anchors{};
  std::array<double, 4> bias_m{};
  Settings solver;
  double rms_limit_m = .06, score_margin_m = .015;
  double max_speed_m_s = 1., continuity_margin_m = .1, max_prior_age_s = .1;
  bool enforce_b4_height_integrity = false, ground_subset_initialization = false;
  bool fresh_b4_recovery_enabled = false;
  void validate() const;
};
struct RangeSample { Time stamp_us; int anchor; double range_m; };
struct Fit {
  bool ok = false, converged = false;
  std::string reason = "invalid_input";
  std::optional<XY> xy, velocity;
  std::optional<double> rms_m, max_m, condition;
  std::array<double, 7> coefficients{};
  std::array<int, 4> counts{};
  int n = 0, mask = 0, iterations = 0, rank = 0;
};
Fit fit_h80(const Anchors&, const std::vector<RangeSample>&, Time reference_us,
            const Settings&);

// Adapter boundary: validated tag/session, calibrated clock, and measured
// antenna heights. No FC XY, commanded XY, or integrated IMU positions enter.
enum class HeightSource { Missing, MeasuredTofImu, GroundReference };
struct Frame {
  Time start_us = 0, end_us = 0;
  std::array<Time, 4> sample_us{};
  std::array<double, 4> raw_range_m{};
  std::array<std::optional<double>, 4> height_m{};
  HeightSource height_source = HeightSource::Missing;
};
struct Hypothesis {
  int excluded_anchor = 0;  // A1..A4; never a zero-based public identifier.
  Fit fit;
  std::optional<double> physical_range_rms_m;
};
struct Decision {
  bool ok = false;
  Time source_stamp_us = 0;
  std::optional<XY> xy;
  std::string source = "rejected", reason = "B_not_fresh_valid";
  Fit b4;
  std::vector<Hypothesis> hypotheses;
  std::optional<double> best_score_m, score_margin_m;
  std::optional<double> tracking_prior_age_s, tracking_step_m, tracking_limit_m;
  std::optional<double> b4_physical_range_rms_m;
  int best_excluded_anchor = 0;
  std::size_t history_samples = 0;
  int fresh_observation_count = 0;
  std::optional<Time> newest_sample_us;
  std::string reset_reason, b4_height_reason;
  int recovery_frames = 0;
  std::optional<double> recovery_raw_residual_m;
  bool b4_height_blocked = false;
};
class Processor {
 public:
  explicit Processor(Config);
  ~Processor();
  Processor(Processor&&) noexcept;
  Processor& operator=(Processor&&) noexcept;
  Processor(const Processor&) = delete;
  Processor& operator=(const Processor&) = delete;
  Decision process(const Frame&);
  void reset();  // Use at an independently validated session/clock boundary.
  void reset_models(bool preserve_motion, bool preserve_height_conflict = true);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace btf
