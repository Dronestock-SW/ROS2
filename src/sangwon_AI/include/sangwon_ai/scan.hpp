#pragma once
#include "sangwon_ai/types.hpp"

namespace sangwon {
enum class ScanStage { Approach, Acquire, Align, Settle, Reader, Camera, Persist, Egress, Complete, Preempted };
const char* scan_stage_name(ScanStage);
struct ScanResult {
  std::string id, task_id, label_id, code, raw;
  unsigned scanner_attempts{};
  bool succeeded{}, camera_used{}, egress_complete{}, preempted{};
  std::optional<QrSource> source;
  std::optional<double> decoded_s;
};
struct ScanDecision {
  Pose target;
  bool move{}, open_reader{}, enable_camera{}, persist{}, complete{}, record_failed{};
  bool flight_failed{};
  std::string window_id;
};
class ScanAction {
 public:
  ScanAction(ScanConfig, Config flight, ScanPlan, std::string execution, double now, double active);
  ScanDecision tick(const State&, double now, double active, bool movement_complete=false);
  void suspend();
  void preempt(const std::string&);
  ScanStage stage() const { return stage_; }
  const std::optional<ScanResult>& result() const { return result_; }
  unsigned attempts() const { return attempts_; }
  const std::string& window_id() const { return window_; }
  const ScanPlan& plan() const { return plan_; }
  const ScanConfig& config() const { return c_; }
 private:
  ScanConfig c_; Config flight_; ScanPlan plan_;
  std::string execution_, window_, marker_producer_, qr_producer_;
  std::uint64_t marker_sequence_{}, qr_sequence_{}, window_sequence_{};
  ScanStage stage_{ScanStage::Approach};
  unsigned attempts_{}, marker_samples_{};
  bool camera_used_{}, camera_mode_{}, suspended_{}, window_open_{};
  double start_active_{}, phase_active_{}, window_open_s_{}, window_active_{}, last_active_{}, last_now_{};
  double camera_budget_used_{};
  std::optional<double> stable_active_;
  std::optional<MarkerObservation> marker_;
  Pose target_;
  std::optional<ScanResult> result_;
  bool inside(Pose) const;
  bool segment_clear(Vec3,Vec3) const;
  bool path_clear(Pose,Pose) const;
  bool marker_valid(const State&, double now);
  bool aligned(const State&, double now);
  Pose marker_target() const;
  bool qr_valid(const QrObservation&, double now) const;
  void enter(ScanStage, double active);
  void finish(bool succeeded, const std::string& code, double active, const QrObservation* qr=nullptr);
  void open(QrSource, double now, double active);
};
} // namespace sangwon
