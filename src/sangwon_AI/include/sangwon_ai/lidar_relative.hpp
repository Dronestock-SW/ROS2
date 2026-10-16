#pragma once
#include "sangwon_ai/service/json.hpp"
#include <array>
#include <string>
#include <vector>

namespace sangwon::observe {
struct ScanPoint { double x{}, y{}, nx{}, ny{}; };
struct ScanMatch {
  bool valid{};
  std::string reason{"INSUFFICIENT_POINTS"};
  double dx{}, dy{}, dyaw{}, rms{}, overlap{}, diversity{}, sigma{};
  unsigned correspondences{};
};
// Current scan -> previous scan, point-to-line SE(2). No FC positions as a prior.
ScanMatch match_scans(const std::vector<ScanPoint>& previous,
                      const std::vector<ScanPoint>& current);
std::vector<ScanPoint> scan_points(const service::Json& scan,
                                 const std::array<double,3>& mount={}, double mount_yaw=0);

// Independent displacement monitor, anchored once to a fresh UWB observation.
// It does not publish an EKF input, integrate velocity, or change a target.
class LidarGapSupport {
 public:
  explicit LidarGapSupport(const service::Json& config);
  service::Json update(const service::Json& state, double now);
 private:
  bool enabled_{}, qualified_{}, anchored_{}, failed_{}, match_valid_{};
  std::string frame_{"laser_frame"}, mount_ref_, timing_ref_, scene_ref_, reason_{"MISSING"};
  std::string input_source_{"sdk_pointcloud"};
  std::array<double,3> mount_{};
  int map_sign_{1};
  double mount_yaw_{}, last_scan_s_{}, last_scan_age_{}, anchor_s_{}, variance_{}, last_yaw_{}, last_roll_{}, last_pitch_{};
  std::int64_t scan_stamp_{}, uwb_stamp_{};
  unsigned consecutive_{};
  std::array<double,2> anchor_xy_{}, displacement_{};
  std::vector<ScanPoint> previous_;
  ScanMatch match_;
};
} // namespace sangwon::observe
