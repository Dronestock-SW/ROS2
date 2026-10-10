#pragma once
#include "sangwon_ai/service/json.hpp"
#include <cstdint>
#include <map>

namespace sangwon::observe {
// Observation-only adapter. No dependency on State, Runtime, ROS or FC output.
// Receipt clocks belong to one capture host boot; header stamps stay in ROS time.
class CaptureObservations {
 public:
  explicit CaptureObservations(const service::Json& manifest);
  void ingest(const service::Json& captured_row);
  service::Json snapshot(std::int64_t receipt_monotonic_ns) const;
  std::int64_t last_receipt_ns() const { return last_receipt_ns_; }
 private:
  struct Stream {
    std::uint64_t received{}, accepted{};
    std::int64_t receipt_ns{}, accepted_ns{}, high_header_ns{}, header_ns{}, ros_ns{};
    std::int64_t max_receipt_gap_ns{}, max_accepted_gap_ns{};
    bool valid{};
    std::string reason{"MISSING"};
    service::Json value;
    std::map<std::string, std::uint64_t> rejections;
  };
  service::Json identity_;
  std::map<std::string, Stream> streams_;
  std::uint64_t rows_{}, ignored_{};
  std::int64_t last_receipt_ns_{};
};
}  // namespace sangwon::observe
