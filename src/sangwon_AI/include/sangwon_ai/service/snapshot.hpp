#pragma once
#include "sangwon_ai/types.hpp"
#include "sangwon_ai/service/json.hpp"

namespace sangwon::service {
struct CompiledSnapshot {
  Mission mission;
  Config motion;
  Pose launch;
  bool has_scan{};
  std::string validation_scope{"ALLOWLISTED_SYNTHETIC_ROUTE_ONLY"};
};
// Only explicit, hash-allowlisted REPLAY input. Optional bounded static XY
// detour requires provided boundaries, capability and approved local limits.
// Simple polygon volumes and horizontal, front-facing labels are supported.
// Allowed segments must fit in one volume per altitude/yaw/boundary group.
// Missing sensor extrinsics are never inferred for physical flight.
CompiledSnapshot compile_replay_snapshot(const Json& snapshot, const Json& config, double utc_now);
} // namespace sangwon::service
