#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace sangwon {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double a) { return std::remainder(a, 2 * pi); }
struct Vec3 { double x{}, y{}, z{}; };
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 a, double k) { return {a.x*k,a.y*k,a.z*k}; }
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline double norm(Vec3 a) { return std::sqrt(dot(a,a)); }
inline double xy(Vec3 a) { return std::hypot(a.x,a.y); }
inline bool finite(Vec3 a) { return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z); }
struct Pose { Vec3 p; double yaw{}; };
inline bool finite(Pose a) { return finite(a.p)&&std::isfinite(a.yaw); }
// Simple, vertical extrusions in the immutable mission map. Holes are separate
// forbidden volumes; vertex winding does not change their meaning.
struct PolygonVolume { std::vector<Vec3> polygon; double z_min{}, z_max{}; };
struct FlightSpace {
  std::vector<PolygonVolume> forbidden, altitude, yaw, boundary;
  bool boundary_provided{};
};
// Bounded synthetic planner settings. These are not measured flight defaults.
struct PlannerLimits {
  double resolution_m{0.25}, max_length_m{50}, budget_s{0.05};
  unsigned max_nodes{8192}, max_expansions{4096};
  void validate() const;
};
enum class Quality { Unknown, Invalid, Valid };
enum class Mode { Manual, Offboard, Land, Takeoff, Hold, Unknown };
enum class Profile { Replay, Sitl, Flight };
enum class TakeoffPolicy { Offboard, NativePx4 };
enum class ControlOperation { None, Offboard, Takeoff, Land, Arm, Disarm };
enum class ReplyResult { Accepted, InProgress, Rejected };
const char* mode_name(Mode);
const char* control_operation_name(ControlOperation);
struct ControlRequest {
  ControlOperation operation{ControlOperation::None};
  std::string execution_id, boot_session;
  std::uint64_t request_id{}, frame_epoch{}, transform_revision{};
  double issued_s{}, deadline_s{};
  // Map/body-reference target, NOT a MAVLink TAKEOFF AMSL altitude.
  Pose target;
};
struct ControlReply {
  ControlRequest request;
  ReplyResult result{ReplyResult::Rejected};
  double observed_s{};
};
struct ControlCompletion { ControlRequest request; double observed_s{}; };

// Normalized perception contract. Raw camera/UWB protocols belong to adapters.
struct MarkerObservation {
  std::string producer_id, dictionary, calibration_ref, mounting_ref, layout_ref;
  std::uint64_t sequence{}, frame_epoch{};
  int marker_id{-1};
  double observed_s{}, marker_size_m{}, reprojection_px{};
  Pose desired_vehicle_pose_map;
  bool valid{};
};
enum class QrSource { Scanner, Camera };
struct QrObservation {
  std::string producer_id, execution_id, task_id, window_id, raw;
  std::uint64_t sequence{};
  double observed_s{};
  QrSource source{QrSource::Scanner};
};
struct ScanConfig {
  double marker_age_s{0.1}, qr_age_s{0.25}, marker_acquire_s{5}, align_s{10};
  double settle_s{1}, reader_s{2}, camera_s{3}, task_s{40}, persist_s{3};
  double speed_mps{0.05}, position_error_m{0.02}, yaw_error_rad{3*pi/180};
  double hover_speed_mps{0.02};
  double max_adjustment_m{0.1}, max_adjustment_yaw{10*pi/180}, reprojection_px{1};
  double acceleration_mps2{0.05}, yaw_rate_rps{0.1}, yaw_acceleration_rps2{0.2};
  unsigned continuous_marker_samples{3};
  void validate() const;
};
struct ScanPlan {
  std::string task_id, label_id, dictionary, calibration_ref, mounting_ref, layout_ref;
  int marker_id{-1};
  double marker_size_m{};
  Pose staging, reader, camera;
  // Coarse centre envelope. Compiled map plans also retain the physical polygon
  // and immutable map; the envelope alone does not establish collision freedom.
  Vec3 centre_min, centre_max;
  std::map<std::string,std::string> expected_qr_fields;
  ScanConfig config;
  bool space_validated{};
  std::optional<PolygonVolume> workspace;
  std::shared_ptr<const FlightSpace> space;
  double clearance_xy_m{}, clearance_z_m{};
};
struct ScanInput {
  bool camera_healthy{}, scanner_healthy{}, camera_decoder_enabled{};
  std::string camera_activation_id, stored_result_id;
  std::optional<MarkerObservation> marker;
  std::optional<QrObservation> qr;
};

// Engineering fixtures only. No inherited FLIGHT defaults are supported.
struct Config {
  Profile profile{Profile::Replay};
  TakeoffPolicy takeoff_policy{TakeoffPolicy::Offboard};
  double tick_s{0.05}, lease_s{0.5}, pose_age_s{0.25};
  double xy_error_m{0.2}, z_error_m{0.15}, yaw_error_rad{10*pi/180};
  double arrival_speed_mps{0.1}, stable_s{1}, recovery_s{8}, recovery_stable_s{1};
  double prestream_s{1.5}, transition_s{6}, blocked_s{10}, stall_s{5};
  double xy_speed_mps{0.5}, z_speed_mps{0.3}, accel_mps2{0.4};
  double yaw_rate_rps{0.4}, yaw_accel_rps2{0.5};
  double battery_return{0.3}, log_age_s{2};
  double control_retry_s{1}, native_takeoff_timeout_s{20};
  unsigned control_max_attempts{3};
  void validate() const;
};
struct State {
  Pose pose; Vec3 velocity;
  Quality pose_quality{Quality::Unknown}, yaw_quality{Quality::Unknown};
  double pose_time_s{}, mode_time_s{}, quality_time_s{};
  bool connected{}, armed{}, landed{true}, can_hold{}, rc_override{}, px4_failsafe{};
  bool observation_valid{}, route_revalidated{};
  bool blocked{}; // Outbound segment. Return clearance is evaluated separately.
  bool return_route_valid{true};
  bool logging_ok{}, battery_sensor_failed{};
  double log_time_s{}, battery_time_s{};
  std::optional<double> battery;
  Mode mode{Mode::Manual};
  std::uint64_t frame_epoch{1};
  ScanInput scan;
  Quality global_position_quality{Quality::Unknown};
  double global_position_time_s{};
  bool native_takeoff_target_validated{};
  std::optional<ControlReply> control_reply;
  std::optional<ControlCompletion> takeoff_completion;
};
inline bool fresh(double now, double stamp, double limit) {
  return std::isfinite(now)&&std::isfinite(stamp)&&stamp<=now&&now-stamp<=limit;
}
inline bool pose_valid(const State& s, double now, const Config& c) {
  return s.pose_quality==Quality::Valid && finite(s.pose) && finite(s.velocity)
      && fresh(now,s.pose_time_s,c.pose_age_s);
}
inline bool yaw_valid(const State& s, double now) {
  return s.yaw_quality==Quality::Valid && std::isfinite(s.pose.yaw)
      && fresh(now,s.quality_time_s,1.0);
}
inline bool stopped(const State& s, const Config& c) {
  return xy(s.velocity)<=c.arrival_speed_mps && std::abs(s.velocity.z)<=c.arrival_speed_mps;
}
inline bool at_pose(const State& s, Pose goal, const Config& c) {
  const auto d=s.pose.p-goal.p;
  return xy(d)<=c.xy_error_m && std::abs(d.z)<=c.z_error_m
      && std::abs(wrap(s.pose.yaw-goal.yaw))<=c.yaw_error_rad && stopped(s,c);
}
struct Waypoint { Vec3 p; std::optional<double> yaw; double hover_s{}; std::optional<ScanPlan> scan{}; };
struct Mission {
  std::string execution_id;
  std::uint64_t transform_revision{1};
  double takeoff_z_m{1}, requested_speed_mps{0.5};
  std::optional<double> start_yaw;
  std::vector<Waypoint> points;
  bool approved_fixture{}; // Synthetic approved route, NOT a map validation result.
  std::shared_ptr<const FlightSpace> space;
  double clearance_xy_m{}, clearance_z_m{};
  bool global_detour_enabled{};
  PlannerLimits planner;
};
enum class IntentKind { Prestream, Position, Hold, Offboard, Arm, Land, AbortGround, Release, Takeoff };
struct Intent {
  unsigned schema_version{1};
  std::string execution_id, boot_session;
  std::uint64_t generation{}, sequence{}, frame_epoch{}, transform_revision{};
  IntentKind kind{IntentKind::Release};
  Pose target;
  double issued_s{}, expires_s{};
};
enum class OutputKind { None, Position, RequestOffboard, RequestArm, RequestLand, GroundDisarm, Release, RequestTakeoff };
struct Output {
  OutputKind kind{OutputKind::None}; Pose target; std::string reason;
  std::optional<ControlRequest> request{};
  // Stream this fixed target while awaiting an Offboard/arm command. A command
  // request and the continuous Offboard position stream are separate outputs.
  bool stream_position{};
};
enum class Command { Pause, Resume, Cancel, Land };
enum class Phase { Idle, Acquire, Takeoff, StartYaw, Vertical, Travel, Rotate, Dwell,
                   Scan, Return, HomeYaw, Landing, Complete, Manual, Aborted, Handoff };
const char* phase_name(Phase p);
} // namespace sangwon
