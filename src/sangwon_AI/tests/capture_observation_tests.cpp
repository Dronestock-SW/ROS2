#include "sangwon_ai/capture_observations.hpp"
#include <iostream>
#include <functional>

using sangwon::service::Json;
using sangwon::observe::CaptureObservations;
namespace {
constexpr std::int64_t wall = 1791544480000000000LL;
void check(bool v, const char* reason) { if (!v) throw std::runtime_error(reason); }
const Json manifest{{"schema", 1}, {"scope", "manual_flight_receive_only"}, {"tag", "B"},
  {"tag_id", "6"}, {"ros_domain_id", 2}, {"boot_id", "fixture-boot"}, {"source_revision", "fixture"}};
Json cov(std::size_t n, double variance) {
  auto out = Json::array(); for (std::size_t i = 0; i < n*n; ++i) out.push_back(i/n == i%n ? variance : 0.);
  return out;
}
const Json v{{"x", 1.}, {"y", 2.}, {"z", 3.}};
const Json q{{"x", 0.}, {"y", 0.}, {"z", 0.}, {"w", 1.}};
Json pose(double variance) { return {{"pose", {{"position", v}, {"orientation", q}}}, {"covariance", cov(6, variance)}}; }
Json row(const std::string& topic, const std::string& type, const std::string& frame, Json data,
         std::int64_t mono = 1000000000, std::int64_t age = 10000000) {
  const auto header = wall + mono - age;
  data["header"] = {{"frame_id", frame}, {"stamp", {{"sec", header/1000000000}, {"nanosec", header%1000000000}}}};
  return {{"topic", topic}, {"type", type}, {"received_monotonic_ns", mono},
    {"received_ros_ns", wall+mono}, {"header_ns", header}, {"data", data}};
}
Json uwb(std::int64_t mono = 1000000000, std::int64_t age = 10000000) {
  auto p = pose(.09); p["pose"]["position"]["z"] = 0.;
  return row("/uwb/btf_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", "uwb_map", {{"pose", p}}, mono, age);
}
Json stream(const CaptureObservations& c, const char* key, std::int64_t now = 1000000000) {
  return c.snapshot(now).at("streams").at(key);
}
void refused(const std::function<void()>& f) {
  bool failed = false; try { f(); } catch (const std::exception&) { failed = true; }
  check(failed, "expected rejection");
}
void reject_uwb(Json r, const char* reason) {
  CaptureObservations c(manifest); c.ingest(r);
  const auto s = stream(c, "uwb_xy", c.last_receipt_ns());
  check(s.at("reason") == reason && s.at("value").is_null() && s.at("accepted") == 0, reason);
}
}  // namespace
int main() {
  try {
    auto bad = manifest; bad["tag_id"] = "5"; refused([&] { CaptureObservations c(bad); });
    bad = manifest; bad["ros_domain_id"] = 1; refused([&] { CaptureObservations c(bad); });
    bad = manifest; bad["boot_id"] = nullptr; refused([&] { CaptureObservations c(bad); });
    bad = manifest; bad["config_evidence"] = Json::array({{{"name", "fixture.json"},
      {"sha256", "bad"}, {"content_utf8", "{}"}}});
    refused([&] { CaptureObservations c(bad); });
    bad["config_evidence"][0]["sha256"] = sangwon::service::digest("{}");
    CaptureObservations bound(bad);
    check(bound.snapshot(1)["identity"]["config_refs"][0]["sha256"] == sangwon::service::digest("{}"), "config binding");
    auto a = manifest; a["tag"] = "A"; a["tag_id"] = "5"; a["ros_domain_id"] = 1; CaptureObservations tag_a(a);
    CaptureObservations c(manifest);
    check(stream(c, "uwb_xy").at("reason") == "MISSING", "missing");
    c.ingest(uwb()); auto s = stream(c, "uwb_xy");
    check(s.at("sample_valid") && s.at("header_ns") == wall+990000000, "ns precision");
    check(s["value"]["xy_m"] == Json::array({1.,2.}) && s["value"]["z_m"].is_null()
      && s["value"]["yaw_rad"].is_null() && s["value"]["position_reference"] == "uwb_antenna", "xy only");
    check(stream(c, "uwb_xy", 1240000000).at("sample_valid"), "original age plus elapsed boundary");
    check(stream(c, "uwb_xy", 1240000001).at("reason") == "EXPIRED", "expired");
    c.ingest(uwb(1020000000, 30000000));
    check(stream(c, "uwb_xy", 1020000000).at("reason") == "HEADER_NOT_NEW", "duplicate must invalidate");
    c.ingest(uwb(1040000000));
    check(stream(c, "uwb_xy", 1040000000).at("accepted") == 2, "same coordinates new sample accepted");
    refused([&] { c.ingest(uwb()); });
    refused([&] { c.snapshot(1); });
    auto r = uwb(); r["data"]["header"]["frame_id"] = "map"; reject_uwb(r, "FRAME_MISMATCH");
    r = uwb(); r["type"] = "geometry_msgs/msg/PoseStamped"; reject_uwb(r, "MESSAGE_TYPE_MISMATCH");
    r = uwb(); r["data"]["pose"]["pose"]["position"]["x"] = "NaN"; reject_uwb(r, "NONFINITE_OR_MISSING");
    r = uwb(); r["data"]["pose"]["covariance"][0] = 0.; reject_uwb(r, "UWB_VARIANCE_UNKNOWN");
    r = uwb(); r["data"]["pose"]["covariance"][1] = .2; reject_uwb(r, "COVARIANCE_ASYMMETRIC");
    r["data"]["pose"]["covariance"][6] = .2; reject_uwb(r, "COVARIANCE_NOT_PSD");
    r = uwb(); r["header_ns"] = wall; reject_uwb(r, "HEADER_COPY_MISMATCH");
    reject_uwb(uwb(1000000000, 250000001), "STALE_AT_RECEIPT");
    reject_uwb(uwb(1000000000, -1), "FUTURE_HEADER");
    CaptureObservations px4(manifest);
    Json odom{{"pose", pose(0)}, {"child_frame_id", "base_link"},
      {"twist", {{"twist", {{"linear", v}, {"angular", v}}}, {"covariance", cov(6, 0)}}}};
    px4.ingest(row("/mavros/local_position/odom", "nav_msgs/msg/Odometry", "map", odom));
    s = stream(px4, "px4_odom");
    check(s.at("sample_valid") && s["value"]["pose_covariance_nonzero"] == false
      && s["value"]["position_m"] == Json::array({1.,2.,3.}), "PX4 zero covariance remains unknown");
    odom["pose"]["pose"]["orientation"]["w"] = 0.;
    px4.ingest(row("/mavros/local_position/odom", "nav_msgs/msg/Odometry", "map", odom, 1100000000));
    check(stream(px4, "px4_odom", 1100000000).at("value").is_null(), "bad latest pose cannot hold old valid pose");
    CaptureObservations tof(manifest);
    Json range{{"range", .01}, {"min_range", .1}, {"max_range", 35.}};
    tof.ingest(row("/mavros/downward_0", "sensor_msgs/msg/Range", "downward_0", range));
    check(stream(tof, "tof_0").at("reason") == "RANGE_OUT_OF_BOUNDS", "no ground substitute");
    range["range"] = .35;
    tof.ingest(row("/mavros/downward_0", "sensor_msgs/msg/Range", "downward_0", range, 1100000000));
    s = stream(tof, "tof_0", 1100000000);
    check(s.at("sample_valid") && s["value"]["warehouse_z_m"].is_null(), "slant range is not world z");
    range["range"] = "Infinity";
    tof.ingest(row("/mavros/downward_0", "sensor_msgs/msg/Range", "downward_0", range, 1200000000));
    check(stream(tof, "tof_0", 1200000000).at("value").is_null(), "nonfinite sensor");
    // Unknown channels still advance the replay clock, expiring old observations.
    tof.ingest(row("/uwb/received", "std_msgs/msg/String", "", {}, 2000000000));
    const auto report = tof.snapshot(2000000000);
    check(report.at("ignored_rows") == 1, "unknown topic count");
    for (const auto* key : {"can_start", "flight_authority", "physical_output_enabled", "alignment_confirmed", "timing_confirmed", "fusion_verified", "mounting_confirmed", "source_sample_time_verified"})
      check(report.at(key) == false, "no authority promotion");
    // Time sync offsets are preserved as integers, not rounded into doubles.
    CaptureObservations timesync(manifest);
    timesync.ingest(row("/mavros/timesync_status", "mavros_msgs/msg/TimesyncStatus", "",
      {{"remote_timestamp_ns", 999}, {"observed_offset_ns", wall+7}, {"estimated_offset_ns", wall+9}, {"round_trip_time_ms", 1.2}}));
    check(stream(timesync, "timesync")["value"]["estimated_offset_ns"] == wall+9, "time precision");
    CaptureObservations status(manifest);
    status.ingest(row("/mavros/state", "mavros_msgs/msg/State", "", {{"connected", true}, {"armed", true}, {"mode", "POSCTL"}}));
    status.ingest(row("/mavros/extended_state", "mavros_msgs/msg/ExtendedState", "", {{"landed_state", 2}, {"vtol_state", 0}}));
    status.ingest(row("/mavros/rc/in", "mavros_msgs/msg/RCIn", "", {{"channels", Json::array({1000,1500,2000})}, {"rssi", 255}}));
    Json flags;
    for (const auto* key : {"attitude_status_flag", "velocity_horiz_status_flag", "velocity_vert_status_flag",
      "pos_horiz_rel_status_flag", "pos_horiz_abs_status_flag", "pos_vert_abs_status_flag", "pos_vert_agl_status_flag",
      "const_pos_mode_status_flag", "pred_pos_horiz_rel_status_flag", "pred_pos_horiz_abs_status_flag",
      "gps_glitch_status_flag", "accel_error_status_flag"}) flags[key] = true;
    status.ingest(row("/mavros/estimator_status", "mavros_msgs/msg/EstimatorStatus", "", flags));
    const auto status_report = status.snapshot(1000000000);
    check(status_report["streams"]["estimator"]["value"]["flags"]["const_pos_mode_status_flag"] == true
      && status_report["streams"]["estimator"]["value"]["fusion_verified"] == false
      && status_report["streams"]["rc"]["value"]["takeover_verified"] == false
      && status_report["can_start"] == false, "FC flags and sticks cannot promote authority");
    CaptureObservations imu(manifest);
    Json inertial{{"orientation", q}, {"orientation_covariance", cov(3, .01)},
      {"angular_velocity", v}, {"linear_acceleration", v}};
    imu.ingest(row("/mavros/imu/data", "sensor_msgs/msg/Imu", "base_link", inertial));
    check(stream(imu, "imu").at("sample_valid"), "IMU");
    inertial["orientation_covariance"][0] = -1;
    imu.ingest(row("/mavros/imu/data", "sensor_msgs/msg/Imu", "base_link", inertial, 1100000000));
    check(stream(imu, "imu", 1100000000).at("reason") == "COVARIANCE_NEGATIVE", "unknown IMU orientation");
    std::cout << "PASS capture identity, source clocks, duplicate/expiry, frames, covariance, XY-only, sensor absence and authority boundaries\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
