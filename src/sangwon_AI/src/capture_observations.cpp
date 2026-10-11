#include "sangwon_ai/capture_observations.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sangwon::observe {
namespace {
using service::Json;
using service::require;
struct Spec { const char* channel; const char* type; std::int64_t age_ns; };
const std::map<std::string, Spec> specs{
  {"/scan", {"lidar_scan", "sensor_msgs/msg/LaserScan", 250000000}},
  {"/uwb/btf_pose", {"uwb_xy", "geometry_msgs/msg/PoseWithCovarianceStamped", 250000000}},
  {"/mavros/local_position/odom", {"px4_odom", "nav_msgs/msg/Odometry", 250000000}},
  {"/mavros/imu/data", {"imu", "sensor_msgs/msg/Imu", 250000000}},
  {"/mavros/downward_0", {"tof_0", "sensor_msgs/msg/Range", 250000000}},
  {"/mavros/downward_1", {"tof_1", "sensor_msgs/msg/Range", 250000000}},
  {"/mavros/estimator_status", {"estimator", "mavros_msgs/msg/EstimatorStatus", 500000000}},
  {"/mavros/state", {"state", "mavros_msgs/msg/State", 1500000000}},
  {"/mavros/extended_state", {"landed", "mavros_msgs/msg/ExtendedState", 1500000000}},
  {"/mavros/rc/in", {"rc", "mavros_msgs/msg/RCIn", 1500000000}},
  {"/mavros/timesync_status", {"timesync", "mavros_msgs/msg/TimesyncStatus", 500000000}}
};
std::int64_t integer(const Json& j, std::int64_t lo, std::int64_t hi) {
  require(j.is_number_integer() && j >= lo && j <= hi, "INTEGER_INVALID");
  return j.get<std::int64_t>();
}
std::int64_t ns(const Json& j) { return integer(j, 1, std::numeric_limits<std::int64_t>::max()); }
double number(const Json& j) {
  require(j.is_number() && std::isfinite(j.get<double>()), "NONFINITE_OR_MISSING");
  return j.get<double>();
}
bool boolean(const Json& j) { require(j.is_boolean(), "BOOLEAN_REQUIRED"); return j.get<bool>(); }
std::string text(const Json& j) {
  require(j.is_string() && !j.get_ref<const std::string&>().empty()
    && j.get_ref<const std::string&>().size() <= 160, "TEXT_REQUIRED");
  return j.get<std::string>();
}
Json vector(const Json& j) {
  return Json::array({number(j.at("x")), number(j.at("y")), number(j.at("z"))});
}
Json quaternion(const Json& q) {
  auto result = vector(q); result.push_back(number(q.at("w")));
  double norm = 0; for (const auto& x : result) norm += x.get<double>() * x.get<double>();
  require(std::abs(norm - 1.) <= .02, "QUATERNION_INVALID");
  return result;
}
// Small symmetric PSD check, including semidefinite zero matrices. Zero output
// covariance remains UNKNOWN, never perfect confidence. No variance is invented.
bool covariance(const Json& j, std::size_t n) {
  require(j.is_array() && j.size() == n*n, "COVARIANCE_SHAPE");
  std::vector<double> a; double scale = 1.; bool nonzero = false;
  for (const auto& v : j) { const auto x = number(v); a.push_back(x); scale = std::max(scale, std::abs(x)); nonzero |= x != 0.; }
  const auto tolerance = 1e-10 * scale;
  for (std::size_t i = 0; i < n; ++i) {
    require(a[i*n+i] >= 0., "COVARIANCE_NEGATIVE");
    for (std::size_t k = 0; k < i; ++k)
      require(std::abs(a[i*n+k] - a[k*n+i]) <= tolerance, "COVARIANCE_ASYMMETRIC");
  }
  for (std::size_t k = 0; k < n; ++k) {
    require(a[k*n+k] >= -tolerance, "COVARIANCE_NOT_PSD");
    if (a[k*n+k] <= tolerance) {
      for (std::size_t i = k+1; i < n; ++i) require(std::abs(a[i*n+k]) <= tolerance, "COVARIANCE_NOT_PSD");
      continue;
    }
    for (std::size_t i = k+1; i < n; ++i)
      for (std::size_t z = i; z < n; ++z) {
        a[z*n+i] -= a[i*n+k] * a[z*n+k] / a[k*n+k]; a[i*n+z] = a[z*n+i];
      }
  }
  return nonzero;
}
Json fields(const std::string& channel, const Json& d) {
  const auto frame = d.at("header").at("frame_id");
  if (channel == "lidar_scan") {
    text(frame); // Sensor frame is retained; it is not assumed to be base_link/map.
    const auto lo = number(d.at("range_min")), hi = number(d.at("range_max"));
    const auto begin = number(d.at("angle_min")), end = number(d.at("angle_max"));
    const auto step = number(d.at("angle_increment"));
    const auto dt = number(d.at("time_increment")), period = number(d.at("scan_time"));
    const auto& ranges = d.at("ranges"); const auto& intensity = d.at("intensities");
    require(lo >= 0 && hi > lo && step > 0 && end > begin, "SCAN_GEOMETRY_INVALID");
    require(ranges.is_array() && ranges.size() >= 2 && ranges.size() <= 65536, "SCAN_SIZE_INVALID");
    require(intensity.is_array() && (intensity.empty() || intensity.size() == ranges.size()), "SCAN_INTENSITY_SIZE");
    require(std::abs(begin + step * (ranges.size()-1) - end) <= step * .51, "SCAN_ANGLE_COUNT_MISMATCH");
    require(dt >= 0 && period > 0 && dt * (ranges.size()-1) <= period * 1.01, "SCAN_TIMING_INVALID");
    auto clean = Json::array(); std::size_t valid = 0;
    for (const auto& r : ranges) {
      // Capture encodes NaN/Inf as strings. Missing returns are not free space.
      require(r.is_number() || (r.is_string() && (r == "NaN" || r == "Infinity" || r == "-Infinity")), "SCAN_RANGE_ENCODING");
      const auto x = r.is_number() ? r.get<double>() : std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(x) && lo <= x && x <= hi) { clean.push_back(x); ++valid; }
      else clean.push_back(nullptr);
    }
    require(valid >= 2, "SCAN_NO_USABLE_RETURNS");
    return {{"frame_id", frame}, {"ranges_m", clean}, {"valid_returns", valid},
      {"range_min_m", lo}, {"range_max_m", hi}, {"angle_min_rad", begin},
      {"angle_max_rad", end}, {"angle_increment_rad", step}, {"time_increment_s", dt},
      {"scan_time_s", period}, {"beam_timing_available", dt > 0},
      {"invalid_returns_are_free_space", false}, {"deskew_applied", false},
      {"mounting_confirmed", false}, {"scan_matching_available", false},
      {"px4_aiding_ready", false}, {"z_observed", false}};
  }
  if (channel == "uwb_xy") {
    require(frame == "uwb_map", "FRAME_MISMATCH");
    const auto& p = d.at("pose").at("pose").at("position");
    const auto& c = d.at("pose").at("covariance");
    require(c.is_array() && c.size() == 36, "COVARIANCE_SHAPE");
    Json xy = Json::array({c.at(0), c.at(1), c.at(6), c.at(7)});
    covariance(xy, 2);
    require(number(c.at(0)) > 0 && number(c.at(7)) > 0, "UWB_VARIANCE_UNKNOWN");
    return {{"frame_id", frame}, {"position_reference", "uwb_antenna"},
      {"xy_m", Json::array({number(p.at("x")), number(p.at("y"))})},
      {"covariance_xy_m2", xy}, {"z_observed", false}, {"yaw_observed", false},
      {"z_m", nullptr}, {"yaw_rad", nullptr}, {"map_transform_applied", false}};
  }
  if (channel == "px4_odom") {
    require(frame == "map" && d.at("child_frame_id") == "base_link", "FRAME_MISMATCH");
    const auto& p = d.at("pose"); const auto& t = d.at("twist");
    const bool pc = covariance(p.at("covariance"), 6), tc = covariance(t.at("covariance"), 6);
    return {{"frame_id", frame}, {"position_axes", "ENU"}, {"child_frame_id", "base_link"},
      {"position_m", vector(p.at("pose").at("position"))},
      {"quaternion_xyzw", quaternion(p.at("pose").at("orientation"))},
      {"velocity_body_flu_mps", vector(t.at("twist").at("linear"))},
      {"angular_body_flu_rps", vector(t.at("twist").at("angular"))},
      {"pose_covariance", p.at("covariance")}, {"twist_covariance", t.at("covariance")},
      {"pose_covariance_nonzero", pc}, {"twist_covariance_nonzero", tc},
      {"warehouse_transform_applied", false}};
  }
  if (channel == "imu") {
    require(frame == "base_link", "FRAME_MISMATCH");
    const bool known = covariance(d.at("orientation_covariance"), 3);
    return {{"frame_id", frame}, {"quaternion_xyzw", quaternion(d.at("orientation"))},
      {"orientation_covariance", d.at("orientation_covariance")}, {"orientation_covariance_nonzero", known},
      {"angular_body_flu_rps", vector(d.at("angular_velocity"))},
      {"acceleration_body_flu_mps2", vector(d.at("linear_acceleration"))}};
  }
  if (channel == "tof_0" || channel == "tof_1") {
    require(frame == (channel == "tof_0" ? "downward_0" : "downward_1"), "FRAME_MISMATCH");
    const auto low = number(d.at("min_range")), high = number(d.at("max_range")), range = number(d.at("range"));
    require(low >= 0 && high > low && range >= low && range <= high, "RANGE_OUT_OF_BOUNDS");
    return {{"frame_id", frame}, {"slant_range_m", range}, {"min_range_m", low},
      {"max_range_m", high}, {"warehouse_z_m", nullptr}, {"ground_substitute_used", false}};
  }
  if (channel == "estimator") {
    Json flags = Json::object();
    for (const auto* key : {"attitude_status_flag", "velocity_horiz_status_flag", "velocity_vert_status_flag",
      "pos_horiz_rel_status_flag", "pos_horiz_abs_status_flag", "pos_vert_abs_status_flag", "pos_vert_agl_status_flag",
      "const_pos_mode_status_flag", "pred_pos_horiz_rel_status_flag", "pred_pos_horiz_abs_status_flag",
      "gps_glitch_status_flag", "accel_error_status_flag"}) flags[key] = boolean(d.at(key));
    return {{"flags", flags}, {"fusion_verified", false}};
  }
  if (channel == "state") return {{"connected", boolean(d.at("connected"))},
    {"armed", boolean(d.at("armed"))}, {"mode", text(d.at("mode"))}};
  if (channel == "landed") return {{"landed_state", integer(d.at("landed_state"), 0, 4)},
    {"vtol_state", integer(d.at("vtol_state"), 0, 4)}};
  if (channel == "rc") {
    const auto& channels = d.at("channels"); require(channels.is_array() && channels.size() <= 32, "RC_CHANNELS_INVALID");
    for (const auto& v : channels) integer(v, 0, 65535);
    return {{"channels", channels}, {"rssi", integer(d.at("rssi"), 0, 255)}, {"takeover_verified", false}};
  }
  const auto rtt = number(d.at("round_trip_time_ms")); require(rtt >= 0, "RTT_INVALID");
  return {{"remote_timestamp_ns", ns(d.at("remote_timestamp_ns"))},
    {"observed_offset_ns", integer(d.at("observed_offset_ns"), INT64_MIN, INT64_MAX)},
    {"estimated_offset_ns", integer(d.at("estimated_offset_ns"), INT64_MIN, INT64_MAX)},
    {"round_trip_time_ms", rtt}, {"fixed_delay_verified", false}};
}
}  // namespace

CaptureObservations::CaptureObservations(const Json& manifest) {
  require(manifest.at("schema") == 1 && manifest.at("scope") == "manual_flight_receive_only", "CAPTURE_SCHEMA_UNSUPPORTED");
  const auto tag = text(manifest.at("tag"));
  require((tag == "A" && manifest.at("tag_id") == "5" && manifest.at("ros_domain_id") == 1)
    || (tag == "B" && manifest.at("tag_id") == "6" && manifest.at("ros_domain_id") == 2), "TAG_DOMAIN_MISMATCH");
  identity_ = {{"boot_id", text(manifest.at("boot_id"))}, {"tag", tag},
    {"tag_id", manifest.at("tag_id")}, {"ros_domain_id", manifest.at("ros_domain_id")},
    {"source_revision", text(manifest.at("source_revision"))}, {"aircraft_identity_verified", false}};
  identity_["config_refs"] = Json::array();
  if (manifest.contains("config_evidence")) {
    const auto& evidence = manifest.at("config_evidence");
    require(evidence.is_array() && evidence.size() <= 64, "CONFIG_EVIDENCE_INVALID");
    for (const auto& item : evidence) {
      const auto name = text(item.at("name")), hash = text(item.at("sha256"));
      require(item.at("content_utf8").is_string(), "CONFIG_EVIDENCE_INVALID");
      require(service::digest(item.at("content_utf8").get<std::string>()) == hash, "CONFIG_DIGEST_MISMATCH");
      identity_["config_refs"].push_back({{"name", name}, {"sha256", hash}});
    }
  }
  for (const auto& [topic, spec] : specs) { (void)topic; streams_.emplace(spec.channel, Stream{}); }
}

void CaptureObservations::ingest(const Json& row) {
  const auto mono = ns(row.at("received_monotonic_ns"));
  require(mono >= last_receipt_ns_, "CAPTURE_MONOTONIC_REGRESSION");
  last_receipt_ns_ = mono; ++rows_;
  const auto found = specs.find(text(row.at("topic")));
  if (found == specs.end()) { ++ignored_; return; }
  const auto& spec = found->second; auto& s = streams_.at(spec.channel);
  if (s.received) s.max_receipt_gap_ns = std::max(s.max_receipt_gap_ns, mono - s.receipt_ns);
  ++s.received; s.receipt_ns = mono; s.valid = false; s.value = nullptr; s.header_ns = s.ros_ns = 0;
  try {
    require(row.at("type") == spec.type, "MESSAGE_TYPE_MISMATCH");
    s.ros_ns = ns(row.at("received_ros_ns")); s.header_ns = ns(row.at("header_ns"));
    const auto& d = row.at("data"); const auto& stamp = d.at("header").at("stamp");
    const auto sec = integer(stamp.at("sec"), 0, INT32_MAX), nano = integer(stamp.at("nanosec"), 0, 999999999);
    require(sec * 1000000000LL + nano == s.header_ns, "HEADER_COPY_MISMATCH");
    require(s.header_ns <= s.ros_ns, "FUTURE_HEADER");
    require(s.ros_ns - s.header_ns <= spec.age_ns, "STALE_AT_RECEIPT");
    require(s.header_ns > s.high_header_ns, "HEADER_NOT_NEW");
    s.high_header_ns = s.header_ns;
    s.value = fields(spec.channel, d);
    if (std::string(spec.channel) == "lidar_scan") {
      // LaserScan header is the first ray. A completed scan cannot contain
      // beams later than receipt. Unknown (zero) beam timing stays unverified.
      const auto span = number(d.at("time_increment")) * (d.at("ranges").size()-1);
      require(span <= (s.ros_ns-s.header_ns)/1e9 + 1e-9, "SCAN_FUTURE_BEAMS");
    }
    if (s.accepted) s.max_accepted_gap_ns = std::max(s.max_accepted_gap_ns, mono - s.accepted_ns);
    ++s.accepted; s.accepted_ns = mono; s.valid = true; s.reason = "SAMPLE_VALID";
  } catch (const std::invalid_argument& e) {
    s.value = nullptr; s.reason = e.what(); ++s.rejections[s.reason];
  } catch (const Json::exception&) {
    s.reason = "MESSAGE_SHAPE_INVALID"; ++s.rejections[s.reason];
  }
}

Json CaptureObservations::snapshot(std::int64_t now) const {
  require(now >= last_receipt_ns_, "SNAPSHOT_BEFORE_RECEIPT");
  Json result{{"schema", "sangwon-capture-observations/1"}, {"scope", "OFFLINE_OBSERVE"},
    {"identity", identity_}, {"rows", rows_}, {"ignored_rows", ignored_}, {"receipt_monotonic_ns", now},
    {"can_start", false}, {"flight_authority", false}, {"physical_output_enabled", false},
    {"alignment_confirmed", false}, {"timing_confirmed", false}, {"fusion_verified", false},
    {"mounting_confirmed", false}, {"source_sample_time_verified", false},
    {"streams", Json::object()}};
  for (const auto& [topic, spec] : specs) {
    const auto& s = streams_.at(spec.channel);
    const auto elapsed = now - s.receipt_ns;
    const auto age = s.header_ns > 0 && s.ros_ns >= s.header_ns ? s.ros_ns - s.header_ns : 0;
    const bool valid = s.valid && elapsed <= spec.age_ns && age <= spec.age_ns - elapsed;
    result["streams"][spec.channel] = {{"topic", topic}, {"sample_valid", valid},
      {"reason", s.valid && !valid ? "EXPIRED" : s.reason}, {"received", s.received}, {"accepted", s.accepted},
      {"rejections", s.rejections}, {"max_receipt_gap_ns", s.max_receipt_gap_ns},
      {"max_accepted_gap_ns", s.max_accepted_gap_ns}, {"diagnostic_max_age_ns", spec.age_ns},
      {"header_ns", s.header_ns ? Json(s.header_ns) : Json()},
      {"received_ros_ns", s.ros_ns ? Json(s.ros_ns) : Json()},
      {"received_monotonic_ns", s.received ? Json(s.receipt_ns) : Json()},
      {"value", valid ? s.value : Json()}};
  }
  return result;
}
}  // namespace sangwon::observe
