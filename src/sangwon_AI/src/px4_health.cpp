#include "sangwon_ai/px4_health.hpp"
#include <cmath>
#include <regex>
#include <set>

namespace sangwon::observe {
namespace {
using service::require;
bool exact_keys(const Json& value, const std::set<std::string>& expected) {
  if (!value.is_object() || value.size() != expected.size()) return false;
  for (const auto& key : expected) if (!value.contains(key)) return false;
  return true;
}
bool integer(const Json& v, int lo, int hi) {
  return v.is_number_integer() && v >= lo && v <= hi;
}
bool number(const Json& v, double lo, double hi, bool nullable = false) {
  return (nullable && v.is_null()) ||
    (v.is_number() && std::isfinite(v.get<double>()) && v >= lo && v <= hi);
}
bool fields_valid(const std::string& channel, const Json& f) {
  if (channel == "local_position") {
    if (!exact_keys(f, {"frame_id", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"})
        || f.at("frame_id") != "map") return false;
    for (const auto* key : {"x_m", "y_m", "z_m"}) if (!number(f.at(key), -1e6, 1e6)) return false;
    double norm = 0;
    for (const auto* key : {"qw", "qx", "qy", "qz"}) {
      if (!number(f.at(key), -1, 1)) return false;
      const auto v = f.at(key).get<double>(); norm += v*v;
    }
    return std::abs(norm-1) < .02;
  }
  if (channel == "state") {
    if (!exact_keys(f, {"connected", "armed", "guided", "manual_input", "mode", "system_status"})) return false;
    for (const auto* key : {"connected", "armed", "guided", "manual_input"})
      if (!f.at(key).is_boolean()) return false;
    return f.at("mode").is_string() && f.at("mode").get_ref<const std::string&>().size() <= 32
      && std::regex_match(f.at("mode").get<std::string>(), std::regex("[A-Z0-9_.]{1,32}"))
      && integer(f.at("system_status"), 0, 8);
  }
  if (channel == "extended_state")
    return exact_keys(f, {"landed_state", "vtol_state"}) && integer(f.at("landed_state"), 0, 4) && integer(f.at("vtol_state"), 0, 4);
  if (channel == "battery")
    return exact_keys(f, {"present", "voltage_v", "current_a", "percentage", "health"}) && f.at("present").is_boolean()
      && number(f.at("voltage_v"), 0, 1000) && number(f.at("current_a"), -10000, 10000, true)
      && number(f.at("percentage"), 0, 1, true) && integer(f.at("health"), 0, 8);
  return exact_keys(f, {"channel_count", "rssi", "rssi_known"}) && integer(f.at("channel_count"), 0, 32)
    && integer(f.at("rssi"), 0, 255) && f.at("rssi_known").is_boolean()
    && f.at("rssi_known") == (f.at("rssi") != 255);
}
}  // namespace

Px4Health::Px4Health(Json config, std::string boot, std::string session)
  : config_(std::move(config)), boot_(std::move(boot)), session_(std::move(session)) {
  require(exact_keys(config_, {"schema_version", "profile", "flight_authority", "physical_output_enabled",
    "observation_source", "ros_domain_id", "topics", "max_header_age_ms"}), "INVALID_PX4_OBSERVER_CONFIG");
  require(config_.at("schema_version") == "sangwon-px4-observe-config/1" && config_.at("profile") == "HOST_OBSERVE"
    && config_.at("flight_authority") == false && config_.at("physical_output_enabled") == false, "PX4_OBSERVER_AUTHORITY_REFUSED");
  const auto source = config_.at("observation_source");
  require(source == "MAVROS_TOPICS_UNVERIFIED_AIRCRAFT" || source == "SYNTHETIC_MAVROS_TEST", "PX4_OBSERVER_SOURCE_REQUIRED");
  require(integer(config_.at("ros_domain_id"), 0, 232), "INVALID_ROS_DOMAIN");
  std::set<std::string> names{"state", "extended_state", "battery", "rc"};
  if (config_.at("topics").contains("local_position")) names.insert("local_position");
  require(exact_keys(config_.at("topics"), names) &&
    exact_keys(config_.at("max_header_age_ms"), names), "INVALID_PX4_OBSERVER_CHANNELS");
  const int domain = config_.at("ros_domain_id").get<int>();
  const bool synthetic = source == "SYNTHETIC_MAVROS_TEST";
  require(synthetic == (domain >= 170 && domain <= 199), "PX4_OBSERVER_SOURCE_DOMAIN_MISMATCH");
  std::set<std::string> topics;
  for (const auto& channel : names) {
    const auto& value = config_.at("topics").at(channel);
    require(value.is_string(), "INVALID_PX4_OBSERVER_TOPIC");
    const auto topic = value.get<std::string>();
    require(topic.size() <= 192 && std::regex_match(topic, std::regex("(/[A-Za-z_][A-Za-z0-9_]*)+")), "INVALID_PX4_OBSERVER_TOPIC");
    require(!synthetic || topic.rfind("/sangwon_synthetic_", 0) == 0, "SYNTHETIC_TOPIC_REQUIRED");
    require(topics.insert(topic).second && integer(config_.at("max_header_age_ms").at(channel), 10, 10000), "INVALID_PX4_OBSERVER_LIMIT");
    streams_.emplace(channel, Stream{});
  }
}

void Px4Health::publishers(const std::string& channel, std::size_t count) {
  auto& s = streams_.at(channel);
  if (s.publishers != count) {
    s.valid = false; s.fields = Json::object(); s.code = "NO_OBSERVATION";
  }
  s.publishers = count;
  if (count != 1) {
    s.valid = false; s.fields = Json::object();
    s.code = count == 0 ? "NO_PUBLISHER" : "AMBIGUOUS_PUBLISHERS";
  }
}

void Px4Health::sample(const std::string& channel, std::int64_t header_ns, double utc_now,
                       double monotonic_now, const Json& fields) {
  auto& s = streams_.at(channel);
  ++s.received;
  auto reject = [&](const char* code) { ++s.rejected; s.valid = false; s.fields = Json::object(); s.code = code; };
  if (s.publishers != 1) { reject("PUBLISHER_NOT_UNIQUE"); return; }
  if (!fields_valid(channel, fields)) { reject("INVALID_MAVROS_MESSAGE"); return; }
  if (header_ns <= 0 || !std::isfinite(utc_now) || !std::isfinite(monotonic_now) || monotonic_now < 0) {
    reject("HEADER_TIMESTAMP_INVALID"); return;
  }
  const double age_ms = (utc_now - double(header_ns) / 1e9) * 1000;
  if (age_ms < 0 || age_ms >= config_.at("max_header_age_ms").at(channel).get<int>()) {
    reject("HEADER_TIMESTAMP_NOT_CURRENT"); return;
  }
  if (header_ns <= s.last_header_ns) { reject("HEADER_TIMESTAMP_NOT_NEW"); return; }
  s.last_header_ns = header_ns; s.received_monotonic = monotonic_now; s.header_age_ms = age_ms;
  s.fields = fields; s.valid = true; s.code = "OK"; ++s.accepted;
}

Json Px4Health::view(const std::string& channel, double now) const {
  const auto& s = streams_.at(channel);
  Json value{{"topic", config_.at("topics").at(channel)}, {"publisher_count", s.publishers},
    {"received_count", s.received}, {"accepted_count", s.accepted}, {"rejected_count", s.rejected},
    {"status", s.received ? "WARN" : "UNKNOWN"}, {"code", s.code}, {"transport_observation_valid", false},
    {"header_age_ms", nullptr}, {"max_header_age_ms", config_.at("max_header_age_ms").at(channel)},
    {"fields", Json::object()}};
  if (!s.valid) return value;
  const double age = s.header_age_ms + (now - s.received_monotonic) * 1000;
  if (!std::isfinite(age) || now < s.received_monotonic || age >= value.at("max_header_age_ms").get<int>()) {
    value["status"] = "STALE"; value["code"] = "BRIDGE_OBSERVATION_EXPIRED"; return value;
  }
  value["header_age_ms"] = age; value["status"] = "LIVE"; value["transport_observation_valid"] = true;
  value["fields"] = s.fields;
  return value;
}

Json Px4Health::snapshot(double now, const std::string& generated_at) {
  service::require(std::isfinite(now) && now >= 0, "MONOTONIC_TIME_INVALID");
  Json streams = Json::object();
  for (const auto& item : streams_) streams[item.first] = view(item.first, now);
  return {{"schema_version", "sangwon-px4-health/1"}, {"scope", "MAVROS_TRANSPORT_ONLY"},
    {"boot_id", boot_}, {"monitor_session_id", session_}, {"monitor_seq", ++sequence_},
    {"generated_at", generated_at}, {"generated_monotonic_s", now}, {"display_ttl_s", 5},
    {"profile", "HOST_OBSERVE"}, {"observation_source", config_.at("observation_source")},
    {"ros_domain_id", config_.at("ros_domain_id")}, {"can_start", false}, {"flight_authority", false},
    {"physical_output_enabled", false}, {"source_sample_time_verified", false}, {"aircraft_identity_verified", false},
    {"runtime_code", "OK"}, {"streams", streams},
    {"remaining", {"AIRCRAFT_ID_FIRMWARE_BOOT_BINDING_REQUIRED", "FC_SOURCE_TIME_NOT_PROVEN_BY_MAVROS_HEADER",
      "PREARM_FAILSAFE_POSITION_YAW_PROFILE_UNVERIFIED", "RC_OVERRIDE_AND_SWITCH_CONFIG_UNVERIFIED"}}};
}
}  // namespace sangwon::observe
