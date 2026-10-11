#include "sangwon_ai/px4_health.hpp"
#include <fstream>
#include <iostream>

using sangwon::observe::Px4Health;
using sangwon::service::Json;
namespace {
void check(bool yes) { if (!yes) throw std::runtime_error("PX4 health boundary failed"); }
const Json state{{"connected", true}, {"armed", false}, {"guided", false}, {"manual_input", true}, {"mode", "POSCTL"}, {"system_status", 3}};
const Json extended{{"landed_state", 1}, {"vtol_state", 0}};
const Json battery{{"present", true}, {"voltage_v", 16.2}, {"current_a", nullptr}, {"percentage", nullptr}, {"health", 0}};
const Json rc{{"channel_count", 8}, {"rssi", 255}, {"rssi_known", false}};
std::uint64_t accepted(const Json& report, const char* key) { return report.at("streams").at(key).at("accepted_count").get<std::uint64_t>(); }
}
int main(int argc, char** argv) {
  try {
    check(argc == 2); std::ifstream input(argv[1]);
    Json config = sangwon::service::parse({std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()});
    const auto make = [&]() { return Px4Health(config, "test-boot", "0123456789abcdef0123456789abcdef"); };
    for (const auto* key : {"flight_authority", "physical_output_enabled"}) {
      auto invalid = config; invalid[key] = true;
      bool refused = false; try { Px4Health bad(invalid, "boot", "session"); } catch (const std::exception&) { refused = true; }
      check(refused);
    }
    auto invalid = config; invalid["observation_source"] = "SYNTHETIC_MAVROS_TEST";
    bool refused = false; try { Px4Health bad(invalid, "boot", "session"); } catch (const std::exception&) { refused = true; }
    check(refused);
    auto h = make();
    for (const auto* key : sangwon::observe::channels) check(h.snapshot(10, "test").at("streams").at(key).at("status") == "UNKNOWN");
    h.publishers("state", 1); h.sample("state", 200000000000LL, 200.02, 10, state);
    auto report = h.snapshot(10.5, "test");
    check(report.at("streams").at("state").at("status") == "LIVE" && accepted(report, "state") == 1);
    for (const auto* key : {"can_start", "flight_authority", "physical_output_enabled", "source_sample_time_verified", "aircraft_identity_verified"}) check(report.at(key) == false);
    check(h.snapshot(12, "test").at("streams").at("state").at("status") == "STALE");
    h.sample("state", 200000000000LL, 200.03, 12.1, state);
    report = h.snapshot(12.1, "test");
    check(accepted(report, "state") == 1 && report.at("streams").at("state").at("code") == "HEADER_TIMESTAMP_NOT_NEW");
    check(report.at("streams").at("state").at("fields").empty());
    h.sample("state", 203000000000LL, 203.02, 13, state);
    check(h.snapshot(13, "test").at("streams").at("state").at("status") == "LIVE");
    h.publishers("state", 2); check(h.snapshot(13.1, "test").at("streams").at("state").at("code") == "AMBIGUOUS_PUBLISHERS");
    h.publishers("state", 1); check(!h.snapshot(13.2, "test").at("streams").at("state").at("transport_observation_valid").get<bool>());
    h.sample("state", 204000000000LL, 204.01, 14, state);
    h.publishers("state", 0); h.publishers("state", 1);
    h.sample("state", 204000000000LL, 204.02, 14.1, state);
    check(h.snapshot(14.1, "test").at("streams").at("state").at("code") == "HEADER_TIMESTAMP_NOT_NEW");
    h.sample("state", 206000000000LL, 205, 15, state);
    check(h.snapshot(15, "test").at("streams").at("state").at("code") == "HEADER_TIMESTAMP_NOT_CURRENT");
    h.sample("state", 200000000000LL, 205, 15, state);
    check(h.snapshot(15, "test").at("streams").at("state").at("code") == "HEADER_TIMESTAMP_NOT_CURRENT");
    h.sample("state", 0, 205, 15, state);
    check(h.snapshot(15, "test").at("streams").at("state").at("code") == "HEADER_TIMESTAMP_INVALID");
    for (const auto& entry : std::map<std::string, Json>{{"extended_state", extended}, {"battery", battery}, {"rc", rc}}) {
      h.publishers(entry.first, 1); h.sample(entry.first, 210000000000LL, 210.01, 16, entry.second);
      check(h.snapshot(16, "test").at("streams").at(entry.first).at("status") == "LIVE");
    }
    report = h.snapshot(16, "test");
    check(report.at("streams").at("battery").at("fields").at("percentage").is_null());
    check(report.dump().find("NaN") == std::string::npos && report.dump().find("channels\"") == std::string::npos);
    auto bad_battery = battery; bad_battery["percentage"] = 2;
    h.sample("battery", 211000000000LL, 211.01, 17, bad_battery);
    check(h.snapshot(17, "test").at("streams").at("battery").at("code") == "INVALID_MAVROS_MESSAGE");
    auto bad_rc = rc; bad_rc["channel_count"] = 33;
    h.sample("rc", 211000000000LL, 211.01, 17, bad_rc);
    check(h.snapshot(17, "test").at("streams").at("rc").at("fields").empty());
    auto bad_mode = state; bad_mode["mode"] = "UNSAFE\nMODE";
    h.sample("state", 211000000000LL, 211.01, 17, bad_mode);
    check(h.snapshot(17, "test").at("streams").at("state").at("code") == "INVALID_MAVROS_MESSAGE");
    auto xyz_config = config;
    xyz_config["topics"]["local_position"] = "/mavros/local_position/pose";
    xyz_config["max_header_age_ms"]["local_position"] = 200;
    Px4Health xyz(xyz_config, "boot", "session");
    xyz.publishers("local_position", 1);
    Json position{{"frame_id", "map"}, {"x_m", 1.2}, {"y_m", -.3}, {"z_m", .5},
      {"qw", 1.}, {"qx", 0.}, {"qy", 0.}, {"qz", 0.}};
    xyz.sample("local_position", 220000000000LL, 220.01, 20, position);
    check(xyz.snapshot(20.1, "test")["streams"]["local_position"]["fields"]["z_m"] == .5);
    check(xyz.snapshot(20.2, "test")["streams"]["local_position"]["fields"].empty());
    position["frame_id"] = "uwb_map";
    xyz.sample("local_position", 221000000000LL, 221.01, 21, position);
    check(xyz.snapshot(21, "test")["streams"]["local_position"]["code"] == "INVALID_MAVROS_MESSAGE");
    position["frame_id"] = "map"; position["qw"] = 0.;
    xyz.sample("local_position", 222000000000LL, 222.01, 22, position);
    check(xyz.snapshot(22, "test")["streams"]["local_position"]["fields"].empty());
    check(xyz.snapshot(22, "test")["can_start"] == false);
    std::cout << "PASS observer authority, freshness/duplicate/recovery, XYZ frame/expiry, bounded fields and unknown battery/RC\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
