#pragma once
#include "sangwon_ai/runtime.hpp"
#include "sangwon_ai/fake_px4.hpp"
#include "sangwon_ai/service/ledger.hpp"
#include "sangwon_ai/service/snapshot.hpp"
#include <map>
#include <deque>

namespace sangwon::service {
inline constexpr const char* contract = "1.1-draft.4";
double steady_seconds();
double utc_seconds();
double utc_parse(std::string text);
std::string utc_text(double seconds);
std::string random_id(const std::string& prefix);
class Engine {
 public:
  Engine(Json config, const std::string& boot, const std::string& database);
  Json handle(const std::string& method, const Json& payload);
  void tick(double steady_now);
  Json status() const;
 private:
  Json config_;
  Ledger ledger_;
  std::string boot_, runtime_id_, session_, assignment_, snapshot_id_, preparation_, execution_, flight_;
  std::string link_code_{"WEB_NOT_CONNECTED"}, server_version_, last_phase_;
  std::uint64_t last_route_revision_{};
  std::string last_mode_status_;
  std::string preparation_code_{"NO_MISSION"};
  std::uint64_t readiness_revision_{1}, sequence_{}, event_sequence_{};
  mutable std::uint64_t readiness_sequence_{};
  bool replay_{}, connected_{}, active_{}, recovery_locked_{}, cancel_requested_{}, land_requested_{};
  double link_time_{}, last_tick_{}, simulation_time_{};
  Json snapshot_, prepared_context_, last_command_;
  Json planner_plan_;
  Pose launch_;
  bool has_scan_{};
  std::string validation_scope_{"ALLOWLISTED_SYNTHETIC_ROUTE_ONLY"};
  std::map<std::string,int> scan_revisions_;
  std::map<std::string,Json> scan_reports_;
  std::uint64_t perception_sequence_{};
  Json sensor_request_;
  std::map<std::string,std::uint64_t> sensor_sequences_;
  std::map<std::string,Json> sensor_receipts_;
  std::deque<std::string> sensor_receipt_order_;
  double sensor_camera_time_{-1e30}, sensor_scanner_time_{-1e30}, sensor_marker_time_{-1e30}, sensor_qr_time_{-1e30}, sensor_ack_time_{-1e30};
  double sensor_qr_utc_{};
  bool external_sensors() const;
  std::string scan_source() const;
  Json scan_request(bool renew=true);
  Json scan_observe(const Json&);
  void external_scan_input(double steady_now);
  void replay_scan_input();
  void persist_scan_results();
  Json host_report_;
  Json host_diagnostics() const;
  Json pending_commands_ = Json::array();
  Mission mission_;
  Config motion_;
  testing::FakePx4 vehicle_;
  std::unique_ptr<Runtime> runtime_;
  std::unique_ptr<FlightGuard> guard_;
  Json envelope(const std::string& type) const;
  Json context() const;
  Json readiness() const;
  Json telemetry() const;
  Json mode_control() const;
  Json accept_command(const Json& command);
  Json result(const Json& command, const std::string& state, const std::string& code, int revision) const;
  void save_result(const Json& command, const Json& response);
  void emit_event(const std::string& type, Json details);
  void invalidate_preparation();
  bool link_fresh() const;
  std::string wire_phase() const;
};
}  // namespace sangwon::service
