#pragma once
#include "sangwon_ai/service/json.hpp"
#include <array>
#include <cstdint>
#include <map>

namespace sangwon::observe {
using service::Json;
inline constexpr std::array<const char*, 4> channels{"state", "extended_state", "battery", "rc"};
// Read-only bridge observations. Neither this reducer nor its report can grant
// readiness, establish aircraft identity, or prove FC source-sample freshness.
class Px4Health {
 public:
  Px4Health(Json config, std::string boot, std::string session);
  void publishers(const std::string& channel, std::size_t count);
  void sample(const std::string& channel, std::int64_t header_ns,
              double utc_now, double monotonic_now, const Json& fields);
  Json snapshot(double monotonic_now, const std::string& generated_at);
  const Json& config() const { return config_; }
 private:
  struct Stream {
    std::size_t publishers{};
    std::uint64_t received{}, accepted{}, rejected{};
    std::int64_t last_header_ns{};
    double received_monotonic{}, header_age_ms{};
    bool valid{};
    std::string code{"NO_OBSERVATION"};
    Json fields = Json::object();
  };
  Json config_;
  std::string boot_, session_;
  std::uint64_t sequence_{};
  std::map<std::string, Stream> streams_;
  Json view(const std::string& channel, double now) const;
};
}  // namespace sangwon::observe
