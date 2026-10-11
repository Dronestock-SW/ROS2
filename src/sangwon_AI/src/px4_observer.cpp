// Subscriptions only. No setpoint publisher, command client or vehicle port.
#include "sangwon_ai/px4_health.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <openssl/rand.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <cstring>
#include <cerrno>

namespace fs = std::filesystem;
using sangwon::service::Json;
using sangwon::service::require;
namespace {
double clock_seconds(clockid_t id) {
  timespec value{}; require(clock_gettime(id, &value) == 0, "CLOCK_UNAVAILABLE");
  return double(value.tv_sec) + double(value.tv_nsec) / 1e9;
}
std::string read_file(const fs::path& path) {
  require(fs::file_size(path) <= 16384, "OBSERVER_FILE_TOO_LARGE");
  std::ifstream input(path); require(bool(input), "OBSERVER_FILE_UNAVAILABLE");
  std::string value{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  return value;
}
bool below(const fs::path& child, const fs::path& parent) {
  return child.string().rfind(parent.string() + "/", 0) == 0;
}
std::string session_id() {
  unsigned char bytes[16]; require(RAND_bytes(bytes, sizeof(bytes)) == 1, "SESSION_RANDOM_FAILED");
  std::ostringstream value;
  for (auto b : bytes) value << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
  return value.str();
}
std::string utc_text() {
  const time_t value = time(nullptr); tm utc{}; require(gmtime_r(&value, &utc) != nullptr, "UTC_UNAVAILABLE");
  char result[32]{}; require(strftime(result, sizeof(result), "%Y-%m-%dT%H:%M:%SZ", &utc) > 0, "UTC_UNAVAILABLE");
  return result;
}
void atomic_write(const fs::path& path, const Json& value) {
  auto pattern = (path.parent_path() / ".health-XXXXXX").string();
  std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
  const int fd = mkstemp(name.data()); require(fd >= 0, "OBSERVATION_WRITE_FAILED");
  try {
    const auto text = value.dump() + "\n"; std::size_t offset = 0;
    while (offset < text.size()) {
      const auto n = write(fd, text.data() + offset, text.size() - offset);
      if (n < 0 && errno == EINTR) continue;
      require(n > 0, "OBSERVATION_WRITE_FAILED"); offset += std::size_t(n);
    }
    require(fsync(fd) == 0, "OBSERVATION_SYNC_FAILED");
    require(rename(name.data(), path.c_str()) == 0, "OBSERVATION_REPLACE_FAILED");
  } catch (...) { close(fd); unlink(name.data()); throw; }
  close(fd);
  const int directory = open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  require(directory >= 0, "OBSERVATION_DIRECTORY_UNAVAILABLE");
  const int result = fsync(directory); close(directory); require(result == 0, "OBSERVATION_DIRECTORY_SYNC_FAILED");
}
void notify(const std::string& value) {
  const char* path = std::getenv("NOTIFY_SOCKET"); if (!path || !*path) return;
  const std::size_t length = std::strlen(path); sockaddr_un address{}; address.sun_family = AF_UNIX;
  require(length < sizeof(address.sun_path), "NOTIFY_PATH_INVALID"); std::memcpy(address.sun_path, path, length + 1);
  if (address.sun_path[0] == '@') address.sun_path[0] = '\0';
  const int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0); require(fd >= 0, "NOTIFY_SOCKET_FAILED");
  const auto sent = sendto(fd, value.data(), value.size(), MSG_NOSIGNAL,
    reinterpret_cast<sockaddr*>(&address), socklen_t(offsetof(sockaddr_un, sun_path) + length + (path[0] == '@' ? 0 : 1)));
  close(fd); require(sent == std::ptrdiff_t(value.size()), "NOTIFY_SEND_FAILED");
}
std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec <= 0 || stamp.nanosec >= 1000000000) return 0;
  return std::int64_t(stamp.sec) * 1000000000 + stamp.nanosec;
}
Json nullable_nan(float value) { return std::isnan(value) ? Json(nullptr) : Json(double(value)); }
// JSON itself cannot retain infinity: preserve it as an invalid input sentinel
// rather than silently converting it to the allowed unknown/NaN representation.
Json battery_fields(const sensor_msgs::msg::BatteryState& m) {
  if (std::isinf(m.voltage) || std::isinf(m.current) || std::isinf(m.percentage)) return Json::object();
  return {{"present", m.present}, {"voltage_v", double(m.voltage)}, {"current_a", nullable_nan(m.current)},
    {"percentage", nullable_nan(m.percentage)}, {"health", m.power_supply_health}};
}

class Observer : public rclcpp::Node {
 public:
  Observer(sangwon::observe::Px4Health& health, const rclcpp::NodeOptions& options)
    : Node("sangwon_px4_observer_" + std::to_string(getpid()), options), health_(health) {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto topic = [&](const char* name) { return health_.config().at("topics").at(name).get<std::string>(); };
    state_ = create_subscription<mavros_msgs::msg::State>(topic("state"), qos,
      [this](mavros_msgs::msg::State::ConstSharedPtr m) {
        receive("state", m->header.stamp, {{"connected", m->connected}, {"armed", m->armed}, {"guided", m->guided},
          {"manual_input", m->manual_input}, {"mode", m->mode}, {"system_status", m->system_status}});
      });
    extended_ = create_subscription<mavros_msgs::msg::ExtendedState>(topic("extended_state"), qos,
      [this](mavros_msgs::msg::ExtendedState::ConstSharedPtr m) {
        receive("extended_state", m->header.stamp, {{"landed_state", m->landed_state}, {"vtol_state", m->vtol_state}});
      });
    battery_ = create_subscription<sensor_msgs::msg::BatteryState>(topic("battery"), qos,
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) { receive("battery", m->header.stamp, battery_fields(*m)); });
    rc_ = create_subscription<mavros_msgs::msg::RCIn>(topic("rc"), qos,
      [this](mavros_msgs::msg::RCIn::ConstSharedPtr m) {
        receive("rc", m->header.stamp, {{"channel_count", m->channels.size()}, {"rssi", m->rssi}, {"rssi_known", m->rssi != 255}});
      });
    if (health_.config().at("topics").contains("local_position")) {
      position_ = create_subscription<geometry_msgs::msg::PoseStamped>(topic("local_position"), qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          const auto& p = m->pose.position; const auto& q = m->pose.orientation;
          receive("local_position", m->header.stamp, {{"frame_id", m->header.frame_id},
            {"x_m", p.x}, {"y_m", p.y}, {"z_m", p.z},
            {"qw", q.w}, {"qx", q.x}, {"qy", q.y}, {"qz", q.z}});
        });
    }
  }
  void refresh_graph() {
    for (const auto& item : health_.config().at("topics").items())
      health_.publishers(item.key(), count_publishers(item.value().get<std::string>()));
  }
 private:
  void receive(const std::string& channel, const builtin_interfaces::msg::Time& stamp, const Json& fields) {
    health_.publishers(channel, count_publishers(health_.config().at("topics").at(channel).get<std::string>()));
    health_.sample(channel, stamp_ns(stamp), clock_seconds(CLOCK_REALTIME), clock_seconds(CLOCK_MONOTONIC), fields);
  }
  sangwon::observe::Px4Health& health_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_;
};
}  // namespace

int main(int argc, char** argv) {
  try {
    umask(0077);
    std::map<std::string, std::string> args;
    require(argc >= 5 && argc % 2 == 1, "Usage: --root PACKAGE --config FILE [--state-dir .runtime/px4] [--duration-s SECONDS]");
    for (int i = 1; i < argc; i += 2) {
      const std::string key(argv[i]);
      require(key == "--root" || key == "--config" || key == "--state-dir" || key == "--duration-s", "OBSERVER_ARGUMENT_INVALID");
      require(args.emplace(key, argv[i + 1]).second, "OBSERVER_ARGUMENT_DUPLICATE");
    }
    require(args.count("--root") && args.count("--config"), "OBSERVER_ARGUMENT_MISSING");
    const auto root = fs::canonical(args.at("--root"));
    const auto config_path = fs::canonical(args.at("--config"));
    require(below(config_path, root), "OBSERVER_CONFIG_OUTSIDE_PACKAGE");
    const auto state = fs::weakly_canonical(root / (args.count("--state-dir") ? args.at("--state-dir") : ".runtime/px4"));
    require(below(state, root / ".runtime"), "STATE_PATH_OUTSIDE_RUNTIME");
    double duration = 0;
    if (args.count("--duration-s")) {
      std::size_t consumed = 0; duration = std::stod(args.at("--duration-s"), &consumed);
      require(consumed == args.at("--duration-s").size() && std::isfinite(duration) && duration >= .1 && duration <= 30, "OBSERVER_DURATION_INVALID");
    }
    auto boot = read_file("/proc/sys/kernel/random/boot_id"); while (!boot.empty() && boot.back() == '\n') boot.pop_back();
    sangwon::observe::Px4Health health(sangwon::service::parse(read_file(config_path)), boot, session_id());
    const auto domain = health.config().at("ros_domain_id").get<int>();
    const auto domain_text = std::to_string(domain);
    require(setenv("ROS_DOMAIN_ID", domain_text.c_str(), 1) == 0, "ROS_DOMAIN_SET_FAILED");
    fs::create_directories(state); fs::permissions(state, fs::perms::owner_all);
    const int lock = open((state / "observer.lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    require(lock >= 0 && flock(lock, LOCK_EX | LOCK_NB) == 0, "PX4_OBSERVER_ALREADY_RUNNING");
    // Config controls subscriptions; argv remaps/parameters cannot replace it.
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
    auto node = std::make_shared<Observer>(health, options);
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    const double start = clock_seconds(CLOCK_MONOTONIC); double next_report = 0; bool first = true;
    std::cout << "PX4 transport observer; domain=" << domain << "; flight_authority=false\n" << std::flush;
    while (rclcpp::ok() && (duration == 0 || clock_seconds(CLOCK_MONOTONIC) - start < duration)) {
      executor.spin_some(std::chrono::milliseconds(20));
      const double now = clock_seconds(CLOCK_MONOTONIC);
      if (now >= next_report) {
        node->refresh_graph(); atomic_write(state / "health.json", health.snapshot(now, utc_text()));
        notify((first ? "READY=1\n" : "") + std::string("WATCHDOG=1\nSTATUS=PX4 subscriber active; no flight authority"));
        first = false; next_report = now + .5;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    notify("STOPPING=1\nSTATUS=PX4 observer stopping");
    executor.remove_node(node); node.reset(); if (rclcpp::ok()) rclcpp::shutdown(); close(lock); return 0;
  } catch (...) {
    if (rclcpp::ok()) rclcpp::shutdown();
    // Never print transport fields, config or exception text to the journal.
    std::cerr << "PX4 observer stopped: OBSERVER_FAILED\n"; return 2;
  }
}
