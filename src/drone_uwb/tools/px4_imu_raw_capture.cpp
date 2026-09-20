// Read-only PX4 IMU snapshots through the official MAVLink shell protocol.
// Allowed payloads are fixed below; no parameters, stream rates or flight state are written.
#include <mavlink/v2.0/common/mavlink.h>
#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
static const std::vector<std::string> survey_commands = {
    "listener sensor_mag -n 1", "listener sensor_baro -n 1",
    "listener vehicle_attitude -n 1", "listener vehicle_local_position -n 1",
    "listener vehicle_global_position -n 1", "listener vehicle_angular_velocity -n 1",
    "listener vehicle_acceleration -n 1", "listener vehicle_air_data -n 1",
    "listener vehicle_magnetometer -n 1", "listener estimator_status -n 1",
    "listener estimator_status_flags -n 1", "listener estimator_sensor_bias -n 1",
    "listener vehicle_status -n 1", "listener vehicle_land_detected -n 1",
    "listener failsafe_flags -n 1", "listener battery_status -n 1",
    "listener system_power -n 1", "listener cpuload -n 1",
    "listener sensor_gps -n 1", "listener distance_sensor -n 1",
    "listener sensor_optical_flow -n 1", "listener input_rc -n 1",
    "listener actuator_outputs -n 1", "listener esc_status -n 1"
};
static volatile std::sig_atomic_t interrupted = 0;
static void stop(int) { interrupted = 1; }
static int64_t mono_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
static std::string json_string(const std::string &s) {
    std::string out = "\"";
    const char *hex = "0123456789abcdef";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
        else out += c;
    }
    return out + '"';
}

class Connection {
public:
    explicit Connection(const std::filesystem::path &directory)
        : wire(directory / "mavlink_rx.bin", std::ios::binary),
          console(directory / "console.raw", std::ios::binary),
          queries(directory / "queries.jsonl") {
        fd = open("/dev/pixhawk", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) throw std::runtime_error("cannot open /dev/pixhawk");
        if (flock(fd, LOCK_EX | LOCK_NB) != 0 || tcgetattr(fd, &original) != 0) {
            close(fd); fd = -1; throw std::runtime_error("port busy or termios unavailable");
        }
        auto config = original;
        cfmakeraw(&config);
        cfsetispeed(&config, B921600); cfsetospeed(&config, B921600);
        config.c_cflag |= CLOCAL | CREAD;
        config.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd, TCSANOW, &config) != 0) {
            close(fd); fd = -1; throw std::runtime_error("cannot configure host serial port");
        }
    }
    ~Connection() {
        if (fd >= 0) {
            try { shell_bytes("", 0); } catch (...) {}
            tcsetattr(fd, TCSANOW, &original);
            close(fd);
        }
    }
    bool heartbeat_seen = false;
    bool armed = false;
    std::map<unsigned, unsigned> message_counts;
    unsigned sent_queries = 0;
    void poll_once(int wait_ms = 10) {
        if (Clock::now() >= next_heartbeat) {
            mavlink_message_t msg{};
            mavlink_msg_heartbeat_pack(245, 191, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID,
                                      0, 0, MAV_STATE_STANDBY);
            send(msg);
            next_heartbeat = Clock::now() + std::chrono::seconds(1);
        }
        pollfd item{fd, POLLIN, 0};
        const int ready = ::poll(&item, 1, wait_ms);
        if (ready < 0) {
            if (errno == EINTR) return;
            throw std::runtime_error("serial poll failed");
        }
        if (item.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("serial disconnected");
        if (!(item.revents & POLLIN)) return;
        std::array<uint8_t, 8192> bytes{};
        const auto size = read(fd, bytes.data(), bytes.size());
        if (size <= 0) return;
        wire.write(reinterpret_cast<const char *>(bytes.data()), size);
        mavlink_message_t msg{};
        mavlink_status_t status{};
        for (ssize_t i = 0; i < size; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &msg, &status)) continue;
            if (msg.sysid != 1 || msg.compid != 1) continue;
            ++message_counts[msg.msgid];
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t hb{};
                mavlink_msg_heartbeat_decode(&msg, &hb);
                heartbeat_seen = true;
                armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
            } else if (msg.msgid == MAVLINK_MSG_ID_SERIAL_CONTROL) {
                mavlink_serial_control_t payload{};
                mavlink_msg_serial_control_decode(&msg, &payload);
                if (payload.device == SERIAL_CONTROL_DEV_SHELL && payload.count <= 70) {
                    std::string chunk(reinterpret_cast<char *>(payload.data), payload.count);
                    pending += chunk;
                    console.write(chunk.data(), chunk.size());
                }
            }
        }
    }
    std::string query(const std::string &command, const std::string &phase) {
        static const std::vector<std::string> allowed = {
            "", "ver all", "sensors status", "ekf2 status", "listener sensor_selection -n 1",
            "listener sensor_accel -n 1", "listener sensor_gyro -n 1", "listener vehicle_imu -n 1",
            "listener vehicle_imu_status -n 1", "param show CAL_ACC*", "param show CAL_GYRO*"
        };
        bool ok = false;
        for (const auto &value : allowed) if (value == command) ok = true;
        for (const auto &value : survey_commands) if (value == command) ok = true;
        if (!ok) throw std::runtime_error("command not allowed");
        if (armed) throw std::runtime_error("bench capture requires disarmed FC");
        pending.clear();
        const auto begin = mono_ns();
        shell_bytes(command + "\n", SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND);
        const auto deadline = Clock::now() + std::chrono::seconds(4);
        bool prompt = false;
        while (!interrupted && Clock::now() < deadline) {
            poll_once();
            if (pending.find("nsh> ") != std::string::npos) { prompt = true; break; }
        }
        ++sent_queries;
        queries << "{\"query\":" << sent_queries << ",\"phase\":" << json_string(phase)
                << ",\"command\":" << json_string(command) << ",\"host_send_monotonic_ns\":" << begin
                << ",\"host_end_monotonic_ns\":" << mono_ns() << ",\"prompt_received\":"
                << (prompt ? "true" : "false") << ",\"text\":" << json_string(pending) << "}\n";
        queries.flush(); console.flush(); wire.flush();
        if (!prompt && !interrupted) throw std::runtime_error("PX4 shell prompt timeout");
        return pending;
    }
private:
    int fd = -1;
    termios original{};
    std::ofstream wire, console, queries;
    std::string pending;
    Clock::time_point next_heartbeat{};
    void send(const mavlink_message_t &msg) {
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
        const auto size = mavlink_msg_to_send_buffer(bytes.data(), &msg);
        size_t used = 0;
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        while (used < size) {
            auto written = write(fd, bytes.data() + used, size - used);
            if (written > 0) used += written;
            else if (errno != EAGAIN && errno != EINTR) throw std::runtime_error("serial write failed");
            if (Clock::now() > deadline) throw std::runtime_error("serial write timeout");
        }
    }
    void shell_bytes(const std::string &command, uint8_t flags) {
        if (command.size() > 70) throw std::runtime_error("shell payload too long");
        std::array<uint8_t, 70> data{};
        std::memcpy(data.data(), command.data(), command.size());
        mavlink_message_t msg{};
        mavlink_msg_serial_control_pack(245, 191, &msg, SERIAL_CONTROL_DEV_SHELL, flags,
                                       0, 0, command.size(), data.data(), 1, 1);
        send(msg);
    }
};

int main(int argc, char **argv) {
    if (argc != 3 || (std::string(argv[1]) != "probe" && std::string(argv[1]) != "capture" && std::string(argv[1]) != "survey" && std::string(argv[1]) != "gyro")) {
        std::cerr << "Usage: px4_imu_raw_capture probe|capture|survey|gyro NEW_OUTPUT_DIRECTORY\n";
        return 2;
    }
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    const std::filesystem::path directory(argv[2]);
    try {
        if (!std::filesystem::create_directories(directory)) throw std::runtime_error("output must be a new directory");
        Connection link(directory);
        const auto deadline = Clock::now() + std::chrono::seconds(6);
        while (!link.heartbeat_seen && !interrupted && Clock::now() < deadline) link.poll_once();
        if (!link.heartbeat_seen) throw std::runtime_error("no FC heartbeat");
        link.query("", "setup");
        if (std::string(argv[1]) == "gyro") {
            for (int sample = 0; sample < 5 && !interrupted; ++sample) {
                std::cout << link.query("listener sensor_gyro -n 1", "gyro") << std::flush;
                if (sample < 4) {
                    const auto next = Clock::now() + std::chrono::milliseconds(200);
                    while (!interrupted && Clock::now() < next) link.poll_once(5);
                }
            }
            return interrupted ? 130 : 0;
        }
        for (const auto &command : {"ver all", "sensors status", "listener sensor_selection -n 1",
                "listener sensor_accel -n 1", "listener sensor_gyro -n 1", "listener vehicle_imu -n 1"}) {
            const auto text = link.query(command, "identification");
            std::cout << text << std::flush;
        }
        if (std::string(argv[1]) == "survey") {
            for (const auto &command : survey_commands) {
                if (interrupted) break;
                std::cout << link.query(command, "survey") << std::flush;
            }
        }
        if (std::string(argv[1]) == "capture") {
            const auto start = Clock::now();
            const auto start_ns = mono_ns();
            auto next = start;
            auto progress = start;
            std::cout << "CAPTURE_STARTED: both sensor instances, 60 seconds, alternating accel/gyro snapshots\n" << std::flush;
            while (!interrupted && Clock::now() - start < std::chrono::seconds(60)) {
                if (Clock::now() >= next) {
                    link.query("listener sensor_accel -n 1", "capture");
                    link.query("listener sensor_gyro -n 1", "capture");
                    next += std::chrono::milliseconds(200);
                    if (next < Clock::now()) next = Clock::now();
                } else link.poll_once(5);
                if (Clock::now() - progress > std::chrono::seconds(15)) {
                    std::cout << "CAPTURE_PROGRESS_SECONDS "
                              << std::chrono::duration<double>(Clock::now()-start).count() << "\n" << std::flush;
                    progress = Clock::now();
                }
            }
            std::ofstream meta(directory / "capture_window.json");
            meta << "{\"host_start_monotonic_ns\":" << start_ns << ",\"host_end_monotonic_ns\":" << mono_ns()
                 << ",\"requested_seconds\":60,\"requested_pair_rate_hz\":5,\"interrupted\":"
                 << (interrupted ? "true" : "false") << ",\"sensor_rate_changed\":false,\"px4_parameters_written\":false}\n";
        }
        if (!interrupted) std::cout << link.query("listener vehicle_imu_status -n 1", "final") << std::flush;
        std::cout << "DONE queries=" << link.sent_queries << ", output=" << directory << "\n";
        return interrupted ? 130 : 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
