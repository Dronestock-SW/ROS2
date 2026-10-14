// Minimal PX4/I2C API doubles for the real patched TF02PRO.cpp, no devices.
#pragma once
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

using hrt_abstime = uint64_t;
using perf_counter_t = unsigned;
constexpr int OK = 0, PX4_OK = 0, PX4_ERROR = -1;
constexpr int PC_COUNT = 0, PC_ELAPSED = 1, DRV_DIST_DEVTYPE_TF02PRO = 224;
#define MODULE_NAME "tf02pro"
#define ORB_ID(name) 0
struct vehicle_status_s {
    static constexpr uint8_t ARMING_STATE_DISARMED = 1, ARMING_STATE_ARMED = 2;
    uint64_t timestamp{};
    uint8_t arming_state{};
};
using Rotation = int;
struct distance_sensor_s {
    static constexpr int MAV_DISTANCE_SENSOR_LASER = 0, ROTATION_DOWNWARD_FACING = 25;
};
struct I2CSPIDriverConfig { int rotation = 25, custom1 = 0, custom2 = 0; };
// CLI API shape for syntax-checking the real tf02pro_main.cpp (not a parser test).
struct BusCLIArguments {
    BusCLIArguments(bool, bool) {}
    int rotation = 0, default_i2c_frequency = 0, i2c_address = 0, custom1 = 0, custom2 = 0;
    int getOpt(int, char **, const char *) { return EOF; }
    const char *optArg() const { return nullptr; }
};
struct BusInstanceIterator { BusInstanceIterator(const char *, const BusCLIArguments &, int) {} };
#define __EXPORT
#define PRINT_MODULE_USAGE_NAME(...) ((void)0)
#define PRINT_MODULE_USAGE_SUBCATEGORY(...) ((void)0)
#define PRINT_MODULE_USAGE_COMMAND(...) ((void)0)
#define PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(...) ((void)0)
#define PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(...) ((void)0)
#define PRINT_MODULE_USAGE_PARAM_INT(...) ((void)0)
#define PRINT_MODULE_USAGE_PARAM_FLAG(...) ((void)0)
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS(...) ((void)0)

namespace mock {
struct Exchange { std::vector<uint8_t> tx, rx; int result = 0; };
inline uint64_t now = 1000000;
inline bool status_available = true;
inline vehicle_status_s status{now, vehicle_status_s::ARMING_STATE_DISARMED};
inline std::deque<Exchange> exchanges;
inline unsigned writes = 0, reads = 0, sleeps = 0;
inline void reset() {
    now = 1000000; writes = reads = sleeps = 0;
    status_available = true; status = {now, vehicle_status_s::ARMING_STATE_DISARMED};
    exchanges.clear();
}
template<typename... Args> void log(const char *, Args...) {}
inline void expect(std::vector<uint8_t> tx, std::vector<uint8_t> rx = {}, int result = 0) {
    exchanges.push_back({tx, rx, result});
}
}
#define PX4_INFO(...) mock::log(__VA_ARGS__)
#define PX4_WARN(...) mock::log(__VA_ARGS__)
#define PX4_ERR(...) mock::log(__VA_ARGS__)
#define PX4_DEBUG(...) mock::log(__VA_ARGS__)
inline uint64_t hrt_absolute_time() { return mock::now; }
inline void px4_usleep(unsigned us) { ++mock::sleeps; mock::now += us; }
inline perf_counter_t perf_alloc(int, const char *) { return 0; }
inline void perf_free(perf_counter_t) {}
inline void perf_begin(perf_counter_t) {}
inline void perf_end(perf_counter_t) {}
inline void perf_count(perf_counter_t) {}
inline void perf_print_counter(perf_counter_t) {}
namespace math { inline float radians(float x) { return x * 0.017453292519943295f; } }
namespace device {
class I2C {
public:
    explicit I2C(const I2CSPIDriverConfig &) {}
    virtual ~I2C() = default;
    virtual int init() { return 0; }
    int get_device_id() { return 1; }
    int transfer(const uint8_t *tx, unsigned nt, uint8_t *rx, unsigned nr) {
        assert(!mock::exchanges.empty());
        auto next = mock::exchanges.front(); mock::exchanges.pop_front();
        assert(next.tx.size() == nt);
        if (nt) { ++mock::writes; assert(std::memcmp(tx, next.tx.data(), nt) == 0); }
        if (nr) { ++mock::reads; assert(next.rx.size() == nr); }
        mock::now += 180;  // Deterministic bus cost, not a replay or device model.
        if (!next.result && nr) { std::memcpy(rx, next.rx.data(), nr); }
        return next.result;
    }
};
}
template<typename T> class I2CSPIDriver {
public:
    explicit I2CSPIDriver(const I2CSPIDriverConfig &) {}
    virtual ~I2CSPIDriver() = default;
    virtual void print_status() {}
    static int module_start(const BusCLIArguments &, BusInstanceIterator &) { return 0; }
    static int module_stop(BusInstanceIterator &) { return 0; }
    static int module_status(BusInstanceIterator &) { return 0; }
    void ScheduleAt(uint64_t due) { scheduled_at = due; }
    void ScheduleDelayed(unsigned delay) { scheduled_at = mock::now + delay; }
    void ScheduleNow() { scheduled_at = mock::now; }
    void ScheduleClear() { scheduled_at = 0; }
    uint64_t scheduled_at = 0;
};
struct I2CSPIDriverBase { static void print_status() {} };
class PX4Rangefinder {
public:
    PX4Rangefinder(int, int) {}
    void set_device_type(int) {}
    void set_rangefinder_type(int) {}
    void set_max_distance(float x) { max_distance = x; }
    void set_min_distance(float x) { min_distance = x; }
    void set_fov(float x) { fov = x; }
    void update(uint64_t stamp, float distance) { ++count; last_stamp = stamp; last_distance = distance; }
    unsigned count = 0;
    uint64_t last_stamp = 0;
    float last_distance = 0, min_distance = 0, max_distance = 0, fov = 0;
};
namespace uORB {
class Subscription {
public:
    explicit Subscription(int) {}
    bool copy(vehicle_status_s *s) { *s = mock::status; return mock::status_available; }
};
}
