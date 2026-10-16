#include "px4_test.hpp"
#define private public
#include "TF02PRO.hpp"
#undef private
#include "TFMiniPlus.hpp"

static const std::vector<uint8_t> request{0x5a, 0x05, 0x00, 0x06, 0x65};
static std::vector<uint8_t> frame(uint16_t mm = 1500, uint16_t strength = 200) {
    std::vector<uint8_t> f{0x59, 0x59, uint8_t(mm), uint8_t(mm >> 8),
        uint8_t(strength), uint8_t(strength >> 8), 0, 0, 0};
    f[8] = tfmini_plus::checksum(f.data(), 8); return f;
}
static void due(TF02PRO &driver) { mock::now = driver.scheduled_at; driver.RunImpl(); }
static void start(TF02PRO &driver) {
    mock::expect(request);
    if (driver._configure_once) { mock::expect({}, frame(0, 0)); } // drain, do not publish
    assert(driver.init() == 0);
}
static void sample(TF02PRO &driver, std::vector<uint8_t> f = frame()) {
    mock::expect(request); mock::expect({}, f); due(driver);
}
static void test_ranging_and_deadlines() {
    mock::reset(); TF02PRO d({25, 1, 0}); start(d);
    const auto first = d.scheduled_at;
    sample(d);
    assert(d._published == 1 && d._last_transaction_us == 360);
    assert(d._read_timestamp == first + 180 && d._request_timestamp == first);
    assert(d._px4_rangefinder.last_stamp == d._read_timestamp);
    assert(d.scheduled_at == first + 20000 && mock::sleeps == 0);
    sample(d); assert(d._last_publish_interval_us == 20000);
    const auto deadline = d.scheduled_at;
    mock::expect(request); mock::expect({}, frame());
    mock::now = deadline + 45000; d.RunImpl();
    assert(d._skipped_slots == 2 && d.scheduled_at == deadline + 60000);
    assert(d._last_schedule_late_us == 45000);
    assert(mock::exchanges.empty());
}
static void test_transport_and_frame_failures() {
    mock::reset(); TF02PRO d({25, 1, 0}); start(d);
    mock::expect(request, {}, -1); due(d);
    assert(d._request_errors == 1 && mock::reads == 0 && d._published == 0);
    mock::expect(request); mock::expect({}, frame(), -1); due(d);
    assert(d._read_errors == 1 && d._published == 0);
    auto bad = frame(); bad[0] = 0; sample(d, bad);
    assert(d._header_errors == 1);
    bad = frame(); bad[8] ^= 1; sample(d, bad); assert(d._checksum_errors == 1);
    sample(d, frame(1500, 99)); sample(d, frame(1500, 65535));
    assert(d._signal_rejects == 2);
    sample(d, frame(99)); sample(d, frame(12001)); assert(d._range_rejects == 2);
    assert(d._published == 0 && d._consecutive_failures == 8);
    sample(d, frame(100, 100)); sample(d, frame(12000, 100));
    assert(d._published == 2 && d._consecutive_failures == 0);
}
static void test_configuration_success_once() {
    mock::reset(); TF02PRO d({25, 1, 1}); start(d);
    mock::expect({0x5a, 6, 0x35, 0, 0, 0x95}); due(d);
    auto sent = mock::now;
    assert(d.scheduled_at == sent + 100000);
    mock::expect({}, {0x5a, 6, 0x35, 0, 0, 0x95}); due(d);
    mock::expect({0x5a, 6, 3, 100, 0, 0xc7}); due(d);
    mock::expect({}, {0x5a, 6, 3, 100, 0, 0xc7}); due(d);
    mock::expect({0x5a, 4, 0x11, 0x6f}); due(d);
    sent = mock::now; assert(d.scheduled_at == sent + 1000000);
    mock::expect({}, {0x5a, 5, 0x11, 0, 0x70}); due(d);
    assert(d._configuration_acknowledged && d._config_errors == 0 && d._published == 0);
    sample(d); sample(d);
    assert(mock::writes == 6 && mock::sleeps == 0 && mock::exchanges.empty());
}
static void test_configuration_fail_closed() {
    mock::reset(); mock::status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
    TF02PRO armed({25, 1, 1}); assert(armed.init() == -1 && mock::writes == 0);
    mock::reset(); mock::status_available = false;
    TF02PRO unknown({25, 1, 1}); assert(unknown.init() == -1);
    mock::reset(); TF02PRO d({25, 1, 1}); start(d);
    mock::expect({0x5a, 6, 0x35, 0, 0, 0x95}); due(d);
    // Well-formed echo, but wrong setting, must fail without SAVE or ranging.
    mock::expect({}, {0x5a, 6, 0x35, 1, 0, 0x96}); due(d);
    assert(d._config_errors == 1 && d.scheduled_at == 0 && d._published == 0);
    d.RunImpl(); assert(mock::writes == 2);
    mock::reset(); TF02PRO changed({25, 1, 1}); start(changed);
    mock::status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
    due(changed); assert(changed._config_errors == 1 && mock::writes == 1);
}
static void test_absent_device_and_save_failure() {
    mock::reset(); TF02PRO absent({25, 1, 0});
    mock::expect(request, {}, -1); assert(absent.init() == -1);
    assert(absent.scheduled_at == 0 && mock::reads == 0 && mock::sleeps == 0);
    mock::reset(); TF02PRO undrained({25, 1, 1});
    mock::expect(request); mock::expect({}, frame(), -1);
    assert(undrained.init() == -1 && undrained.scheduled_at == 0 && undrained._read_errors == 1);
    mock::reset(); TF02PRO d({25, 1, 1}); start(d);
    mock::expect({0x5a, 6, 0x35, 0, 0, 0x95}); due(d);
    mock::expect({}, {0x5a, 6, 0x35, 0, 0, 0x95}); due(d);
    mock::expect({0x5a, 6, 3, 100, 0, 0xc7}); due(d);
    mock::expect({}, {0x5a, 6, 3, 100, 0, 0xc7}); due(d);
    mock::expect({0x5a, 4, 0x11, 0x6f}); due(d);
    mock::expect({}, {0x5a, 5, 0x11, 1, 0x71}); due(d);
    assert(d._config_errors == 1 && !d._configuration_acknowledged && d.scheduled_at == 0);
    d.RunImpl(); assert(mock::writes == 4 && d._published == 0);
}
static void test_legacy_profile() {
    mock::reset(); TF02PRO d({25, 0, 0});
    mock::expect(request); assert(d.init() == 0);
    mock::expect(request); due(d);
    auto next = d.scheduled_at;
    mock::expect({}, frame(20000, 60)); mock::expect(request); due(d);
    assert(d._px4_rangefinder.count == 1 && d._px4_rangefinder.max_distance == 35.f);
    assert(d.scheduled_at == next + 360 + 100000 && mock::sleeps == 1);
}
int main() {
    test_ranging_and_deadlines(); test_transport_and_frame_failures();
    test_configuration_success_once(); test_configuration_fail_closed();
    test_absent_device_and_save_failure(); test_legacy_profile();
    std::puts("TFmini Plus: 6 driver test groups passed (mock I2C/clock, no hardware/replay)");
}
