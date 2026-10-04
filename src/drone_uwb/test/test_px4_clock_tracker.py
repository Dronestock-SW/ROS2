"""Clock validity uses observed PX4 progress, never host extrapolation."""
from types import SimpleNamespace
import pytest

from drone_uwb.integration.px4_clock_tracker import PX4ClockTracker


class Reply(SimpleNamespace):
    def get_type(self):
        return 'TIMESYNC'

    def get_srcSystem(self):
        return getattr(self, 'system', 1)

    def get_srcComponent(self):
        return 1


def tracker():
    now, sent = [1_000_000_000], []
    clock = PX4ClockTracker(SimpleNamespace(timesync_send=lambda *args: sent.append(args)),
                            host_now_ns=lambda: now[0])
    return clock, now, sent


def reply(clock, now, sent, px4_ns):
    clock.poll()
    return clock.handle(Reply(ts1=sent[-1][1], tc1=px4_ns))


def test_actual_progress_only_and_repeated_paused_tick_does_not_renew_freshness():
    c, now, sent = tracker()
    reply(c, now, sent, 10_000_000_000)
    now[0] += 100_000_000
    assert c.now_us() == 10_000_000
    reply(c, now, sent, 10_000_000_000)
    now[0] += 110_000_000
    reply(c, now, sent, 10_000_000_000)
    with pytest.raises(ValueError, match='stalled'):
        c.now_us()


def test_delayed_older_request_reply_is_not_a_px4_clock_reset():
    c, now, sent = tracker()
    c.poll()
    old = sent[-1][1]
    now[0] += 50_000_000
    reply(c, now, sent, 10_050_000_000)
    result = c.handle(Reply(ts1=old, tc1=10_000_000_000))
    assert result['reason'] == 'older_request_reply'
    assert c.now_us() == 10_050_000 and c.fault is None


def test_new_request_with_reversed_px4_time_latches_fault():
    c, now, sent = tracker()
    reply(c, now, sent, 10_000_000_000)
    now[0] += 50_000_000
    reply(c, now, sent, 9_000_000_000)
    with pytest.raises(ValueError, match='px4_time_reversed'):
        c.now_us()


def test_foreign_high_rtt_and_future_samples_are_not_accepted():
    c, now, sent = tracker()
    c.poll()
    nonce = sent[-1][1]
    assert c.handle(Reply(ts1=nonce, tc1=10_000_000_000, system=2)) is None
    now[0] += 51_000_000
    assert c.handle(Reply(ts1=nonce, tc1=10_000_000_000))['reason'] == 'round_trip_exceeded'
    with pytest.raises(ValueError, match='unavailable'):
        c.now_us()
    reply(c, now, sent, 10_000_000_000)
    with pytest.raises(ValueError, match='newer_than_clock'):
        c.sample_age_s(10_000_001, now[0])
    assert c.sample_age_s(9_970_000, now[0]-80_000_000) == pytest.approx(.08)


def test_host_time_reversal_invalidates_clock():
    c, now, sent = tracker()
    reply(c, now, sent, 10_000_000_000)
    now[0] -= 1
    c.poll()
    with pytest.raises(ValueError, match='host_time_reversed'):
        c.now_us()
