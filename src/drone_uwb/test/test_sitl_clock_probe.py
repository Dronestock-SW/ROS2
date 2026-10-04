"""PX4 boot-clock diagnostic must reject unmatched replies and stay read-only."""
from collections import deque
from types import SimpleNamespace

import pytest

from drone_uwb.integration.sitl_clock_probe import probe, timesync_sample


class Message:
    def __init__(self, *, system=1, component=1, target=True, **values):
        self._system = system
        self._component = component
        self._values = values
        if target:
            self.target_system = 245
            self.target_component = 191
        for name, value in values.items():
            setattr(self, name, value)

    def get_type(self):
        return self._values.get('kind', 'TIMESYNC')

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


class Clock:
    def __init__(self):
        self.value = 10_000_000_000

    def now_ns(self):
        value = self.value
        self.value += 1_000_000
        return value


class Link:
    def __init__(self, *, legacy=False, spoof=False):
        self.legacy = legacy
        self.spoof = spoof
        self.messages = deque()
        self.calls = []
        self.mav = SimpleNamespace(timesync_send=self.send)

    def wait_heartbeat(self, timeout):
        return Message(kind='HEARTBEAT', autopilot=12, type=2)

    def send(self, *args):
        self.calls.append(args)
        tc1, sent_ns = args
        if self.spoof:
            self.messages.append(Message(tc1=sent_ns+5_500_000,
                                         ts1=sent_ns, system=2))
            self.messages.append(Message(tc1=sent_ns+5_500_000,
                                         ts1=sent_ns, target=True))
            self.messages[-1].target_component = 2
        reply_midpoint_ns = 1_500_000 if self.spoof else 500_000
        self.messages.append(Message(tc1=sent_ns+5_000_000+reply_midpoint_ns,
                                     ts1=sent_ns, target=not self.legacy))

    def recv_match(self, blocking, timeout):
        return self.messages.popleft() if self.messages else None


def test_timesync_sample_preserves_round_trip_and_offset():
    message = Message(tc1=10_005_500_000, ts1=10_000_000_000)
    result = timesync_sample(message, sent_ns=10_000_000_000,
                             received_ns=10_001_000_000)
    assert result['round_trip_ns'] == 1_000_000
    assert result['px4_minus_host_midpoint_ns'] == 5_000_000
    assert result['reply_target_verified'] is True


def test_probe_sends_only_timesync_and_discards_spoofed_replies():
    link, clock = Link(spoof=True), Clock()
    result = probe(link, duration_s=2., samples=3, interval_s=0,
                   now_ns=clock.now_ns)
    assert result['status'] == 'complete'
    assert result['median_offset_ns'] == 5_000_000
    assert result['message_counts']['TIMESYNC_unmatched'] == 6
    assert link.calls == [(0, row['sent_host_monotonic_ns'])
                          for row in result['samples']]
    assert result['all_reply_targets_verified'] is True
    assert result['clock_mapping_verified'] is False
    assert result['external_output_allowed'] is False


def test_old_pymavlink_reply_without_target_stays_diagnostic_only():
    link, clock = Link(legacy=True), Clock()
    result = probe(link, duration_s=2., samples=2, interval_s=0,
                   now_ns=clock.now_ns)
    assert result['status'] == 'complete'
    assert result['all_reply_targets_verified'] is False
    assert not any(row['reply_target_verified'] for row in result['samples'])
    assert result['clock_mapping_verified'] is False


def test_wrong_target_and_time_order_are_rejected():
    message = Message(tc1=5, ts1=2)
    message.target_system = 1
    with pytest.raises(ValueError, match='target'):
        timesync_sample(message, sent_ns=2, received_ns=3)
    message.target_system = 245
    with pytest.raises(ValueError, match='unmatched_timesync_reply'):
        timesync_sample(message, sent_ns=2, received_ns=1)


def test_no_heartbeat_sends_nothing():
    link = Link()
    link.wait_heartbeat = lambda timeout: None
    result = probe(link, duration_s=2., samples=2)
    assert result['status'] == 'no_heartbeat'
    assert link.calls == []
