"""Only PX4 clock requests may train the future SITL sender clock path."""
from types import SimpleNamespace

import pytest

from drone_uwb.integration.sitl_timesync import SITLTimesyncResponder


class Message:
    def __init__(self, stamp=1_000_000_000, *, source=(1, 1), target=(0, 0), tc1=0):
        self.ts1, self.tc1 = stamp, tc1
        self.source = source
        if target is not None:
            self.target_system, self.target_component = target

    def get_type(self):
        return 'TIMESYNC'

    def get_srcSystem(self):
        return self.source[0]

    def get_srcComponent(self):
        return self.source[1]


class Sender:
    def __init__(self):
        self.calls = []

    def timesync_send(self, *args, **kwargs):
        self.calls.append((args, kwargs))


def make_responder(*, targeted=True, clock=None):
    fields = ['tc1', 'ts1']+(['target_system', 'target_component'] if targeted else [])
    dialect = SimpleNamespace(MAVLink_timesync_message=SimpleNamespace(fieldnames=fields))
    sender = Sender()
    times = iter(clock or [8_000_000_000, 8_100_000_000, 8_200_000_000])
    return sender, SITLTimesyncResponder(sender, dialect, now_ns=lambda: next(times))


def test_targeted_reply_echoes_px4_request_and_uses_sender_clock():
    sender, responder = make_responder()
    row = responder.handle(Message())
    assert sender.calls == [((8_000_000_000, 1_000_000_000),
                             dict(target_system=1, target_component=1))]
    assert row['action'] == 'responded'
    assert row['clock_mapping_verified'] is False
    assert row['observations_sent'] is False


def test_legacy_dialect_uses_only_two_fields_and_records_limit():
    sender, responder = make_responder(targeted=False)
    row = responder.handle(Message(target=None))
    assert sender.calls == [((8_000_000_000, 1_000_000_000), {})]
    assert row['reply_target_encoded'] is False
    assert row['request_target_available'] is False


def test_other_sources_targets_and_responses_do_not_send_or_consume_clock():
    sender, responder = make_responder(clock=[8_000_000_000])
    for msg in (Message(source=(2, 1)), Message(target=(2, 191)),
                Message(target=(245, 2)), Message(tc1=123), Message(stamp=0)):
        assert responder.handle(msg)['action'] == 'ignored'
    assert sender.calls == []
    assert responder.handle(Message())['sender_reply_ns'] == 8_000_000_000


def test_duplicate_does_not_reply_and_reverse_latches_until_mapping_reset():
    sender, responder = make_responder()
    responder.handle(Message())
    assert responder.handle(Message())['reason'] == 'duplicate_px4_request'
    row = responder.handle(Message(stamp=1))
    assert row['mapping_invalidation_required'] is True
    assert row['reason'] == 'px4_request_time_reversed'
    assert responder.handle(Message(stamp=1_100_000_000))['action'] == 'suspended'
    assert len(sender.calls) == 1
    responder.reset_epoch()
    row = responder.handle(Message(stamp=2))
    assert row['action'] == 'responded' and row['epoch'] == 1
    assert len(sender.calls) == 2


def test_host_clock_reversal_and_transport_error_require_reset():
    sender, responder = make_responder(clock=[100, 99])
    responder.handle(Message())
    assert responder.handle(Message(stamp=2_000_000_000))['reason'] == 'sender_time_not_advancing'
    assert len(sender.calls) == 1
    sender, responder = make_responder()
    def fail(*args, **kwargs):
        raise OSError('test_transport_failure')
    sender.timesync_send = fail
    with pytest.raises(OSError, match='test_transport_failure'):
        responder.handle(Message())
    assert responder.handle(Message(stamp=2_000_000_000))['reason'] == 'timesync_send_failed'
