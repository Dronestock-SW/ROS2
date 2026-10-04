"""PX4 parameter snapshot must decode bytewise integers without changing FC state."""
from collections import deque
import struct
from types import SimpleNamespace
import pytest

from drone_uwb.integration.sitl_link_probe import PARAMETERS, decode_px4_param, probe


INT32, REAL32 = 6, 9


def encoded_int(value):
    return struct.unpack('<f', struct.pack('<i', value))[0]


class Message:
    def __init__(self, kind, **values):
        self.kind = kind
        self.__dict__.update(values)

    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class Link:
    def __init__(self, messages):
        self.messages = deque(messages)
        self.requests = []
        self.mav = SimpleNamespace(param_request_read_send=self._request)

    def _request(self, *args):
        self.requests.append(args)

    def wait_heartbeat(self, timeout):
        return Message('HEARTBEAT', autopilot=12, type=2)

    def recv_match(self, blocking, timeout):
        return self.messages.popleft() if self.messages else None


def test_int_parameter_uses_px4_bytewise_encoding():
    decoded = decode_px4_param('EKF2_EV_CTRL', encoded_int(1), INT32,
                               int32_type=INT32, real32_type=REAL32)
    assert decoded['value'] == 1
    assert decoded['raw_float'] != 1.
    negative = decode_px4_param('EKF2_EV_CTRL', encoded_int(-1), INT32,
                                int32_type=INT32, real32_type=REAL32)
    assert negative['value'] == -1
    assert negative['raw_float'] is None and negative['raw_bits_hex'] == 'ffffffff'
    float_value = decode_px4_param('EKF2_EV_DELAY', 12.5, REAL32,
                                   int32_type=INT32, real32_type=REAL32)
    assert float_value['value'] == 12.5


def test_navigation_parameter_allowlist_does_not_expand_default_probe_scope():
    with pytest.raises(ValueError, match='unexpected_parameter'):
        decode_px4_param('NAV_DLL_ACT', encoded_int(3), INT32,
                         int32_type=INT32, real32_type=REAL32)
    result = decode_px4_param('NAV_DLL_ACT', encoded_int(3), INT32,
        int32_type=INT32, real32_type=REAL32, allowed_names=('NAV_DLL_ACT',))
    assert result['value'] == 3 and result['encoding'] == 'px4_bytewise'


def test_probe_reads_every_required_parameter_without_set_or_flight_command():
    messages = []
    for name in PARAMETERS:
        integer = name in ('EKF2_EV_CTRL', 'EKF2_EV_NOISE_MD',
                           'EKF2_GPS_CTRL', 'EKF2_OF_CTRL',
                           'EKF2_RNG_CTRL', 'EKF2_HGT_REF')
        value = encoded_int(1) if integer else .05
        kind = INT32 if integer else REAL32
        messages.append(Message('PARAM_VALUE', param_id=name.encode(),
                                param_type=kind, param_value=value))
    link = Link(messages)
    result = probe(link, SimpleNamespace(MAV_PARAM_TYPE_INT32=INT32,
                                         MAV_PARAM_TYPE_REAL32=REAL32),
                   duration_s=5.)
    assert result['status'] == 'complete'
    assert not result['missing']
    assert result['parameters']['EKF2_EV_CTRL']['value'] == 1
    assert len(link.requests) == len(PARAMETERS)
    assert all(args[0:2] == (1, 1) and args[3] == -1 for args in link.requests)
    assert all(args[2].decode('ascii') in PARAMETERS for args in link.requests)
    assert result['external_output_allowed'] is False


def test_missing_heartbeat_is_explicit_and_sends_no_requests():
    link = Link([])
    link.wait_heartbeat = lambda timeout: None
    result = probe(link, SimpleNamespace(), duration_s=3.)
    assert result['status'] == 'no_heartbeat'
    assert result['missing'] == list(PARAMETERS)
    assert link.requests == []
