"""Real pymavlink checks in memory and loopback UDP, without a live PX4."""
import io
import math
import socket
import time

import pytest

from drone_uwb.integration.sitl.sitl_odometry_contract import (
    SITLOdometrySettings, odometry_fields, send_odometry_fields,
)
from drone_uwb.integration.sitl.sitl_timesync import SITLTimesyncResponder


mavlink = pytest.importorskip('pymavlink.dialects.v20.common')


def test_odometry_wire_round_trip_keeps_remote_time_covariance_and_unknowns():
    settings = SITLOdometrySettings(
        enabled=True, alignment_confirmed=True, mount_confirmed=True,
        height_confirmed=True, time_mapping_confirmed=True,
        px4_reference_confirmed=True)
    params = dict(EKF2_EV_CTRL=1, EKF2_EV_NOISE_MD=0, EKF2_EV_DELAY=0.,
                  EKF2_EV_POS_X=0., EKF2_EV_POS_Y=0., EKF2_EV_POS_Z=0.)
    candidate = dict(position_ned_m=[1.25, -2.5, -1.],
                     covariance_ned_xy_m2=[[.001, .0002], [.0002, .002]])
    fields = odometry_fields(candidate, 7_999_980_000, settings, params,
                             sample_clock_domain='wsl_monotonic_us',
                             fc_connected=True, param_age_s=.1, sample_age_s=.02)
    stream = io.BytesIO()
    encoder = mavlink.MAVLink(stream, srcSystem=245, srcComponent=191)
    send_odometry_fields(encoder, mavlink, fields)
    packet = stream.getvalue()
    assert packet[0] == 0xfd  # ODOMETRY requires MAVLink 2.
    messages = mavlink.MAVLink(None).parse_buffer(packet)
    assert len(messages) == 1
    decoded = messages[0]
    assert decoded.get_type() == 'ODOMETRY'
    assert decoded.get_srcSystem() == 245 and decoded.get_srcComponent() == 191
    assert decoded.time_usec == 7_999_980_000
    assert decoded.frame_id == mavlink.MAV_FRAME_LOCAL_NED
    assert decoded.child_frame_id == mavlink.MAV_FRAME_BODY_FRD
    assert [decoded.x, decoded.y, decoded.z] == [1.25, -2.5, -1.]
    assert all(math.isnan(value) for value in decoded.q)
    assert all(math.isnan(getattr(decoded, key)) for key in ('vx', 'vy', 'vz'))
    assert decoded.pose_covariance[0] == pytest.approx(.001)
    assert decoded.pose_covariance[1] == pytest.approx(.0002)
    assert decoded.pose_covariance[6] == pytest.approx(.002)
    assert decoded.pose_covariance[11] == 1e6
    assert math.isnan(decoded.velocity_covariance[0])
    assert decoded.estimator_type == mavlink.MAV_ESTIMATOR_TYPE_VISION


def test_timesync_reply_wire_echoes_request_with_sender_monotonic_clock():
    request_stream = io.BytesIO()
    px4_encoder = mavlink.MAVLink(request_stream, srcSystem=1, srcComponent=1)
    px4_encoder.timesync_send(0, 30_000_000_000)
    request = mavlink.MAVLink(None).parse_buffer(request_stream.getvalue())[0]
    reply_stream = io.BytesIO()
    sender = mavlink.MAVLink(reply_stream, srcSystem=245, srcComponent=191)
    responder = SITLTimesyncResponder(sender, mavlink, now_ns=lambda: 8_000_000_000_000)
    event = responder.handle(request)
    reply = mavlink.MAVLink(None).parse_buffer(reply_stream.getvalue())[0]
    assert reply.get_type() == 'TIMESYNC'
    assert reply.get_srcSystem() == 245 and reply.get_srcComponent() == 191
    assert reply.ts1 == 30_000_000_000
    assert reply.tc1 == 8_000_000_000_000
    if event['reply_target_encoded']:
        assert reply.target_system == 1 and reply.target_component == 1
    assert event['clock_mapping_verified'] is False


def test_observer_actual_udp_loopback_keeps_sensor_stamp_and_receives_reset(monkeypatch):
    """A fake PX4 peer exercises real pymavlink UDP, not EKF fusion."""
    monkeypatch.setenv('MAVLINK20', '1')
    from pymavlink import mavutil
    from drone_uwb.integration.sitl.sitl_observer import SITLObserver
    from test_sitl_observer import make_observer, result, PARAMS, parameter

    prepared, _, host, clock = make_observer()
    # OS-assigned ports avoid touching a running SITL's 14540.
    link = mavutil.mavlink_connection('udpin:127.0.0.1:0', dialect='common',
                                      source_system=245, source_component=191)
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0))
    peer.settimeout(.5)
    destination = link.port.getsockname()

    class Writer:
        def write(self, data):
            peer.sendto(data, destination)

    px4 = mavlink.MAVLink(Writer(), srcSystem=1, srcComponent=1)
    observer = SITLObserver(prepared.settings, prepared.runtime, clock, mode='send',
                             connection=link, dialect=mavutil.mavlink, host_now_ns=lambda: host[0])
    try:
        px4.heartbeat_send(2, mavlink.MAV_AUTOPILOT_PX4, 0, 0, 3)
        for i, (name, value) in enumerate(PARAMS.items()):
            msg = parameter(name, value)
            px4.param_value_send(name.encode(), msg.param_value, msg.param_type, len(PARAMS), i)
        def send_px4_odometry(stamp, reset):
            px4.odometry_send(stamp, mavlink.MAV_FRAME_LOCAL_NED, mavlink.MAV_FRAME_BODY_FRD,
                0., 0., 0., [1., 0., 0., 0.], 0., 0., 0., 0., 0., 0.,
                [0.]*21, [0.]*21, reset, mavlink.MAV_ESTIMATOR_TYPE_AUTOPILOT, 0)
        send_px4_odometry(1_000_000, 0)
        px4.timesync_send(0, 1_000_000_000)
        deadline = time.monotonic()+1.
        input_events = []
        while observer.timesync_replies < 1 and time.monotonic() < deadline:
            input_events.extend(observer.poll_link())
        assert observer.timesync_replies == 1
        px4_state = next(event for event in input_events if event['type'] == 'ODOMETRY')
        assert px4_state['payload']['time_usec'] == 1_000_000
        assert px4_state['payload']['child_frame_id'] == mavlink.MAV_FRAME_BODY_FRD
        assert px4_state['payload']['velocity_covariance'] == [0.]*21
        assert px4_state['fusion_verified'] is False
        clock.update(1_040_000_000)
        px4.timesync_send(0, 1_040_000_000)
        while observer.timesync_replies < 2 and time.monotonic() < deadline:
            observer.poll_link()
        event = observer.observe(result(), raw_received_host_ns=host[0])
        assert event['transmitted']
        receiver = mavlink.MAVLink(None)
        received = []
        while not any(msg.get_type() == 'ODOMETRY' for msg in received):
            data, _ = peer.recvfrom(65535)
            received.extend(receiver.parse_buffer(data) or [])
        odometry = next(msg for msg in received if msg.get_type() == 'ODOMETRY')
        assert odometry.get_srcSystem() == 245 and odometry.get_srcComponent() == 191
        assert odometry.time_usec == 1_000_000
        assert [odometry.x, odometry.y, odometry.z] == pytest.approx([1.68, 2.23, -.7])
        assert all(math.isnan(v) for v in odometry.q)
        assert {msg.get_type() for msg in received} <= {'PARAM_REQUEST_READ', 'TIMESYNC', 'ODOMETRY'}
        send_px4_odometry(1_040_000, 1)
        deadline = time.monotonic()+1.
        while observer.fault is None and time.monotonic() < deadline:
            observer.poll_link()
        assert observer.fault == 'px4_reference_reset'
        assert not observer.observe(result(1, 1_040_000), raw_received_host_ns=host[0])['transmitted']
    finally:
        link.close()
        peer.close()
