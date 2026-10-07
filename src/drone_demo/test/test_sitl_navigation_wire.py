"""Actual common dialect, in-memory PX4 peer. Not an actual SITL flight."""
import io
import math

import pytest

from drone_demo.sitl_mission import PX4MissionMonitor
from drone_uwb.integration.sitl.sitl_target_contract import (
    NavigationReadiness, PX4GlobalReference, RepositionProgress, SITLTargetSettings,
    reposition_fields, send_reposition_fields)
from drone_uwb.integration.sitl.sitl_odometry_contract import SITLOdometrySettings
from test_sitl_mission import CONFIG


mavlink = pytest.importorskip('pymavlink.dialects.v20.common')


def wire_message(method, *args):
    stream = io.BytesIO()
    peer = mavlink.MAVLink(stream, srcSystem=1, srcComponent=1)
    getattr(peer, method)(*args)
    return mavlink.MAVLink(None).parse_buffer(stream.getvalue())[0]


def prepared():
    alignment = SITLOdometrySettings(alignment_confirmed=True, px4_reference_confirmed=True)
    origin = wire_message('gps_global_origin_send', 473979860, 85461920, 200, 3_224_000)
    reference = PX4GlobalReference.from_message(origin)
    state = NavigationReadiness(.02, .02, reference.reference_timestamp_us, 0,
                                 True, True, True, True, True, True, True)
    settings = SITLTargetSettings(enabled=True, sitl_session_confirmed=True,
                                  observation_fusion_confirmed=True)
    packet = reposition_fields((2.59, 1.68), (2.09, 1.68), settings, alignment,
                               reference, state, expected_reset_counter=0)
    stream = io.BytesIO()
    sender = mavlink.MAVLink(stream, srcSystem=245, srcComponent=191)
    send_reposition_fields(sender, mavlink, packet)
    return alignment, packet, stream.getvalue()


def echo(packet, stamp_ms):
    return wire_message('position_target_global_int_send', stamp_ms,
        mavlink.MAV_FRAME_GLOBAL_INT, 0, packet['x'], packet['y'], .7,
        0., 0., 0., 0., 0., 0., 0., 0.)


def test_command_int_preserves_centimetre_target_and_unknown_altitude():
    _, packet, data = prepared()
    command = mavlink.MAVLink(None).parse_buffer(data)[0]
    assert command.get_type() == 'COMMAND_INT'
    assert command.command == mavlink.MAV_CMD_DO_REPOSITION
    assert command.frame == mavlink.MAV_FRAME_GLOBAL
    assert (command.x, command.y) == (packet['x'], packet['y'])
    assert command.param1 == pytest.approx(.3)
    assert command.param2 == 0  # PX4 must already be in Hold.
    assert all(math.isnan(value) for value in (command.param3, command.param4, command.z))


def test_ack_and_target_echo_require_new_px4_samples_before_arrival():
    alignment, packet, _ = prepared()
    progress = RepositionProgress(packet, mavlink, sent_host_s=2., sent_px4_us=3_000_000)
    m = PX4MissionMonitor(CONFIG, alignment, mavlink)
    m.set_target(*packet['target_map_xy_m'])
    ack = wire_message('command_ack_send', mavlink.MAV_CMD_DO_REPOSITION,
                        mavlink.MAV_RESULT_ACCEPTED, 0, 0, 245, 191)
    assert progress.handle(ack, received_host_s=2.01)
    assert not progress.evaluate(2.01, {})['target_applied']
    for i in range(1, 81):
        now = 2.+i/40
        stamp = 3_000_000+i*25000
        odometry = wire_message('odometry_send', stamp, mavlink.MAV_FRAME_LOCAL_NED,
            mavlink.MAV_FRAME_LOCAL_NED, 1.68, 2.59, -.7, [1., 0., 0., 0.],
            0., 0., 0., 0., 0., 0., [.001]*21, [.001]*21, 0,
            mavlink.MAV_ESTIMATOR_TYPE_AUTOPILOT, 0)
        estimator = wire_message('estimator_status_send', stamp,
            mavlink.ESTIMATOR_ATTITUDE | mavlink.ESTIMATOR_VELOCITY_HORIZ | mavlink.ESTIMATOR_POS_HORIZ_REL,
            0., 0., 0., 0., 0., 0., .02, .02)
        m.update_uwb(100_000_000_000+i*25000000, now, 0.)
        assert m.update_px4(odometry, estimator, received_s=now, sample_age_s=0., estimator_age_s=0.)
        assert progress.handle(echo(packet, stamp//1000), received_host_s=now, sample_age_s=0.)
        assessment = m.evaluate(now)
        result = progress.evaluate(now, assessment)
        if i < 61:
            assert not result['arrival_valid']
    assert result['command_accepted'] and result['target_applied'] and result['arrival_valid']
    assert not result['flight_valid'] and not result['fusion_verified']
    # Reusing the old arrival record cannot extend the result indefinitely.
    assert not progress.evaluate(now+.1, assessment)['arrival_valid']


def test_foreign_ack_stale_echo_and_changed_fc_target_are_not_completion():
    _, packet, _ = prepared()
    progress = RepositionProgress(packet, mavlink, sent_host_s=2., sent_px4_us=3_000_000)
    foreign = wire_message('command_ack_send', mavlink.MAV_CMD_DO_REPOSITION,
                            mavlink.MAV_RESULT_ACCEPTED, 0, 0, 250, 190)
    assert not progress.handle(foreign, received_host_s=2.01)
    assert not progress.handle(echo(packet, 2900), received_host_s=2.02, sample_age_s=.01)
    ack = wire_message('command_ack_send', mavlink.MAV_CMD_DO_REPOSITION,
                        mavlink.MAV_RESULT_ACCEPTED, 0, 0, 245, 191)
    progress.handle(ack, received_host_s=2.03)
    progress.handle(echo(packet, 3040), received_host_s=2.04, sample_age_s=0.)
    different = dict(packet, x=packet['x']+1000)
    assert not progress.handle(echo(different, 3050), received_host_s=2.05, sample_age_s=0.)
    assert progress.evaluate(2.05, {})['reason'] == 'px4_target_changed'
    assert not progress.handle(echo(packet, 3060), received_host_s=2.06, sample_age_s=0.)


def test_response_timeout_latches_without_a_resend():
    _, packet, _ = prepared()
    progress = RepositionProgress(packet, mavlink, sent_host_s=2., sent_px4_us=3_000_000)
    assert progress.evaluate(4.001, {})['reason'] == 'command_response_timeout'
    ack = wire_message('command_ack_send', mavlink.MAV_CMD_DO_REPOSITION,
                        mavlink.MAV_RESULT_ACCEPTED, 0, 0, 245, 191)
    assert not progress.handle(ack, received_host_s=4.01)
