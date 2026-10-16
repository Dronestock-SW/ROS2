"""Physical frame, covariance and excluded-observation regressions."""
from dataclasses import replace
import math

import numpy as np
import pytest

from drone_uwb.integration.local_odometry import (
    LocalObservationSettings, horizontal_odometry, parameter_gate, alignment_yaw_deg,
    trial_start_allowed, guarded_horizontal_odometry, SenderStartGate)
from drone_uwb.processing.observation_guard import ObservationGuard
from drone_uwb.integration.local_odometry import fresh_bridge_guard


def test_validated_bridge_recovers_on_first_fresh_pose_without_extra_wait():
    guard = fresh_bridge_guard()
    assert guard.check((2., 2.), 1_000_000_000) == 'observation_acquiring'
    assert guard.check((2., 2.), 1_250_000_000) == 'ready'
    assert guard.check((2.01, 2.), 1_950_000_000) == 'ready'
    assert guard.check((2.01, 2.), 1_950_000_000) == 'duplicate_or_backward_observation'
    assert guard.check((float('nan'), 2.), 1_975_000_000) == 'invalid_observation'
from drone_uwb.integration.planar import map_yaw_to_enu


def settings():
    return LocalObservationSettings(enabled=True, layout_confirmed=True,
        sensor_mount_confirmed=True, host_boot_id='synthetic-test-boot',
        fc_remote_ns_at_alignment=1,
        alignment_method='operator_cardinal_and_stationary_px4_yaw')


def test_sender_jump_policy_is_removed_and_migrates_legacy_boolean():
    assert settings().jump_guard_enabled is False
    assert replace(settings(), jump_guard_enabled=True).jump_guard_enabled is False
    assert replace(settings(), jump_guard_enabled=False).jump_guard_enabled is False
    for invalid in (0, 1, 'false', None):
        with pytest.raises(ValueError, match='invalid_confirmation:jump_guard_enabled'):
            replace(settings(), jump_guard_enabled=invalid)


@pytest.mark.parametrize('reject_jumps', [True, False])
def test_ros_sender_boundary_acquires_new_measurements_and_applies_jump_policy(reject_jumps):
    config = replace(settings(), jump_guard_enabled=reject_jumps)
    guard = ObservationGuard(motion_recovery=True)
    def send(x, stamp):
        return guarded_horizontal_odometry(config, guard, frame='uwb_map',
            stamp_ns=stamp, now_ns=stamp+10_000_000, last_stamp_ns=None,
            x=x, y=2., covariance=[[.09, 0.], [0., .09]], motion_eligible=False)
    for i in range(20):
        reason, fields = send(2., 1_000_000_000+i*25_000_000)
    assert reason == 'ready'
    assert fields['time_usec'] == 1_475_000
    reason, fields = send(3., 1_500_000_000)
    assert reason == 'ready'
    assert fields['x'] == -2. and fields['y'] == 3.
    assert send(3., 1_500_000_000)[0] == 'duplicate_or_backward_observation'
    with pytest.raises(ValueError, match='invalid XY observation'):
        send(float('nan'), 1_525_000_000)


def test_sender_trial_starts_only_with_fresh_disarmed_ground_evidence():
    row = dict(connected=True, armed=False, landed_state=1,
               state_age_s=.1, landed_age_s=.1)
    assert trial_start_allowed(**row)
    for key, value in (('connected',False),('armed',True),('armed',None),
                       ('landed_state',True),('landed_state',2),
                       ('state_age_s',2.6),('landed_age_s',.6),
                       ('state_age_s',-.1),('landed_age_s',float('nan'))):
        assert not trial_start_allowed(**{**row, key:value})


def test_new_sender_requires_ground_but_started_fusion_continues_through_arm():
    state = dict(connected=True, armed=True, landed_state=2, state_age_s=.1, landed_age_s=.1)
    gate = SenderStartGate()
    assert not gate.check(**state)
    assert not gate.check(**{**state, 'armed':False, 'landed_state':1, 'landed_age_s':.6})
    assert gate.check(**{**state, 'armed':False, 'landed_state':1})
    assert gate.check(**state)
    assert not SenderStartGate().check(**state)


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('body_map', [0., 90., -90., 180.])
def test_operator_heading_matches_px4_for_both_warehouse_charts(sign, body_map):
    angle = alignment_yaw_deg(47., body_map, sign)
    config = replace(settings(), map_y_axis_sign=sign, enu_yaw_deg=angle)
    assert math.remainder(map_yaw_to_enu(body_map, config)-47., 360.) == 0.


def test_covariance_swap_cross_term_stamp_and_unobserved_fields():
    row = horizontal_odometry(replace(settings(), enu_yaw_deg=0.), frame='uwb_map',
        stamp_ns=1_950_123_000, now_ns=2_000_000_000, last_stamp_ns=None,
        x=2., y=3., covariance=[[.04, .01], [.01, .09]], reset_counter=7)
    np.testing.assert_allclose([row['x'], row['y']], [-3., 2.])
    np.testing.assert_allclose([row['pose_covariance'][i] for i in (0, 1, 6)], [.09, -.01, .04])
    assert row['time_usec'] == 1_950_123 and row['reset_counter'] == 7 and row['z'] == 0.
    assert all(math.isnan(v) for v in row['q'])
    assert all(math.isnan(row[key]) for key in ('vx', 'vy', 'vz', 'rollspeed', 'pitchspeed', 'yawspeed'))
    assert all(row['pose_covariance'][i] == 1e6 for i in (11, 15, 18, 20))


def test_gate_rejects_vertical_yaw_velocity_fusion_and_unverified_inputs():
    params = dict(EKF2_GPS_CTRL=0, EKF2_EV_CTRL=1, EKF2_EV_NOISE_MD=0,
        EKF2_EV_DELAY=0., EKF2_EV_POS_X=-.1, EKF2_EV_POS_Y=0., EKF2_EV_POS_Z=-.1, EKF2_EV_QMIN=0)
    assert parameter_gate(settings(), params) == 'ready'
    for value in (0, 3, 5, 9, 15, True):
        assert parameter_gate(settings(), {**params, 'EKF2_EV_CTRL': value}) != 'ready'
    for field in ('enabled', 'layout_confirmed', 'sensor_mount_confirmed'):
        assert parameter_gate(replace(settings(), **{field: False}), params) != 'ready'
    assert parameter_gate(replace(settings(), alignment_method=''), params) != 'ready'
    assert parameter_gate(settings(), {**params, 'EKF2_EV_QMIN': 1}) != 'ready'


@pytest.mark.parametrize('change', [dict(frame='map'), dict(stamp_ns=0),
    dict(stamp_ns=2_000_000_001), dict(last_stamp_ns=1_950_000_000),
    dict(stamp_ns=1_000_000_000), dict(covariance=[[.1,.2],[.2,.1]])])
def test_old_or_invalid_observations_cannot_be_transmitted(change):
    data = dict(frame='uwb_map', stamp_ns=1_950_000_000, now_ns=2_000_000_000,
        last_stamp_ns=None, x=2., y=3., covariance=[[.04, 0.], [0., .09]])
    with pytest.raises(ValueError):
        horizontal_odometry(settings(), **{**data, **change})


def test_actual_mavlink_v2_wire_retains_unknown_attitude_and_capture_stamp():
    pytest.importorskip('pymavlink')
    from pymavlink.dialects.v20 import common as dialect
    fields = horizontal_odometry(settings(), frame='uwb_map',
        stamp_ns=1_950_123_000, now_ns=2_000_000_000, last_stamp_ns=None,
        x=2., y=3., covariance=[[.04, .01], [.01, .09]])
    encoder = dialect.MAVLink(None, srcSystem=255, srcComponent=196)
    packet = encoder.odometry_encode(fields['time_usec'], dialect.MAV_FRAME_LOCAL_NED,
        dialect.MAV_FRAME_BODY_FRD, fields['x'], fields['y'], fields['z'], fields['q'],
        fields['vx'], fields['vy'], fields['vz'], fields['rollspeed'], fields['pitchspeed'],
        fields['yawspeed'], fields['pose_covariance'], fields['velocity_covariance'],
        fields['reset_counter'], dialect.MAV_ESTIMATOR_TYPE_VISION, fields['quality'])
    decoded = dialect.MAVLink(None).decode(bytearray(packet.pack(encoder)))
    assert decoded.get_msgId() == 331 and decoded.time_usec == 1_950_123
    assert decoded.get_srcSystem() == 255 and decoded.get_srcComponent() == 196
    assert decoded.frame_id == dialect.MAV_FRAME_LOCAL_NED
    assert all(math.isnan(v) for v in decoded.q) and math.isnan(decoded.vz)
    assert decoded.z == 0. and decoded.pose_covariance[11] == 1e6
