"""PX4 velocity/frame/reset evidence governs arrival, never Gazebo truth."""
from dataclasses import replace
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from drone_demo.mission import MissionConfig
from drone_demo.sitl_mission import PX4MissionMonitor
from drone_uwb.integration.sitl_odometry_contract import SITLOdometrySettings


CONFIG = MissionConfig(**json.loads((Path(__file__).parents[1]/'config/mission.json')
                                    .read_text(encoding='utf-8')))
DIALECT = SimpleNamespace(MAV_ESTIMATOR_TYPE_AUTOPILOT=8, MAV_FRAME_LOCAL_NED=1,
    MAV_FRAME_BODY_FRD=12, ESTIMATOR_ATTITUDE=1, ESTIMATOR_VELOCITY_HORIZ=2,
    ESTIMATOR_POS_HORIZ_REL=8, ESTIMATOR_CONST_POS_MODE=128, ESTIMATOR_ACCEL_ERROR=2048)


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return getattr(self, 'sysid', 1)

    def get_srcComponent(self):
        return 1


def monitor():
    alignment = SITLOdometrySettings(alignment_confirmed=True, px4_reference_confirmed=True)
    result = PX4MissionMonitor(CONFIG, alignment, DIALECT)
    result.set_target(2., 1.)
    return result


def feed(monitor, t, *, velocity=(0., 0., 0.), odometry_changes=None, flags=11,
         estimator_age_s=0., uwb=True):
    stamp = 1_000_000+round(t*1e6)
    odometry = Message(kind='ODOMETRY', time_usec=stamp, reset_counter=0,
        x=1., y=2., vx=velocity[0], vy=velocity[1], vz=velocity[2],
        q=[1., 0., 0., 0.], frame_id=1, child_frame_id=1, estimator_type=8)
    if odometry_changes:
        odometry.__dict__.update(odometry_changes)
    estimator = Message(kind='ESTIMATOR_STATUS', time_usec=stamp, flags=flags)
    if uwb:
        # Different epoch than PX4; the monitor must not subtract these clocks.
        monitor.update_uwb(100_000_000_000+round(t*1e9), t, 0.)
    accepted = monitor.update_px4(odometry, estimator, received_s=t,
                                 sample_age_s=0., estimator_age_s=estimator_age_s)
    return accepted, monitor.evaluate(t)


def settle(m):
    for i in range(81):
        _, result = feed(m, i/40)
    assert result['arrival_valid']
    return result


def test_uses_px4_velocity_even_when_positions_are_repeated():
    m = monitor()
    for i in range(100):
        _, result = feed(m, i/40, velocity=(.4, 0., 0.))
        assert not result['arrival_valid']
    assert result['speed_m_s'] == .4
    assert result['velocity_source'] == result['position_source'] == 'px4_ekf2'
    assert not result['demo'] and not result['flight_output']


def test_arrives_only_after_fresh_px4_velocity_distance_and_dwell():
    result = settle(monitor())
    assert result['distance_m'] == 0.
    assert result['reference_point'] == 'px4_reference'
    assert result['source'] == 'simulation' and not result['fusion_verified']


def test_body_velocity_is_rotated_with_full_attitude_before_xy_selection():
    m = monitor()
    # At +90 degree pitch, body downward velocity is NED north velocity.
    _, result = feed(m, 0., velocity=(0., 0., .4), odometry_changes={
        'child_frame_id': 12, 'q': [math.sqrt(.5), 0., math.sqrt(.5), 0.]})
    assert result['speed_m_s'] == pytest.approx(.4)
    assert not result['arrival_valid']


def test_velocity_rotation_does_not_include_map_translation():
    m = monitor()
    m.alignment = replace(m.alignment, enu_offset_x_m=1000., enu_offset_y_m=-1000., enu_yaw_deg=37.)
    _, result = feed(m, 0., velocity=(.03, .04, 0.))
    assert result['speed_m_s'] == pytest.approx(.05)


@pytest.mark.parametrize('flags', [0, 9, 3, 11 | 128, 11 | 2048])
def test_invalid_estimator_state_cancels_existing_arrival(flags):
    m = monitor()
    settle(m)
    accepted, result = feed(m, 2.025, flags=flags)
    assert not accepted and not result['arrival_valid']
    assert result['last_input_rejection'] == 'px4_xy_velocity_or_attitude_invalid'


@pytest.mark.parametrize('changes,reason', [
    ({'reset_counter': 1}, 'px4_reference_reset'),
    ({'time_usec': 2_999_000}, 'px4_time_reversed'),
])
def test_reset_and_reversed_px4_time_latch(changes, reason):
    m = monitor()
    settle(m)
    _, result = feed(m, 2.025, odometry_changes=changes)
    assert not result['arrival_valid'] and m.fault == reason
    assert not feed(m, 2.05)[0]


@pytest.mark.parametrize('changes', [
    {'frame_id': 20}, {'child_frame_id': 20}, {'vx': float('nan')},
    {'child_frame_id': 12, 'q': [0., 0., 0., 0.]},
])
def test_invalid_frame_and_unknown_velocity_are_not_zero_speed(changes):
    m = monitor()
    settle(m)
    accepted, result = feed(m, 2.025, odometry_changes=changes)
    assert not accepted and not result['arrival_valid']
    assert result['speed_m_s'] is None


def test_stale_status_and_lost_uwb_block_arrival_independently():
    m = monitor()
    settle(m)
    assert not feed(m, 2.025, estimator_age_s=.201)[1]['arrival_valid']
    for i in range(82, 161):
        result = feed(m, i/40)[1]
    assert result['arrival_valid']
    for i in range(161, 181):
        result = feed(m, i/40, uwb=False)[1]
    assert result['reason'] == 'uwb_unavailable'
    assert result['pose_age_s'] == 0.


def test_duplicate_foreign_and_vision_odometry_do_not_refresh_pose():
    m = monitor()
    settle(m)
    for i in range(81, 94):
        changes = ({'time_usec': 3_000_000}, {'sysid': 42}, {'estimator_type': 2})[i % 3]
        accepted, result = feed(m, i/40, odometry_changes=changes)
        assert not accepted
    assert not result['arrival_valid']


def test_host_time_reversal_cannot_resume_old_target():
    m = monitor()
    settle(m)
    assert not m.evaluate(1.9)['arrival_valid']
    assert m.fault == 'host_time_reversed'
    assert not feed(m, 2.025)[0]
