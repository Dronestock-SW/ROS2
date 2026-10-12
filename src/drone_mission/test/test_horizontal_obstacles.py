from dataclasses import replace
import math

import numpy as np
import pytest

from drone_mission.horizontal_obstacles import (
    GuardConfig, HorizontalGuard, scan_obstacles, stopping_distance, speed_for_clearance,
    ordered_obstacles,
)


def test_deskewed_obstacles_keep_unknown_bins_and_oldest_capture_time():
    angles = (np.arange(144)+.5)*2*np.pi/144
    points = np.column_stack((3*np.cos(angles), 3*np.sin(angles)))
    valid = np.ones(144, dtype=bool)
    valid[0] = False
    value = ordered_obstacles(relative_enu_xy=points, valid=valid,
        bearings_enu_rad=angles, sensor_origin_xy_m=(2., 1.),
        stamp_ns=1_000_000_000, sequence=1, clock_id='clock',
        source_id='lidar', epoch='boot', bins=72, acquisition_motion_bound_m=.12)
    assert len(value.points_xy_m)==143
    assert not value.known_bins[0] and all(value.known_bins[1:])
    assert value.points_xy_m[0]==pytest.approx(points[1]+[2., 1.])
    assert value.acquisition_motion_bound_m==.12
    g=HorizontalGuard(CFG, source_id='lidar', clock_id='clock', epoch='boot')
    assert g.observe(value)=='ready'
    assert limit(g,current_xy_m=(2.,1.))['velocity_xy_m_s']==(0.,0.)
    assert limit(g,current_xy_m=(2.,1.),now_ns=1_250_000_001)['reason']=='scan_stale_or_future'


CFG = GuardConfig(radius_m=.2, clearance_m=.1, brake_accel_m_s2=1.,
    command_latency_s=.1, max_age_s=.25, max_speed_m_s=2., range_error_m=.02,
    pose_error_m=.03, obstacle_speed_bound_m_s=0., velocity_error_m_s=0.)


def box_scan(front=3., rear=3., side=3., yaw=0.):
    angles = np.linspace(-math.pi, math.pi, 721)
    ranges = []
    for theta in angles + yaw:
        x, y = math.cos(theta), math.sin(theta)
        xhit = front/x if x > 1e-10 else -rear/x if x < -1e-10 else 1e9
        yhit = side/abs(y) if abs(y) > 1e-10 else 1e9
        ranges.append(min(xhit, yhit))
    return dict(angle_min=-math.pi, angle_max=math.pi,
                angle_increment=2*math.pi/720, ranges=ranges,
                range_min=.03, range_max=12.)


def frame(value=None, **kw):
    args = dict(stamp_ns=1_000_000_000, sequence=1, clock_id='clock', source_id='lidar',
                epoch='boot', sensor_origin_xy_m=(0., 0.), sensor_yaw_rad=0.,
                bins=72, mount_confirmed=True, horizontal_plane_confirmed=True,
                oldest_stamp_confirmed=True, acquisition_motion_bound_m=0.)
    args.update(kw)
    return scan_obstacles(box_scan() if value is None else value, **args)


def guard(value=None, config=CFG):
    g = HorizontalGuard(config, source_id='lidar', clock_id='clock', epoch='boot')
    assert g.observe(frame(value)) == 'ready'
    return g


def limit(g, **kw):
    args = dict(desired_xy_m_s=(1., 0.), measured_xy_m_s=(0., 0.),
                current_xy_m=(0., 0.), now_ns=1_050_000_000)
    args.update(kw)
    return g.limit(**args)


def test_open_box_preserves_command_and_near_wall_slows_it():
    assert limit(guard())['velocity_xy_m_s'] == pytest.approx((1., 0.))
    result = limit(guard(box_scan(front=.65)))
    assert result['reason'] == 'limited'
    assert 0 < result['velocity_xy_m_s'][0] < 1.
    assert stopping_distance(result['velocity_xy_m_s'][0], CFG, .05) == pytest.approx(result['clearance_m'])


def test_existing_momentum_brakes_even_with_reverse_requested():
    result = limit(guard(box_scan(front=.65)), desired_xy_m_s=(-.2, 0.), measured_xy_m_s=(1., 0.))
    assert result['velocity_xy_m_s'] == (0., 0.)
    assert result['reason'].startswith('brake:')


def test_user_clearance_is_fifty_cm_outside_propeller_radius():
    cfg = replace(CFG, clearance_m=.5)
    # Centre distance 0.70 m = 0.20 m assumed rotor envelope + 0.50 m gap.
    result = limit(guard(box_scan(front=.70), config=cfg))
    assert result['velocity_xy_m_s'] == (0., 0.)
    assert result['reason'].startswith('brake:')


def test_velocity_uncertainty_starts_braking_earlier():
    nominal = limit(guard(box_scan(front=1.5)))
    bounded = limit(guard(box_scan(front=1.5), config=replace(CFG, velocity_error_m_s=.35)))
    assert bounded['velocity_xy_m_s'][0] < nominal['velocity_xy_m_s'][0]


@pytest.mark.parametrize('invalid', [0., None, math.nan, math.inf, 'NaN'])
def test_missing_forward_ray_is_unknown_not_infinite_free_space(invalid):
    scan = box_scan(); scan['ranges'][360] = invalid
    result = limit(guard(scan))
    assert result['velocity_xy_m_s'] == (0., 0.)
    assert result['reason'] == 'direction_has_unknown_returns'


def test_unknown_rear_does_not_authorize_reverse_motion():
    scan = box_scan(); scan['ranges'][0] = None
    g = guard(scan)
    assert limit(g)['reason'] == 'clear'
    result = limit(g, desired_xy_m_s=(-1., 0.))
    assert result['velocity_xy_m_s'] == (0., 0.)


def test_epoch_duplicate_and_out_of_order_disable_previous_clear_scan():
    g = guard()
    assert g.observe(frame()) == 'scan_duplicate_or_out_of_order'
    assert limit(g)['velocity_xy_m_s'] == (0., 0.)
    assert g.observe(frame(sequence=2, stamp_ns=1_020_000_000)) == 'ready'
    assert g.observe(frame(sequence=3, stamp_ns=1_030_000_000, epoch='new')) == 'scan_source_or_epoch_changed'
    assert limit(g)['velocity_xy_m_s'] == (0., 0.)


def test_stale_future_and_unconfirmed_input_never_allow_motion():
    g = guard()
    assert limit(g, now_ns=1_250_000_001)['reason'] == 'scan_stale_or_future'
    assert limit(g, now_ns=999_999_999)['reason'] == 'scan_stale_or_future'
    for flag in ('mount_confirmed', 'horizontal_plane_confirmed', 'oldest_stamp_confirmed'):
        assert g.observe(frame(**{flag: False})) == 'mount_plane_or_acquisition_time_unverified'
        assert limit(g)['velocity_xy_m_s'] == (0., 0.)


def test_mount_rotation_and_translation_are_applied_before_query():
    g = HorizontalGuard(CFG, source_id='lidar', clock_id='clock', epoch='boot')
    value = frame(box_scan(front=.65, yaw=math.pi/2),
                  sensor_yaw_rad=math.pi/2, sensor_origin_xy_m=(2., 1.))
    assert g.observe(value) == 'ready'
    result = limit(g, current_xy_m=(2., 1.))
    assert result['reason'] == 'limited'
    assert 0 < result['velocity_xy_m_s'][0] < 1.


def test_robot_edge_obstacle_is_detected_by_swept_footprint():
    base = frame()
    # Not on the centre ray, but the propeller footprint clips it.
    obstacle = replace(base, points_xy_m=base.points_xy_m + ((.5, .19),))
    g = HorizontalGuard(CFG, source_id='lidar', clock_id='clock', epoch='boot')
    g.observe(obstacle)
    assert limit(g)['reason'] == 'limited'


def test_larger_latency_uncertainty_and_obstacle_motion_reduce_speed():
    scan = box_scan(front=1.2)
    first = limit(guard(scan))['velocity_xy_m_s'][0]
    conservative = replace(CFG, command_latency_s=.3, pose_error_m=.08,
                           obstacle_speed_bound_m_s=.25)
    second = limit(guard(scan, conservative))['velocity_xy_m_s'][0]
    assert 0 < second < first


def test_braking_inverse_over_speed_age_and_moving_obstacle_grid():
    for other in (0., .1, .5):
        config = replace(CFG, obstacle_speed_bound_m_s=other)
        for age in (0., .1, .25):
            for speed in (.01, .1, .5, 1., 1.9):
                distance = stopping_distance(speed, config, age)
                assert speed_for_clearance(distance, config, age) == pytest.approx(speed)


def test_current_position_is_used_without_rebasing_old_obstacle_points():
    g = guard(box_scan(front=1.5))
    before = limit(g)['velocity_xy_m_s'][0]
    after = limit(g, current_xy_m=(1., 0.))['velocity_xy_m_s'][0]
    assert after < before
