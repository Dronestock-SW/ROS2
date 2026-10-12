"""Independent rigid transforms, ray-cast motion, and data-gap contracts."""
import numpy as np
import pytest

from drone_uwb.processing.experiments.lidar_scanmatch import (
    Cloud, KeyframeMatcher, compose, inverse, match, rotation, uwb_velocity_from_lidar)
from drone_uwb.processing.experiments.lidar_synthetic import CORRIDOR, fixture_cloud


@pytest.mark.parametrize('pose', [[.08, -.06, .04], [-.07, .1, -.08], [0., 0., 0.]])
def test_rigid_transform_direction_and_additive_jacobian(pose):
    target = fixture_cloud(sigma_m=0.)
    # Point coordinates generated algebraically, not by the matcher helpers.
    angle = pose[2]
    r = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    source = Cloud((target.points - pose[:2]) @ r, target.normals @ r)
    result = match(target, source)
    assert result.accepted, result.reasons
    np.testing.assert_allclose(result.pose, pose, atol=1e-6)
    reverse = match(source, target)
    assert reverse.accepted
    np.testing.assert_allclose(compose(result.pose, reverse.pose), 0., atol=1e-6)


def test_ray_cast_translation_and_rotation_are_not_frozen():
    tracker = KeyframeMatcher()
    for index in range(81):
        pose = np.array([.01 * index, .004 * index, .003 * index])
        result = tracker.observe(fixture_cloud(pose, seed=index, sigma_m=.001), index * .1, imu_yaw_rad=pose[2])
        if index:
            assert result.accepted, (index, result.reasons)
            np.testing.assert_allclose(result.pose, pose, atol=.007)
    assert tracker.keyframe >= 2
    assert result.velocity_scan_midpoint_mps is not None
    assert result.sample_midpoint_s == pytest.approx(7.95)


def test_parallel_walls_reject_unobservable_motion():
    wall = fixture_cloud(segments=CORRIDOR, sigma_m=0.)
    result = match(wall, wall)
    assert not result.accepted
    assert 'degenerate_geometry' in result.reasons


def test_imu_disagreement_and_missing_imu_withhold_velocity():
    tracker = KeyframeMatcher()
    tracker.observe(fixture_cloud(), 1., imu_yaw_rad=0.)
    result = tracker.observe(fixture_cloud((0., 0., .05)), 1.1, imu_yaw_rad=0.)
    assert not result.accepted
    assert 'imu_rotation_inconsistent' in result.reasons
    assert result.velocity_scan_midpoint_mps is None
    result = tracker.observe(fixture_cloud(), 1.2)
    assert result.reasons == ['imu_unavailable']
    result = tracker.observe(fixture_cloud(), 1.3, imu_yaw_rad=0.)
    assert not result.accepted and result.reasons == ['reacquired_pose_only']
    result = tracker.observe(fixture_cloud(), 1.4, imu_yaw_rad=0.)
    assert result.accepted


def test_duplicate_reverse_time_and_long_gap_never_generate_stale_velocity():
    tracker = KeyframeMatcher()
    tracker.observe(fixture_cloud(), 1., imu_yaw_rad=0.)
    for time_s in (1., .9):
        result = tracker.observe(fixture_cloud(), time_s, imu_yaw_rad=0.)
        assert result.reasons == ['nonmonotonic_time']
        assert result.delta is None
    result = tracker.observe(fixture_cloud((.8, 0., 0.)), 2., imu_yaw_rad=0.)
    assert result.reasons == ['new_segment']
    assert result.segment == 1
    assert result.velocity_scan_midpoint_mps is None


@pytest.mark.parametrize('height', [.05, .10, .15])
def test_mount_sign_and_separate_reference_point(height):
    omega = np.array([0., np.deg2rad(10.), 0.])
    v = uwb_velocity_from_lidar([0., 0., 0.], omega, [0., 0., height])
    np.testing.assert_allclose(v, [-np.deg2rad(10.) * height, 0., 0.])
    # Pure yaw and a purely vertical lever arm do not change XY velocity.
    np.testing.assert_allclose(uwb_velocity_from_lidar([1., 2., 0.], [0., 0., 1.], [0., 0., height]), [1., 2., 0.])


def test_invalid_cloud_and_time_rejected():
    with pytest.raises(ValueError):
        Cloud([[np.nan, 0.]], [[1., 0.]])
    with pytest.raises(ValueError):
        KeyframeMatcher().observe(fixture_cloud(), float('nan'))


def test_transform_composition():
    a = np.array([.3, -.7, .4])
    b = np.array([-.2, .4, -.6])
    point = np.array([1.3, -.8])
    composed = compose(a, b)
    np.testing.assert_allclose(rotation(composed[2]) @ point + composed[:2],
                               rotation(a[2]) @ (rotation(b[2]) @ point + b[:2]) + a[:2])
    np.testing.assert_allclose(compose(a, inverse(a)), 0., atol=1e-12)
