import math

import numpy as np
import pytest

from drone_uwb.integration.lidar_contract import (
    SingleUseObservation, TimingProvenance, audit_scan, beam_timestamps,
    horizontal_lever_arm, interval_velocity,
)


def scan(count=401, increment=.00025):
    return dict(angle_min=-math.pi, angle_max=math.pi,
                angle_increment=2*math.pi/(count-1), ranges=[2.] * count,
                range_min=.03, range_max=12., scan_time=.1,
                time_increment=increment)


VERIFIED = TimingProvenance('synthetic-clock', True, True, True,
                            'fixture generated in acquisition order')


def audit(value, provenance=VERIFIED, **kw):
    args = dict(header_ns=1_000_000_000, received_ns=1_110_000_000,
                now_ns=1_110_000_000, provenance=provenance)
    args.update(kw)
    return audit_scan(value, **args)


def test_fixed_resolution_count_mismatch_preserves_geometry_but_rejects_deskew():
    # Same 4000 Hz raw point interval, but 430 angular bins after resampling.
    result = audit(scan(430))
    assert result.geometry_valid and result.fresh
    assert result.span_period_ratio == pytest.approx(1.0725)
    assert not result.declared_timing_consistent and not result.deskew_ready


def test_numerically_consistent_bins_do_not_prove_acquisition_order():
    result = audit(scan(), TimingProvenance(clock_id='ros', evidence='angular bins'))
    assert result.geometry_valid and result.declared_timing_consistent
    assert not result.deskew_ready
    with pytest.raises(ValueError, match='acquisition_order'):
        beam_timestamps(scan(), header_ns=1_000_000_000,
                        received_ns=1_110_000_000,
                        provenance=TimingProvenance())


def test_verified_ray_times_and_future_last_ray():
    values = beam_timestamps(scan(), header_ns=1_000_000_000,
                             received_ns=1_110_000_000, provenance=VERIFIED)
    assert values[0] == 1_000_000_000 and values[-1] == 1_100_000_000
    assert np.diff(values).tolist() == [250_000] * 400
    assert not audit(scan(), received_ns=1_050_000_000).deskew_ready


@pytest.mark.parametrize('changes', [dict(time_increment=0.), dict(scan_time=0.),
                                    dict(time_increment=math.nan)])
def test_missing_timing_is_not_invented_from_frequency(changes):
    value = scan(); value.update(changes)
    result = audit(value)
    assert result.geometry_valid and not result.deskew_ready


def test_clock_order_and_staleness_rejected():
    assert not audit(scan(), now_ns=1_300_000_001).fresh
    assert not audit(scan(), header_ns=1_200_000_000).fresh
    assert not audit(scan(), now_ns=1_100_000_000).fresh


def test_nonfinite_ranges_are_unknown_and_geometry_failure_is_separate():
    value = scan(); value['ranges'] = ['NaN', 'Infinity', '-Infinity', None] * 100 + [2.]
    result = audit(value)
    assert result.valid_returns == 1 and not result.geometry_valid
    assert result.declared_timing_consistent


def test_interval_average_has_midpoint_time_two_axes_and_explicit_uncertainty():
    result = interval_velocity(displacement_xy_m=(.1, -.2),
        displacement_covariance_m2=((.01, .002), (.002, .02)),
        start_ns=1_000_000_000, end_ns=1_200_000_000,
        additional_covariance_m2_s2=((.03, 0.), (0., .04)),
        frame_id='fixed_scan_reference', reference_point='lidar',
        source_lineage=('lidar:scan0', 'lidar:scan1', 'imu:rotation'))
    assert result['representative_stamp_ns'] == 1_100_000_000
    assert result['velocity_xy_m_s'] == pytest.approx((.5, -1.))
    assert np.asarray(result['covariance_xy_m2_s2']) == pytest.approx(np.array([[.28, .05], [.05, .54]]))
    assert not result['z_observed'] and not result['px4_aiding_ready']
    assert 'vz' not in result and result['source_lineage'][-1] == 'imu:rotation'


def test_pitch_about_stationary_antenna_removes_lidar_lever_arm_motion():
    omega = math.radians(10)
    # FRD: LiDAR is 0.1 m below the antenna, positive pitch gyro about +Y.
    velocity, cov = horizontal_lever_arm(lidar_velocity_body_xy_m_s=(omega*.1, 0.),
        omega_body_rad_s=(0., omega, 0.), uwb_to_lidar_body_m=(0., 0., .1),
        joint_covariance=np.diag([.01, .02, 0., 0., 0., 0., 0., 0.]))
    assert velocity == pytest.approx((0., 0.))
    assert cov == pytest.approx(np.diag([.01, .02]))


def test_lever_arm_preserves_correlated_input_uncertainty():
    # All measured vLx uncertainty originates from the SAME uncertain pitch.
    joint = np.zeros((8, 8)); joint[0, 0] = .0001; joint[3, 3] = .01
    joint[0, 3] = joint[3, 0] = .001
    _, c = horizontal_lever_arm(lidar_velocity_body_xy_m_s=(0., 0.),
        omega_body_rad_s=(0., 0., 0.), uwb_to_lidar_body_m=(0., 0., .1),
        joint_covariance=joint)
    assert c[0, 0] == pytest.approx(0., abs=1e-15)
    # This zero is an analytic synthetic case, never a default sensor variance.


def test_freshness_accepts_only_once_and_epoch_change_needs_explicit_reset():
    gate = SingleUseObservation('lidar', 'clock', 'boot1')
    args = dict(source_id='lidar', clock_id='clock', epoch='boot1',
                stamp_ns=1_000_000_000, now_ns=1_050_000_000, max_age_s=.2)
    assert gate.accept(**args) == 'accepted'
    assert gate.accept(**args) == 'duplicate_or_out_of_order'
    assert gate.accept(**dict(args, stamp_ns=999_000_000)) == 'duplicate_or_out_of_order'
    assert gate.accept(**dict(args, epoch='boot2')) == 'epoch_or_source_changed'
    assert gate.accept(**dict(args, now_ns=1_300_000_000)) == 'observation_stale_or_future'
