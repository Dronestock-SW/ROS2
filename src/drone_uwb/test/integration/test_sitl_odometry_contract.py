"""Keep SITL position, goal and MAVLink measurement references consistent."""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from drone_uwb.integration.sitl.sitl_odometry_contract import (
    SITLOdometrySettings, map_reference_xy_to_ned, map_tag_to_px4_ned,
    ned_xy_to_map_reference, odometry_fields, send_odometry_fields, transmission_gate,
)


PARAMS = dict(EKF2_EV_CTRL=1, EKF2_EV_NOISE_MD=0, EKF2_EV_DELAY=40.,
              EKF2_EV_POS_X=0., EKF2_EV_POS_Y=0., EKF2_EV_POS_Z=0.)


def confirmed():
    return SITLOdometrySettings(enabled=True, alignment_confirmed=True,
                                mount_confirmed=True, height_confirmed=True,
                                time_mapping_confirmed=True, px4_reference_confirmed=True,
                                enu_yaw_deg=90., enu_offset_x_m=2., enu_offset_y_m=-1.,
                                px4_origin_height_world_m=.3,
                                tag_offset_body_flu_m=(0., 0., .3),
                                px4_reference_offset_body_flu_m=(.14, 0., 0.),
                                mount_sigma_xy_m=.02, expected_ev_delay_ms=40.)


def test_mount_frame_goal_round_trip_and_covariance():
    settings = confirmed()
    # 90-degree body yaw moves the FC's 14 cm forward offset along world +y.
    yaw_quat = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
    mapped = map_tag_to_px4_ned([1., 2., 1.3], yaw_quat,
                                [[.0009, .0002], [.0002, .0016]], settings)
    reference = mapped['reference_world_xyz_m']
    np.testing.assert_allclose(reference, [1., 2.14, 1.], atol=1e-12)
    np.testing.assert_allclose(mapped['position_ned_m'], [0., -.14, -.7], atol=1e-12)
    np.testing.assert_allclose(ned_xy_to_map_reference(mapped['position_ned_m'][:2], settings),
                               reference[:2], atol=1e-12)
    np.testing.assert_allclose(map_reference_xy_to_ned(reference[:2], settings),
                               mapped['position_ned_m'][:2], atol=1e-12)
    covariance = np.asarray(mapped['covariance_ned_xy_m2'])
    assert np.trace(covariance) == pytest.approx(.0009+.0016+2*.02**2)
    assert covariance[0, 1] == pytest.approx(-.0002)


def test_packet_requires_live_horizontal_only_fc_contract_and_preserves_unknowns():
    settings = confirmed()
    candidate = map_tag_to_px4_ned([1., 2., 1.3], [1., 0., 0., 0.],
                                   [[.01, 0.], [0., .02]], settings)
    assert transmission_gate(SITLOdometrySettings(), PARAMS) == 'disabled'
    assert transmission_gate(settings, PARAMS) == 'fc_parameters_unavailable_or_stale'
    assert transmission_gate(settings, PARAMS, fc_connected=True, param_age_s=.1,
                             sample_age_s=.2) == 'sensor_sample_unavailable_or_stale'
    with pytest.raises(ValueError, match='fc_parameters_unavailable_or_stale'):
        odometry_fields(candidate, 1_000_000, settings, PARAMS,
                        sample_clock_domain='wsl_monotonic_us')
    wrong = dict(PARAMS, EKF2_EV_CTRL=3)
    with pytest.raises(ValueError, match='require_horizontal_only_ev_ctrl_1'):
        odometry_fields(candidate, 1_000_000, settings, wrong,
                        sample_clock_domain='wsl_monotonic_us',
                        fc_connected=True, param_age_s=.1, sample_age_s=.01)
    wrong_mount = dict(PARAMS, EKF2_EV_POS_X=.14)
    with pytest.raises(ValueError, match='require_zero_ev_sensor_offset'):
        odometry_fields(candidate, 1_000_000, settings, wrong_mount,
                        sample_clock_domain='wsl_monotonic_us',
                        fc_connected=True, param_age_s=.1, sample_age_s=.01)
    fields = odometry_fields(candidate, 1_000_000, settings, PARAMS,
                             sample_clock_domain='wsl_monotonic_us',
                             fc_connected=True, param_age_s=.1, sample_age_s=.01)
    assert fields['message'] == 'ODOMETRY' and fields['time_usec'] == 1_000_000
    assert fields['frame_id'] == 'MAV_FRAME_LOCAL_NED'
    assert all(math.isnan(v) for v in fields['q'])
    assert all(math.isnan(fields[key]) for key in ('vx', 'vy', 'vz'))
    assert fields['pose_covariance'][11] == 1e6
    assert fields['sender_handoff_ready'] is True and fields['transmitted'] is False


def test_unconfirmed_origin_and_missing_height_are_not_packaged():
    settings = replace(confirmed(), px4_reference_confirmed=False)
    assert transmission_gate(settings, PARAMS, fc_connected=True, param_age_s=.1,
                             sample_age_s=.01) == 'px4_reference_confirmed_required'
    with pytest.raises(ValueError, match='finite_tag_position_required'):
        map_tag_to_px4_ned([1., 2., float('nan')], [1., 0., 0., 0.],
                           [[.01, 0.], [0., .01]], settings)


def test_mavlink_adapter_refuses_repeat_and_preserves_unknown_orientation():
    class Recorder:
        def __init__(self):
            self.calls = []

        def odometry_send(self, *args):
            self.calls.append(args)

    settings = confirmed()
    candidate = map_tag_to_px4_ned([1., 2., 1.3], [1., 0., 0., 0.],
                                   [[.01, 0.], [0., .01]], settings)
    fields = odometry_fields(candidate, 1_000_000, settings, PARAMS,
                             sample_clock_domain='wsl_monotonic_us',
                             fc_connected=True, param_age_s=.1, sample_age_s=.01)
    enums = SimpleNamespace(MAV_FRAME_LOCAL_NED=1, MAV_FRAME_BODY_FRD=12,
                            MAV_ESTIMATOR_TYPE_VISION=4)
    sender = Recorder()
    last = send_odometry_fields(sender, enums, fields)
    assert last == 1_000_000 and len(sender.calls) == 1
    assert sender.calls[0][0:3] == (1_000_000, 1, 12)
    assert all(math.isnan(value) for value in sender.calls[0][6])
    with pytest.raises(ValueError, match='duplicate_or_old'):
        send_odometry_fields(sender, enums, fields, last)
    assert len(sender.calls) == 1


def test_packet_uses_sender_clock_and_refuses_already_mapped_px4_stamp():
    settings = confirmed()
    candidate = map_tag_to_px4_ned([1., 2., 1.3], [1., 0., 0., 0.],
                                   [[.01, 0.], [0., .02]], settings)
    sender_measurement_us = 7_999_980_000
    fields = odometry_fields(candidate, sender_measurement_us, settings, PARAMS,
                             sample_clock_domain='wsl_monotonic_us',
                             fc_connected=True, param_age_s=.1, sample_age_s=.02)
    assert fields['time_usec'] == sender_measurement_us
    assert fields['time_clock_domain'] == 'wsl_monotonic_us'
    # PX4 Timesync::sync_stamp adds its boot-minus-remote offset once.
    px4_boot_minus_sender_us = -7_970_000_000
    assert fields['time_usec']+px4_boot_minus_sender_us == 29_980_000
    with pytest.raises(ValueError, match='sender_measurement_clock_mismatch'):
        odometry_fields(candidate, 29_980_000, settings, PARAMS,
                        sample_clock_domain='px4_boot_us', fc_connected=True,
                        param_age_s=.1, sample_age_s=.02)
    with pytest.raises(ValueError, match='sender_measurement_clock_mismatch'):
        send_odometry_fields(None, None, dict(fields, time_clock_domain='px4_boot_us'))


def test_gazebo_clock_packet_preserves_measurement_stamp_and_cannot_mix_reply_clock():
    settings = replace(confirmed(), sender_clock_domain='gazebo_sim_us')
    candidate = map_tag_to_px4_ned([1., 2., 1.3], [1., 0., 0., 0.],
                                   [[.01, 0.], [0., .02]], settings)
    fields = odometry_fields(candidate, 29_980_000, settings, PARAMS,
                             sample_clock_domain='gazebo_sim_us', fc_connected=True,
                             param_age_s=.1, sample_age_s=.02)
    assert fields['time_usec'] == 29_980_000
    assert fields['time_clock_domain'] == fields['timesync_clock_domain'] == 'gazebo_sim_us'
    with pytest.raises(ValueError, match='sender_measurement_clock_mismatch'):
        send_odometry_fields(None, None, dict(fields, timesync_clock_domain='wsl_monotonic_us'))
