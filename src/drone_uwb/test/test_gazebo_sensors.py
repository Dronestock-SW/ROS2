"""Gazebo ToF/IMU record contracts with representative protobuf-shaped input."""
from types import SimpleNamespace as Obj

import pytest

from drone_uwb.integration.gazebo_sensors import attitude_record, tof_record


def header(sec=5, nsec=123456000):
    return Obj(stamp=Obj(sec=sec, nsec=nsec))


def test_single_beam_range_keeps_simulation_time_and_missing_return():
    scan = Obj(header=header(), frame='tof_link', ranges=[.17155],
               range_min=.1, range_max=12.)
    row = tof_record(scan, '/tof')
    assert row['time_us'] == 5_123456
    assert row['distance_m'] == .17155 and row['valid'] is True
    assert row['clock_domain'] == 'gazebo_sim_us'
    scan.ranges = [float('inf')]
    missing = tof_record(scan, '/tof')
    assert missing['distance_m'] is None and missing['reason'] == 'no_valid_return'
    scan.ranges = [.2, .3]
    with pytest.raises(ValueError, match='unexpected_tof_scan_geometry'):
        tof_record(scan, '/tof')


def test_imu_orientation_stays_unverified_and_invalid_quaternion_is_blocked():
    imu = Obj(header=header(), entity_name='imu_sensor',
              orientation=Obj(w=1., x=0., y=0., z=0.),
              angular_velocity=Obj(x=.1, y=.2, z=.3))
    row = attitude_record(imu, '/imu')
    assert row['time_us'] == 5_123456 and row['valid'] is True
    assert row['orientation_reference'] == 'initial_imu_frame_unverified'
    assert row['quaternion_wxyz'] == [1., 0., 0., 0.]
    imu.orientation.w = 0.
    invalid = attitude_record(imu, '/imu')
    assert invalid['valid'] is False and invalid['quaternion_wxyz'] is None


def test_bad_gazebo_stamp_rejected_before_recording():
    scan = Obj(header=header(nsec=1_000_000_000), frame='tof_link', ranges=[.2],
               range_min=.1, range_max=12.)
    with pytest.raises(ValueError, match='invalid_gazebo_timestamp'):
        tof_record(scan, '/tof')
