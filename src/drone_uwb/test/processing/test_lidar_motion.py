"""Physical adapter contracts: measured timing, mounting, axes and continuity."""
from dataclasses import replace
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from drone_uwb.processing.lidar_motion import LidarMotionSettings, MeasuredLidarMotion, sdk_points
from drone_uwb.integration.lidar_velocity import (parameter_gate, velocity_fields, VelocityReadinessWindow,
    ground_test_gate, validate_ground_test_duration)


def settings(**kwargs):
    return replace(LidarMotionSettings(enabled=True, sensor_to_body_flu_m=(0.,0.,.10),
        sensor_to_body_yaw_rad=0., mount_position_ref='synthetic-fixture',
        mount_yaw_ref='synthetic-fixture', scan_plane_parallel_confirmed=True,
        timing_ref='synthetic-capture-clock'), **kwargs)


def fixture(motion, start, *, velocity=(.12,-.08), yaw_rate=.12, roll_rate=0.):
    count = 430
    times = start+np.arange(count)*.1/count
    angles = -math.pi+np.arange(count)*2*math.pi/count
    yaw_mount = motion.settings.sensor_to_body_yaw_rad
    sensor = np.column_stack((np.cos(angles+yaw_mount),np.sin(angles+yaw_mount),np.zeros(count)))
    def attitude(t):
        return Rotation.from_euler('z',yaw_rate*t).as_matrix()@Rotation.from_euler('x',roll_rate*t).as_matrix()
    rotations = np.array([attitude(t) for t in times])
    directions = np.einsum('nij,nj->ni',rotations,sensor)
    position = np.tile([2.3,2.6,1.],(count,1))
    position[:,:2] += (times-1.)[:,None]*np.asarray(velocity)
    position += np.einsum('nij,j->ni',rotations,np.asarray(motion.settings.sensor_to_body_flu_m))
    distances = np.full((count,2),np.inf)
    np.divide(5.-position[:,:2],directions[:,:2],out=distances,where=directions[:,:2]>1e-10)
    negative = np.full_like(distances,np.inf)
    np.divide(-position[:,:2],directions[:,:2],out=negative,where=directions[:,:2]<-1e-10)
    ranges = np.minimum(distances,negative).min(axis=1)
    points = np.column_stack((ranges*np.cos(angles),ranges*np.sin(angles),np.zeros(count)))
    # Sampled, capture-time IMU stream; no vehicle translation enters estimator.
    for t in np.arange(start-.03,start+.121,.005):
        motion.imu.add_attitude(float(t),attitude(t))
        body_gyro = Rotation.from_euler('x',-roll_rate*t).apply([0.,0.,yaw_rate])+[roll_rate,0.,0.]
        motion.imu.add_gyro(float(t),body_gyro)
    return dict(points=points,offsets=times-start,first_stamp_ns=round(start*1e9),
        now_ns=round((start+.12)*1e9),receipt_age_s=.02,frame_id='laser_frame',
        floor_range_m=1.,floor_stamp_ns=round((start+.115)*1e9))


def test_unknown_mount_and_yaw_cannot_estimate_or_publish():
    default = LidarMotionSettings()
    assert default.geometry_gate() == 'mount_position_unconfirmed'
    assert replace(settings(),sensor_to_body_yaw_rad=None).geometry_gate() == 'mount_yaw_unconfirmed'
    assert replace(settings(),timing_ref='').geometry_gate() == 'acquisition_timing_unconfirmed'
    assert replace(settings(),scan_plane_parallel_confirmed=False).geometry_gate() == 'scan_plane_unconfirmed'


@pytest.mark.parametrize('mount_yaw', [0., math.pi/2, -math.pi/2])
def test_known_translation_rotating_scans_and_nonzero_sensor_yaw(mount_yaw):
    model = MeasuredLidarMotion(settings(sensor_to_body_yaw_rad=mount_yaw))
    accepted = []
    for i in range(9):
        row = model.observe(**fixture(model,1.+i*.1))
        if row['accepted']:
            accepted.append(row)
    assert len(accepted) >= 6
    velocities = np.array([r['velocity_ne_m_s'] for r in accepted])
    assert np.max(np.linalg.norm(velocities-[-.08,.12],axis=1)) < .025
    assert all(r['z_observed'] is False and r['reference_point']=='fc_imu' for r in accepted)
    assert all(r['interval_start_ns'] < r['representative_stamp_ns'] < r['interval_end_ns'] for r in accepted)


def test_pure_body_rotation_with_lever_arm_does_not_create_translation():
    model = MeasuredLidarMotion(settings(sensor_to_body_flu_m=(.15,-.12,.10)))
    rows = [model.observe(**fixture(model,1.+i*.1,velocity=(0.,0.),yaw_rate=.15)) for i in range(9)]
    accepted = [r for r in rows if r['accepted']]
    assert len(accepted) >= 6
    assert max(np.linalg.norm(r['velocity_ne_m_s']) for r in accepted) < .025


def test_below_or_above_mount_during_roll_uses_imu_origin():
    for height in (-.15,.10):
        model = MeasuredLidarMotion(settings(sensor_to_body_flu_m=(0.,0.,height)))
        rows = [model.observe(**fixture(model,1.+i*.1,velocity=(0.,0.),yaw_rate=0.,roll_rate=.04)) for i in range(7)]
        accepted = [r for r in rows if r['accepted']]
        assert len(accepted) >= 4
        assert max(np.linalg.norm(r['velocity_ne_m_s']) for r in accepted) < .02


def test_duplicate_stale_gap_and_missing_imu_withhold_velocity():
    model = MeasuredLidarMotion(settings())
    first = fixture(model,1.)
    model.observe(**first)
    assert model.observe(**first)['reasons'] == ['lidar_capture_time_nonmonotonic']
    second = fixture(model,1.1)
    assert not model.observe(**dict(second,now_ns=2_000_000_000))['accepted']
    model.imu.gyros.clear()
    assert model.observe(**second)['reasons'] == ['imu_not_bracketed']
    model.reset_clock()
    rows = [model.observe(**fixture(model,2.+i*.1)) for i in range(4)]
    assert not rows[0]['accepted']
    assert rows[-1]['accepted']


def test_sdk_padding_removed_without_reinventing_time():
    points = np.column_stack((np.arange(42),np.ones(42),np.zeros(42)))
    points[-2:] = points[-3]
    offsets = np.arange(42)*.002
    p,t,dropped = sdk_points(points,offsets)
    assert len(p)==40 and dropped==2
    assert t[-1]==offsets[-3]
    with pytest.raises(ValueError,match='sdk_times_invalid'):
        sdk_points(points,offsets[::-1])
    with pytest.raises(ValueError,match='sdk_points_or_times_shape'):
        sdk_points(points,[])


def test_wire_ne_covariance_and_nan_vertical_without_mode_switch_dependency():
    model = MeasuredLidarMotion(settings())
    rows = [model.observe(**fixture(model,1.+i*.1)) for i in range(5)]
    row = rows[-1]
    fields = velocity_fields(row,now_ns=row['representative_stamp_ns']+120_000_000,
        last_stamp_ns=None,max_age_s=.18)
    assert math.isnan(fields['vz']) and math.isnan(fields['z'])
    assert all(math.isnan(v) for v in fields['q'])
    assert fields['velocity_covariance'][0] > 0 and fields['velocity_covariance'][6] > 0
    assert fields['time_usec']==row['representative_stamp_ns']//1000
    with pytest.raises(ValueError,match='duplicate'):
        velocity_fields(row,now_ns=row['representative_stamp_ns'],last_stamp_ns=row['representative_stamp_ns'],max_age_s=.18)
    params = dict(EKF2_LID2_VER=1,EKF2_LID2_EN=1,EKF2_LID2_AGE=180.,EKF2_AVEL_DELAY=5.)
    assert parameter_gate(settings(),params)=='ready'
    assert parameter_gate(settings(),dict(params,EKF2_LID2_VER=None))=='dedicated_lidar2d_firmware_required'
    assert parameter_gate(settings(),dict(params,EKF2_LID2_EN=0))=='px4_lidar2d_not_enabled'
    assert parameter_gate(settings(),dict(params,EKF2_LID2_AGE=150.))=='px4_lidar2d_age_mismatch'
    independent = settings(floor_policy='metric_scan',vertical_structure_ref='synthetic-walls')
    assert parameter_gate(independent,params)=='independent_lidar2d_firmware_required'
    assert parameter_gate(independent,dict(params,EKF2_LID2_VER=2))=='ready'


@pytest.mark.parametrize('floor,stamp', [(math.nan,0),(.02,0),(1.,1),(-1.,1)])
def test_metric_scan_survives_unobserved_floor_without_inventing_height(floor,stamp):
    cfg = settings(floor_policy='metric_scan',vertical_structure_ref='synthetic-vertical-walls')
    model = MeasuredLidarMotion(cfg)
    rows = [model.observe(**dict(fixture(model,1.+i*.1),floor_range_m=floor,floor_stamp_ns=stamp)) for i in range(9)]
    accepted = [r for r in rows if r['accepted']]
    assert len(accepted) >= 6
    assert all(r['sensor_height_m'] is None and r['floor_range_valid'] is False for r in rows)
    assert max(np.linalg.norm(np.asarray(r['velocity_ne_m_s'])-[-.08,.12]) for r in accepted) < .025
    assert all(r['height_used_for_xy_scale'] is False and r['z_observed'] is False for r in rows)


def test_metric_scan_tof_loss_and_recovery_do_not_reset_horizontal_continuity():
    model = MeasuredLidarMotion(settings(floor_policy='metric_scan',vertical_structure_ref='synthetic-vertical-walls'))
    rows=[]
    for i in range(12):
        scan=fixture(model,1.+i*.1,velocity=(0.,0.),yaw_rate=0.)
        # Ascending through a wall scene: height never supplies XY scale.
        scan['floor_range_m']=.02+i*.15 if i not in (4,5,6) else math.nan
        rows.append(model.observe(**scan))
    assert all(r['accepted'] for r in rows[3:])
    assert len({r['segment'] for r in rows[3:]})==1
    assert max(np.linalg.norm(r['velocity_ne_m_s']) for r in rows[3:]) < .01
    assert rows[5]['sensor_height_m'] is None and rows[10]['sensor_height_m'] > 1.


def test_unqualified_scene_and_required_floor_still_block():
    cfg=settings(floor_policy='metric_scan')
    assert cfg.geometry_gate()=='vertical_structure_unconfirmed'
    model=MeasuredLidarMotion(settings())
    assert model.observe(**dict(fixture(model,1.),floor_range_m=math.nan))['reasons']==['measured_floor_range_unavailable']


def test_ground_bench_observes_horizontal_motion_without_claiming_floor_height():
    model=MeasuredLidarMotion(settings(floor_policy='ground_static'))
    rows=[model.observe(**dict(fixture(model,1.+i*.1,velocity=(0.,0.),yaw_rate=0.),
        floor_range_m=math.nan,floor_stamp_ns=0)) for i in range(6)]
    assert all(r['accepted'] for r in rows[3:])
    assert all(r['sensor_height_m'] is None and not r['z_observed'] for r in rows)
    assert max(np.linalg.norm(r['velocity_ne_m_s']) for r in rows[3:]) < .01


def test_ground_bench_never_latches_permission_to_continue_armed_or_airborne():
    state=dict(connected=True,armed=False,landed_state=1,state_age_s=.1,landed_age_s=.1)
    assert ground_test_gate(**state)=='ready'
    for change in (dict(armed=True),dict(connected=False),dict(landed_state=2),
                   dict(landed_state=None),dict(state_age_s=2.6),dict(landed_age_s=.51)):
        assert ground_test_gate(**{**state,**change})!='ready'
    cfg=settings(floor_policy='ground_static')
    validate_ground_test_duration(cfg,30.)
    for seconds in (0.,-1.,61.,math.nan,math.inf):
        with pytest.raises(ValueError,match='bounded_duration'):
            validate_ground_test_duration(cfg,seconds)


def test_metric_scan_keeps_tilt_and_scan_support_limits():
    cfg=settings(floor_policy='metric_scan',vertical_structure_ref='synthetic-vertical-walls')
    model=MeasuredLidarMotion(cfg)
    row=model.observe(**dict(fixture(model,1.,roll_rate=.2),floor_range_m=math.nan))
    assert row['reasons']==['scan_tilt_limit']
    model=MeasuredLidarMotion(cfg)
    for i in range(5):model.observe(**fixture(model,1.+i*.1))
    scan=fixture(model,1.5)
    # Vertical plane transition into an unobservable scene must withhold XY.
    scan['points'][:]=0.
    row=model.observe(**dict(scan,floor_range_m=math.nan))
    assert not row['accepted']


def test_readiness_uses_measured_intervals_without_smoothing_fc_input():
    window=VelocityReadinessWindow()
    def sample(i,v,reset=0):
        return dict(interval_start_ns=i*100_000_000,interval_end_ns=(i+1)*100_000_000,
            representative_stamp_ns=i*100_000_000+50_000_000,reset_counter=reset,velocity_ne_m_s=[v,0.])
    for i in range(10):window.observe(sample(i,.2 if i%2 else -.2))
    status=window.status()
    assert status['ready'] and status['speed_m_s']<1e-8
    assert status['observed_s']==pytest.approx(1.) and not status['fc_input_filtered']
    assert window.rows[-1]['velocity_ne_m_s']==[.2,0.]
    window.observe(sample(9,0.));assert window.status()==status
    for i in range(10,21):window.observe(sample(i,.2))
    assert window.status()['ready'] and window.status()['speed_m_s']==pytest.approx(.2)
    window.observe(sample(30,.2));assert not window.status()['ready']
    for i in range(31,40):window.observe(sample(i,.2))
    window.observe(sample(40,.2,reset=1));assert window.status()['samples']==1


def test_readiness_does_not_count_missing_intervals_as_zero_motion():
    window=VelocityReadinessWindow()
    for i in (0,2,4,6,8):
        window.observe(dict(interval_start_ns=i*100_000_000,interval_end_ns=(i+1)*100_000_000,
            representative_stamp_ns=i*100_000_000+50_000_000,reset_counter=0,velocity_ne_m_s=[.2,0.]))
    status=window.status()
    assert not status['ready'] and status['observed_s']==pytest.approx(.5)
    assert status['speed_m_s']==pytest.approx(.2)
