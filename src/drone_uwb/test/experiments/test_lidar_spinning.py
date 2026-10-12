import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from drone_uwb.processing.experiments.lidar_timing import ClockAlignment, ImuTimeline, TimingError
from drone_uwb.processing.experiments.lidar_spinning import PoseTimeline, SpinningLidar, SpinningScanOdometry


def test_clock_drift_asymmetric_delay_and_stale_map():
    clock = ClockAlignment()
    sensor = SpinningLidar(42)
    for time in np.arange(0, 1.01, .02):
        clock.add_exchange(*sensor.exchange(time))
    observed, bound, scale = clock.convert(sensor.local_time(.8), now_local_s=sensor.local_time(1.001))
    assert abs(observed-.8)<bound<.002
    assert abs(scale-sensor.scale)<.0004
    with pytest.raises(TimingError, match='clock_stale'):
        clock.convert(3., now_local_s=3.)
    clock.add_exchange(2., 2.09, 2.001)
    assert clock.epoch==1
    with pytest.raises(TimingError, match='clock_warmup'):
        clock.convert(2., now_local_s=2.)


def test_capture_time_gyro_integration_and_gap_rejection():
    imu = ImuTimeline()
    imu.add_attitude(0., np.eye(3))
    for time in np.arange(0., .121, .005):
        imu.add_gyro(time, [0, 0, .7])
    rotations, _ = imu.rotations(np.linspace(.001, .1, 430))
    assert Rotation.from_matrix(rotations[-1]).as_rotvec()[2]==pytest.approx(.07, abs=1e-8)
    assert not imu.add_gyro(.1, [0, 0, .7])
    broken = ImuTimeline()
    broken.add_attitude(0., np.eye(3))
    for time in [0., .005, .09, .12]:
        broken.add_gyro(time, [0, 0, .7])
    with pytest.raises(TimingError, match='imu_gap'):
        broken.rotations(np.array([.001, .1]))


def run_motion(deskew, missing=None):
    timeline = PoseTimeline()
    tracker = SpinningScanOdometry(deskew=deskew, simulation_covariance_model=True)
    sensor = SpinningLidar(12, range_noise_m=.001, scan_bias_noise_m=0.)
    for time in np.arange(0., 2.001, .005):
        yaw = .4*np.sin(2*time)+.25*time
        rate = .8*np.cos(2*time)+.25
        rot = Rotation.from_euler('z', yaw).as_matrix()
        timeline.add(time, [2+.2*time, 2+.08*time, 1.7], rot, [.2, .08, 0.])
        tracker.imu.add_gyro(time, [0., 0., rate])
        tracker.imu.add_attitude(time, rot)
    rows = []
    for first in np.arange(.3, 1.81, .1):
        for time in np.arange(max(0., first-.3), first+.101, .02):
            # Feed each exchange once, as a live transport would.
            exchange = sensor.exchange(time)
            if not tracker.clock.exchanges or (exchange[0]+exchange[2])/2>tracker.clock.exchanges[-1][0]:
                tracker.clock.add_exchange(*exchange)
        scan = sensor.observe(timeline, first_ray_fc_s=first, missing_sector=missing)
        rows.append(tracker.observe_scan(scan, now_local_s=sensor.local_time(first+.101), height_above_floor_m=1.7))
    accepted = [r for r in rows if r['accepted']]
    errors = np.array([r['velocity_ne_m_s'] for r in accepted])-[.08, .2]
    return rows, np.sqrt(np.mean(np.sum(errors**2, axis=1))) if len(errors) else np.inf


def test_spinning_motion_deskew_improves_velocity_without_truth_input():
    corrected, error = run_motion(True)
    uncorrected, baseline = run_motion(False)
    assert sum(r['accepted'] for r in corrected)>=12
    assert error<.018
    assert error<baseline*.7
    assert all('truth' not in ' '.join(r.get('source_lineage', [])) for r in corrected)


def test_partial_occlusion_keeps_observable_scans_and_interval_midpoint():
    rows, error = run_motion(True, missing=(-np.pi/2, -.15))
    accepted = [r for r in rows if r['accepted']]
    assert len(accepted)>=10
    assert error<.025
    assert accepted[-1]['match']['translation_information_ratio']>=.06
    timing = accepted[-1]['timing']
    assert timing['last_ray_fc_s']-timing['first_ray_fc_s']==pytest.approx(.1*429/430, abs=.0001)
    assert timing['reference_fc_s']-accepted[-1]['sample_midpoint_s']==pytest.approx(.05, abs=.001)


def test_invalid_beam_time_is_withheld_not_relabelled_as_arrival():
    from drone_uwb.processing.experiments.lidar_timing import TimedScan
    tracker = SpinningScanOdometry(simulation_covariance_model=True)
    scan = TimedScan(np.ones(430), 1., 0., .1, -np.pi, 2*np.pi/430)
    result = tracker.observe_scan(scan, now_local_s=2., height_above_floor_m=1.7)
    assert not result['accepted'] and result['reasons']==['invalid_beam_timing']
    assert 'velocity_ne_m_s' not in result


def test_common_range_offset_is_not_misreported_as_vehicle_translation():
    from drone_uwb.processing.experiments.lidar_sitl_observation import VirtualLidar
    from drone_uwb.processing.experiments.lidar_scanmatch import Config, match, scan_cloud
    sensor=VirtualLidar(1,range_noise_m=0.,scan_bias_noise_m=0.)
    ranges=sensor.observe(position_enu_m=[2.,2.,1.7],rotation_enu_flu=np.eye(3))
    config=Config(estimate_range_bias=True,spatial_support=True,normal_half_window=4)
    first=scan_cloud(ranges,-np.pi,2*np.pi/430,.2,4.,config)
    shifted=scan_cloud(ranges+.015,-np.pi,2*np.pi/430,.2,4.,config)
    result=match(first,shifted,config=config)
    assert result.accepted
    assert np.linalg.norm(result.pose[:2])<.0003
    assert result.range_bias_m==pytest.approx(-.015,abs=.0003)


def test_parallel_surfaces_do_not_pass_directional_observability_gate():
    from drone_uwb.processing.experiments.lidar_scanmatch import Cloud,Config,match
    x=np.linspace(-2.,2.,90)
    points=np.r_[np.c_[x,np.ones(len(x))],np.c_[x,-np.ones(len(x))]]
    normals=np.tile([0.,1.],(len(points),1))
    cloud=Cloud(points,normals)
    result=match(cloud,cloud,config=Config(spatial_support=True,estimate_range_bias=True))
    assert not result.accepted and 'weak_translation_direction' in result.reasons
