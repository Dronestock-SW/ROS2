import math

import numpy as np
import pytest

from drone_uwb.processing.experiments.lidar_sitl_observation import (
    PlanarScanOdometry,VirtualLidar,enu_flu_rotation)


def attitude(yaw=0.,roll=0.,pitch=0.):
    cx,sx=np.cos(roll),np.sin(roll);cy,sy=np.cos(pitch),np.sin(pitch)
    cz,sz=np.cos(yaw),np.sin(yaw)
    return np.array([[cz,-sz,0.],[sz,cz,0.],[0.,0.,1.]])@np.array([
        [cy,0.,sy],[0.,1.,0.],[-sy,0.,cy]])@np.array([[1.,0.,0.],[0.,cx,-sx],[0.,sx,cx]])


def test_north_east_axes_and_no_fabricated_vertical_velocity():
    assert np.allclose(enu_flu_rotation(np.eye(3))@np.array([1.,0.,0.]),[0.,1.,0.])
    sensor=VirtualLidar(1,range_noise_m=0.,scan_bias_noise_m=0.)
    tracker=PlanarScanOdometry(simulation_covariance_model=True)
    accepted=[]
    for i in range(12):
        t=i*.1;rot=attitude(.3)
        ranges=sensor.observe(position_enu_m=[2.+.2*t,2.+.1*t,1.7],rotation_enu_flu=rot)
        result=tracker.observe(ranges=ranges,stamp_s=t,rotation_enu_flu=rot,height_above_floor_m=1.7)
        if result['accepted']:accepted.append(result)
    assert len(accepted)>=9
    assert np.allclose(accepted[-1]['velocity_ne_m_s'],[.1,.2],atol=.008)
    assert accepted[-1]['sample_midpoint_s']==pytest.approx(1.05)
    assert 'vz' not in accepted[-1]


def test_roll_pitch_projection_and_relative_yaw_with_no_truth_in_estimator():
    sensor=VirtualLidar(3,range_noise_m=.003,scan_bias_noise_m=.001)
    tracker=PlanarScanOdometry(simulation_covariance_model=True)
    observations=[]
    for i in range(22):
        t=i*.1;rot=attitude(.2+.02*t,math.radians(4)*np.sin(t),math.radians(3)*np.cos(t))
        result=tracker.observe(ranges=sensor.observe(position_enu_m=[2+.15*t,2,1.7],rotation_enu_flu=rot),
            stamp_s=t,rotation_enu_flu=rot,height_above_floor_m=1.7)
        if result['accepted']:observations.append(result)
    assert len(observations)>=18
    errors=np.array([v['velocity_ne_m_s'] for v in observations])-[0.,.15]
    assert np.sqrt(np.mean(np.sum(errors**2,axis=1)))<.03


def test_gap_cannot_bridge_unobserved_motion_and_physical_covariance_is_blocked():
    with pytest.raises(ValueError,match='physical_covariance'):
        PlanarScanOdometry(simulation_covariance_model=False)
    sensor=VirtualLidar(4);tracker=PlanarScanOdometry(simulation_covariance_model=True)
    def observe(t):
        r=sensor.observe(position_enu_m=[2,2,1.7],rotation_enu_flu=np.eye(3))
        return tracker.observe(ranges=r,stamp_s=t,rotation_enu_flu=np.eye(3),height_above_floor_m=1.7)
    observe(0.);observe(.1)
    gap=observe(1.)
    assert not gap['accepted'] and 'new_segment' in gap['reasons']
    assert gap['reset_counter']==1
    assert observe(1.1)['sample_midpoint_s']==pytest.approx(1.05)


def test_invalid_attitude_is_rejected_before_indexing():
    tracker=PlanarScanOdometry(simulation_covariance_model=True)
    with pytest.raises(ValueError,match='invalid_scan_attitude'):
        tracker.observe(ranges=np.ones(430),stamp_s=0.,rotation_enu_flu=[1.,0.,0.],height_above_floor_m=1.7)


def test_two_interval_velocity_keeps_midpoint_and_gap_invalidates_history():
    sensor=VirtualLidar(8,range_noise_m=0.,scan_bias_noise_m=0.)
    tracker=PlanarScanOdometry(simulation_covariance_model=True,velocity_intervals=2,
        simulation_position_quantum_m=.012)
    def observe(t):
        ranges=sensor.observe(position_enu_m=[2.+.1*t,2.,1.7],rotation_enu_flu=np.eye(3))
        return tracker.observe(ranges=ranges,stamp_s=t,rotation_enu_flu=np.eye(3),height_above_floor_m=1.7)
    assert not observe(0.)['accepted']
    assert not observe(.1)['accepted']
    result=observe(.2)
    assert result['accepted']
    assert result['interval_s']==pytest.approx(.2)
    assert result['sample_midpoint_s']==pytest.approx(.1)
    assert result['velocity_ne_m_s']==pytest.approx([0.,.1],abs=.001)
    assert np.linalg.eigvalsh(result['covariance_ne_m2_s2']).min()>=.03**2+.012**2/(6*.2**2)-1e-9
    assert not observe(1.)['accepted']
    assert not observe(1.1)['accepted']
    assert observe(1.2)['accepted']
