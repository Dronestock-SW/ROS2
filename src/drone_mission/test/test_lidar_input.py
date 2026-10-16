import math
import pytest
from drone_mission.lidar_input import beam_attitudes, nearest_uwb, quaternion_at, point_cloud_samples, cloud_scan


def test_attitude_interpolation_does_not_use_latest_receive_time():
    q0=(1.,0.,0.,0.);q1=(math.cos(.01),0.,0.,math.sin(.01))
    q=quaternion_at([(1_000_000_000,q0),(1_020_000_000,q1)],1_010_000_000)
    assert math.atan2(q[3],q[0])*2==pytest.approx(.01)
    assert sum(x*x for x in q)==pytest.approx(1.)


@pytest.mark.parametrize('stamp',[999_000_000,1_021_000_000])
def test_no_orientation_extrapolation(stamp):
    with pytest.raises(ValueError,match='not_bracketed'):
        quaternion_at([(1_000_000_000,(1.,0.,0.,0.)),(1_020_000_000,(1.,0.,0.,0.))],stamp)


def test_beam_time_and_attitude_holes_cannot_be_faked():
    history=[(1_000_000_000,(1.,0.,0.,0.)),(1_100_000_000,(1.,0.,0.,0.))]
    with pytest.raises(ValueError,match='measured_beam_timing'):
        beam_attitudes(history,1_000_000_000,0.,430)
    with pytest.raises(ValueError,match='history_gap'):
        beam_attitudes(history,1_000_000_000,.0001,430)


def test_uwb_anchor_preserves_original_stamp_and_rejects_alignment_gap():
    history=[(1_000_000_000,(2.,2.)),(1_020_000_000,(2.01,2.))]
    assert nearest_uwb(history,1_010_000_000)==history[0]
    with pytest.raises(ValueError,match='alignment_missing'):nearest_uwb(history,1_100_000_000)


def sdk_points():
    # Acquisition order wraps across +/-pi; angular bin order differs.
    angles=[(2.+i*.13+math.pi)%(2*math.pi)-math.pi for i in range(80)]
    points=[dict(x=2*math.cos(a),y=2*math.sin(a),z=0.) for a in angles]
    times=[.001+i*.001 for i in range(80)]
    return points,[dict(name='stamps',values=times)]


def test_sdk_acquisition_times_survive_angle_rebinning_and_padding():
    points,channels=sdk_points();points+=points[-1:]*5
    channels[0]['values']+=[.081+i*.001 for i in range(5)]
    ranges,angles,times,dropped=point_cloud_samples(points,channels)
    assert len(ranges)==80 and dropped==5 and times[-1]==pytest.approx(.08)
    assert any(b<a for a,b in zip(angles,angles[1:]))
    history=[(1_000_000_000+i*20_000_000,(1.,0.,0.,0.)) for i in range(6)]
    row=cloud_scan(points,channels,history,1_000_000_000,'laser_frame',1_090_000_000,.015,lambda y:y)
    assert row['stamp_ns']==1_080_000_000 and row['age_s']==.015
    assert row['point_times_s']==times and 'time_increment' not in row


@pytest.mark.parametrize('fault',['missing','reversed','nan','size'])
def test_sdk_time_faults_are_rejected_without_inventing_timing(fault):
    points,channels=sdk_points()
    if fault=='missing':channels=[]
    if fault=='reversed':channels[0]['values'][20]=.0001
    if fault=='nan':channels[0]['values'][20]=float('nan')
    if fault=='size':channels[0]['values'].pop()
    with pytest.raises(ValueError):point_cloud_samples(points,channels)


def test_cloud_requires_actual_attitude_brackets_and_fresh_original_time():
    points,channels=sdk_points()
    history=[(1_000_000_000+i*20_000_000,(1.,0.,0.,0.)) for i in range(6)]
    with pytest.raises(ValueError,match='not_bracketed'):
        cloud_scan(points,channels,history[:-2],1_000_000_000,'laser_frame',1_090_000_000,.01,lambda y:y)
    with pytest.raises(ValueError,match='stale_or_future'):
        cloud_scan(points,channels,history,1_000_000_000,'laser_frame',2_000_000_000,.01,lambda y:y)
