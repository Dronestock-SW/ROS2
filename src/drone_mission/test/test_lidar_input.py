import math
import pytest
from drone_mission.lidar_input import beam_attitudes, nearest_uwb, quaternion_at


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
