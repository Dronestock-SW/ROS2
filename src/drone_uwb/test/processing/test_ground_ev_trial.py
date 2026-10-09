import pytest
from drone_uwb.processing.ground_ev_trial import ground_trial_gate, inflated_xy_covariance
from drone_uwb.processing.ground_ev_trial import GroundAuthorityLatch


@pytest.mark.parametrize('event', [dict(armed=True),dict(connected=False),dict(landed=2),
    dict(landed=3),dict(landed=4),dict(kill_channel=1000)])
def test_fresh_authority_loss_cannot_resume_the_same_ground_stream(event):
    latch=GroundAuthorityLatch()
    assert not latch.observe(fresh=False,**event)
    assert not latch.observe(fresh=True)
    assert latch.observe(fresh=True,**event)
    assert latch.observe(fresh=True)  # Fresh normal values never undo a takeover.


@pytest.mark.parametrize('source', [(-.1,0,0,.1),(0,0,0,0),(.1,.2,.2,.1),
    (.1,0,.01,.1),(.1,0,0,float('nan')),(float('inf'),0,0,.1),(.1,0,0)])
def test_test_allowance_cannot_repair_invalid_source_covariance(source):
    with pytest.raises(ValueError):inflated_xy_covariance(source)


def test_trial_allowance_preserves_cross_covariance():
    assert inflated_xy_covariance((.1,.02,.02,.2))==pytest.approx((.35,.02,.02,.45))


def sample():
    return dict(connected=True,armed=False,state_age_s=.1,landed=1,landed_age_s=.1,
        kill_channel=1999,rc_age_s=.1,bridge_disabled=True,bridge_age_s=.1,
        execution_disabled=True,flight_age_s=.1,parameter_age_s=.1,measured_mount_frd_m=[-.1,0.,-.1],
        parameters=dict(EKF2_EV_CTRL=1,EKF2_EV_NOISE_MD=0,EKF2_EV_DELAY=0.,
            EKF2_EV_POS_X=-.1,EKF2_EV_POS_Y=0.,EKF2_EV_POS_Z=-.1,RC_MAP_KILL_SW=7))


def test_disarmed_ground_transport_only():
    assert ground_trial_gate(**sample())=='ready'


@pytest.mark.parametrize('change',[
    dict(armed=True),dict(connected=False),dict(state_age_s=2),dict(landed=2),
    dict(landed_age_s=2),dict(kill_channel=1000),dict(rc_age_s=.4),
    dict(bridge_disabled=False),dict(bridge_age_s=1),
    dict(execution_disabled=False),dict(flight_age_s=1),dict(parameter_age_s=4),
])
def test_closes_output_on_every_live_authority_loss(change):
    assert ground_trial_gate(**(sample()|change))!='ready'


@pytest.mark.parametrize('key,value', [('EKF2_EV_CTRL',3),('EKF2_EV_POS_X',0),
    ('RC_MAP_KILL_SW',8),('EKF2_EV_DELAY',50),('EKF2_EV_NOISE_MD',1)])
def test_horizontal_only_and_expected_mount_parameters(key,value):
    data=sample();data['parameters'][key]=value
    assert ground_trial_gate(**data)=='parameter_mismatch:'+key


@pytest.mark.parametrize('mount', [None,[],[0,0],[0,0,float('nan')],[0,0,2],[True,0,0]])
def test_invalid_mount_never_opens_trial(mount):
    assert ground_trial_gate(**(sample()|dict(measured_mount_frd_m=mount)))=='measured_mount_required'


def test_mount_is_supplied_by_field_evidence_not_a_default():
    data=sample();data['measured_mount_frd_m']=[-.2,.03,-.12]
    data['parameters'].update(EKF2_EV_POS_X=-.2,EKF2_EV_POS_Y=.03,EKF2_EV_POS_Z=-.12)
    assert ground_trial_gate(**data)=='ready'
