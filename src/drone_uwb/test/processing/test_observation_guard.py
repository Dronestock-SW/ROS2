import pytest
from drone_uwb.processing.observation_guard import ObservationGuard


def test_causal_guard_rejects_coherent_step_replays_and_never_generates_measurements():
    g=ObservationGuard();decisions=[]
    for i in range(60):
        decisions.append(g.check((2.+i*.0025,2.),1000000000+i*25000000))
    assert decisions[-1]=='ready'
    before=g.prior_xy
    for i in range(100):
        assert g.check((3.,2.),2500000000+i*25000000)=='observation_jump_quarantined'
    assert g.prior_xy==before
    assert g.check(before,2500000000)=='duplicate_or_backward_observation'
    assert g.check(before,5100000000)=='observation_recovering'
    assert g.check(before,5300000000)=='ready'


def test_explicit_sender_trial_accepts_step_but_keeps_time_and_value_checks():
    g = ObservationGuard(motion_recovery=True)
    for i in range(20):
        result = g.check((2., 2.), 1_000_000_000+i*25_000_000, reject_jumps=False)
    assert result == 'ready'
    assert g.check((3., 2.), 1_500_000_000, reject_jumps=False) == 'ready'
    assert g.check((3., 2.), 1_500_000_000, reject_jumps=False) == 'duplicate_or_backward_observation'
    assert g.check((float('nan'), 2.), 1_525_000_000, reject_jumps=False) == 'invalid_observation'
    assert g.check((4., 2.), 2_000_000_000, reject_jumps=False) == 'observation_recovering'
    assert g.check((4., 2.), 2_150_000_000, reject_jumps=False) == 'ready'


@pytest.mark.parametrize('policy', [0, 1, None, 'false'])
def test_jump_policy_cannot_be_implicitly_coerced(policy):
    with pytest.raises(ValueError, match='boolean_jump_policy_required'):
        ObservationGuard().check((2., 2.), 1_000_000_000, reject_jumps=policy)
