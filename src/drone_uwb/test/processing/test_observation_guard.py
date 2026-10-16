import pytest
from drone_uwb.processing.observation_guard import ObservationGuard


def test_causal_guard_accepts_coherent_step_but_rejects_replayed_time():
    g=ObservationGuard();decisions=[]
    for i in range(60):
        decisions.append(g.check((2.+i*.0025,2.),1000000000+i*25000000))
    assert decisions[-1]=='ready'
    before=g.prior_xy
    for i in range(100):
        assert g.check((3.,2.),2500000000+i*25000000)=='ready'
    assert g.prior_xy==(3.,2.)
    assert g.check(before,2500000000)=='duplicate_or_backward_observation'
    assert g.check(before,5300000000)=='observation_recovering'
    assert g.check(before,5500000000)=='ready'


@pytest.mark.parametrize('legacy_policy', [True, False])
def test_legacy_policy_cannot_restore_removed_jump_quarantine(legacy_policy):
    g = ObservationGuard()
    for i in range(20):
        g.check((1., 1.), 1_000_000_000+i*25_000_000)
    assert g.check((20., -10.), 1_500_000_000, reject_jumps=legacy_policy) == 'ready'
    assert g.prior_xy == (20., -10.)


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
