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
