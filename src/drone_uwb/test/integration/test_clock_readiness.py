import math
import pytest
from drone_uwb.integration.clock_readiness import ClockReadiness


def sample(clock, i, *, now=None, rtt=2., offset=1_000_000_000):
    return clock.observe(now=i*.1 if now is None else now, remote_ns=i*100_000_000,
                         offset_ns=offset, rtt_ms=rtt)


def qualified():
    clock = ClockReadiness()
    for i in range(1,31):
        sample(clock,i)
    assert clock.ready(3.)
    return clock


def test_warmup_still_requires_thirty_continuous_good_samples():
    clock = ClockReadiness()
    for i in range(1,30):
        sample(clock,i)
        assert not clock.ready(i*.1)
    sample(clock,35)
    assert clock.stable_samples == 1 and not clock.ready(3.5)


@pytest.mark.parametrize('rtt', [21., 1800., -1., math.nan, math.inf])
def test_bad_probe_never_refreshes_last_valid_time_or_offset(rtt):
    clock = qualified()
    before = clock.__dict__.copy()
    assert sample(clock,31,rtt=rtt) == (False,False)
    assert clock.__dict__ == before
    assert not clock.ready(3.5)


def test_transport_gap_stays_closed_until_fresh_same_epoch_probe():
    clock = qualified()
    assert not clock.ready(5.)
    sample(clock,35,now=5.,rtt=1500.)
    assert not clock.ready(5.)
    sample(clock,51,now=5.1,offset=1_000_050_000)
    assert clock.ready(5.1)
    assert not clock.ready(5.6)


@pytest.mark.parametrize('rtt', [2., 1800.])
def test_offset_jump_revokes_qualification_and_requires_new_warmup(rtt):
    clock = qualified()
    accepted, changed = sample(clock,31,rtt=rtt,offset=1_010_000_000)
    assert changed and accepted == (rtt == 2.)
    assert not clock.ready(3.1)
    for i in range(32,60):
        sample(clock,i,offset=1_010_000_000)
        assert not clock.ready(i*.1)
    sample(clock,60,offset=1_010_000_000)
    if rtt == 2.:
        assert clock.ready(6.)
    else:
        assert not clock.ready(6.)
        sample(clock,61,offset=1_010_000_000)
        assert clock.ready(clock.last_valid_s)


def test_out_of_order_probe_cannot_revert_offset_or_refresh_time():
    clock = qualified()
    before = clock.__dict__.copy()
    assert sample(clock,29,now=3.1,offset=1_100_000_000) == (False,False)
    assert clock.__dict__ == before


def test_disconnect_reset_forgets_old_fc_epoch():
    clock = qualified()
    clock.reset()
    assert not clock.ready(3.)
    sample(clock,1,now=3.1)
    assert clock.remote_ns == 100_000_000 and clock.stable_samples == 1
    assert not clock.ready(3.1)
