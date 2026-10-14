"""Synthetic clock transitions only: no ROS node, replay file or FC output."""
import pytest

from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.timing.clock import ClockMap
from drone_uwb.processing.timing.recovery import ClockRecovery
from test_tdma_stream import warmed_measured_processor, pair, event, status

OFFSET = 1_700_000_000_000_000_000


def retained(identity=(10, 20)):
    c = ClockMap(PreimuSettings())
    for seq in range(1, 81):
        c.update(seq*25_000, 10_000_000_000+seq*25_000_000)
    assert c.ready
    r = ClockRecovery()
    r.qualified(c, 2_000_000, 12_000_000_000, OFFSET, 80, identity, 100_000_000_000)
    return r, r.expire(c, 100_001_000_000), c


def add(r, c, seq, *, residual=0, offset=0, local=None):
    return r.update(c, seq*25_000, 10_000_000_000+seq*25_000_000+residual,
                    OFFSET+offset, seq, (10, 20),
                    98_000_000_000+seq*25_000_000 if local is None else local)


def test_three_pairs_requalify_without_mutating_retained_predictor():
    r, c, saved = retained()
    original = (list(saved.points), saved.base, saved.slope, saved.beta)
    for count, seq in enumerate((81, 83), 1):  # Skips inside the time window are allowed.
        c = add(r, c, seq, residual=10_000_000)
        assert not c.ready
        assert len(c.points) == count  # Cold learning starts immediately.
    c = add(r, c, 85, residual=10_000_000)
    assert c.ready and r.reason == 'retained_map_requalified'
    assert c is not saved and len(c.points) == 83
    assert (list(saved.points), saved.base, saved.slope, saved.beta) == original


@pytest.mark.parametrize('residual,offset,accepted', [
    (-2_000_001, 0, False), (-2_000_000, 0, True),
    (50_000_000, 0, True), (50_000_001, 0, False),
    (0, 5_000_000, True), (0, -5_000_000, True),
    (0, 5_000_001, False), (0, -5_000_001, False),
])
def test_individual_limits_checked_before_refit(residual, offset, accepted):
    r, c, saved = retained()
    c = add(r, c, 83, residual=residual, offset=offset)
    assert bool(r.checks) is accepted
    assert len(c.points) == 1 and len(saved.points) == 80


def test_bad_point_clears_confirmation_but_does_not_wait_to_learn():
    r, c, _ = retained()
    c = add(r, c, 81)
    c = add(r, c, 82, offset=5_000_001)
    assert not r.checks and len(c.points) == 2
    for seq in (83, 84):
        c = add(r, c, seq)
        assert not c.ready
    c = add(r, c, 85)
    assert c.ready and r.reason == 'retained_map_requalified'


def test_median_rejection_and_standard_acquisition_do_not_wait_for_ttl():
    r, c, _ = retained()
    for seq in range(81, 111):
        c = add(r, c, seq, residual=21_000_000)
        if seq == 83:
            assert not r.checks and r.reason == 'revalidation_median_rejected'
        assert c.ready == (seq == 110)
    assert r.reason == 'new_map_qualified' and r.held is None


@pytest.mark.parametrize('source_step,local_step,retained_ok', [
    (2_000_000_000, 2_000_000_000, True),
    (2_000_000_001, 100_000_000, False),
    (100_000_000, 2_000_000_001, False),
])
def test_ttl_applies_to_both_source_and_local_clock(source_step, local_step, retained_ok):
    r, c, _ = retained()
    r.update(c, 2_000_000+source_step//1000, 12_000_000_000+source_step,
             OFFSET, 160, (10, 20), 100_000_000_000+local_step)
    assert (r.held is not None) is retained_ok


@pytest.mark.parametrize('local', [False, True])
def test_confirmation_window_applies_to_both_clocks(local):
    r, c, _ = retained()
    c = add(r, c, 81)
    c = add(r, c, 82)
    c = add(r, c, 83 if local else 86,
            local=100_125_000_001 if local else None)
    assert not c.ready and len(r.checks) == 1
    assert len(c.points) == 3


def test_repeat_expiry_does_not_extend_ttl_or_erase_cold_learning():
    r, c, _ = retained()
    c = add(r, c, 81)
    assert r.expire(c, 101_999_000_000) is c and len(c.points) == 1
    assert r.held.steady_ns == 100_000_000_000 and not r.checks
    assert r.expire(c, 102_000_000_001) is c
    assert r.held is None and len(c.points) == 1


def test_unverified_stream_and_changed_map_cannot_be_retained():
    r, c, saved = retained(identity=None)
    assert r.held is None and not c.ready
    r = ClockRecovery()
    r.qualified(saved, 2_000_000, 12_000_000_000, OFFSET, 80, (10, 20), 100_000_000_000)
    saved.update(2_025_000, 12_025_000_000)
    assert not r.expire(saved, 100_050_000_000).ready and r.held is None


def test_failed_refit_falls_back_to_cold_map(monkeypatch):
    r, c, saved = retained()
    bad = ClockMap(saved.settings)
    monkeypatch.setattr(saved, 'updated_copy', lambda points: bad)
    for seq in (81, 82, 83):
        c = add(r, c, seq)
    assert not c.ready and len(c.points) == 3
    assert r.reason == 'retained_refit_rejected'


def test_pending_loss_clears_consecutive_count_status_and_sidebands_do_not():
    p, mono = warmed_measured_processor('5')
    p.reject_queued_input()
    raw, diag = pair(81)
    p.process(event(raw, mono+25_000_000))
    p.process(event(diag, mono+30_000_000))
    assert len(p.clock_recovery.checks) == 1
    p.process(event(status(), mono+31_000_000))
    side = dict(type='lora_rx', schema=1, tag_id='5', rssi_dbm=-50, snr_db=10, payload='{}')
    p.process(event(side, mono+32_000_000))
    assert len(p.clock_recovery.checks) == 1
    raw, _ = pair(82)
    p.process(event(raw, mono+50_000_000))
    raw, diag = pair(83)
    p.process(event(raw, mono+75_000_000))
    assert not p.clock_recovery.checks
    p.process(event(diag, mono+80_000_000))
    assert len(p.clock_recovery.checks) == 1


@pytest.mark.parametrize('change', ['session', 'boot', 'host_backward', 'ros_backward'])
def test_real_discontinuity_clears_retention(change):
    p, mono = warmed_measured_processor('5')
    p.reject_queued_input()
    assert p.clock_recovery.held is not None
    if change in ('session', 'boot'):
        raw, diag = pair(81, session=21 if change == 'session' else 20,
                         boot=11 if change == 'boot' else 10)
        p.process(event(raw, mono+25_000_000))
        p.process(event(diag, mono+30_000_000))
    else:
        e = event(status(), mono if change == 'host_backward' else mono+6_000_000)
        if change == 'ros_backward':
            e['host_received_ros_ns'] -= 2_000_000  # <250ms offset gate, still backward.
        assert p.process(e)['reason'] == 'host_clock_jump'
    assert p.clock_recovery.held is None and not p.clock.ready


def test_predictor_residual_uses_the_same_units_and_origin_as_clock_map():
    r, c, saved = retained()
    add(r, c, 81, residual=13_645_000)
    assert r.residual_ns == pytest.approx(12_038_645_000-saved.host_s(2_025_000)*1e9, abs=1)
