import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.b4_integrity import B4HeightIntegrity
from drone_uwb.processing.experiments.h80_b import BSettings, H80Window
from drone_uwb.processing.experiments.tof_subset import ToFTrackedSubsetCandidate
from drone_uwb.processing.measured_btf import MeasuredBtf, height_independent_b4


ANCHORS = np.array([[0, 0, .15], [6.3, 0, .15], [0, 4.6, .15], [6.3, 4.6, .15]])


def row(stamp=1_000_000, velocity=(.8, -.5), height=True, bias=0.):
    ts = [stamp-12000+i*4000 for i in range(4)]
    xy = np.array([2., 2.])+np.asarray(velocity)*(stamp-1_000_000)/1e6
    positions = [np.r_[xy+np.asarray(velocity)*(t-stamp)/1e6, 1.2] for t in ts]
    ranges = [np.linalg.norm(p-a)+bias for p, a in zip(positions, ANCHORS)]
    return dict(time_us=stamp, sample_time_us=ts, cal_slant_m=ranges,
                height_m=1.2 if height else None, sample_height_m=[1.2 if height else None]*4,
                height_source='measured_tof_imu' if height else None,
                models={'B': dict(xy_m=xy.tolist(), velocity_m_s=list(velocity))})


def guard():
    return B4HeightIntegrity(ANCHORS, limit_m=.06, window_s=.8)


def config():
    c = json.loads((Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json').read_text(encoding='utf-8'))
    c['anchors_xyz_m'] = ANCHORS.tolist()
    c['enforce_b4_height_integrity'] = True
    return c


def test_normal_translation_is_not_a_fixed_start_displacement_gate():
    g = guard()
    for i in range(160):
        allowed, evidence = g.check(row(1_000_000+i*25_000))
        assert allowed and evidence['physical_range_rms_m'] < 1e-12
    assert not g.blocked


def test_slow_range_bias_cannot_pass_just_because_it_has_no_jump():
    g = guard(); blocked = []
    for i in range(160):
        allowed, evidence = g.check(row(1_000_000+i*25_000, bias=-i*.001))
        if not allowed: blocked.append(i)
    assert blocked and blocked[-1] == 159
    assert evidence['reason'] == 'b4_height_inconsistent'


def test_geometrically_coherent_false_motion_needs_an_independent_sensor():
    # Identical ranges/height can represent real movement or coordinated bias.
    # No range-only gate can decide which is true. Do not add a stationary clamp.
    g = guard()
    assert all(g.check(row(1_000_000+i*25_000, velocity=(.01, 0.)))[0] for i in range(200))


@pytest.mark.parametrize('interruption', ['missing', 'ground', 'nan', 'gap', 'backward', 'duplicate'])
def test_bad_height_conflict_survives_unavailable_data_and_requires_fresh_recovery(interruption):
    g = guard(); assert not g.check(row(bias=-.15))[0]
    assert not g.check(row(1_025_000))[0]
    d = row(1_050_000)
    if interruption == 'missing': d = row(1_050_000, height=False)
    if interruption == 'ground': d['height_source'] = 'ground_antenna_reference'
    if interruption == 'nan': d['sample_height_m'][2] = float('nan')
    if interruption == 'gap': d = row(1_200_000)
    if interruption == 'backward': d = row(1_000_000)
    if interruption == 'duplicate': d = row(1_025_000)
    assert not g.check(d)[0]
    for i in range(7):
        allowed, e = g.check(row(2_000_000+i*25_000))
        assert allowed == (i == 6)
    assert e['state'] == 'consistent'


def test_unknown_height_is_diagnostic_xy_only_and_never_claims_validation():
    allowed, e = guard().check(row(height=False))
    assert allowed and not e['checked'] and e['state'] == 'unavailable'


def test_future_anchor_time_does_not_clear_a_conflict():
    d = row(); d['sample_time_us'][0] += 1_000_000
    assert guard().check(d)[0] is False


def test_resetting_fit_or_clock_does_not_clear_physical_conflict():
    p = MeasuredBtf(config()); p.candidate.b4_integrity.blocked = True
    for action in (p.reset_models, p.reject_queued_input, p.reset):
        action()
        assert p.candidate.b4_integrity.blocked
    assert p.start_ground_session(connected=True, armed=False, landed=1,
        state_age_s=.1, landed_age_s=.1, stationary=True, velocity_age_s=.1)[0]
    assert not p.candidate.b4_integrity.blocked


@pytest.mark.parametrize('velocity,climb', [((0., 0.), .0), ((.7, -.3), .0), ((.2, .2), .4)])
@pytest.mark.parametrize('height_period_us', [100_000, 102_000])
def test_causal_sequential_ranges_and_slower_height_do_not_block_normal_motion(velocity, climb, height_period_us):
    c = config(); w = H80Window(ANCHORS, [0]*4, BSettings(**c['B']))
    p = ToFTrackedSubsetCandidate(c, **c['tof_subset']); output = []
    for i in range(240):
        stamp = 1_000_000+i*25_000; d = row(stamp, velocity=velocity)
        # 40Hz UWB, staggered anchors, 10/9.8Hz height. Keep the 100ms age limit.
        ranges = []; heights = []
        for j, t in enumerate(d['sample_time_us']):
            elapsed = (t-1_000_000)/1e6
            point = np.r_[np.array([2., 2.])+np.array(velocity)*elapsed, 1.2+climb*elapsed]
            ranges.append(float(np.linalg.norm(point-ANCHORS[j])))
            height_t = (t//height_period_us)*height_period_us
            heights.append(1.2+climb*(height_t-1_000_000)/1e6 if t-height_t<=100_000 else None)
        d.update(cal_slant_m=ranges, sample_height_m=heights, height_m=heights[-1])
        cy = Cycle(i, d['sample_time_us'][0], stamp, 15, np.array(ranges),
                   d['sample_time_us'], ['ok']*4, list(range(4)))
        d['models']['B'] = w.process(cy, i)
        result = p.process(d)
        if i > 40:
            output.append(result)
            assert result['ok']
            assert result['xy_m'] == pytest.approx(np.array([2., 2.])+np.array(velocity)*(stamp-1_000_000)/1e6, abs=.002)
    assert len(output) == 199


def test_persistent_bad_ranges_do_not_reopen_when_tof_expires():
    c = config(); w = H80Window(ANCHORS, [0]*4, BSettings(**c['B']))
    p = ToFTrackedSubsetCandidate(c, **c['tof_subset'])
    for i in range(200):
        d = row(1_000_000+i*25_000, velocity=(0., 0.), bias=-.15, height=i<100)
        cy = Cycle(i, d['sample_time_us'][0], d['time_us'], 15,
                   np.array(d['cal_slant_m']), d['sample_time_us'], ['ok']*4, list(range(4)))
        d['models']['B'] = w.process(cy, i)
        result = p.process(d)
        if i > 40:
            assert not result['ok'] and result['xy_m'] is None
            assert p.prior_xy is None
            assert not height_independent_b4(dict(d, **result))
    assert result['reason'] == 'b4_height_conflict_unresolved'


@pytest.mark.parametrize('policy', ['missing', False, True])
def test_fresh_install_retains_field_b4_output_and_veto_requires_explicit_policy(policy):
    c = config()
    if policy == 'missing':
        del c['enforce_b4_height_integrity']
    else:
        c['enforce_b4_height_integrity'] = policy
    w = H80Window(ANCHORS, [0]*4, BSettings(**c['B']))
    p = ToFTrackedSubsetCandidate(c, **c['tof_subset'])
    checked = 0
    for i in range(100):
        d = row(1_000_000+i*25_000, velocity=(0., 0.), bias=-.15)
        cy = Cycle(i, d['sample_time_us'][0], d['time_us'], 15,
                   np.array(d['cal_slant_m']), d['sample_time_us'], ['ok']*4, list(range(4)))
        d['models']['B'] = w.process(cy, i)
        result = p.process(d)
        if i <= 40:
            continue
        checked += 1
        assert result['b4_height_check']['state'] == 'inconsistent'
        if policy is True:
            assert not result['ok'] and result['xy_m'] is None
        else:
            assert result['ok'] and result['xy_m'] == d['models']['B']['xy_m']
            assert result['source'] == 'unchanged_B4'
            assert result['b4_height_check']['enforced'] is False
    assert checked == 59


@pytest.mark.parametrize('policy', ['false', 0, 1, None])
def test_invalid_policy_cannot_silently_enable_or_disable_veto(policy):
    c = config(); c['enforce_b4_height_integrity'] = policy
    with pytest.raises(ValueError, match='boolean_b4_height_integrity_policy'):
        ToFTrackedSubsetCandidate(c, **c['tof_subset'])
