"""Baseline A: mathematical recovery, causal replay, missing inputs, and provenance."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.processing.experiments.baseline_a import BaselineA, digest, main, run
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.runner import json_line
from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.simulator import simulated_events


CONFIG_DIR = Path(__file__).resolve().parents[2] / 'config'
LAYOUT = json.loads((CONFIG_DIR/'anchors_20260906.json').read_text(encoding='utf-8'))


def config():
    return json.loads((CONFIG_DIR/'baseline_a_simulation_20260921.json').read_text(encoding='utf-8'))


def events():
    settings = PreimuSettings(range_bias_m=config()['range_bias_m'])
    return list(simulated_events(LAYOUT, settings, duration_s=1))


def process(stream, cfg=None):
    model = BaselineA(cfg or config(), LAYOUT)
    return [model.process(row, index) for index, row in enumerate(stream, 1)]


def test_noiseless_recovery_with_unequal_anchor_heights_and_sample_z():
    anchors = np.array(LAYOUT['anchors_xyz_m'])
    anchors[:, 2] = [2.1, 2.4, 2.2, 2.5]
    xy, z = np.array([1.2, 2.4]), np.array([.8, .81, .82, .83])
    ranges = np.sqrt(np.sum((anchors[:, :2]-xy)**2, axis=1)+(anchors[:, 2]-z)**2)
    fit = solve_uniform_xy(anchors, ranges, z)
    assert fit.ok and fit.rms_m < 1e-10
    np.testing.assert_allclose(fit.xy_m, xy, atol=1e-10)


@pytest.mark.parametrize('bad', ['three', 'nonfinite', 'collinear', 'iterations'])
def test_invalid_or_unconverged_fit_does_not_emit_position(bad):
    anchors = np.array(LAYOUT['anchors_xyz_m'])
    ranges = np.linalg.norm(anchors-[.7, 1.8, 1.1], axis=1)
    settings = {}
    if bad == 'three':
        anchors, ranges = anchors[:3], ranges[:3]
    elif bad == 'nonfinite':
        ranges[0] = np.nan
    elif bad == 'collinear':
        anchors[:, 1] = 0
    else:
        ranges[0] += .4
        settings['max_iterations'] = 1
    fit = solve_uniform_xy(anchors, ranges, 1.1, **settings)
    assert not fit.ok and fit.xy_m is None


def test_bias_sign_original_preservation_and_causal_height():
    stream = events()
    original = deepcopy(stream)
    rows = process(stream)
    assert stream == original
    cycles = [r for r in rows if r['event_type'] == 'uwb_raw_cycle']
    assert [r['reason'] for r in cycles[:29]] == ['clock_unsynced']*29
    assert all(r['status'] == 'ok' for r in cycles[29:])
    for row in cycles[29:]:
        diag = row['diagnostics']
        np.testing.assert_allclose(np.array(diag['raw_slant_m'])-diag['cal_slant_m'],
                                   config()['range_bias_m'])
        for height, meta in zip(diag['z_antenna_m'], diag['z_selection']):
            assert meta['attitude_time_us'] <= meta['tof_time_us'] <= meta['target_time_us']
            tof = next(e['message'] for e in stream if e['message']['type'] == 'tof_sample'
                       and e['message']['measurement_time_us'] == meta['tof_time_us'])
            assert height == tof['distance_m']
    assert all(not r['external_output_allowed'] and not r['flight_valid'] for r in rows)
    assert rows[-1]['estimate'] is None and not rows[-1]['fresh']


def test_tilt_and_sensor_levers_produce_antenna_height():
    cfg = config()
    cfg['tof_geometry']['lever_tof_B_m'] = [.2, 0, -.1]
    cfg['tof_geometry']['lever_uwb_B_m'] = [-.1, 0, .05]
    model = BaselineA(cfg, LAYOUT)
    pitch = np.deg2rad(30)
    # A known FC height and tilted downward ray intersect a level floor.
    distance = (1.2-.2*np.sin(pitch)-.1*np.cos(pitch))/np.cos(pitch)
    model.sensors.add(dict(type='tof_sample', clock_domain='host_monotonic_us',
                           measurement_time_us=1_000_000, valid=True, distance_m=distance),
                      'simulation', 1_001_000_000)
    model.sensors.add(dict(type='attitude_sample', clock_domain='host_monotonic_us',
                           measurement_time_us=1_000_000, valid=True,
                           rotation_convention='R_WB',
                           quaternion_wxyz=[float(np.cos(pitch/2)), 0.,
                                            float(np.sin(pitch/2)), 0.]),
                      'simulation', 1_001_000_000)
    z_ant, _ = model.height_at(1_010_000)
    assert z_ant == pytest.approx(1.2+.1*np.sin(pitch)+.05*np.cos(pitch), abs=1e-12)


@pytest.mark.parametrize('failure', ['missing', 'stale', 'invalid', 'future', 'malformed'])
def test_unusable_tof_blocks_instead_of_reusing_position(failure):
    stream = events()
    tof_rows = [e for e in stream if e['message']['type'] == 'tof_sample']
    if failure == 'missing':
        stream = [e for e in stream if e not in tof_rows]
    elif failure == 'stale':
        stream = [e for e in stream if e not in tof_rows[1:]]
    else:
        for event in tof_rows:
            if failure == 'invalid':
                event['message']['valid'] = False
            elif failure == 'future':
                event['message']['measurement_time_us'] += 100_000
            else:
                event['message']['distance_m'] = -1
    cycles = [r for r in process(stream) if r['event_type'] == 'uwb_raw_cycle']
    assert all(r['estimate'] is None for r in cycles)
    assert cycles[-1]['reason'] == 'tof_sample_unavailable'


def test_malformed_latest_tof_invalidates_older_sample():
    stream = events()
    model = BaselineA(config(), LAYOUT)
    for i, e in enumerate(stream[:-4]):
        model.process(e, i)
    event = deepcopy(stream[-4])
    assert event['message']['type'] == 'tof_sample'
    event['message']['distance_m'] = -1
    assert model.process(event, 120)['reason'] == 'invalid_tof_distance'
    model.process(stream[-3], 121)
    assert model.process(stream[-2], 122)['reason'] == 'tof_sample_unavailable'


@pytest.mark.parametrize('flag', ['calibration_confirmed', 'geometry_confirmed'])
def test_unconfirmed_inputs_block(flag):
    cfg = config()
    cfg[flag] = False
    assert all(r['estimate'] is None for r in process(events(), cfg))


def test_source_mixing_and_boot_reset():
    model = BaselineA(config(), LAYOUT)
    stream = events()
    for i, event in enumerate(stream[:-4]):
        model.process(event, i)
    foreign = deepcopy(stream[-4])
    foreign['source'] = 'measured'
    assert model.process(foreign, 120)['reason'] == 'input_source_mismatch'
    assert not model.clock.ready and not model.sensors.rows['tof_sample']
    reboot = deepcopy(stream[0])
    reboot['host_received_monotonic_ns'] = foreign['host_received_monotonic_ns']
    assert model.process(reboot, 121)['reason'] == 'status_recorded'
    assert not model.clock.ready


def test_three_anchor_cycle_is_blocked_and_duplicate_is_rejected():
    stream = events()
    event = stream[-2]
    event['message']['valid_mask'] = 7
    event['message']['raw_slant_m'][3] = None
    event['message']['failure'][3] = 'report_timeout'
    model = BaselineA(config(), LAYOUT)
    for i, row in enumerate(stream[:-2]):
        model.process(row, i)
    assert model.process(event, 120)['reason'] == 'four_anchors_required'
    assert model.process(event, 121)['reason'] == 'duplicate_or_out_of_order'


def test_input_truth_fields_do_not_affect_calculation():
    stream = events()
    altered = deepcopy(stream)
    for event in altered:
        event['simulation_truth'] = {'z_fc_m': 1000, 'xy_m': [-999, 999]}
    assert process(stream) == process(altered)


def test_replay_determinism_and_separate_evaluation(tmp_path):
    incoming = tmp_path/'events.jsonl'
    incoming.write_text(''.join(map(json_line, events())), encoding='utf-8')
    cfg = config()
    cfg['input_sha256'] = digest(incoming)
    ref = {k: cfg[k] for k in ('input_sha256', 'dataset_id', 'frame_id', 'source')}
    ref.update(method='known_stationary_simulator_fixture', reference_point='uwb_antenna',
               xy_m=[.7, 1.8])
    first, second = tmp_path/'a'/'replay', tmp_path/'b'/'replay'
    summary = run(incoming, first, cfg, LAYOUT, ref)
    ref['xy_m'] = [10, 10]
    other = run(incoming, second, cfg, LAYOUT, ref)
    assert summary['position_error']['rmse_m'] < .02
    assert other['position_error']['rmse_m'] > 10
    assert (first/'results.jsonl').read_bytes() == (second/'results.jsonl').read_bytes()
    assert (first/'input.jsonl').read_bytes() == incoming.read_bytes()
    assert summary['positions'] == 11
    with pytest.raises(FileExistsError):
        run(incoming, first, cfg, LAYOUT)
    cfg['input_sha256'] = 'wrong'
    with pytest.raises(ValueError, match='input_hash_mismatch'):
        run(incoming, tmp_path/'wrong', cfg, LAYOUT)
    assert not (tmp_path/'wrong').exists()


def test_cli_rejects_wrong_anchor_file_before_output(tmp_path):
    cfg = config()
    cfg['layout_sha256'] = 'wrong'
    path = tmp_path/'config.json'
    path.write_text(json.dumps(cfg), encoding='utf-8')
    with pytest.raises(SystemExit):
        main(['--input', str(tmp_path/'unused.jsonl'), '--output', str(tmp_path/'output'),
              '--config', str(path), '--anchors', str(CONFIG_DIR/'anchors_20260906.json')])
    assert not (tmp_path/'output').exists()
