"""Pipeline integration: preserve evidence and keep every numerical/output gate closed."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.preimu import h80, height, qs10, rawxy, transform
from drone_uwb.processing.pipeline import Pipeline, PipelineConfig
from drone_uwb.processing.ranges import RangeGate
from drone_uwb.processing.runner import json_line, run_lines
from drone_uwb.processing.timing.sensors import SensorInputs
from drone_uwb.processing.simulator import simulated_events

CONFIG_DIR = Path(__file__).resolve().parents[2] / 'config'
LAYOUT = json.loads((CONFIG_DIR / 'anchors_20260906.json').read_text(encoding='utf-8'))


def config():
    return PipelineConfig.from_mapping(json.loads((CONFIG_DIR / 'preimu_pipeline.json').read_text(encoding='utf-8')))


def events(seconds=1.0):
    return list(simulated_events(LAYOUT, config().preimu, duration_s=seconds))


def test_full_pipeline_buffers_simulated_z_but_never_calls_calculators(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('a closed calculation gate invoked a numerical function')
    for module, name in [(h80, 'fit_h80'), (rawxy, 'solve_raw_xy'), (qs10, 'solve_q_s10'),
                         (height, 'tof_to_fc_height'), (transform, 'warehouse_to_px4')]:
        monkeypatch.setattr(module, name, forbidden)
    pipeline = Pipeline(config())
    # Longer than the old 10s boot-status timeout: file sessions latch a valid boot status.
    results = [pipeline.process(e) for e in events(11)]
    cycles = [r for r in results if r['event_type'] == 'uwb_raw_cycle']
    assert len(cycles) == 440
    assert all(r['stages']['input'] == 'accepted' for r in cycles)
    assert cycles[-1]['clock']['state'] == 'ok'
    assert cycles[-1]['sensor_inputs']['tof_sample']['available']
    assert cycles[-1]['sensor_inputs']['attitude_sample']['available']
    assert cycles[-1]['z_source'] == 'simulation'
    assert all(n > 3 for n in cycles[-1]['history_counts'])
    assert all(r['solver'] == 'none' and not r['valid'] and not r['fresh'] for r in results)
    assert all(r['x_m'] is None and r['y_m'] is None and r['z_m'] is None for r in results)
    assert results[-1]['input_stale'] is True


def test_bias_sign_and_original_data_are_preserved():
    pipeline = Pipeline(config())
    stream = events()
    original = deepcopy(stream)
    results = [pipeline.process(e) for e in stream]
    assert stream == original
    row = next(r for r in results if r['event_type'] == 'uwb_raw_cycle')
    np.testing.assert_allclose(np.array(row['raw_slant_m']) - row['cal_slant_m'], config().preimu.range_bias_m)
    assert row['range_bias_calibrated'] is False
    assert row['range_bias_source'].startswith('esp32_reference_trial')


def test_range_gate_rejects_isolated_spike_then_reacquires_consistent_level():
    gate = RangeGate(config().preimu)
    assert gate.update(1_000_000, 2.0).accepted
    assert not gate.update(1_025_000, 3.0).accepted
    assert gate.update(1_050_000, 2.01).accepted
    assert not gate.update(1_075_000, 3.01).accepted
    assert gate.update(1_100_000, 3.03).pending_count == 2
    result = gate.update(1_125_000, 3.02)
    assert result.accepted and result.reason == 'reacquired'
    assert result.value_m == pytest.approx(3.02)
    assert not gate.update(1_125_000, 3.02).accepted


def test_invalid_anchor_does_not_gain_a_history_sample_and_duplicate_is_rejected():
    pipeline = Pipeline(config())
    stream = events()
    pipeline.process(stream[0])
    frame = deepcopy(stream[3])
    frame['message']['valid_mask'] = 7
    frame['message']['raw_slant_m'][3] = None
    frame['message']['failure'][3] = 'report_timeout'
    result = pipeline.process(frame)
    assert result['range_accepted'] == [True, True, True, False]
    assert result['accepted_range_m'][3] is None
    assert result['history_counts'] == [1, 1, 1, 0]
    assert pipeline.process(frame)['reason'] == 'duplicate_or_out_of_order'
    assert pipeline.history.counts() == [1, 1, 1, 0]


def test_source_gap_clears_window_and_boot_clears_clock_and_sensors():
    pipeline = Pipeline(config())
    stream = events()
    for event in stream[:-1]:
        pipeline.process(event)
    frame = deepcopy(stream[-2])
    frame['host_received_monotonic_ns'] += 1_000_000_000
    frame['message']['seq'] += 1
    for key in ('cycle_start_us', 'cycle_end_us'):
        frame['message'][key] += 1_000_000
    frame['message']['sample_time_us'] = [t + 1_000_000 for t in frame['message']['sample_time_us']]
    result = pipeline.process(frame)
    assert result['source_gap_detected']
    assert result['history_counts'] == [1, 1, 1, 1]
    reboot = deepcopy(stream[0])
    reboot['host_received_monotonic_ns'] = frame['host_received_monotonic_ns'] + 1
    pipeline.process(reboot)
    assert pipeline.history.counts() == [0, 0, 0, 0]
    assert not pipeline.clock.ready
    assert not pipeline.sensors.rows['tof_sample']


def test_sensor_selection_preserves_source_and_does_not_use_future_or_stale_samples():
    sensors = SensorInputs()
    msg = {'type': 'tof_sample', 'clock_domain': 'host_monotonic_us',
           'measurement_time_us': 100_000, 'distance_m': 1.1, 'valid': True}
    sensors.add(msg, 'simulation', 100_000_000)
    future = dict(msg, measurement_time_us=200_000, distance_m=1.3)
    sensors.add(future, 'measured', 200_000_000)
    chosen = sensors.snapshot(150_000)['tof_sample']
    assert chosen['sample']['source'] == 'simulation'
    assert chosen['sample']['message']['distance_m'] == 1.1
    assert not sensors.snapshot(400_000)['tof_sample']['available']
    assert not sensors.snapshot(50_000)['tof_sample']['available']
    sensors.add(dict(future, measurement_time_us=250_000, valid=False), 'measured', 250_000_000)
    assert not sensors.snapshot(250_000)['tof_sample']['available']
    with pytest.raises(ValueError, match='sensor_clock_unmapped'):
        sensors.add(dict(msg, clock_domain='fc_boot_us'), 'measured', 250_000_000)
    with pytest.raises(ValueError, match='invalid_sensor_time'):
        sensors.add(future, 'simulation', 100_000_000)


def test_gates_cannot_be_opened_by_config():
    for name in ('calculation_enabled', 'external_output_enabled'):
        with pytest.raises(ValueError, match='must stay closed'):
            PipelineConfig(**{name: True})


def test_replay_is_identical_and_preserves_malformed_original_lines(tmp_path):
    lines = [json_line(e).encode('utf-8') for e in events()]
    lines += [b'{broken\n', b'{"overflow":1e400}\n', b'\xff\n', b'[]\n']
    run_lines(lines, tmp_path / 'a', config())
    run_lines(lines, tmp_path / 'b', config())
    for name in ('input.jsonl', 'diagnostics.jsonl', 'summary.json'):
        assert (tmp_path / 'a' / name).read_bytes() == (tmp_path / 'b' / name).read_bytes()
    assert (tmp_path / 'a/input.jsonl').read_bytes() == b''.join(lines)
    result = [json.loads(line) for line in (tmp_path / 'a/diagnostics.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len(result) == len(lines)
    assert all(row['stages']['input'] == 'rejected' for row in result[-4:])
    with pytest.raises(FileExistsError):
        run_lines(lines, tmp_path / 'a', config())


def test_height_equations_with_tilt_bias_and_nonzero_lever_arm():
    angle = np.pi / 3
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    # Corrected distance 2m, downward vertical projection 1m; sensor is 0.1m below FC.
    z = height.tof_to_fc_height(2.1, 0.1, rotation, [0, 0, -1], [0, 0, -0.2], 0)
    assert z == pytest.approx(1.1)
    fc = height.antenna_to_fc_position([1, 2, 3], rotation, [0, 0, 0.2])
    np.testing.assert_allclose(fc, np.array([1, 2, 3]) - rotation @ [0, 0, 0.2])
    assert height.median3([1.0, 10.0, 1.1]) == 1.1
    z_ab, vz_ab = height.alpha_beta_step(1.0, 0.1, 1.2, 0.1)
    assert z_ab == pytest.approx(1.086)
    assert vz_ab == pytest.approx(0.48)
    with pytest.raises(ValueError):
        height.tof_to_fc_height(1, 0, np.eye(3), [0, 0, 1], [0, 0, 0], 0)
