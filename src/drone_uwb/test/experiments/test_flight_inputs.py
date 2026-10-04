"""Preserve PX4 field semantics and block unpaired UWB/height experiments."""
import numpy as np

from drone_uwb.processing.experiments.flight_inputs import (
    attitude_records, position_intervals, range_records, summarize,
)


def test_unknown_quality_variance_and_missing_measurement_time():
    data = dict(timestamp=[100, 200, 300], current_distance=[1., .01, 1.],
                min_distance=[.1]*3, max_distance=[35.]*3, signal_quality=[-1, 0, 0],
                variance=[0., 0., 0.], type=[0]*3, orientation=[25]*3, device_id=[123]*3)
    rows = range_records(data)
    assert rows[0]['usable_raw'] and not rows[0]['signal_quality_known']
    assert [r['reason'] for r in rows] == ['ok', 'invalid_distance', 'invalid_signal']
    assert all(r['variance_m2'] is None and r['timestamp_sample_us'] is None for r in rows)
    assert all(r['z_antenna_m'] is None and not r['external_output_allowed'] for r in rows)


def test_attitude_stays_in_native_frame_and_checks_sample_time():
    data = {'timestamp': [200, 300], 'timestamp_sample': [190, 310], 'quat_reset_counter': [0, 1],
            'q[0]': [1., 1.], 'q[1]': [0., 0.], 'q[2]': [0., 0.], 'q[3]': [0., 0.]}
    rows = attitude_records(data)
    assert rows[0]['usable_raw'] and not rows[1]['usable_raw']
    assert rows[0]['rotation_convention'] == 'FRD_body_to_NED_earth'
    assert rows[0]['clock_domain'] == 'px4_boot_us'
    assert rows[0]['timestamp_sample_us'] == 190


def test_mode_intervals_are_time_based_and_missing_uwb_blocks_comparison():
    data = dict(timestamp=np.array([0, 1_000_000, 2_000_000, 5_000_000]), nav_state=np.array([1, 2, 2, 1]))
    intervals = position_intervals(data, 6_000_000)
    assert intervals == [dict(start_us=1_000_000, end_us=5_000_000, duration_s=4.)]
    summary = summarize([], [], data, 6_000_000)
    assert not summary['ABC_flight_comparison_ready'] and not summary['antenna_height_calculated']
    assert 'missing_synchronous_uwb_raw' in summary['blocked_reasons']
