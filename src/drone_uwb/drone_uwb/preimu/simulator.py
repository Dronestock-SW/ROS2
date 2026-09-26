"""Deterministic synthetic UWB/ToF/attitude source, not a Gazebo or flight-physics model."""
import math

import numpy as np


def simulated_events(layout, settings, duration_s=10.0, seed=7):
    """Known stationary XY with varying Z; truth stays outside the sensor message.

    ToF is downward, R_WB is identity and lever arms are zero in this synthetic fixture only.
    UWB RAW includes the configured signed bias. Pipeline calibration subtracts it again.
    """
    if not math.isfinite(duration_s) or not 0 < duration_s <= 3600:
        raise ValueError('duration_s must be in (0, 3600]')
    anchors = np.asarray(layout['anchors_xyz_m'], dtype=float)
    if anchors.shape != (4, 3) or not np.isfinite(anchors).all():
        raise ValueError('simulator requires four finite anchor positions')
    rng = np.random.default_rng(seed)
    offset_us = 10_000_000
    start_us = 1_000_000

    def envelope(msg, received_esp_us, truth=None):
        event = {'source': 'simulation',
                 'host_received_monotonic_ns': (offset_us + received_esp_us) * 1000,
                 'message': msg}
        if truth is not None:
            event['simulation_truth'] = truth
        return event

    yield envelope({'type': 'uwb_raw_status', 'schema': 1, 'tag_id': settings.tag_id,
                    'event': 'boot', 'uwb_ready': True, 'firmware': 'uwb-tag-jetson-raw-v1',
                    'anchor_order': ['A1', 'A2', 'A3', 'A4'], 'anchor_count': 4,
                    'range_bias_applied': False, 'height_correction_applied': False,
                    'temporal_filter_applied': False, 'xy_solver_applied': False,
                    'kalman_applied': False, 'ai_applied': False}, start_us)

    def z_at(esp_us):
        return 1.13 + 0.12 * math.sin(2 * math.pi * (esp_us - start_us) / 4_000_000)

    end_us = start_us
    for k in range(math.ceil(duration_s * 40)):
        begin_us = start_us + k * 25_000
        end_us = begin_us + 20_000
        sensor_us = offset_us + begin_us
        z = z_at(begin_us)
        yield envelope({'type': 'tof_sample', 'clock_domain': 'host_monotonic_us',
                        'measurement_time_us': sensor_us, 'valid': True,
                        'distance_m': float(z + rng.normal(0, 0.002))}, begin_us + 1000,
                       {'z_fc_m': z, 'tof_model': 'downward_zero_lever_arm'})
        yield envelope({'type': 'attitude_sample', 'clock_domain': 'host_monotonic_us',
                        'measurement_time_us': sensor_us, 'valid': True,
                        'rotation_convention': 'R_WB', 'quaternion_wxyz': [1.0, 0.0, 0.0, 0.0]}, begin_us + 1000)
        sample_us = [begin_us + 4000 * (i + 1) for i in range(4)]
        raw = [float(np.linalg.norm(anchors[i] - [0.70, 1.80, z_at(sample_us[i])])
                     + settings.range_bias_m[i] + rng.normal(0, 0.005)) for i in range(4)]
        yield envelope({'type': 'uwb_raw_cycle', 'schema': 1, 'tag_id': settings.tag_id,
                        'seq': k + 1, 'cycle_start_us': begin_us, 'cycle_end_us': end_us,
                        'cycle_duration_us': 20_000, 'valid_mask': 15, 'rf_valid_mask': 0,
                        'raw_slant_m': raw, 'sample_time_us': sample_us,
                        'twr_seq': [(4 * k + i) % 256 for i in range(4)], 'failure': ['ok'] * 4}, end_us + 1000)
    # Exercise the input watchdog without fabricating another measurement.
    yield envelope({'type': 'pipeline_tick'}, end_us + 601_000)
