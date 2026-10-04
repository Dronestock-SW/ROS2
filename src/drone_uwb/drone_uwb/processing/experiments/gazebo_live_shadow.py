"""Causal, file-independent Gazebo UWB comparison without simulator truth.

This processor accepts virtual RAW plus separately measured Gazebo ToF/IMU.
It never sends positions to PX4 or treats a missing sensor as a measurement.
"""
from collections import deque
from dataclasses import asdict

import numpy as np

from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.experiments.gazebo_height import GazeboSensorHeight
from drone_uwb.processing.experiments.gazebo_trial import validate_config
from drone_uwb.processing.experiments.h80_b import BSettings, H80Window
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.gazebo_geometry import rotation_world_body
from drone_uwb.processing.intersections import DSettings, make_candidates as model_d
from drone_uwb.processing.triplets import make_candidates as model_c
from drone_uwb.processing.weighted_xy import solve_weighted_xy


MODEL_NAMES = ('A', 'B', 'C', 'D', 'WLS')


def _finite_vector(value, size, name):
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('invalid_'+name) from exc
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise ValueError('invalid_'+name)
    return vector


class GazeboLiveShadow:
    """Single-session, single-writer shadow processor for live Gazebo samples."""

    def __init__(self, config, height_profile):
        self.anchors = validate_config(config)
        if config['faults']:
            raise ValueError('live_raw_fault_schedule_must_be_empty')
        if 'tag_offset_body_flu_m' not in config:
            raise ValueError('tag_mount_required')
        self.config = config
        self.profile = height_profile
        self.bias = _finite_vector(config['bias_m'], 4, 'bias')
        self.range_covariance = np.diag(_finite_vector(config['wls_sigma_m'], 4, 'wls_sigma')**2)
        self.window = H80Window(self.anchors, self.bias, BSettings(**config['B']))
        self.tof = deque()
        self.imu = deque()
        self.last_sensor_us = {'tof_sample': None, 'imu_attitude_sample': None}
        self.last_cycle_us = None
        self.last_seq = None

    def add_sensor(self, row):
        kind, stamp = row.get('type'), row.get('time_us')
        if (kind not in self.last_sensor_us or row.get('schema') != 1
                or row.get('source') != 'gazebo_sensor'
                or row.get('clock_domain') != 'gazebo_sim_us'
                or type(stamp) is not int or stamp < 0):
            raise ValueError('invalid_sensor_record')
        previous = self.last_sensor_us[kind]
        if previous is not None and stamp <= previous:
            raise ValueError('non_increasing_'+kind+'_time')
        self.last_sensor_us[kind] = stamp
        target = self.tof if kind == 'tof_sample' else self.imu
        target.append(row)
        # Keep enough history for configured sensor age plus callback holdback.
        lower = stamp-round(max(2., float(self.profile['max_tof_age_s'])*2.)*1e6)
        while len(target) > 1 and target[1]['time_us'] < lower:
            target.popleft()

    def _height_at(self, stamp):
        if not self.profile['orientation_alignment_confirmed']:
            return None, {'reason': 'orientation_alignment_unconfirmed'}
        if not self.profile['flat_floor_confirmed']:
            return None, {'reason': 'ground_plane_unconfirmed'}
        if not self.tof:
            return None, {'reason': 'tof_unavailable'}
        if not self.imu:
            return None, {'reason': 'attitude_unavailable'}
        selector = GazeboSensorHeight(self.tof, self.imu, self.profile,
                                      self.config['tag_offset_body_flu_m'])
        return selector.height_at(stamp)

    def _attitude_at(self, stamp):
        """Select a causal attitude at RAW time for reference-point conversion.

        Height projection uses the attitude at ToF time separately. Do not
        silently reuse that older attitude when the vehicle is rotating.
        """
        if not self.profile['orientation_alignment_confirmed']:
            return dict(reason='orientation_alignment_unconfirmed')
        row = next((item for item in reversed(self.imu) if item['time_us'] <= stamp), None)
        if row is None:
            return dict(reason='attitude_unavailable')
        age = (stamp-row['time_us'])/1e6
        result = dict(time_us=row['time_us'], age_s=age)
        if age > self.profile['max_attitude_skew_s']:
            return dict(result, reason='attitude_stale')
        if row.get('valid') is not True or row.get('reason') != 'ok':
            return dict(result, reason='attitude_invalid')
        try:
            rotation_world_body(row['quaternion_wxyz'])
        except (ValueError, TypeError, KeyError):
            return dict(result, reason='attitude_invalid')
        return dict(result, reason='ok', quaternion_wxyz=list(row['quaternion_wxyz']))

    def process_cycle(self, row):
        stamp, seq = row.get('time_us'), row.get('seq')
        if (row.get('schema') != 1 or row.get('source') != 'simulation'
                or row.get('type') != 'sim_uwb_cycle'
                or row.get('clock_domain') != 'gazebo_sim_us'
                or row.get('anchor_order') != ['A1', 'A2', 'A3', 'A4']
                or row.get('range_bias_applied') is not False
                or row.get('external_output_allowed') is not False
                or type(stamp) is not int or stamp < 0
                or type(seq) is not int or seq < 0):
            raise ValueError('invalid_sim_uwb_cycle')
        if self.last_cycle_us is not None and (stamp <= self.last_cycle_us or seq <= self.last_seq):
            raise ValueError('non_increasing_cycle_time_or_sequence')
        raw = _finite_vector(row.get('raw_slant_m'), 4, 'raw_slant')
        times = row.get('sample_time_us')
        if times != [stamp]*4:
            raise ValueError('non_simultaneous_range_samples')
        mask = row.get('valid_mask')
        expected_mask = sum(1 << i for i, value in enumerate(raw) if 0 < value <= 80.)
        if type(mask) is not int or mask != expected_mask:
            raise ValueError('invalid_range_mask')
        self.last_cycle_us, self.last_seq = stamp, seq
        corrected = raw-self.bias
        height, height_meta = self._height_at(stamp)
        output = dict(schema=1, source='gazebo_shadow_live',
                      clock_domain='gazebo_sim_us', time_us=stamp, seq=seq,
                      sample_time_us=times, anchor_order=row['anchor_order'],
                      raw_slant_m=raw.tolist(), cal_slant_m=corrected.tolist(),
                      height_m=height, height_source='gazebo_tof_imu' if height is not None else None,
                      height_selection=height_meta, models={},
                      attitude_selection=self._attitude_at(stamp),
                      truth_used=False, external_output_allowed=False, flight_valid=False)
        usable = [i for i in range(4) if mask & (1 << i) and 0 < corrected[i] <= 80.]
        cycle = Cycle(seq, stamp, stamp, sum(1 << i for i in usable), raw,
                      times, ['ok' if i in usable else 'invalid_range' for i in range(4)], usable)
        output['models']['B'] = self.window.process(cycle, seq)
        if len(usable) < 4:
            output['models'].update({name: dict(ok=False, reason='four_valid_ranges_required', xy_m=None)
                                     for name in ('A', 'C', 'D', 'WLS')})
        elif height is None:
            output['models'].update({name: dict(ok=False, reason=height_meta['reason'], xy_m=None)
                                     for name in ('A', 'C', 'D', 'WLS')})
        else:
            obs = [f'{seq}:A{i+1}' for i in range(4)]
            output['models']['A'] = asdict(solve_uniform_xy(self.anchors, corrected, height,
                                                             **self.config['solver']))
            output['models']['C'] = model_c(self.anchors, corrected, height, t_ref_us=stamp,
                                            obs_ids=obs, settings=self.config['solver'])
            output['models']['D'] = model_d(self.anchors, corrected, height, t_ref_us=stamp,
                                            obs_ids=obs, settings=DSettings(**self.config['D']))
            output['models']['WLS'] = solve_weighted_xy(self.anchors, corrected, height,
                                                         self.range_covariance, **self.config['solver'])
        for name in MODEL_NAMES:
            model = output['models'][name]
            if model.get('ok'):
                _finite_vector(model.get('xy_m'), 2, name+'_xy')
        return output
