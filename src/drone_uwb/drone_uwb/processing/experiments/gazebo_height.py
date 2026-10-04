"""Causal Gazebo ToF/IMU height selection for file-only UWB comparisons.

The alignment and floor gates default closed. Gazebo truth is never read here.
"""
from bisect import bisect_right
import math

import numpy as np

from drone_uwb.processing.gazebo_geometry import rotation_world_body
from drone_uwb.processing.height import tof_to_fc_height


def _stream(rows, kind):
    times, previous = [], None
    for row in rows:
        stamp = row.get('time_us')
        if (row.get('schema') != 1 or row.get('source') != 'gazebo_sensor'
                or row.get('type') != kind or row.get('clock_domain') != 'gazebo_sim_us'
                or type(stamp) is not int or stamp < 0
                or previous is not None and stamp <= previous):
            raise ValueError('invalid_or_unordered_'+kind)
        times.append(stamp)
        previous = stamp
    return times


class GazeboSensorHeight:
    def __init__(self, tof_rows, attitude_rows, settings, tag_offset_body_flu_m):
        self.tof_rows, self.attitude_rows = list(tof_rows), list(attitude_rows)
        self.tof_times = _stream(self.tof_rows, 'tof_sample')
        self.attitude_times = _stream(self.attitude_rows, 'imu_attitude_sample')
        if not self.tof_times or not self.attitude_times:
            raise ValueError('empty_gazebo_sensor_stream')
        if set(settings) != {'schema', 'orientation_alignment_confirmed', 'flat_floor_confirmed',
                             'tof_bias_m', 'ground_z_m', 'tof_axis_body_flu',
                             'tof_lever_body_flu_m', 'max_tof_age_s', 'max_attitude_skew_s'}:
            raise ValueError('invalid_height_profile')
        if settings['schema'] != 1 or any(type(settings[key]) is not bool for key in
                ('orientation_alignment_confirmed', 'flat_floor_confirmed')):
            raise ValueError('invalid_height_profile')
        for key in ('tof_bias_m', 'ground_z_m', 'max_tof_age_s', 'max_attitude_skew_s'):
            value = settings[key]
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('invalid_height_profile')
        if min(settings['max_tof_age_s'], settings['max_attitude_skew_s']) <= 0:
            raise ValueError('invalid_height_profile')
        for key in ('tof_axis_body_flu', 'tof_lever_body_flu_m'):
            array = np.asarray(settings[key], float)
            if array.shape != (3,) or not np.isfinite(array).all():
                raise ValueError('invalid_height_profile')
            if key == 'tof_axis_body_flu' and not np.isclose(np.linalg.norm(array), 1., rtol=0, atol=1e-9):
                raise ValueError('invalid_height_profile')
        self.tag_offset = np.asarray(tag_offset_body_flu_m, float)
        if self.tag_offset.shape != (3,) or not np.isfinite(self.tag_offset).all():
            raise ValueError('invalid_tag_mount')
        self.settings = dict(settings)

    def height_at(self, target_us):
        if type(target_us) is not int or target_us < 0:
            raise ValueError('invalid_height_target_time')
        if not self.settings['orientation_alignment_confirmed']:
            return None, dict(reason='orientation_alignment_unconfirmed')
        if not self.settings['flat_floor_confirmed']:
            return None, dict(reason='ground_plane_unconfirmed')
        tof_index = bisect_right(self.tof_times, target_us)-1
        if tof_index < 0:
            return None, dict(reason='tof_unavailable')
        tof = self.tof_rows[tof_index]
        tof_age_us = target_us-tof['time_us']
        if tof_age_us > self.settings['max_tof_age_s']*1e6:
            return None, dict(reason='tof_stale', tof_time_us=tof['time_us'])
        # A newer invalid sample blocks reuse of an older valid range.
        if tof.get('valid') is not True or tof.get('reason') != 'ok':
            return None, dict(reason='tof_invalid', tof_time_us=tof['time_us'])
        value = tof.get('distance_m')
        limits = (tof.get('range_min_m'), tof.get('range_max_m'))
        if (type(value) not in (int, float) or not math.isfinite(value)
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in limits)
                or not 0 < limits[0] <= value <= limits[1]):
            return None, dict(reason='tof_invalid', tof_time_us=tof['time_us'])
        imu_index = bisect_right(self.attitude_times, tof['time_us'])-1
        if imu_index < 0:
            return None, dict(reason='attitude_unavailable', tof_time_us=tof['time_us'])
        imu = self.attitude_rows[imu_index]
        if tof['time_us']-imu['time_us'] > self.settings['max_attitude_skew_s']*1e6:
            return None, dict(reason='attitude_stale', tof_time_us=tof['time_us'],
                              attitude_time_us=imu['time_us'])
        if imu.get('valid') is not True or imu.get('reason') != 'ok':
            return None, dict(reason='attitude_invalid', tof_time_us=tof['time_us'],
                              attitude_time_us=imu['time_us'])
        try:
            rotation = rotation_world_body(imu['quaternion_wxyz'])
            # The existing FC formula computes any reference origin when its lever
            # arms are expressed from that origin; here the origin is body center.
            z_body = tof_to_fc_height(value, self.settings['tof_bias_m'], rotation,
                         self.settings['tof_axis_body_flu'], self.settings['tof_lever_body_flu_m'],
                         self.settings['ground_z_m'])
            z_tag = z_body+float((rotation@self.tag_offset)[2])
        except (ValueError, TypeError, KeyError):
            return None, dict(reason='height_geometry_invalid', tof_time_us=tof['time_us'],
                              attitude_time_us=imu['time_us'])
        return z_tag, dict(reason='ok', tof_time_us=tof['time_us'],
                           attitude_time_us=imu['time_us'], tof_age_s=tof_age_us/1e6,
                           attitude_skew_s=(tof['time_us']-imu['time_us'])/1e6)
