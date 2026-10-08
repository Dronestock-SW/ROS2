"""Measured RAW/ToF/attitude adapter for the frozen B_TF algorithm.

Observation-only: no FC positions, simulator truth, control or MAVLink writes.
Unknown mounting geometry blocks ToF use; missing height is never fabricated.
"""
from collections import deque
from dataclasses import replace
import math
import numpy as np

from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.timing.clock import ClockMap
from drone_uwb.processing.gazebo_geometry import rotation_world_body
from drone_uwb.processing.experiments.h80_b import H80Window, BSettings
from drone_uwb.processing.experiments.tof_subset import ToFTrackedSubsetCandidate


class MeasuredHeight:
    def __init__(self, config):
        self.config = config
        self.samples = {'tof': deque(maxlen=1000), 'imu': deque(maxlen=2000)}

    def add(self, kind, sample):
        target = self.samples[kind]
        if type(sample['stamp_ns']) is not int or sample['stamp_ns'] <= 0:
            return False
        if target and sample['stamp_ns'] <= target[-1]['stamp_ns']:
            target.clear()
            return False
        target.append(dict(sample))
        return True

    def at(self, stamp_ns):
        c = self.config
        tof = next((r for r in reversed(self.samples['tof']) if r['stamp_ns'] <= stamp_ns), None)
        if tof is None:
            return None, {'reason': 'tof_unavailable'}
        meta = {'tof_stamp_ns': tof['stamp_ns'], 'range_m': tof.get('range_m'),
                'tof_age_s': (stamp_ns-tof['stamp_ns'])/1e9}
        if meta['tof_age_s'] > c['max_tof_age_s']:
            return None, dict(meta, reason='tof_stale')
        if not tof['valid']:
            return None, dict(meta, reason='tof_invalid')
        imu = next((r for r in reversed(self.samples['imu']) if r['stamp_ns'] <= tof['stamp_ns']), None)
        if imu is None:
            return None, dict(meta, reason='attitude_unavailable')
        meta.update(imu_stamp_ns=imu['stamp_ns'], attitude_skew_s=(tof['stamp_ns']-imu['stamp_ns'])/1e9)
        if meta['attitude_skew_s'] > c['max_attitude_skew_s']:
            return None, dict(meta, reason='attitude_stale')
        if not imu['valid']:
            return None, dict(meta, reason='attitude_invalid')
        if not c['mount_confirmed']:
            return None, dict(meta, reason='tof_to_tag_mount_unconfirmed')
        if not c['flat_floor_confirmed']:
            return None, dict(meta, reason='ground_plane_unconfirmed')
        try:
            rotation = rotation_world_body(imu['quaternion_wxyz'])
            delta = np.asarray(c['tof_to_tag_body_flu_m'], float)
            if delta.shape != (3,) or not np.isfinite(delta).all():
                raise ValueError('invalid_mount')
            # MAVROS attitude is ENU/FLU. Only vertical projection is used;
            # warehouse yaw alignment is not required to project onto world Z.
            projection = float(rotation[2, 2])
            distance = tof['range_m']-c['tof_bias_m']
            if projection < c['min_downward_projection'] or distance <= 0:
                raise ValueError('invalid_downward_projection')
            height = c['ground_z_m']+distance*projection+float((rotation@delta)[2])
            if not math.isfinite(height):
                raise ValueError('nonfinite_height')
        except (KeyError, TypeError, ValueError):
            return None, dict(meta, reason='height_geometry_invalid')
        return height, dict(meta, reason='ok')


class MeasuredBtf:
    def __init__(self, config):
        self.config = config
        self.input_settings = PreimuSettings(tag_id=config['tag_id'])
        self.validator = InputValidator(self.input_settings)
        self.clock = ClockMap(self.input_settings)
        self.height = MeasuredHeight(config['height'])
        self.last_host_ns = None
        self.last_ros_offset = None
        self.last_output_stamp_ns = None
        self.reset_models()

    def reset_models(self):
        c = self.config
        self.window = H80Window(c['anchors_xyz_m'], c['bias_m'], BSettings(**c['B']))
        self.candidate = ToFTrackedSubsetCandidate(c, **c['tof_subset'])
        self.last_output_stamp_ns = None

    def reset(self):
        self.validator.disconnect()
        self.clock.reset()
        self.reset_models()

    def process(self, event):
        out = {'schema': 1, 'source': 'measured_uwb_btf', 'truth_used': False,
               'external_output_allowed': False, 'flight_valid': False,
               'timestamp_calibrated': False, 'ok': False}
        try:
            msg = event['message']
            mono, ros = event['host_received_monotonic_ns'], event['host_received_ros_ns']
            if any(type(v) is not int or v <= 0 for v in (mono, ros)):
                raise InvalidInput('invalid_host_time', reset=True)
            offset = ros-mono
            if ((self.last_host_ns is not None and mono < self.last_host_ns)
                or (self.last_ros_offset is not None and abs(offset-self.last_ros_offset) > 250_000_000)):
                self.reset()
                self.height.samples['tof'].clear()
                self.height.samples['imu'].clear()
                self.last_host_ns, self.last_ros_offset = mono, offset
                raise InvalidInput('host_clock_jump')
            self.last_host_ns, self.last_ros_offset = mono, offset
            self.validator.check_common(msg)
            if msg.get('type') == 'uwb_raw_status':
                if msg.get('event') == 'boot':
                    self.reset()
                self.validator.on_status(msg, mono)
                if msg.get('range_bias_applied') is True:
                    raise InvalidInput('already_bias_corrected_input', reset=True)
                return dict(out, reason='status', tag_layout_id=msg.get('anchor_layout_id'))
            if msg.get('type') != 'uwb_raw_cycle':
                raise InvalidInput('unsupported_type')
            cycle = self.validator.on_cycle(msg, mono, ros)
            out.update(seq=cycle.seq, seq_gap=cycle.seq_gap, source_cycle_end_us=cycle.end_us,
                       sample_time_us=cycle.sample_us, raw_slant_m=msg['raw_slant_m'])
            if len(cycle.indices) != 4:
                self.reset_models()
                raise InvalidInput('four_valid_ranges_required')
            self.clock.update(cycle.end_us, mono)
            if not self.clock.ready:
                self.reset_models()
                return dict(out, reason='clock_unsynced')
            queue_age = (mono/1e9-self.clock.host_s(cycle.end_us))
            if not 0 <= queue_age <= self.input_settings.max_queue_s:
                self.reset_models()
                return dict(out, reason='queued_sample')
            # Fit at the newest actual anchor measurement, retaining all four times.
            stamp = max(cycle.sample_us)
            cycle = replace(cycle, end_us=stamp)
            stamp_ns = round(self.clock.host_s(stamp)*1e9)+offset
            if stamp_ns > ros or (self.last_output_stamp_ns is not None and stamp_ns <= self.last_output_stamp_ns):
                self.reset_models()
                return dict(out, reason='invalid_or_nonincreasing_stamp')
            heights, selections = [], []
            for sample in cycle.sample_us:
                height, selection = self.height.at(round(self.clock.host_s(sample)*1e9)+offset)
                heights.append(height)
                selections.append(selection)
            latest_index = cycle.sample_us.index(stamp)
            height = heights[latest_index]
            b = self.window.process(cycle, cycle.seq)
            corrected = cycle.raw-np.asarray(self.config['bias_m'])
            candidate_input = {'time_us': stamp, 'sample_time_us': cycle.sample_us,
                 'cal_slant_m': corrected.tolist(), 'height_m': height,
                 'sample_height_m': heights, 'height_source': 'measured_tof_imu' if height is not None else None,
                 'models': {'B': b}}
            btf = self.candidate.process(candidate_input)
            self.last_output_stamp_ns = stamp_ns
            return dict(out, ok=btf['ok'], reason=btf['reason'], xy_m=btf['xy_m'],
                xyz_m=([*btf['xy_m'], height] if btf['ok'] and height is not None
                       and all(h is not None for h in heights) else None),
                height_source='measured_tof_imu' if height is not None else None,
                stamp_ns=stamp_ns, time_us=stamp, cal_slant_m=corrected.tolist(),
                height_m=height, height_selection=selections, height_ready=all(h is not None for h in heights),
                models={'B': b, 'B_TF': btf}, clock_alpha=self.clock.alpha,
                receive_age_s=(ros-stamp_ns)/1e9, queue_age_s=queue_age,
                report_span_s=(max(cycle.sample_us)-min(cycle.sample_us))/1e6,
                layout_id=self.config['layout_id'], position_reference='uwb_antenna')
        except InvalidInput as exc:
            if exc.reset:
                self.reset()
            return dict(out, reason=exc.reason)
        except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
            self.reset_models()
            return dict(out, reason='invalid_input')
