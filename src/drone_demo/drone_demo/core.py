"""Repeatable trajectories and RAW observations using the existing UWB checks."""
from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path
import random

from drone_uwb.processing.solvers.observations import Decision, Processor, Settings


@dataclass(frozen=True)
class DemoConfig:
    scenario: str
    z_min_m: float
    z_max_m: float
    z_period_s: float
    start_xy_m: list
    target_xy_m: list
    rate_hz: float
    duration_s: float
    start_hold_s: float
    speed_m_s: float
    range_noise_stddev_m: float
    gap_start_s: float
    gap_duration_s: float
    seed: int

    def __post_init__(self):
        if self.scenario not in ('stationary', 'move', 'gap'):
            raise ValueError('scenario must be stationary, move, or gap')
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError('seed must be a nonnegative integer')
        for name in ('start_xy_m', 'target_xy_m'):
            values = getattr(self, name)
            if not isinstance(values, (tuple, list)) or len(values) != 2:
                raise ValueError(name + ' must contain two coordinates')
            if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                raise ValueError(name + ' must contain finite numbers')
        for field in fields(self):
            if field.name in ('scenario', 'start_xy_m', 'target_xy_m', 'seed'):
                continue
            value = getattr(self, field.name)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(field.name + ' must be finite and nonnegative')
        if not 1 <= self.rate_hz <= 100:
            raise ValueError('rate_hz must be in [1, 100]')
        if not self.z_min_m < self.z_max_m or self.z_period_s <= 0:
            raise ValueError('z bounds must increase and z_period_s must be positive')
        if self.duration_s <= 0 or self.speed_m_s <= 0:
            raise ValueError('duration_s and speed_m_s must be positive')
        if self.scenario == 'gap' and not (
                self.gap_duration_s > 0 and self.gap_start_s < self.duration_s):
            raise ValueError('gap must overlap the demo duration')

    @property
    def sample_count(self):
        return math.ceil(self.duration_s * self.rate_hz)


def load_inputs(config_file='', **overrides):
    """Use installed shared data, including the one authoritative anchor file."""
    from ament_index_python.packages import get_package_share_directory
    import yaml

    path = Path(config_file) if config_file else (
        Path(get_package_share_directory('drone_demo')) / 'config/demo.json')
    config = json.loads(path.read_text(encoding='utf-8'))
    config.update({name: value for name, value in overrides.items() if value is not None})
    uwb_share = Path(get_package_share_directory('drone_uwb'))
    layout = json.loads((uwb_share / 'config/anchors/anchors_20261004.json').read_text(encoding='utf-8'))
    values = yaml.safe_load((uwb_share / 'config/uwb.yaml').read_text(encoding='utf-8'))
    params = values['uwb_node']['ros__parameters']
    settings = Settings(**{k: v for k, v in params.items() if k in asdict(Settings())})
    settings.source_mode = 'raw_ranges'
    return DemoConfig(**config), layout, settings


class DemoRun:
    """A prescribed path, not motion resulting from a controller's commands."""

    def __init__(self, config, layout, uwb_settings=None):
        self.config = config
        self.layout = layout
        self.processor = Processor(layout, uwb_settings)
        if self.processor.settings.source_mode != 'raw_ranges':
            raise ValueError('demo requires raw_ranges processing')
        self.last_status_time = None
        self.last_index = -1

    @property
    def target_xyz(self):
        c = self.config
        xy = c.start_xy_m if c.scenario == 'stationary' else c.target_xy_m
        return [*xy, self.z_at(c.duration_s)]

    def z_at(self, time_s):
        c = self.config
        midpoint = (c.z_min_m + c.z_max_m) / 2
        amplitude = (c.z_max_m - c.z_min_m) / 2
        return midpoint + amplitude * math.sin(2 * math.pi * time_s / c.z_period_s)

    def truth_at(self, time_s):
        c = self.config
        if c.scenario == 'stationary':
            return [*c.start_xy_m, self.z_at(time_s)], 'STATIONARY'
        dx = c.target_xy_m[0] - c.start_xy_m[0]
        dy = c.target_xy_m[1] - c.start_xy_m[1]
        distance = math.hypot(dx, dy)
        travel = max(0.0, time_s - c.start_hold_s) * c.speed_m_s
        fraction = min(1.0, travel / distance) if distance > 0 else 1.0
        phase = ('ARRIVED' if fraction == 1.0 else
                 'WAITING' if time_s < c.start_hold_s else 'MOVING')
        return [c.start_xy_m[0] + fraction * dx,
                c.start_xy_m[1] + fraction * dy, self.z_at(time_s)], phase

    def sample(self, index, mono_ns=None, ros_ns=None):
        if type(index) is not int or not self.last_index < index < self.config.sample_count:
            raise ValueError('sample index must advance within the demo duration')
        self.last_index = index
        c = self.config
        t = index / c.rate_hz
        mono_ns = int(10e9 + t * 1e9) if mono_ns is None else mono_ns
        ros_ns = int(1_700_000_000e9) + mono_ns if ros_ns is None else ros_ns
        truth, phase = self.truth_at(t)
        in_gap = (c.scenario == 'gap'
                  and c.gap_start_s <= t < c.gap_start_s + c.gap_duration_s)
        received = []
        decision = Decision('demo_gap')

        def receive(message):
            received.append({'demo': True, 'host_received_monotonic_ns': mono_ns,
                             'host_received_ros_ns': ros_ns, 'message': message})
            return self.processor.process(message, mono_ns, ros_ns)

        if not in_gap:
            if self.last_status_time is None or t - self.last_status_time >= 2.0:
                receive(dict(type='uwb_raw_status', schema=1,
                             tag_id=self.processor.settings.tag_id, demo=True,
                             uwb_ready=True, anchor_count=4,
                             anchor_order=self.layout['anchor_order'],
                             anchor_layout_id=self.layout['layout_id'],
                             clock_domain='esp32_monotonic_boot_us',
                             temporal_filter_applied=False))
                self.last_status_time = t
            end_us = 1_000_000 + round(t * 1e6)
            cycle_us = min(20_000, round(0.8e6 / c.rate_hz))
            sample_us = [end_us - round(cycle_us * f) for f in (0.8, 0.6, 0.4, 0.2)]
            # Independent per-index RNG makes dropped ROS timer ticks reproducible.
            rng = random.Random(c.seed + index)
            ranges = []
            for anchor, report_us in zip(self.layout['anchors_xyz_m'], sample_us):
                position, _ = self.truth_at(t - (end_us - report_us) / 1e6)
                distance = math.dist(anchor, position)
                ranges.append(max(0.001, distance + rng.gauss(0, c.range_noise_stddev_m)))
            decision = receive(dict(
                type='uwb_raw_cycle', schema=1,
                tag_id=self.processor.settings.tag_id, demo=True, seq=index,
                cycle_start_us=end_us-cycle_us, cycle_end_us=end_us,
                valid_mask=15, raw_slant_m=ranges, sample_time_us=sample_us,
                failure=['ok']*4, attempt_count=[1]*4))

        observation = asdict(decision.observation) if decision.observation else None
        reference = None
        if observation:
            observation_t = t - (ros_ns - observation['stamp_ns']) / 1e9
            reference = self.truth_at(observation_t)[0][:2]
        return {'demo': True, 'schema': 1, 'time_s': t, 'seq': index,
                'scenario': c.scenario, 'frame_id': self.layout['coordinate_frame'],
                'z_source': 'demo_sine', 'z_measured': False,
                'truth_xyz_m': truth, 'target_xyz_m': self.target_xyz,
                'trajectory_phase': phase, 'uwb_available': not in_gap,
                'decision': decision.reason, 'observation': observation,
                'observation_reference_xy_m': reference, 'received': received,
                'imu_source': 'not_provided', 'lidar_source': 'not_provided',
                'flight_interface': 'none'}
