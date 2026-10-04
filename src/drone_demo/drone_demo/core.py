"""Repeatable XY-only fixtures for display and mission assessment."""
from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path
import random

from drone_uwb.processing.solvers.observations import Settings


@dataclass(frozen=True)
class DemoConfig:
    scenario: str
    start_xy_m: list
    target_xy_m: list
    rate_hz: float
    duration_s: float
    start_hold_s: float
    speed_m_s: float
    xy_noise_stddev_m: float
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
    return DemoConfig(**config), layout, settings


class DemoRun:
    """A prescribed path, not motion resulting from a controller's commands."""

    def __init__(self, config, layout, uwb_settings=None):
        self.config = config
        self.layout = layout
        self.settings = uwb_settings or Settings()
        self.last_index = -1

    @property
    def target_xy(self):
        c = self.config
        xy = c.start_xy_m if c.scenario == 'stationary' else c.target_xy_m
        return list(xy)

    def truth_at(self, time_s):
        c = self.config
        if c.scenario == 'stationary':
            return list(c.start_xy_m), 'STATIONARY'
        dx = c.target_xy_m[0] - c.start_xy_m[0]
        dy = c.target_xy_m[1] - c.start_xy_m[1]
        distance = math.hypot(dx, dy)
        travel = max(0.0, time_s - c.start_hold_s) * c.speed_m_s
        fraction = min(1.0, travel / distance) if distance > 0 else 1.0
        phase = ('ARRIVED' if fraction == 1.0 else
                 'WAITING' if time_s < c.start_hold_s else 'MOVING')
        return [c.start_xy_m[0] + fraction * dx,
                c.start_xy_m[1] + fraction * dy], phase

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
        observation = None
        if not in_gap:
            # Independent per-index RNG makes dropped ROS timer ticks reproducible.
            rng = random.Random(c.seed + index)
            observation = dict(seq=index, stamp_ns=ros_ns,
                               x=truth[0] + rng.gauss(0, c.xy_noise_stddev_m),
                               y=truth[1] + rng.gauss(0, c.xy_noise_stddev_m),
                               variance=self.settings.xy_stddev_m ** 2,
                               source_mode='demo_xy', demo=True)
        return {'demo': True, 'schema': 2, 'time_s': t, 'seq': index,
                'scenario': c.scenario, 'frame_id': self.layout['coordinate_frame'],
                'z_source': 'unobserved', 'z_measured': False, 'z_m': None,
                'truth_xy_m': truth, 'target_xy_m': self.target_xy,
                'trajectory_phase': phase, 'uwb_available': not in_gap,
                'decision': 'demo_gap' if in_gap else 'demo_xy', 'observation': observation,
                'observation_reference_xy_m': truth if observation else None,
                'imu_source': 'not_provided', 'lidar_source': 'not_provided',
                'flight_interface': 'none'}
