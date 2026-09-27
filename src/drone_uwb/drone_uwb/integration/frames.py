"""Fixed observation frame transform, without reading PX4 position/attitude."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BridgeSettings:
    enabled: bool = False
    alignment_confirmed: bool = False
    timing_confirmed: bool = False
    sensor_mount_confirmed: bool = False
    enu_yaw_deg: float = 0.0
    enu_offset_x_m: float = 0.0
    enu_offset_y_m: float = 0.0
    expected_ev_delay_ms: float = 0.0
    source_frame: str = 'uwb_map'
    max_age_s: float = 0.20
    state_timeout_s: float = 2.5

    def __post_init__(self):
        for name in ('enabled', 'alignment_confirmed', 'timing_confirmed', 'sensor_mount_confirmed'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(name + ' must be boolean')
        for name in ('enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m',
                     'expected_ev_delay_ms', 'max_age_s', 'state_timeout_s'):
            if not math.isfinite(getattr(self, name)):
                raise ValueError('nonfinite transform setting')
        if min(self.max_age_s, self.state_timeout_s) <= 0 or not self.source_frame:
            raise ValueError('invalid bridge timing/frame')


def gate(settings, connected, state_age_s, params, param_age_s):
    if not settings.enabled:
        return 'disabled'
    for name in ('alignment_confirmed', 'timing_confirmed', 'sensor_mount_confirmed'):
        if not getattr(settings, name):
            return name + '_required'
    if not connected or not 0 <= state_age_s <= settings.state_timeout_s:
        return 'fcu_disconnected_or_stale'
    if not 0 <= param_age_s <= 5.0:
        return 'fcu_parameters_unavailable'
    if params.get('EKF2_EV_CTRL') != 1:
        return 'require_horizontal_only_ev_ctrl_1'
    delay = params.get('EKF2_EV_DELAY')
    if delay is None or not math.isfinite(delay) or abs(delay - settings.expected_ev_delay_ms) > 0.01:
        return 'ev_delay_mismatch'
    if params.get('EKF2_EV_NOISE_MD') != 0:
        return 'require_message_covariance_mode_0'
    return 'ready'


def rotate_xy_covariance(x, y, covariance, settings):
    vector = np.array([x, y], dtype=float)
    matrix = np.asarray(covariance, dtype=float)
    if not np.isfinite(vector).all() or matrix.shape != (2, 2) or not np.isfinite(matrix).all():
        raise ValueError('invalid XY observation')
    if not np.allclose(matrix, matrix.T, atol=1e-9) or np.min(np.linalg.eigvalsh(matrix)) <= 0:
        raise ValueError('invalid XY covariance')
    angle = math.radians(settings.enu_yaw_deg)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    return (rotation @ vector + [settings.enu_offset_x_m, settings.enu_offset_y_m],
            rotation @ matrix @ rotation.T)
