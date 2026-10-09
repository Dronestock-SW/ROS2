"""Fixed observation frame transform, without reading PX4 position/attitude."""
from dataclasses import dataclass
import math

import numpy as np
from drone_uwb.integration.planar import map_to_enu_matrix, y_axis_sign


@dataclass(frozen=True)
class BridgeSettings:
    enabled: bool = False
    input_source: str = 'uwb_xy'
    tag_id: str = '5'
    ground_only: bool = True
    layout_confirmed: bool = False
    alignment_confirmed: bool = False
    timing_confirmed: bool = False
    sensor_mount_confirmed: bool = False
    enu_yaw_deg: float = 0.0
    map_y_axis_sign: int = 1
    enu_offset_x_m: float = 0.0
    enu_offset_y_m: float = 0.0
    expected_ev_delay_ms: float = 0.0
    # Antenna relative to PX4 body origin, FRD. PX4 applies the lever arm.
    antenna_body_frd_x_m: float = 0.0
    antenna_body_frd_y_m: float = 0.0
    antenna_body_frd_z_m: float = 0.0
    source_frame: str = 'uwb_map'
    max_age_s: float = 0.20
    state_timeout_s: float = 2.5

    def __post_init__(self):
        y_axis_sign(self)
        for name in ('enabled', 'ground_only', 'layout_confirmed', 'alignment_confirmed', 'timing_confirmed', 'sensor_mount_confirmed'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(name + ' must be boolean')
        for name in ('enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m',
                     'expected_ev_delay_ms', 'max_age_s', 'state_timeout_s',
                     'antenna_body_frd_x_m', 'antenna_body_frd_y_m', 'antenna_body_frd_z_m'):
            if type(getattr(self, name)) not in (float, int) or not math.isfinite(getattr(self, name)):
                raise ValueError('nonfinite transform setting')
        if min(self.max_age_s, self.state_timeout_s) <= 0 or not self.source_frame:
            raise ValueError('invalid bridge timing/frame')
        if self.input_source not in ('uwb_xy', 'btf_xy') or self.tag_id not in ('5', '6'):
            raise ValueError('invalid observation source or Tag ID')

    @property
    def input_topic(self):
        return '/uwb/btf_pose' if self.input_source == 'btf_xy' else '/uwb_pose'

    @property
    def ros_domain_id(self):
        return 1 if self.tag_id == '5' else 2


def gate(settings, connected, state_age_s, params, param_age_s, *, armed=False):
    if not settings.enabled:
        return 'disabled'
    for name in ('layout_confirmed', 'alignment_confirmed', 'timing_confirmed', 'sensor_mount_confirmed'):
        if not getattr(settings, name):
            return name + '_required'
    if not connected or not 0 <= state_age_s <= settings.state_timeout_s:
        return 'fcu_disconnected_or_stale'
    if settings.ground_only and armed:
        return 'ground_only_requires_disarmed'
    if not 0 <= param_age_s <= 5.0:
        return 'fcu_parameters_unavailable'
    if params.get('EKF2_EV_CTRL') != 1:
        return 'require_horizontal_only_ev_ctrl_1'
    delay = params.get('EKF2_EV_DELAY')
    if delay is None or not math.isfinite(delay) or abs(delay - settings.expected_ev_delay_ms) > 0.01:
        return 'ev_delay_mismatch'
    if params.get('EKF2_EV_NOISE_MD') != 0:
        return 'require_message_covariance_mode_0'
    for axis in ('x', 'y', 'z'):
        actual = params.get('EKF2_EV_POS_' + axis.upper())
        expected = getattr(settings, 'antenna_body_frd_' + axis + '_m')
        if type(actual) not in (int, float) or not math.isfinite(actual) or abs(actual-expected) > .001:
            return 'antenna_lever_arm_parameter_mismatch'
    return 'ready'


def observation_xy(settings, *, frame, stamp_ns, now_ns, last_stamp_ns, x, y, covariance):
    """Keep the original stamp; transform antenna XY and its full 2x2 covariance."""
    if (frame != settings.source_frame or stamp_ns <= 0
            or not 0 <= (now_ns-stamp_ns)/1e9 <= settings.max_age_s
            or last_stamp_ns is not None and stamp_ns <= last_stamp_ns):
        raise ValueError('invalid_frame_or_timestamp')
    return rotate_xy_covariance(x, y, covariance, settings)


def rotate_xy_covariance(x, y, covariance, settings):
    vector = np.array([x, y], dtype=float)
    matrix = np.asarray(covariance, dtype=float)
    if not np.isfinite(vector).all() or matrix.shape != (2, 2) or not np.isfinite(matrix).all():
        raise ValueError('invalid XY observation')
    if not np.allclose(matrix, matrix.T, atol=1e-9, rtol=0) or np.min(np.linalg.eigvalsh(matrix)) <= 0:
        raise ValueError('invalid XY covariance')
    rotation = map_to_enu_matrix(settings)
    return (rotation @ vector + [settings.enu_offset_x_m, settings.enu_offset_y_m],
            rotation @ matrix @ rotation.T)
