"""Horizontal antenna observation contract, independent of ROS/FC transport.

Warehouse origin is local. No geographic origin, flight height, external yaw,
velocity estimate, or flight command is supplied. PX4 owns the lever arm.
"""
from dataclasses import dataclass
import math

from drone_uwb.integration.ros.frames import observation_xy


PARAMETERS = ('EKF2_GPS_CTRL', 'EKF2_EV_CTRL', 'EKF2_EV_DELAY',
              'EKF2_EV_NOISE_MD', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y',
              'EKF2_EV_POS_Z', 'EKF2_EV_QMIN')
APPLIED_PARAMETERS = {'EKF2_EV_POS_X': -.10, 'EKF2_EV_POS_Y': 0.,
                      'EKF2_EV_POS_Z': -.10, 'EKF2_EV_CTRL': 1}


@dataclass(frozen=True)
class LocalObservationSettings:
    enabled: bool = False
    tag_id: str = '6'
    source_frame: str = 'uwb_map'
    enu_yaw_deg: float = 0.
    map_y_axis_sign: int = -1
    enu_offset_x_m: float = 0.
    enu_offset_y_m: float = 0.
    max_age_s: float = .15
    jump_guard_enabled: bool = True
    host_boot_id: str = ''
    fc_remote_ns_at_alignment: int = 0
    alignment_method: str = ''
    layout_confirmed: bool = False
    sensor_mount_confirmed: bool = False
    expected_ev_delay_ms: float = 0.
    antenna_body_frd_x_m: float = -.10
    antenna_body_frd_y_m: float = 0.
    antenna_body_frd_z_m: float = -.10

    def __post_init__(self):
        if self.tag_id not in ('5', '6') or self.map_y_axis_sign not in (-1, 1):
            raise ValueError('invalid_tag_or_axis_sign')
        for name in ('enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m', 'max_age_s',
                     'expected_ev_delay_ms', 'antenna_body_frd_x_m',
                     'antenna_body_frd_y_m', 'antenna_body_frd_z_m'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('invalid_setting:' + name)
        if not 0 < self.max_age_s <= .2:
            raise ValueError('invalid_observation_age')
        for name in ('enabled', 'layout_confirmed', 'sensor_mount_confirmed',
                     'jump_guard_enabled'):
            if type(getattr(self, name)) is not bool:
                raise ValueError('invalid_confirmation:' + name)

    @property
    def ros_domain_id(self):
        return 1 if self.tag_id == '5' else 2


def parameter_gate(settings, params):
    if not settings.enabled:
        return 'disabled'
    if not settings.layout_confirmed or not settings.sensor_mount_confirmed:
        return 'layout_or_mount_unconfirmed'
    if (settings.alignment_method != 'operator_cardinal_and_stationary_px4_yaw'
            or not settings.host_boot_id or settings.fc_remote_ns_at_alignment <= 0):
        return 'alignment_evidence_required'
    expected = {'EKF2_GPS_CTRL': 0, 'EKF2_EV_CTRL': 1, 'EKF2_EV_NOISE_MD': 0,
                'EKF2_EV_DELAY': settings.expected_ev_delay_ms,
                'EKF2_EV_POS_X': settings.antenna_body_frd_x_m,
                'EKF2_EV_POS_Y': settings.antenna_body_frd_y_m,
                'EKF2_EV_POS_Z': settings.antenna_body_frd_z_m}
    for name, wanted in expected.items():
        actual = params.get(name)
        if (type(actual) not in (int, float) or not math.isfinite(actual)
                or abs(actual-wanted) > .001):
            return 'parameter_mismatch:' + name
    # No quantified quality score is invented for raw UWB ranging.
    if params.get('EKF2_EV_QMIN') != 0:
        return 'require_unscored_observation_qmin_0'
    return 'ready'


def trial_start_allowed(*, connected, armed, landed_state, state_age_s, landed_age_s):
    """Authorize a sender bypass once, on fresh connected DISARM ground evidence."""
    return (connected is True and armed is False
        and type(landed_state) is int and landed_state == 1
        and all(type(v) in (int, float) and math.isfinite(v)
                for v in (state_age_s, landed_age_s))
        and 0 <= state_age_s <= 2.5 and 0 <= landed_age_s <= .5)


def horizontal_odometry(settings, *, frame, stamp_ns, now_ns, last_stamp_ns,
                        x, y, covariance, reset_counter=0):
    xy, cov = observation_xy(settings, frame=frame, stamp_ns=stamp_ns,
        now_ns=now_ns, last_stamp_ns=last_stamp_ns, x=x, y=y, covariance=covariance)
    if type(reset_counter) is not int or not 0 <= reset_counter <= 255:
        raise ValueError('invalid_reset_counter')
    packed = [0.] * 21
    # ENU -> NED swaps XY, including the entire horizontal covariance.
    for index, value in ((0, cov[1, 1]), (1, cov[1, 0]), (6, cov[0, 0]),
                         (11, 1e6), (15, 1e6), (18, 1e6), (20, 1e6)):
        packed[index] = float(value)
    return dict(time_usec=stamp_ns // 1000, x=float(xy[1]), y=float(xy[0]),
        # PX4 requires three finite position components. EV_CTRL=1 excludes z.
        z=0., q=[math.nan] * 4, vx=math.nan, vy=math.nan, vz=math.nan,
        rollspeed=math.nan, pitchspeed=math.nan, yawspeed=math.nan,
        pose_covariance=packed, velocity_covariance=[math.nan] + [0.] * 20,
        reset_counter=reset_counter, quality=0)


def alignment_yaw_deg(body_enu_yaw_deg, body_map_yaw_deg, map_y_axis_sign):
    if map_y_axis_sign not in (-1, 1):
        raise ValueError('invalid_axis_sign')
    if not all(math.isfinite(v) for v in (body_enu_yaw_deg, body_map_yaw_deg)):
        raise ValueError('invalid_alignment_yaw')
    return math.remainder(body_enu_yaw_deg-map_y_axis_sign*body_map_yaw_deg, 360.)
