"""SITL-only UWB position to MAVLink ODOMETRY field contract.

No network transport is implemented here. An unknown pose component never
becomes a made-up measurement: height, frame alignment, mount and time mapping
must all be confirmed before fields can be handed to a MAVLink sender.
"""
from dataclasses import dataclass
import math

import numpy as np

from drone_uwb.processing.gazebo_geometry import rotation_world_body


@dataclass(frozen=True)
class SITLOdometrySettings:
    enabled: bool = False
    alignment_confirmed: bool = False
    mount_confirmed: bool = False
    height_confirmed: bool = False
    time_mapping_confirmed: bool = False
    px4_reference_confirmed: bool = False
    sender_clock_domain: str = 'wsl_monotonic_us'
    enu_yaw_deg: float = 0.
    enu_offset_x_m: float = 0.
    enu_offset_y_m: float = 0.
    px4_origin_height_world_m: float = 0.
    tag_offset_body_flu_m: tuple = (0., 0., 0.)
    px4_reference_offset_body_flu_m: tuple = (0., 0., 0.)
    mount_sigma_xy_m: float = 0.
    expected_ev_delay_ms: float = 0.

    def __post_init__(self):
        if self.sender_clock_domain not in ('wsl_monotonic_us', 'gazebo_sim_us'):
            raise ValueError('unsupported_sender_clock_domain')
        for name in ('enabled', 'alignment_confirmed', 'mount_confirmed',
                     'height_confirmed', 'time_mapping_confirmed', 'px4_reference_confirmed'):
            if type(getattr(self, name)) is not bool:
                raise ValueError('invalid_confirmation_flag:'+name)
        for name in ('enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m',
                     'px4_origin_height_world_m', 'mount_sigma_xy_m', 'expected_ev_delay_ms'):
            value = getattr(self, name)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError('invalid_frame_setting:'+name)
        if self.mount_sigma_xy_m < 0 or not 0 <= self.expected_ev_delay_ms <= 300:
            raise ValueError('invalid_uncertainty_or_delay')
        for name in ('tag_offset_body_flu_m', 'px4_reference_offset_body_flu_m'):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError('invalid_mount_vector:'+name)


def transmission_gate(settings, params, *, fc_connected=False, param_age_s=math.inf,
                      sample_age_s=math.inf):
    """Refuse sender handoff until only horizontal EV is configured."""
    if not settings.enabled:
        return 'disabled'
    for name in ('alignment_confirmed', 'mount_confirmed', 'height_confirmed',
                 'time_mapping_confirmed', 'px4_reference_confirmed'):
        if not getattr(settings, name):
            return name+'_required'
    if not fc_connected or not 0 <= param_age_s <= 5.:
        return 'fc_parameters_unavailable_or_stale'
    if not 0 <= sample_age_s <= .15:
        return 'sensor_sample_unavailable_or_stale'
    if params.get('EKF2_EV_CTRL') != 1:
        return 'require_horizontal_only_ev_ctrl_1'
    if params.get('EKF2_EV_NOISE_MD') != 0:
        return 'require_message_covariance_mode_0'
    delay = params.get('EKF2_EV_DELAY')
    if (type(delay) not in (int, float) or not math.isfinite(delay)
            or abs(delay-settings.expected_ev_delay_ms) > .01):
        return 'ev_delay_mismatch'
    if any(params.get(name) != 0 for name in ('EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z')):
        return 'require_zero_ev_sensor_offset_for_reference_position'
    return 'ready'


def map_reference_xy_to_ned(map_xy_m, settings):
    point = np.asarray(map_xy_m, dtype=float)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise ValueError('finite_map_xy_required')
    angle = math.radians(settings.enu_yaw_deg)
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    enu = rotation@point+np.array([settings.enu_offset_x_m, settings.enu_offset_y_m])
    return [float(enu[1]), float(enu[0])]


def ned_xy_to_map_reference(ned_xy_m, settings):
    point = np.asarray(ned_xy_m, dtype=float)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise ValueError('finite_ned_xy_required')
    angle = math.radians(settings.enu_yaw_deg)
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    enu = np.array([point[1], point[0]])
    world = rotation.T@(enu-np.array([settings.enu_offset_x_m, settings.enu_offset_y_m]))
    return world.tolist()


def map_tag_to_px4_ned(tag_xyz_world_m, attitude_wxyz, covariance_tag_xy_m2, settings):
    """Convert a measured antenna point into an explicitly chosen PX4 reference.

    The attitude maps body FLU to the UWB/Gazebo world. PX4 reference position
    is compensated once here, so the EV sensor-offset parameters must be zero.
    """
    point = np.asarray(tag_xyz_world_m, dtype=float)
    covariance = np.asarray(covariance_tag_xy_m2, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('finite_tag_position_required')
    if (covariance.shape != (2, 2) or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, rtol=0, atol=1e-9)
            or np.min(np.linalg.eigvalsh(covariance)) <= 0):
        raise ValueError('positive_xy_covariance_required')
    body_to_world = rotation_world_body(attitude_wxyz)
    offset = (np.asarray(settings.px4_reference_offset_body_flu_m, dtype=float)
              - np.asarray(settings.tag_offset_body_flu_m, dtype=float))
    reference_world = point+body_to_world@offset
    angle = math.radians(settings.enu_yaw_deg)
    c, s = math.cos(angle), math.sin(angle)
    map_to_enu = np.array([[c, -s], [s, c]])
    swap = np.array([[0., 1.], [1., 0.]])
    ned_xy = map_reference_xy_to_ned(reference_world[:2], settings)
    ned_cov = swap@map_to_enu@covariance@map_to_enu.T@swap.T
    ned_cov += np.eye(2)*settings.mount_sigma_xy_m**2
    return dict(position_ned_m=[float(ned_xy[0]), float(ned_xy[1]),
                                float(settings.px4_origin_height_world_m-reference_world[2])],
                covariance_ned_xy_m2=ned_cov.tolist(),
                reference_world_xyz_m=reference_world.tolist(),
                reference_point='px4_reference',
                orientation_observed=False, velocity_observed=False)


def odometry_fields(candidate, sender_sample_us, settings, params, *, sample_clock_domain,
                    fc_connected=False, param_age_s=math.inf, sample_age_s=math.inf):
    """Return a packet field dictionary, never a sent packet.

    `sender_sample_us` uses the same clock as the sender's TIMESYNC responses.
    For a verified Gazebo clock connection it retains the sensor's simulation
    timestamp. PX4 applies the sender-to-boot offset itself. The confirmed
    gate covers the chosen clock source and PX4's converged conversion.
    """
    reason = transmission_gate(settings, params, fc_connected=fc_connected,
                               param_age_s=param_age_s, sample_age_s=sample_age_s)
    if reason != 'ready':
        raise ValueError(reason)
    if sample_clock_domain != settings.sender_clock_domain:
        raise ValueError('sender_measurement_clock_mismatch')
    if type(sender_sample_us) is not int or not 0 < sender_sample_us < 2**64:
        raise ValueError('verified_sender_sample_time_required')
    position = candidate['position_ned_m']
    covariance = np.asarray(candidate['covariance_ned_xy_m2'], dtype=float)
    if (len(position) != 3 or not np.isfinite(position).all()
            or covariance.shape != (2, 2) or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, rtol=0, atol=1e-9)
            or np.min(np.linalg.eigvalsh(covariance)) <= 0):
        raise ValueError('invalid_observation_candidate')
    # MAVLink upper-triangle packing of [x, y, z, roll, pitch, yaw].
    pose_covariance = [0.]*21
    for index, value in ((0, covariance[0, 0]), (1, covariance[0, 1]),
                         (6, covariance[1, 1]), (11, 1e6),
                         (15, 1e6), (18, 1e6), (20, 1e6)):
        pose_covariance[index] = float(value)
    return dict(message='ODOMETRY', time_usec=sender_sample_us,
                time_clock_domain=sample_clock_domain,
                timesync_clock_domain=settings.sender_clock_domain,
                frame_id='MAV_FRAME_LOCAL_NED', child_frame_id='MAV_FRAME_BODY_FRD',
                x=float(position[0]), y=float(position[1]), z=float(position[2]),
                q=[float('nan')]*4, vx=float('nan'), vy=float('nan'), vz=float('nan'),
                rollspeed=float('nan'), pitchspeed=float('nan'), yawspeed=float('nan'),
                pose_covariance=pose_covariance,
                velocity_covariance=[float('nan')]+[0.]*20,
                reset_counter=0, estimator_type='MAV_ESTIMATOR_TYPE_VISION',
                quality=0, transmission_gate='ready', sender_handoff_ready=True,
                transmitted=False)


def send_odometry_fields(mav, enums, fields, last_sent_sample_us=None):
    """Hand one fresh field set to pymavlink; no acknowledgement is implied.

    `mav` is `connection.mav` and `enums` is `mavutil.mavlink`. A caller must
    establish a loopback-only SITL connection and verify FC state before using
    `odometry_fields`; this adapter does not open a connection by itself.
    """
    if fields.get('message') != 'ODOMETRY' or fields.get('sender_handoff_ready') is not True:
        raise ValueError('unapproved_odometry_fields')
    if (fields.get('time_clock_domain') not in ('wsl_monotonic_us', 'gazebo_sim_us')
            or fields.get('time_clock_domain') != fields.get('timesync_clock_domain')):
        raise ValueError('sender_measurement_clock_mismatch')
    stamp = fields.get('time_usec')
    if (type(stamp) is not int or stamp <= 0 or last_sent_sample_us is not None
            and stamp <= last_sent_sample_us):
        raise ValueError('duplicate_or_old_odometry_sample')
    mav.odometry_send(stamp, enums.MAV_FRAME_LOCAL_NED, enums.MAV_FRAME_BODY_FRD,
                      fields['x'], fields['y'], fields['z'], fields['q'],
                      fields['vx'], fields['vy'], fields['vz'],
                      fields['rollspeed'], fields['pitchspeed'], fields['yawspeed'],
                      fields['pose_covariance'], fields['velocity_covariance'],
                      fields['reset_counter'], enums.MAV_ESTIMATOR_TYPE_VISION,
                      fields['quality'])
    return stamp
