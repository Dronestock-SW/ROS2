"""Dedicated horizontal LiDAR MAVLink contract, independent of flight mode."""
import math
import numpy as np

SOURCE_SYSTEM = 255
SOURCE_COMPONENT = 198
NODE_NAME = 'lidar_velocity_bridge'
RAW_WRITERS = {'mavros', 'uwb_local_odometry_bridge', 'px4_fixed_sensor_readback', NODE_NAME}
PARAMETERS = ('EKF2_LID2_VER', 'EKF2_LID2_EN', 'EKF2_LID2_AGE', 'EKF2_AVEL_DELAY')


def parameter_gate(settings, parameters):
    if not settings.enabled:
        return 'disabled'
    geometry = settings.geometry_gate()
    if geometry != 'ready':
        return geometry
    if parameters.get('EKF2_LID2_VER') != 1:
        return 'dedicated_lidar2d_firmware_required'
    if parameters.get('EKF2_LID2_EN') != 1:
        return 'px4_lidar2d_not_enabled'
    age = parameters.get('EKF2_LID2_AGE')
    if type(age) not in (float, int) or not settings.max_observation_age_s*1000 <= age <= 200:
        return 'px4_lidar2d_age_mismatch'
    delay = parameters.get('EKF2_AVEL_DELAY')
    if type(delay) not in (float, int) or abs(delay-settings.expected_avel_delay_ms) > .01:
        return 'px4_auxiliary_delay_mismatch'
    return 'ready'


def velocity_fields(observation, *, now_ns, last_stamp_ns, max_age_s):
    """Only NE are observed; all unobserved fields, including vz, are NaN."""
    if observation.get('accepted') is not True or observation.get('observed_axes') != ['x','y']:
        raise ValueError('accepted_horizontal_observation_required')
    if (observation.get('z_observed') is not False or observation.get('reference_point') != 'fc_imu'
            or observation.get('frame_id') != 'local_ned'):
        raise ValueError('horizontal_imu_origin_contract_required')
    stamp = observation.get('representative_stamp_ns')
    if type(stamp) is not int or stamp <= 0 or not 0 <= (now_ns-stamp)/1e9 <= max_age_s:
        raise ValueError('lidar_observation_stale_or_future')
    if last_stamp_ns is not None and stamp <= last_stamp_ns:
        raise ValueError('lidar_observation_duplicate')
    v = np.asarray(observation['velocity_ne_m_s'], dtype=float)
    c = np.asarray(observation['covariance_ne_m2_s2'], dtype=float)
    if (v.shape != (2,) or c.shape != (2,2) or not np.isfinite(v).all()
            or not np.isfinite(c).all() or not np.allclose(c,c.T,rtol=1e-9,atol=1e-12)
            or np.linalg.eigvalsh(c).min() <= 0):
        raise ValueError('invalid_lidar_velocity_or_covariance')
    reset = observation.get('reset_counter')
    if type(reset) is not int or not 0 <= reset <= 255:
        raise ValueError('invalid_lidar_reset_counter')
    packed = [math.nan]*21
    packed[0], packed[1], packed[6] = float(c[0,0]), float(c[0,1]), float(c[1,1])
    return dict(time_usec=stamp//1000, x=math.nan, y=math.nan, z=math.nan,
        q=[math.nan]*4, vx=float(v[0]), vy=float(v[1]), vz=math.nan,
        rollspeed=math.nan, pitchspeed=math.nan, yawspeed=math.nan,
        pose_covariance=[math.nan]*21, velocity_covariance=packed,
        reset_counter=reset, quality=1)
