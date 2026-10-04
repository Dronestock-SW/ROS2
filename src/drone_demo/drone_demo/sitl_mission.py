"""PX4 telemetry adapter for the shared arrival monitor, without flight output.

The caller supplies measured transport/sample ages; this adapter never treats
receipt time as PX4 sample time. UWB and PX4 stamps stay in their own domains.
"""
import math

from drone_demo.mission import MissionMonitor
from drone_uwb.integration.sitl_odometry_contract import ned_xy_to_map_reference
from drone_uwb.processing.gazebo_geometry import rotation_world_body


def _px4(message, kind):
    return (message.get_type() == kind and message.get_srcSystem() == 1
            and message.get_srcComponent() == 1)


class PX4MissionMonitor:
    """Reuse demo dwell/hysteresis with measured PX4 position and velocity.

    Heading/reference resets latch a fault. A new alignment needs a new
    instance, not an automatic continuation toward the previous target.
    """

    def __init__(self, config, alignment, dialect):
        if config.frame_id != 'uwb_map':
            raise ValueError('uwb_map_required')
        self.monitor = MissionMonitor(config, position_source='px4_ekf2')
        self.alignment, self.dialect = alignment, dialect
        self.reset_counter = self.last_px4_sample_us = None
        self.fault = None
        self.last_rejection = 'px4_inputs_missing'

    def set_target(self, x, y):
        return self.monitor.set_target(x, y, 'uwb_map')

    def invalidate(self, reason, *, latch=False):
        self.last_rejection = reason
        if latch:
            self.fault = self.fault or reason
        self.monitor.pose = None
        self.monitor.speed = None
        self.monitor.reset_health()
        return False

    def update_uwb(self, sample_sim_ns, received_s, sample_age_s):
        # Coordinates are unused here: only the independent observation's age
        # enters mission health. They must never replace the EKF position.
        return self.monitor.update('uwb', 0., 0., sample_sim_ns, received_s,
                                   sample_age_s, 'uwb_map')

    def update_px4(self, odometry, estimator, *, received_s, sample_age_s,
                   estimator_age_s):
        d = self.dialect
        if self.fault:
            return False
        if not _px4(odometry, 'ODOMETRY') or not _px4(estimator, 'ESTIMATOR_STATUS'):
            return False
        if odometry.estimator_type != d.MAV_ESTIMATOR_TYPE_AUTOPILOT:
            return False
        limit = self.monitor.config.pose_timeout_s
        if (not self.alignment.alignment_confirmed
                or not self.alignment.px4_reference_confirmed):
            return self.invalidate('frame_alignment_unconfirmed')
        ages = (sample_age_s, estimator_age_s)
        if any(type(age) not in (int, float) or not math.isfinite(age)
               or not 0 <= age <= limit for age in ages):
            return self.invalidate('px4_input_stale_or_future')
        required = (d.ESTIMATOR_ATTITUDE | d.ESTIMATOR_VELOCITY_HORIZ
                    | d.ESTIMATOR_POS_HORIZ_REL)
        forbidden = d.ESTIMATOR_CONST_POS_MODE | d.ESTIMATOR_ACCEL_ERROR
        if estimator.flags & required != required or estimator.flags & forbidden:
            return self.invalidate('px4_xy_velocity_or_attitude_invalid')
        stamp, reset = odometry.time_usec, odometry.reset_counter
        if (type(stamp) is not int or stamp <= 0 or type(reset) is not int
                or not 0 <= reset <= 255 or type(estimator.time_usec) is not int
                or estimator.time_usec <= 0
                or abs(estimator.time_usec-stamp)/1e6 > limit):
            return self.invalidate('px4_status_time_mismatch')
        if self.reset_counter is not None and reset != self.reset_counter:
            return self.invalidate('px4_reference_reset', latch=True)
        if self.last_px4_sample_us is not None and stamp < self.last_px4_sample_us:
            return self.invalidate('px4_time_reversed', latch=True)
        if stamp == self.last_px4_sample_us:
            return False
        if odometry.frame_id != d.MAV_FRAME_LOCAL_NED:
            return self.invalidate('px4_position_frame_unsupported')
        try:
            px, py = float(odometry.x), float(odometry.y)
            vx, vy, vz = float(odometry.vx), float(odometry.vy), float(odometry.vz)
            if not all(math.isfinite(v) for v in (px, py, vx, vy, vz)):
                raise ValueError('nonfinite_px4_state')
            if odometry.child_frame_id == d.MAV_FRAME_BODY_FRD:
                # ODOMETRY q rotates its child/body frame into the parent NED.
                q = tuple(float(v) for v in odometry.q)
                if len(q) != 4 or abs(math.hypot(*q)-1.) > .01:
                    raise ValueError('invalid_px4_attitude')
                velocity_ned = rotation_world_body(q) @ [vx, vy, vz]
                vx, vy = float(velocity_ned[0]), float(velocity_ned[1])
            elif odometry.child_frame_id != d.MAV_FRAME_LOCAL_NED:
                raise ValueError('px4_velocity_frame_unsupported')
            position_map = ned_xy_to_map_reference((px, py), self.alignment)
            # A velocity has no translation. Remove the transformed origin.
            origin = ned_xy_to_map_reference((0., 0.), self.alignment)
            endpoint = ned_xy_to_map_reference((vx, vy), self.alignment)
            velocity_map = [endpoint[i]-origin[i] for i in range(2)]
        except (ValueError, TypeError, OverflowError) as exc:
            return self.invalidate(str(exc))
        accepted = self.monitor.update('pose', *position_map, stamp*1000,
            received_s, sample_age_s, 'uwb_map', velocity_xy=velocity_map)
        if accepted:
            self.last_px4_sample_us, self.reset_counter = stamp, reset
            self.last_rejection = None
        return accepted

    def evaluate(self, now):
        previous = self.monitor.last_evaluation_s
        if previous is not None and now < previous:
            self.invalidate('host_time_reversed', latch=True)
        result = self.monitor.evaluate(now)
        if self.fault:
            result.update(state='DEGRADED', reason=self.fault, arrival_valid=False)
        result.update(source='simulation', reference_point='px4_reference',
                      evaluated_host_s=now,
                      position_map_xy_m=([self.monitor.pose.x, self.monitor.pose.y]
                                         if self.monitor.pose is not None else None),
                      px4_sample_time_us=(self.monitor.pose.stamp_ns//1000
                                          if self.monitor.pose is not None else None),
                      px4_reset_counter=self.reset_counter,
                      last_input_rejection=self.last_rejection,
                      fusion_verified=False, flight_valid=False)
        return result
