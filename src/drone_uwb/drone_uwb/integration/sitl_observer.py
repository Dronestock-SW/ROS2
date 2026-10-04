"""Single-session Gazebo observation coordinator for an explicit SITL link.

Shadow never touches MAVLink. The other modes read parameters and reply to
TIMESYNC. Only send mode may hand a new observation to the injected transport.
No parameter writes, arming, mode changes or flight commands are implemented.
"""
from dataclasses import dataclass
import math
import time

import numpy as np

from drone_uwb.integration.gazebo_clock import ClockUnavailable
from drone_uwb.integration.sitl_link_probe import PARAMETERS, _name, decode_px4_param
from drone_uwb.integration.sitl_odometry_contract import (
    map_tag_to_px4_ned, odometry_fields, send_odometry_fields)
from drone_uwb.integration.sitl_timesync import SITLTimesyncResponder


PX4_TELEMETRY_TYPES = frozenset({
    'HEARTBEAT', 'ODOMETRY', 'LOCAL_POSITION_NED', 'POSITION_TARGET_LOCAL_NED',
    'ESTIMATOR_STATUS', 'ESTIMATOR_SENSOR_FUSION_STATUS', 'EXTENDED_SYS_STATE',
    'GPS_GLOBAL_ORIGIN', 'POSITION_TARGET_GLOBAL_INT', 'COMMAND_ACK',
})


def log_value(value):
    """JSON logs encode unknown wire values as null; MAVLink keeps its NaNs."""
    if isinstance(value, dict):
        return {str(k): log_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [log_value(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, bytes):
        return value.decode('utf-8', errors='backslashreplace')
    return value


def px4_telemetry_record(message, received_host_ns):
    """Keep native telemetry for later arrival/fusion/trajectory assessment.

    Receipt is not validity or fusion evidence by itself. Preserve native
    frame and clock fields rather than relabeling them as Gazebo/map values.
    """
    if (message.get_srcSystem() != 1 or message.get_srcComponent() != 1
            or message.get_type() not in PX4_TELEMETRY_TYPES):
        return None
    return dict(type=message.get_type(), source_system=1, source_component=1,
                received_host_monotonic_ns=received_host_ns,
                payload=log_value(message.to_dict()),
                payload_coordinates='native_mavlink_frames',
                payload_clock='native_px4_timestamp_fields',
                nonfinite_wire_values_logged_as='null',
                fusion_verified=False, flight_valid=False)


@dataclass(frozen=True)
class SITLObserverSettings:
    model: str = 'A'
    sigma_xy_m: tuple = (.10, .10)
    uncertainty_source: str = 'declared_sitl_trial_uncertainty_uncalibrated'
    uncertainty_reviewed: bool = False
    max_wall_residence_s: float = .15
    min_timesync_replies: int = 10

    def __post_init__(self):
        if self.model not in ('A', 'B', 'C', 'D', 'WLS'):
            raise ValueError('unsupported_observation_model')
        sigma = np.asarray(self.sigma_xy_m, float)
        if sigma.shape != (2,) or not np.isfinite(sigma).all() or np.min(sigma) <= 0:
            raise ValueError('positive_declared_xy_sigma_required')
        if not isinstance(self.uncertainty_source, str) or not self.uncertainty_source.strip():
            raise ValueError('uncertainty_source_required')
        if type(self.uncertainty_reviewed) is not bool:
            raise ValueError('invalid_uncertainty_review_flag')
        if (type(self.max_wall_residence_s) not in (int, float)
                or not math.isfinite(self.max_wall_residence_s)
                or not 0 < self.max_wall_residence_s <= .15):
            raise ValueError('wall_residence_must_be_0_to_150ms')
        if type(self.min_timesync_replies) is not int or self.min_timesync_replies < 1:
            raise ValueError('positive_timesync_warmup_required')


class SITLObserver:
    """One writer owns MAVLink state and sends exactly once per fresh cycle.

    Any reset fault latches for this run. Restart with new alignment/time
    evidence and a new output directory instead of replaying queued samples.
    """

    def __init__(self, settings, runtime, clock, *, mode='shadow', connection=None,
                 dialect=None, host_now_ns=time.monotonic_ns):
        if mode not in ('shadow', 'timesync', 'send'):
            raise ValueError('unknown_sitl_mode')
        if settings.sender_clock_domain != 'gazebo_sim_us':
            raise ValueError('live_observer_requires_gazebo_clock')
        if mode == 'shadow' and connection is not None:
            raise ValueError('shadow_must_not_have_mavlink_connection')
        if mode != 'shadow' and (connection is None or dialect is None):
            raise ValueError('sitl_connection_required')
        self.settings, self.runtime, self.clock = settings, runtime, clock
        self.mode, self.connection, self.dialect = mode, connection, dialect
        self.host_now_ns = host_now_ns
        self.responder = None if mode == 'shadow' else SITLTimesyncResponder(
            connection.mav, dialect, now_ns=clock.now_ns, clock_domain=clock.clock_domain)
        self.params, self.param_receipts = {}, {}
        self.heartbeat_ns = self.odometry_ns = self.last_request_ns = None
        self.px4_sample_us = self.px4_reset_counter = None
        self.last_reply_host_ns = None
        self.timesync_replies = 0
        self.last_cycle_us = self.last_seq = self.last_sent_sample_us = None
        self.link_drained = False
        self.fault = None
        self.message_listener = None

    def suspend(self, reason):
        self.fault = self.fault or reason

    def _fresh(self, stamp, limit_s):
        return stamp is not None and 0 <= (self.host_now_ns()-stamp)/1e9 <= limit_s

    def handle_message(self, message):
        """Read only PX4 1/1, preserving parameter bits and reset evidence."""
        if self.mode == 'shadow':
            return None
        kind = message.get_type()
        if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            return None
        now = self.host_now_ns()
        result = px4_telemetry_record(message, now)
        if result is None:
            result = dict(type=kind, received_host_monotonic_ns=now)
        if kind == 'HEARTBEAT':
            if message.autopilot != self.dialect.MAV_AUTOPILOT_PX4:
                self.suspend('unexpected_autopilot_identity')
            else:
                self.heartbeat_ns = now
            result.update(autopilot=message.autopilot, vehicle_type=message.type)
        elif kind == 'PARAM_VALUE':
            name = _name(message)
            if name not in PARAMETERS:
                return None
            try:
                decoded = decode_px4_param(name, message.param_value, message.param_type,
                    int32_type=self.dialect.MAV_PARAM_TYPE_INT32,
                    real32_type=self.dialect.MAV_PARAM_TYPE_REAL32)
            except ValueError as exc:
                self.params.pop(name, None)
                self.param_receipts.pop(name, None)
                result.update(name=name, reason=str(exc))
            else:
                self.params[name], self.param_receipts[name] = decoded['value'], now
                result.update(decoded)
        elif kind == 'ODOMETRY':
            if message.estimator_type != self.dialect.MAV_ESTIMATOR_TYPE_AUTOPILOT:
                return None
            stamp, reset = message.time_usec, message.reset_counter
            if (type(stamp) is not int or stamp <= 0 or type(reset) is not int
                    or not 0 <= reset <= 255 or message.frame_id != self.dialect.MAV_FRAME_LOCAL_NED):
                self.suspend('invalid_px4_odometry_reference')
            elif self.px4_reset_counter is not None and reset != self.px4_reset_counter:
                self.suspend('px4_reference_reset')
            elif self.px4_sample_us is not None and stamp < self.px4_sample_us:
                self.suspend('px4_time_reversed')
            elif stamp != self.px4_sample_us:
                self.odometry_ns, self.px4_sample_us, self.px4_reset_counter = now, stamp, reset
            result.update(time_usec=stamp, reset_counter=reset,
                          frame_id=message.frame_id, fault=self.fault)
        elif kind == 'TIMESYNC':
            if self.fault or not self._fresh(self.heartbeat_ns, 3.):
                return dict(result, action='ignored', reason=self.fault or 'heartbeat_unavailable')
            try:
                reply = self.responder.handle(message)
            except Exception:
                self.suspend('timesync_send_failed')
                raise
            result.update(reply)
            if reply['mapping_invalidation_required']:
                self.suspend(reply['reason'])
            if reply['action'] == 'responded':
                self.timesync_replies += 1
                self.last_reply_host_ns = now
        elif kind == 'STATUSTEXT':
            result.update(severity=message.severity, text=log_value(message.text))
        elif kind in PX4_TELEMETRY_TYPES:
            # These are evidence streams only. Never feed FC estimates back
            # into the independent UWB range solver or claim EV fusion here.
            pass
        else:
            return None
        return result

    def poll_link(self, max_messages=512):
        """Bounded nonblocking receive, then periodic read-only parameter refresh."""
        if self.connection is None:
            return []
        self.link_drained = False
        events = []
        for _ in range(max_messages):
            message = self.connection.recv_match(blocking=False)
            if message is None:
                break
            event = self.handle_message(message)
            if self.message_listener is not None:
                self.message_listener(message)
            if event is not None:
                events.append(event)
        else:
            # Queue completeness is unknown: do not send a sample ahead of an
            # unprocessed PX4 reset. Resume only after a later full drain.
            events.append(dict(type='receive_backlog', reason='mavlink_queue_not_drained'))
            return events
        self.link_drained = True
        if not self.fault and self._fresh(self.heartbeat_ns, 3.):
            if self.last_request_ns is None or not self._fresh(self.last_request_ns, 2.):
                for name in PARAMETERS:
                    self.connection.mav.param_request_read_send(1, 1, name.encode('ascii'), -1)
                self.last_request_ns = self.host_now_ns()
                events.append(dict(type='parameter_read_requests', names=list(PARAMETERS)))
        return events

    def observe(self, result, *, raw_received_host_ns, mavlink_drained=True):
        """Create a candidate, record every gate, optionally send once.

        The declared XY uncertainty is a trial setting, not a fitted estimator
        covariance. The selected algorithm and its uncertainty stay unchanged
        throughout this run. PX4 estimates are never used to solve UWB.
        """
        stamp, seq = result['time_us'], result['seq']
        event = dict(type='observation', mode=self.mode, time_us=stamp, seq=seq,
                     model=self.runtime.model, transmitted=False, candidate=None,
                     uncertainty_source=self.runtime.uncertainty_source,
                     uncertainty_reviewed=self.runtime.uncertainty_reviewed,
                     confirmations={key: getattr(self.settings, key) for key in
                        ('enabled', 'alignment_confirmed', 'mount_confirmed',
                         'height_confirmed', 'time_mapping_confirmed', 'px4_reference_confirmed')},
                     timesync_replies=self.timesync_replies, flight_valid=False)
        if self.last_cycle_us is not None and (stamp <= self.last_cycle_us or seq <= self.last_seq):
            if stamp < self.last_cycle_us or seq < self.last_seq:
                self.suspend('observation_time_or_sequence_reversed')
            return dict(event, reason=self.fault or 'duplicate_observation')
        self.last_cycle_us, self.last_seq = stamp, seq
        if self.fault:
            return dict(event, reason=self.fault)
        try:
            if result.get('truth_used') is not False or result.get('clock_domain') != 'gazebo_sim_us':
                raise ValueError('sensor_only_gazebo_observation_required')
            model = result['models'][self.runtime.model]
            if model.get('ok') is not True:
                raise ValueError('model_rejected:'+model.get('reason', 'unknown'))
            if self.runtime.model == 'B' and (model.get('fresh_observation_count', 0) < 1
                                               or model.get('newest_sample_us') != stamp):
                raise ValueError('new_range_observation_required')
            height = result.get('height_m')
            if (result.get('height_source') != 'gazebo_tof_imu'
                    or type(height) not in (int, float) or not math.isfinite(height)):
                raise ValueError('sensor_height_required')
            attitude = result.get('attitude_selection', {})
            if attitude.get('reason') != 'ok':
                raise ValueError('reference_attitude:'+attitude.get('reason', 'unavailable'))
            covariance = np.diag(np.asarray(self.runtime.sigma_xy_m, float)**2)
            candidate = map_tag_to_px4_ned([*model['xy_m'], height],
                attitude['quaternion_wxyz'], covariance, self.settings)
            event.update(candidate=candidate, attitude_time_us=attitude['time_us'],
                         height_selection=result['height_selection'],
                         covariance_tag_xy_m2=covariance.tolist())
            if self.mode != 'send':
                return dict(event, reason='shadow_only' if self.mode == 'shadow' else 'timesync_only')
            if not self.runtime.uncertainty_reviewed:
                raise ValueError('declared_uncertainty_review_required')
            if not mavlink_drained or not self.link_drained:
                raise ValueError('mavlink_queue_not_drained')
            wall_age = (self.host_now_ns()-raw_received_host_ns)/1e9
            event['wall_residence_s'] = wall_age
            if not 0 <= wall_age <= self.runtime.max_wall_residence_s:
                raise ValueError('observation_wall_residence_exceeded')
            now_sim_us = self.clock.now_ns()//1000
            age_s = (now_sim_us-stamp)/1e6
            event['sample_age_sim_s'] = age_s
            if not self._fresh(self.odometry_ns, .5):
                raise ValueError('px4_reset_monitor_unavailable_or_stale')
            if (self.timesync_replies < self.runtime.min_timesync_replies
                    or not self._fresh(self.last_reply_host_ns, 2.)):
                raise ValueError('timesync_warmup_or_fresh_reply_required')
            param_age_s = (max((self.host_now_ns()-self.param_receipts[key])/1e9
                                  for key in PARAMETERS) if all(key in self.param_receipts
                                  for key in PARAMETERS) else math.inf)
            event.update(parameters=dict(self.params), param_age_s=log_value(param_age_s),
                         px4_reset_counter=self.px4_reset_counter)
            fields = odometry_fields(candidate, stamp, self.settings, self.params,
                sample_clock_domain='gazebo_sim_us', fc_connected=self._fresh(self.heartbeat_ns, 3.),
                param_age_s=param_age_s, sample_age_s=age_s)
        except ClockUnavailable as exc:
            if exc.reset_required:
                self.suspend(str(exc))
            return dict(event, reason=str(exc))
        except (ValueError, KeyError, TypeError) as exc:
            return dict(event, reason=str(exc))
        try:
            self.last_sent_sample_us = send_odometry_fields(self.connection.mav, self.dialect,
                                                           fields, self.last_sent_sample_us)
        except Exception:
            self.suspend('odometry_send_failed')
            raise
        return dict(event, reason='sent_to_transport', transmitted=True,
                    packet=log_value(fields), nonfinite_wire_values_logged_as='null',
                    fusion_verified=False)
