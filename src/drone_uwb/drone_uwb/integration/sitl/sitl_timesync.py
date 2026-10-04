"""PX4-requested TIMESYNC replies for a future SITL observation connection.

This module opens no sockets. When attached to a live connection, its replies
update PX4's remote-clock filter; they do not set parameters or command flight.
The caller must invalidate observation time mapping on a clock-reset result.
"""
import time

from drone_uwb.integration.gazebo.gazebo_clock import ClockUnavailable


class SITLTimesyncResponder:
    """Use one declared sender clock for TIMESYNC and ODOMETRY stamps."""

    def __init__(self, mav, dialect, *, now_ns=time.monotonic_ns,
                 clock_domain='wsl_monotonic_us'):
        if clock_domain not in ('wsl_monotonic_us', 'gazebo_sim_us'):
            raise ValueError('unsupported_sender_clock_domain')
        fields = set(dialect.MAVLink_timesync_message.fieldnames)
        if not {'tc1', 'ts1'} <= fields:
            raise ValueError('unsupported_timesync_dialect')
        if ('target_system' in fields) != ('target_component' in fields):
            raise ValueError('incomplete_timesync_target_fields')
        self.mav = mav
        self.target_fields_available = 'target_system' in fields
        self.now_ns = now_ns
        self.clock_domain = clock_domain
        self.epoch = 0
        self.last_request_ns = None
        self.last_sender_ns = None
        self.suspended_reason = None

    def reset_epoch(self):
        """Caller invokes this only after invalidating its observation mapping."""
        self.epoch += 1
        self.last_request_ns = None
        self.last_sender_ns = None
        self.suspended_reason = None

    def handle(self, message):
        """Respond only to PX4 requests; return a loggable disposition."""
        result = dict(action='ignored', reason='not_px4_request', epoch=self.epoch,
                      sender_clock_domain=self.clock_domain,
                      mapping_invalidation_required=False,
                      clock_mapping_verified=False, observations_sent=False)
        if (message.get_type() != 'TIMESYNC'
                or message.get_srcSystem() != 1 or message.get_srcComponent() != 1):
            return result
        tc1, request_ns = getattr(message, 'tc1', None), getattr(message, 'ts1', None)
        if type(tc1) is not int or tc1 != 0:
            result['reason'] = 'not_timesync_request'
            return result
        if type(request_ns) is not int or not 0 < request_ns < 2**63:
            result['reason'] = 'invalid_px4_request_time'
            return result
        target_system = getattr(message, 'target_system', None)
        target_component = getattr(message, 'target_component', None)
        if ((target_system is None) != (target_component is None)
                or target_system is not None
                and (target_system not in (0, 245) or target_component not in (0, 191))):
            result['reason'] = 'request_for_other_component'
            return result
        result['px4_request_ns'] = request_ns
        if self.suspended_reason:
            result.update(action='suspended', reason=self.suspended_reason,
                          mapping_invalidation_required=True)
            return result
        if request_ns == self.last_request_ns:
            result['reason'] = 'duplicate_px4_request'
            return result
        if self.last_request_ns is not None and request_ns < self.last_request_ns:
            self.suspended_reason = 'px4_request_time_reversed'
            result.update(action='suspended', reason=self.suspended_reason,
                          mapping_invalidation_required=True)
            return result
        try:
            sender_ns = self.now_ns()
        except ClockUnavailable as exc:
            if exc.reset_required:
                self.suspended_reason = str(exc)
            result.update(action='clock_unavailable', reason=str(exc),
                          mapping_invalidation_required=exc.reset_required)
            return result
        if (self.clock_domain == 'gazebo_sim_us' and self.last_sender_ns is not None
                and sender_ns == self.last_sender_ns
                and not self.suspended_reason):
            result['reason'] = 'simulation_clock_not_advanced'
            return result
        if (type(sender_ns) is not int or not 0 < sender_ns < 2**63
                or self.last_sender_ns is not None and sender_ns <= self.last_sender_ns):
            self.suspended_reason = 'sender_time_not_advancing'
        if self.suspended_reason:
            result.update(action='suspended', reason=self.suspended_reason,
                          mapping_invalidation_required=True)
            return result
        try:
            if self.target_fields_available:
                self.mav.timesync_send(sender_ns, request_ns,
                                       target_system=1, target_component=1)
            else:
                # Older common dialects omit target extensions. The live
                # adapter must use one dedicated loopback SITL connection.
                self.mav.timesync_send(sender_ns, request_ns)
        except Exception:
            self.suspended_reason = 'timesync_send_failed'
            raise
        self.last_request_ns = request_ns
        self.last_sender_ns = sender_ns
        result.update(action='responded', reason='ok', sender_reply_ns=sender_ns,
                      reply_target_encoded=self.target_fields_available,
                      request_target_available=target_system is not None)
        return result
