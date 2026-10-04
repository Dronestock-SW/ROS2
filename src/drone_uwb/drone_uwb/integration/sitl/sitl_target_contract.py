"""Horizontal SITL reposition contract; altitude selection belongs to PX4.

PX4 c4e4ef98e9 uses an azimuthal-equidistant local map (R=6371000 m).
COMMAND_INT preserves E7 coordinates, unlike float COMMAND_LONG lat/lon.
No socket, arming, takeoff or controller is created by this module.
"""
from dataclasses import dataclass
import math

from drone_uwb.integration.sitl.sitl_odometry_contract import map_reference_xy_to_ned


EARTH_RADIUS_M = 6371000.


def _finite(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class PX4GlobalReference:
    latitude_deg: float
    longitude_deg: float
    reference_timestamp_us: int

    def __post_init__(self):
        if (not _finite((self.latitude_deg, self.longitude_deg))
                or not -89.9 < self.latitude_deg < 89.9
                or not -180 <= self.longitude_deg <= 180
                or type(self.reference_timestamp_us) is not int
                or self.reference_timestamp_us <= 0):
            raise ValueError('valid_px4_global_reference_required')

    @classmethod
    def from_message(cls, message):
        if (message.get_type() != 'GPS_GLOBAL_ORIGIN'
                or message.get_srcSystem() != 1 or message.get_srcComponent() != 1):
            raise ValueError('px4_global_origin_required')
        return cls(message.latitude/1e7, message.longitude/1e7, message.time_usec)


def ned_to_global(north, east, reference):
    """Inverse spherical azimuthal-equidistant projection, limited to a lab."""
    if not _finite((north, east)) or math.hypot(north, east) > 10000.:
        raise ValueError('finite_local_target_within_10km_required')
    lat0, lon0 = map(math.radians, (reference.latitude_deg, reference.longitude_deg))
    distance = math.hypot(north, east)
    if distance == 0:
        return reference.latitude_deg, reference.longitude_deg
    angular_distance = distance/EARTH_RADIUS_M
    bearing = math.atan2(east, north)
    sine, cosine = math.sin(angular_distance), math.cos(angular_distance)
    latitude = math.asin(math.sin(lat0)*cosine + math.cos(lat0)*sine*math.cos(bearing))
    longitude = lon0 + math.atan2(math.sin(bearing)*sine*math.cos(lat0),
                                  cosine-math.sin(lat0)*math.sin(latitude))
    return math.degrees(latitude), (math.degrees(longitude)+180.) % 360. - 180.


def global_to_ned(latitude, longitude, reference):
    """Forward projection, used to expose E7 target quantization separately."""
    if (not _finite((latitude, longitude)) or not -90 <= latitude <= 90
            or not -180 <= longitude <= 180):
        raise ValueError('invalid_global_target')
    lat0, lon0 = map(math.radians, (reference.latitude_deg, reference.longitude_deg))
    lat, lon = map(math.radians, (latitude, longitude))
    delta = lon-lon0
    north = math.cos(lat0)*math.sin(lat)-math.sin(lat0)*math.cos(lat)*math.cos(delta)
    east = math.cos(lat)*math.sin(delta)
    dot = math.sin(lat0)*math.sin(lat)+math.cos(lat0)*math.cos(lat)*math.cos(delta)
    sine = math.hypot(north, east)
    angle = math.atan2(sine, dot)
    if angle*EARTH_RADIUS_M > 10000.:
        raise ValueError('global_target_outside_local_trial')
    factor = EARTH_RADIUS_M * (angle/sine if sine else 1.)
    return north*factor, east*factor


@dataclass(frozen=True)
class SITLTargetSettings:
    enabled: bool = False
    sitl_session_confirmed: bool = False
    observation_fusion_confirmed: bool = False
    # Deliberately inside the configured anchor area; these are trial limits.
    bounds_xy_m: tuple = (.5, 5.3, .5, 3.9)
    speed_m_s: float = .3
    max_leg_m: float = 1.
    state_max_age_s: float = .2
    uwb_max_age_s: float = .25

    def __post_init__(self):
        for name in ('enabled', 'sitl_session_confirmed', 'observation_fusion_confirmed'):
            if type(getattr(self, name)) is not bool:
                raise ValueError('invalid_confirmation_flag:'+name)
        if (len(self.bounds_xy_m) != 4 or not _finite(self.bounds_xy_m)
                or self.bounds_xy_m[0] >= self.bounds_xy_m[1]
                or self.bounds_xy_m[2] >= self.bounds_xy_m[3]):
            raise ValueError('invalid_trial_bounds')
        for name in ('speed_m_s', 'max_leg_m', 'state_max_age_s', 'uwb_max_age_s'):
            value = getattr(self, name)
            if not _finite((value,)) or value <= 0:
                raise ValueError('positive_setting_required:'+name)


@dataclass(frozen=True)
class NavigationReadiness:
    """Snapshot assembled from current FC evidence by the live caller.

    Ages include transport delay. Origin identity/reset counter must match the
    session that established observation alignment. Booleans are not config.
    """
    state_age_s: float
    uwb_age_s: float
    origin_timestamp_us: int
    reset_counter: int
    armed: bool = False
    in_air: bool = False
    hold_mode: bool = False
    xy_valid: bool = False
    vxy_valid: bool = False
    attitude_solution_valid: bool = False
    global_position_valid: bool = False


def navigation_gate(settings, alignment, reference, state, expected_reset_counter):
    for name in ('enabled', 'sitl_session_confirmed', 'observation_fusion_confirmed'):
        if getattr(settings, name) is not True:
            return name+'_required'
    for name in ('alignment_confirmed', 'px4_reference_confirmed'):
        if getattr(alignment, name) is not True:
            return name+'_required'
    for name in ('armed', 'in_air', 'hold_mode', 'xy_valid', 'vxy_valid',
                 'attitude_solution_valid', 'global_position_valid'):
        if getattr(state, name) is not True:
            return name+'_required'
    for age, limit in ((state.state_age_s, settings.state_max_age_s),
                       (state.uwb_age_s, settings.uwb_max_age_s)):
        if not _finite((age,)) or not 0 <= age <= limit:
            return 'stale_or_future_navigation_input'
    if (type(state.origin_timestamp_us) is not int
            or state.origin_timestamp_us != reference.reference_timestamp_us):
        return 'px4_global_reference_changed'
    if (type(expected_reset_counter) is not int or not 0 <= expected_reset_counter <= 255
            or type(state.reset_counter) is not int or state.reset_counter != expected_reset_counter):
        return 'px4_reference_reset'
    return 'ready'


def reposition_fields(target_map_xy_m, current_map_xy_m, settings, alignment,
                      reference, state, *, expected_reset_counter):
    """Build one reference-point target, using the observation's same transform."""
    reason = navigation_gate(settings, alignment, reference, state, expected_reset_counter)
    if reason != 'ready':
        raise ValueError(reason)
    low_x, high_x, low_y, high_y = settings.bounds_xy_m
    for point in (target_map_xy_m, current_map_xy_m):
        if len(point) != 2 or not _finite(point):
            raise ValueError('finite_map_xy_required')
        if not (low_x <= point[0] <= high_x and low_y <= point[1] <= high_y):
            raise ValueError('outside_trial_bounds')
    if math.dist(target_map_xy_m, current_map_xy_m) > settings.max_leg_m:
        raise ValueError('leg_exceeds_trial_limit')
    north, east = map_reference_xy_to_ned(target_map_xy_m, alignment)
    latitude, longitude = ned_to_global(north, east, reference)
    lat_e7, lon_e7 = round(latitude*1e7), round(longitude*1e7)
    encoded = global_to_ned(lat_e7/1e7, lon_e7/1e7, reference)
    return dict(message='COMMAND_INT', command='MAV_CMD_DO_REPOSITION',
        target_system=1, target_component=1, frame_id='MAV_FRAME_GLOBAL',
        current=0, autocontinue=0, param1=settings.speed_m_s, param2=0.,
        param3=float('nan'), param4=float('nan'), x=lat_e7, y=lon_e7,
        z=float('nan'), target_map_xy_m=list(target_map_xy_m),
        target_ned_xy_m=[north, east], encoded_target_ned_xy_m=list(encoded),
        target_quantization_error_m=math.dist((north, east), encoded),
        reference_point='px4_reference', origin_timestamp_us=reference.reference_timestamp_us,
        reset_counter=expected_reset_counter, altitude_policy='px4_selects_current_altitude',
        sender_handoff_ready=True, transmitted=False, target_applied=False, arrival_valid=False)


def send_reposition_fields(mav, dialect, fields):
    """Hand a fresh approved packet to the caller's SITL transport, once.

    The runtime must re-evaluate readiness immediately before calling this.
    No ACK, target application or arrival is implied by a successful write.
    """
    if (fields.get('message') != 'COMMAND_INT' or fields.get('command') != 'MAV_CMD_DO_REPOSITION'
            or fields.get('sender_handoff_ready') is not True or fields.get('transmitted') is not False
            or fields.get('target_system') != 1 or fields.get('target_component') != 1
            or fields.get('frame_id') != 'MAV_FRAME_GLOBAL'
            or fields.get('param2') != 0. or not _finite((fields.get('param1'),))
            or fields['param1'] <= 0 or fields.get('current') != 0 or fields.get('autocontinue') != 0
            or any(type(fields.get(name)) is not float or not math.isnan(fields[name])
                   for name in ('param3', 'param4', 'z'))
            or type(fields.get('x')) is not int or not -900000000 <= fields['x'] <= 900000000
            or type(fields.get('y')) is not int or not -1800000000 <= fields['y'] <= 1800000000):
        raise ValueError('unapproved_or_repeated_reposition')
    # Mark an attempted handoff before I/O: a transport exception is ambiguous,
    # so the same object must not silently be retried.
    fields['sender_handoff_ready'] = False
    mav.command_int_send(1, 1, dialect.MAV_FRAME_GLOBAL, dialect.MAV_CMD_DO_REPOSITION,
                         0, 0, fields['param1'], 0., fields['param3'], fields['param4'],
                         fields['x'], fields['y'], fields['z'])
    fields['transmitted'] = True


class RepositionProgress:
    """Observe one command's ACK and actual FC target, separately from arrival.

    A COMMAND_ACK has no waypoint ID. A fresh matching FC target is therefore
    required as additional evidence. This helper sends/retries no commands.
    """

    def __init__(self, fields, dialect, *, sent_host_s, sent_px4_us,
                 sender_system=245, sender_component=191, response_timeout_s=2.):
        if (fields.get('transmitted') is not True or not _finite((sent_host_s, response_timeout_s))
                or response_timeout_s <= 0 or type(sent_px4_us) is not int or sent_px4_us <= 0):
            raise ValueError('transmitted_command_and_send_times_required')
        self.fields, self.dialect = dict(fields), dialect
        self.sent_host_s, self.sent_px4_us = sent_host_s, sent_px4_us
        self.sender_system, self.sender_component = sender_system, sender_component
        self.response_timeout_s = response_timeout_s
        self.accepted = False
        self.ever_target_matched = False
        self.target_sample_ms = self.target_received_s = None
        self.failure = None
        self.last_now = sent_host_s

    def handle(self, message, *, received_host_s, sample_age_s=None):
        if not _finite((received_host_s,)) or received_host_s < self.last_now:
            self.failure = self.failure or 'host_time_reversed'
            return False
        self.last_now = received_host_s
        if (self.failure or message.get_srcSystem() != 1 or message.get_srcComponent() != 1
                or received_host_s < self.sent_host_s):
            return False
        if (not self.accepted or self.target_received_s is None) and (
                received_host_s-self.sent_host_s > self.response_timeout_s):
            self.failure = 'command_response_timeout'
            return False
        d = self.dialect
        if message.get_type() == 'COMMAND_ACK':
            if (message.command != d.MAV_CMD_DO_REPOSITION
                    or getattr(message, 'target_system', None) != self.sender_system
                    or getattr(message, 'target_component', None) != self.sender_component):
                return False
            if message.result == d.MAV_RESULT_ACCEPTED:
                self.accepted = True
            elif message.result != d.MAV_RESULT_IN_PROGRESS:
                self.failure = 'command_rejected:'+str(message.result)
            return True
        if message.get_type() == 'POSITION_TARGET_GLOBAL_INT':
            if (not _finite((sample_age_s,)) or not 0 <= sample_age_s <= .2
                    or type(message.time_boot_ms) is not int
                    or message.time_boot_ms*1000 <= self.sent_px4_us
                    or self.target_sample_ms is not None and message.time_boot_ms <= self.target_sample_ms):
                return False
            if message.coordinate_frame != d.MAV_FRAME_GLOBAL_INT or message.type_mask & 3:
                return False
            self.target_sample_ms = message.time_boot_ms
            # PX4's target stream truncates E7 while COMMAND_INT starts with
            # integer E7; permit one unit of float round-trip truncation.
            matches = (abs(message.lat_int-self.fields['x']) <= 1
                       and abs(message.lon_int-self.fields['y']) <= 1)
            if not matches:
                self.target_received_s = None
                if self.ever_target_matched:
                    self.failure = 'px4_target_changed'
                return False
            self.ever_target_matched = True
            self.target_received_s = received_host_s-sample_age_s
            return True
        return False

    def evaluate(self, now, arrival):
        if not _finite((now,)):
            raise ValueError('finite_host_time_required')
        if now < self.last_now:
            self.failure = self.failure or 'host_time_reversed'
        self.last_now = now
        if (not self.accepted or self.target_received_s is None) and (
                now-self.sent_host_s > self.response_timeout_s):
            self.failure = self.failure or 'command_response_timeout'
        target_fresh = (self.target_received_s is not None
                        and 0 <= now-self.target_received_s <= .5)
        if self.ever_target_matched and not target_fresh:
            self.failure = self.failure or 'px4_target_stream_stale'
        applied = self.accepted and target_fresh and self.failure is None
        # Caller must pass this tick's PX4MissionMonitor result. Never accept
        # demo/truth arrival or a previous waypoint's arrival state.
        evaluated = arrival.get('evaluated_host_s')
        recent_assessment = _finite((evaluated,)) and 0 <= now-evaluated <= .05
        arrived = (applied and recent_assessment and arrival.get('arrival_valid') is True
                   and arrival.get('position_source') == 'px4_ekf2'
                   and arrival.get('reference_point') == 'px4_reference'
                   and arrival.get('target_xy_m') == self.fields['target_map_xy_m'])
        return dict(command_accepted=self.accepted, target_applied=applied,
                    arrival_valid=arrived, reason=self.failure,
                    target_quantization_error_m=self.fields['target_quantization_error_m'],
                    flight_valid=False, fusion_verified=False)
