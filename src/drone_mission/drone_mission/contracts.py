"""Validate local flight trials against the Platform v1 coordinate contract."""

from dataclasses import dataclass
from datetime import datetime
import math


LAYOUT = 'warehouse-rectangle-6p3x4p6-z2p2-20261004'


def finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class Settings:
    execute: bool = False
    full_mission: bool = False
    takeoff_settle_s: float = 2.0
    observation_recovery_s: float = 3.0
    observation_settle_s: float = .5
    scan_timeout_s: float = 20.0
    scan_speed_m_s: float = .05
    layout_confirmed: bool = False
    alignment_confirmed: bool = False
    fusion_confirmed: bool = False
    timing_confirmed: bool = False
    sensor_mount_confirmed: bool = False
    takeoff_settings_confirmed: bool = False
    heading_control_confirmed: bool = False
    expected_ekf2_mag_type: int = 0
    expected_mis_takeoff_alt_m: float = 0.6
    enu_yaw_deg: float = 0.0
    enu_offset_x_m: float = 0.0
    enu_offset_y_m: float = 0.0
    expected_ev_delay_ms: float = 0.0
    expected_ev_pos_x_m: float = 0.0
    expected_ev_pos_y_m: float = 0.0
    expected_ev_pos_z_m: float = 0.0
    bounds_xy_m: tuple = (0.5, 5.3, 0.5, 3.9)
    max_leg_m: float = 1.0
    speed_m_s: float = 0.3
    web_timeout_s: float = 2.0
    state_timeout_s: float = 2.5
    pose_timeout_s: float = .2
    command_timeout_s: float = 3.0
    takeoff_timeout_s: float = 30.0
    leg_timeout_s: float = 20.0
    landing_timeout_s: float = 30.0
    request_ttl_s: float = 30.0
    drone_id: str = '5'
    layout_id: str = LAYOUT

    def __post_init__(self):
        if type(self.expected_ekf2_mag_type) is not int or self.expected_ekf2_mag_type not in (0, 1, 6):
            raise ValueError('supported_native_heading_policy_required')
        for name in ('execute', 'full_mission', 'layout_confirmed', 'alignment_confirmed', 'fusion_confirmed',
                     'takeoff_settings_confirmed', 'heading_control_confirmed', 'timing_confirmed', 'sensor_mount_confirmed'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name + '_must_be_boolean')
        if (len(self.bounds_xy_m) != 4 or not finite(*self.bounds_xy_m)
                or self.bounds_xy_m[0] >= self.bounds_xy_m[1]
                or self.bounds_xy_m[2] >= self.bounds_xy_m[3]):
            raise ValueError('invalid_trial_bounds')
        for name in ('expected_mis_takeoff_alt_m', 'max_leg_m', 'speed_m_s',
                     'web_timeout_s', 'state_timeout_s', 'pose_timeout_s', 'command_timeout_s',
                     'takeoff_timeout_s', 'leg_timeout_s', 'landing_timeout_s',
                     'request_ttl_s', 'takeoff_settle_s', 'observation_recovery_s',
                     'observation_settle_s', 'scan_timeout_s', 'scan_speed_m_s'):
            if not finite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError('positive_setting_required:' + name)
        if self.scan_speed_m_s > .1:
            raise ValueError('scan_speed_m_s_must_not_exceed_0p1')
        if not finite(self.enu_yaw_deg, self.enu_offset_x_m, self.enu_offset_y_m,
                      self.expected_ev_delay_ms, self.expected_ev_pos_x_m,
                      self.expected_ev_pos_y_m, self.expected_ev_pos_z_m):
            raise ValueError('invalid_alignment')
        if self.drone_id not in ('5', '6') or not self.layout_id:
            raise ValueError('device_and_layout_required')

    def inside(self, xy):
        return (len(xy) == 2 and finite(*xy)
                and self.bounds_xy_m[0] <= xy[0] <= self.bounds_xy_m[1]
                and self.bounds_xy_m[2] <= xy[1] <= self.bounds_xy_m[3])


def parse_request(payload, settings, wall_s):
    if (not isinstance(payload, dict) or payload.get('ok') is not True
            or payload.get('contract_version') != '1.0'
            or str(payload.get('drone_id')) != settings.drone_id):
        raise ValueError('invalid_device_contract')
    action = payload.get('control_action')
    if action is None:
        return None
    if action not in ('start', 'land', 'return_to_home'):
        raise ValueError('unsupported_control_action')
    request_id = payload.get('control_request_id')
    if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
        raise ValueError('control_request_id_required')
    requested = payload.get('control_requested_at')
    try:
        stamp = datetime.fromisoformat(requested.replace('Z', '+00:00'))
        if stamp.tzinfo is None:
            raise ValueError('timezone_required')
        requested_s = stamp.timestamp()
    except (AttributeError, TypeError, ValueError):
        raise ValueError('control_requested_at_required') from None
    if not 0 <= wall_s - requested_s <= settings.request_ttl_s:
        raise ValueError('expired_or_future_control_request')
    request = dict(action=action, request_id=request_id, requested_at=requested,
                   requested_s=requested_s)
    if action != 'start':
        return request
    expected = dict(coordinate_frame='UWB_ANCHOR_LOCAL', origin='A1',
                    x_axis='A1_TO_A2', y_axis='A1_TO_A3',
                    z_axis='UP_FROM_FLOOR', unit='meter')
    if any(payload.get(k) != v for k, v in expected.items()):
        raise ValueError('coordinate_contract_mismatch')
    if payload.get('anchor_layout_id') != settings.layout_id:
        raise ValueError('anchor_layout_mismatch')
    if payload.get('status') != 'ACTIVE':
        raise ValueError('active_mission_required')
    for name in ('mission_code', 'route_revision'):
        if not isinstance(payload.get(name), str) or not payload[name]:
            raise ValueError(name + '_required')
    if type(payload.get('mission_db_id')) is not int or payload['mission_db_id'] <= 0:
        raise ValueError('mission_db_id_required')
    tasks = payload.get('route_tasks')
    if not isinstance(tasks, list) or not 1 <= len(tasks) <= 20:
        raise ValueError('one_to_twenty_waypoints_required')
    waypoints, ids = [], set()
    for item in tasks:
        if not isinstance(item, dict):
            raise ValueError('invalid_waypoint')
        point_id = item.get('id')
        if not isinstance(point_id, str) or not point_id or point_id in ids:
            raise ValueError('unique_waypoint_ids_required')
        kind = item.get('type', 'waypoint')
        if kind not in (('waypoint', 'hover', 'scan') if settings.full_mission else ('waypoint', 'hover')):
            raise ValueError('flight_trial_supports_waypoint_and_hover')
        xy = (item.get('x'), item.get('y'))
        if not settings.inside(xy):
            raise ValueError('waypoint_outside_trial_bounds')
        ids.add(point_id)
        point = dict(id=point_id, xy=tuple(map(float, xy)), type=kind)
        if settings.full_mission:
            yaw = item.get('yaw_deg')
            dwell = item.get('dwell_s', 1.)
            if ((yaw is not None and not finite(yaw)) or not finite(dwell) or not 0 <= dwell <= 30):
                raise ValueError('invalid_waypoint_yaw_or_dwell')
            point.update(yaw_deg=yaw, dwell_s=float(dwell))
            if kind == 'scan':
                # Web scan x/y identifies the label, never the aircraft target.
                staging = item.get('staging_xy_m')
                if not isinstance(staging, list) or not settings.inside(staging):
                    raise ValueError('scan_validated_staging_xy_required')
                if not isinstance(item.get('path_validation_ref'), str) or not item['path_validation_ref']:
                    raise ValueError('scan_path_validation_ref_required')
                point.update(label_xy=point['xy'], xy=tuple(map(float, staging)),
                             path_validation_ref=item['path_validation_ref'])
                if type(item.get('marker_id')) is not int or item['marker_id'] < 0 or yaw is None:
                    raise ValueError('scan_marker_and_yaw_required')
                for name in ('label_id', 'calibration_ref', 'mounting_ref'):
                    if not isinstance(item.get(name), str) or not item[name]:
                        raise ValueError('scan_'+name+'_required')
                    point[name] = item[name]
                point['marker_id'] = item['marker_id']
        waypoints.append(point)
    return dict(request, mission_db_id=payload['mission_db_id'],
                mission_code=payload['mission_code'], route_revision=payload['route_revision'],
                waypoints=waypoints)
