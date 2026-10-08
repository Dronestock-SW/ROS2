"""Fresh, read-only observations for Platform telemetry."""

import math
import time


class Observations:
    def __init__(self):
        self.pose = None
        self.battery = None
        self.height = self.fc = self.position = self.mission = None

    def receive_pose(self, x, y, frame, stamp_ns, now_ns=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        if (frame != 'uwb_map' or not all(map(math.isfinite, (x, y)))
                or stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= 500_000_000):
            self.pose = None
            return
        self.pose = (float(x), float(y), stamp_ns, time.monotonic())

    def receive_battery(self, percentage, voltage, connected=True):
        if not connected:
            self.battery = None
            return
        percent = percentage * 100 if math.isfinite(percentage) and 0 <= percentage <= 1 else None
        volts = voltage if math.isfinite(voltage) and voltage > 0 else None
        self.battery = (percent, volts, time.monotonic()) if percent is not None or volts is not None else None

    def receive_height(self, z, frame, stamp_ns):
        if frame == 'uwb_map' and math.isfinite(z) and 0 <= time.time_ns()-stamp_ns <= 200_000_000:
            self.height = (z, stamp_ns, time.monotonic())
        else:
            self.height = None

    def receive_fc(self, connected, armed, mode):
        self.fc = (connected, armed, mode, time.monotonic())

    def receive_position(self, xyz, frame, stamp_ns):
        if frame == 'map' and all(map(math.isfinite, xyz)) and 0 <= time.time_ns()-stamp_ns <= 200_000_000:
            self.position = (list(xyz), stamp_ns, time.monotonic())
        else:
            self.position = None

    def receive_mission(self, value):
        if isinstance(value, dict):
            self.mission = (dict(value), time.monotonic())

    def fields(self):
        result = {'fix': False, 'telemetry_verified': False, 'x': None, 'y': None,
                  'uwb_age_ms': None, 'battery': None, 'battery_voltage': None,
                  'current_z_m': None, 'current_z_source': None,
                  'fc_connected': None, 'fc_armed': None, 'fc_mode': None,
                  'px4_position_enu_m': None}
        now = time.monotonic()
        if self.pose is not None:
            x, y, stamp, received = self.pose
            age_ms = round(max(now-received, (time.time_ns()-stamp)/1e9)*1000)
            if 0 <= age_ms <= 500:
                result.update(fix=True, telemetry_verified=True, x=x, y=y,
                              uwb_age_ms=age_ms)
        if self.battery is not None:
            percent, volts, received = self.battery
            if 0 <= now - received <= 3:
                result.update(battery=percent, battery_voltage=volts)
        if self.height is not None:
            z, stamp, received = self.height
            if 0 <= max(now-received, (time.time_ns()-stamp)/1e9) <= .2:
                result.update(current_z_m=z, current_z_source='tof_imu_uwb_antenna',
                              current_z_reference='floor', coordinate_frame='UWB_ANCHOR_LOCAL')
        if self.position is not None:
            xyz, stamp, received = self.position
            if 0 <= max(now-received, (time.time_ns()-stamp)/1e9) <= .2:
                result.update(px4_position_enu_m=xyz, px4_position_source='px4_ekf2',
                              px4_position_frame='map')
        if self.fc is not None:
            connected, armed, mode, received = self.fc
            if 0 <= now-received <= 2.5:
                result.update(fc_connected=connected, fc_armed=armed, fc_mode=mode)
        if self.mission is not None:
            value, received = self.mission
            if 0 <= now-received <= .5:
                for key in ('mission_db_id', 'mission_code', 'route_revision',
                            'active_waypoint_index', 'active_waypoint_id', 'flight_control_enabled',
                            'mission_complete', 'landing_verified', 'control_ack', 'target_applied',
                            'px4_map_xy_m'):
                    if key in value:
                        result[key] = value[key]
                result.update(flight_state=value.get('state'), flight_reason=value.get('reason'),
                              target_validation=value.get('validation'))
        return result
