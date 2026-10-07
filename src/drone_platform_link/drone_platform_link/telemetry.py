"""Fresh observation metadata; source time and receiver age stay distinct."""
import math
import time

SOURCES = {
    'uwb_xy': ('/uwb_pose', 'uwb_map', 'uwb_antenna', 'observation'),
    'btf_xy': ('/uwb/btf_pose', 'uwb_map', 'uwb_antenna', 'observation'),
    'px4_local': ('/mavros/local_position/pose', 'map', 'fc', 'state_estimate'),
}


class Observations:
    def __init__(self, source='uwb_xy'):
        if source not in SOURCES:
            raise ValueError('unsupported_pose_source')
        self.source = source
        self.pose = None
        self.battery = None
        self.last_stamp_ns = None

    def receive_pose(self, x, y, frame, stamp_ns, now_ns=None, z=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        values = (x, y, z) if self.source == 'px4_local' else (x, y)
        if (frame != SOURCES[self.source][1]
                or not all(type(v) in (int,float) and math.isfinite(v) for v in values)
                or stamp_ns <= 0 or not 0 <= now_ns-stamp_ns <= 500_000_000):
            self.pose = None
            return False
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return False
        self.last_stamp_ns = stamp_ns
        self.pose = dict(x=float(x),y=float(y),z=float(z) if self.source=='px4_local' else None,
            stamp_ns=stamp_ns,received_ns=now_ns,received_mono=time.monotonic(),
            initial_age_ms=(now_ns-stamp_ns)/1e6)
        return True

    def receive_battery(self, percentage, voltage, connected=True):
        if not connected:
            self.battery = None
            return
        percent = percentage*100 if math.isfinite(percentage) and 0 <= percentage <= 1 else None
        volts = voltage if math.isfinite(voltage) and voltage > 0 else None
        self.battery = (percent,volts,time.monotonic()) if percent is not None or volts is not None else None

    def fields(self):
        topic, _, reference, kind = SOURCES[self.source]
        result = dict(fix=False,telemetry_verified=False,x=None,y=None,current_z_m=None,
            current_z_source=None,current_z_trusted=False,xyz_valid=False,
            uwb_age_ms=None,source_age_ms=None,source_stamp_ns=None,source_received_at_ms=None,
            pose_source=self.source,pose_topic=topic,position_kind=kind,
            coordinate_frame='px4_local_enu' if self.source=='px4_local' else 'uwb_map',
            position_reference=reference,xy_source=self.source,z_source=None,
            battery=None,battery_voltage=None)
        now = time.monotonic()
        if self.pose is not None:
            p=self.pose
            elapsed=(now-p['received_mono'])*1000
            age=p['initial_age_ms']+elapsed
            result.update(source_age_ms=round(age,3),source_stamp_ns=p['stamp_ns'],
                          source_received_at_ms=p['received_ns']//1_000_000)
            if self.source != 'px4_local':
                result['uwb_age_ms']=round(age,3)
            if elapsed >= 0 and 0 <= age <= 500:
                result.update(fix=True,telemetry_verified=True,x=p['x'],y=p['y'])
                if self.source=='px4_local':
                    result.update(current_z_m=p['z'],current_z_source='px4_local',
                                  current_z_trusted=True,z_source='px4_local',xyz_valid=True)
        if self.battery is not None:
            percent,volts,received=self.battery
            if 0 <= now-received <= 3:
                result.update(battery=percent,battery_voltage=volts)
        return result
