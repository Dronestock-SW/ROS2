"""Read FC feedback when MAVROS global/setpoint plugins are disabled."""
from drone_uwb.integration.sitl.sitl_target_contract import PX4GlobalReference


def feedback(packet, clock, now, ros_ns):
    if packet.get_srcSystem() != 1 or packet.get_srcComponent() != 1:
        return None
    if packet.get_type() == 'GPS_GLOBAL_ORIGIN':
        return ('origin', PX4GlobalReference.from_message(packet))
    if packet.get_type() != 'POSITION_TARGET_GLOBAL_INT' or not clock.ready(now):
        return None
    if packet.coordinate_frame not in (0, 5) or packet.type_mask & 3:
        return None
    stamp = packet.time_boot_ms * 1_000_000 + clock.offset_ns
    if packet.time_boot_ms <= 0 or not 0 <= ros_ns-stamp <= 500_000_000:
        return None
    lat, lon = packet.lat_int/1e7, packet.lon_int/1e7
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        return None
    return ('target', dict(latitude=lat, longitude=lon, stamp_ns=stamp,
                           source='fc_mavlink_POSITION_TARGET_GLOBAL_INT'))
