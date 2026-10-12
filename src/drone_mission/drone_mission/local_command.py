"""Transport-independent local command fields in ENU for MAVROS.

MAVROS performs the ENU to NED conversion. Direct MAVLink test transports
must explicitly convert these fields exactly once. Unobserved components
remain masked, and there is never a companion vertical velocity command.
"""
import math

from drone_uwb.integration.planar import map_to_enu_matrix, map_xy_to_enu, map_yaw_to_enu
from .contracts import finite


def local_command_fields(goal, settings):
    position = (*map_xy_to_enu(goal['xy'], settings), goal['altitude_enu_m'])
    yaw = math.radians(map_yaw_to_enu(goal['yaw_deg'], settings))
    if not finite(*position, yaw):
        raise ValueError('nonfinite_local_command')
    fields = dict(position_enu=position, velocity_enu=(0., 0., 0.), yaw_enu_rad=yaw,
                  type_mask=8 | 16 | 32 | 64 | 128 | 256 | 2048)
    if goal.get('horizontal_control', 'position') == 'velocity':
        value = goal.get('velocity_xy_m_s', ())
        if len(value) != 2 or not finite(*value):
            raise ValueError('finite_horizontal_velocity_required')
        velocity = map_to_enu_matrix(settings) @ value
        fields.update(type_mask=1 | 2 | 32 | 64 | 128 | 256 | 2048,
                      velocity_enu=(float(velocity[0]), float(velocity[1]), 0.))
    elif goal.get('horizontal_control', 'position') != 'position':
        raise ValueError('unsupported_horizontal_control')
    return fields
