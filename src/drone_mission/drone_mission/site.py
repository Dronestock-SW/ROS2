"""Site ceiling constraints; never a flight command or a survey assertion."""
import copy
from .contracts import finite


def ceiling_value(value):
    if not finite(value) or not .5 <= value <= 100:
        raise ValueError('ceiling_height_m_must_be_0p5_to_100')
    return float(value)


def ceiling_map(source, ceiling):
    """Only tighten the measured map. Preserve obstacles and survey provenance."""
    ceiling = ceiling_value(ceiling)
    result = copy.deepcopy(source)
    height, clearance = result.get('native_body_floor_height_m'), result.get('clearance_z_m')
    if not finite(height, clearance) or clearance <= 0:
        raise ValueError('measured_body_floor_height_and_clearance_required')
    if height + clearance > ceiling:
        raise ValueError('native_height_exceeds_site_ceiling_clearance')
    for group in ('boundary', 'altitude_zones', 'yaw_zones'):
        for volume in result[group]:
            volume['z_max_m'] = min(volume['z_max_m'], ceiling)
    return result
