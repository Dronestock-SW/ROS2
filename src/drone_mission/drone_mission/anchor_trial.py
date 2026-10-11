"""Bounded anchor-coordinate trials; no obstacle-map or survey claims."""
import math
from .contracts import finite, parse_request


def validate_anchor_trial(payload, launch_xy, settings, wall_s):
    if not settings.anchor_trial:
        raise ValueError('anchor_trial_profile_required')
    request = parse_request(payload, settings, wall_s)
    if request is None or request['action'] != 'start':
        raise ValueError('anchor_trial_start_required')
    if not settings.inside(launch_xy):
        raise ValueError('anchor_trial_launch_outside_bounds')
    ceiling = payload.get('ceiling_height_m')
    if not finite(ceiling) or ceiling < settings.expected_mis_takeoff_alt_m + .25:
        raise ValueError('anchor_trial_takeoff_ceiling_clearance_required')
    if settings.speed_m_s > .25:
        raise ValueError('anchor_trial_max_speed_0p25_m_s')
    points = [launch_xy] + [point['xy'] for point in request['waypoints']]
    if any(math.dist(a, b) > settings.max_leg_m for a, b in zip(points, points[1:])):
        raise ValueError('route_leg_exceeds_trial_limit')
    return dict(profile='anchor_trial', launch_xy_m=list(launch_xy),
                bounds_xy_m=list(settings.bounds_xy_m), speed_m_s=settings.speed_m_s,
                obstacle_map_checked=False, surveyed_map_required=False,
                vertical_controller='PX4', waypoint_count=len(request['waypoints']))
