"""Test routes from a validated PX4 body origin. No Z or approvals."""
import math
from .contracts import finite


def field_route(case, start_xy, settings):
    offsets = {'hover': [(0, 0)], 'x': [(0, 0), (1, 0), (0, 0)],
               'y': [(0, 0), (0, 1), (0, 0)],
               'xy': [(0, 0), (1, 0), (1, 1), (1, 0), (0, 0)]}
    if case not in offsets or len(start_xy) != 2 or not finite(*start_xy):
        raise ValueError('invalid_case_or_measured_start')
    points=[(0.,0.)]
    # Leave room for measured arrival error under the existing per-command
    # distance limit. The requested endpoints stay exactly 1 m from home.
    for a,b in zip(offsets[case],offsets[case][1:]):
        steps=math.ceil(math.dist(a,b)/min(.5,settings.max_leg_m/2))
        points.extend((a[0]+(b[0]-a[0])*j/steps,a[1]+(b[1]-a[1])*j/steps) for j in range(1,steps+1))
    if len(points)>20:
        raise ValueError('test_route_exceeds_task_limit')
    route = []
    for i, (dx, dy) in enumerate(points):
        xy = (start_xy[0]+dx, start_xy[1]+dy)
        if not settings.inside(xy):
            raise ValueError('test_route_outside_configured_bounds')
        route.append(dict(id=f'{case.upper()}{i+1}', type='hover' if i == 0 else 'waypoint',
                          x=xy[0], y=xy[1], dwell_s=2.0 if i == 0 else 0.0))
    return route
