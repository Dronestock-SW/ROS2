import math
import pytest
from drone_mission.contracts import Settings
from drone_mission.field_presets import field_route


@pytest.mark.parametrize('case,count', [('hover',1),('x',5),('y',5),('xy',9)])
def test_presets_keep_px4_height_and_return_over_visited_segments(case,count):
    route = field_route(case, (4.2,2.), Settings(drone_id='6'))
    assert len(route) == count and route[0]['dwell_s'] == 2
    assert all('z' not in p and 'yaw_deg' not in p for p in route)
    assert (route[-1]['x'],route[-1]['y']) == (4.2,2.)
    assert all(math.dist((a['x'],a['y']),(b['x'],b['y']))<=.5000001 for a,b in zip(route,route[1:]))
    if case in ('x','xy'):assert max(p['x'] for p in route)==5.2
    if case in ('y','xy'):assert max(p['y'] for p in route)==3.


def test_presets_reject_missing_start_and_outside_boundary():
    for xy in [(), (True,2), (math.nan,2), (5.2,2)]:
        with pytest.raises(ValueError):
            field_route('x',xy,Settings())
