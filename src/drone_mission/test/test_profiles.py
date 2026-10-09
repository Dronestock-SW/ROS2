import json
from pathlib import Path
import yaml

import pytest

from drone_mission.profiles import trial_profile
from drone_uwb.integration.ros.frames import BridgeSettings

MISSION = Path(__file__).parents[1]
UWB = MISSION.parent / 'drone_uwb'


@pytest.mark.parametrize('role,tag,domain,height', [('A','5',1,.6), ('B','6',2,1.3)])
def test_profile_matches_every_input_and_keeps_physical_confirmations_off(role,tag,domain,height):
    settings, actual_domain, _, btf_path, anchors_path, bridge = trial_profile(role, MISSION, UWB)
    btf = json.loads(btf_path.read_text(encoding='utf-8'))
    anchors = json.loads(anchors_path.read_text(encoding='utf-8'))
    assert actual_domain == domain and settings.drone_id == tag == btf['tag_id']
    assert settings.layout_id == btf['layout_id'] == anchors['layout_id']
    assert settings.expected_mis_takeoff_alt_m == height
    assert btf['tdma_mode'] == 'required'
    assert not settings.execute and not settings.layout_confirmed and not settings.fusion_confirmed
    parsed = BridgeSettings(**bridge)
    assert not parsed.enabled and parsed.ground_only and parsed.input_topic == '/uwb/btf_pose'
    assert parsed.ros_domain_id == domain


def test_wrong_tag_and_layout_rejected_before_any_process(tmp_path):
    with pytest.raises(ValueError, match='match selected Tag'):
        trial_profile('B', MISSION, UWB, config=MISSION/'config/flight.json')
    config = json.loads((MISSION/'config/flight_tag_b.json').read_text(encoding='utf-8'))
    config['layout_id'] = 'warehouse-rectangle-6p3x4p6-z2p2-20261004'
    path = tmp_path/'wrong-layout.json'
    path.write_text(json.dumps(config),encoding='utf-8')
    with pytest.raises(ValueError, match='layout must match'):
        trial_profile('B', MISSION, UWB, config=path)


def test_real_launch_loads_rc_plugin_required_by_mission_start():
    profile = yaml.safe_load((UWB/'config/runtime/mavros_test_flight.yaml').read_text(encoding='utf-8'))
    plugins = profile['/**']['ros__parameters']['plugin_allowlist']
    assert 'rc_io' in plugins, 'MissionChain requires /mavros/rc/in on real domains'
    for plugin in ('sys_status', 'command', 'param', 'global_position', 'local_position', 'vision_pose'):
        assert plugin in plugins
