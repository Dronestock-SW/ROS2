"""Inspect launch configuration without starting serial/MAVROS processes."""
import importlib.util
import json
from pathlib import Path
import pytest

launch=pytest.importorskip('launch')
from launch import LaunchContext
from launch_ros.actions import Node

ROOT=Path(__file__).parents[2]
spec=importlib.util.spec_from_file_location('multitag_bench_launch',ROOT/'launch/uwb_btf_bench.launch.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('role,tag,domain',[('A','5','1'),('B','6','2')])
def test_each_tag_uses_its_profile_and_domain_without_duplicate_mavros(monkeypatch,role,tag,domain):
    monkeypatch.setattr(module,'get_package_share_directory',lambda _:str(ROOT))
    context=LaunchContext()
    context.launch_configurations.update(tag=role,btf_config='',record_directory='/tmp/unused-inspection',
        uwb_port='/dev/uwb-test',stop_after_s='60',start_mavros='false',fcu_url='/dev/unused')
    actions=module.setup(context)
    assert len([a for a in actions if isinstance(a,Node)])==2
    actions[0].execute(context)
    assert context.environment['ROS_DOMAIN_ID']==domain
    config=json.loads((ROOT/f'config/runtime/uwb_btf_tag_{role.lower()}.json').read_text())
    assert config['tag_id']==tag and config['tdma_mode']=='required'
    assert config['external_output_allowed'] is False
    if role=='B':
        assert config['height']['mount_confirmed'] is False
        assert config['height']['flat_floor_confirmed'] is False


def test_wrong_tag_config_is_rejected_before_launch(monkeypatch):
    monkeypatch.setattr(module,'get_package_share_directory',lambda _:str(ROOT))
    context=LaunchContext()
    context.launch_configurations.update(tag='B',btf_config=str(ROOT/'config/runtime/uwb_btf_tag_a.json'))
    with pytest.raises(ValueError,match='match selected Tag'):
        module.setup(context)
