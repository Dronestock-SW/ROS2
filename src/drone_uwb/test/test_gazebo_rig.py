"""Sensor geometry and asset consistency, without claiming a Gazebo runtime."""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from drone_uwb.integration.gazebo_rig import build, install_assets, read_xml, MODEL, WORLD
from drone_uwb.processing.gazebo_geometry import tag_position, VirtualRanges
from drone_uwb.processing.experiments.gazebo_trial import compare_poses


CONFIG = Path(__file__).resolve().parents[1]/'config'


def test_antenna_offset_rotates_in_world_and_fc_remains_distinct():
    pose = dict(position_xyz_m=[1., 2., 3.], quaternion_wxyz=[2**-.5, 0, 0, 2**-.5])
    np.testing.assert_allclose(tag_position(pose, [.14, 0, .3]), [1, 2.14, 3.3], atol=1e-12)
    pose['quaternion_wxyz'] = [2**-.5, 2**-.5, 0, 0]
    np.testing.assert_allclose(tag_position(pose, [0, 0, .3]), [1, 1.7, 3], atol=1e-12)
    with pytest.raises(ValueError):
        tag_position(dict(pose, quaternion_wxyz=[0, 0, 0, 0]), [0, 0, .3])


def test_live_sensor_and_replay_share_ranges_but_truth_is_separate():
    config = json.loads((CONFIG/'gazebo_shadow.json').read_text(encoding='utf-8'))
    config['tag_offset_body_flu_m'] = [0, 0, .3]
    sample = dict(source='gazebo', clock_domain='gazebo_sim_us', model='test', time_us=100,
                  position_xyz_m=[.7, .3, 1.1], quaternion_wxyz=[1, 0, 0, 0])
    sensor = VirtualRanges(config)
    raw, reference = sensor.sample(sample)
    replay = next(compare_poses([sample], config))
    np.testing.assert_allclose(raw['raw_slant_m'], replay['raw_slant_m'])
    np.testing.assert_allclose(reference['tag_xyz_m'], replay['truth_xyz_m'])
    assert 'tag_xyz_m' not in raw and 'truth_xyz_m' not in raw
    assert raw['clock_domain'] == 'gazebo_sim_us' and not raw['range_bias_applied']
    with pytest.raises(ValueError):
        sensor.sample(sample)


@pytest.fixture
def native(tmp_path):
    """Minimal structural substitutes; production native SDF is checked separately."""
    root = tmp_path/'native'
    files = {
        'models/x500_base/model.sdf': '<sdf version="1.9"><model name="x500_base"><link name="base_link"><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><sensor name="imu_sensor" type="imu"/></link></model></sdf>',
        'models/x500/model.sdf':'<sdf><model><include><uri>x500_base</uri></include><plugin name="motor" filename="motor"/></model></sdf>',
        'models/x500_lidar_down/model.sdf':'<sdf><model><link name="lidar_sensor_link"><sensor name="lidar" type="gpu_lidar"/></link></model></sdf>',
        'models/optical_flow/model.sdf':'<sdf version="1.9"><model><link name="flow_link"><sensor name="optical_flow" type="custom" gz:type="optical_flow"/></link></model></sdf>',
        'models/lidar_2d_v2/model.sdf':'<sdf><model><link name="link"><sensor name="lidar_2d_v2" type="gpu_lidar"/></link></model></sdf>',
        'models/mono_cam/model.sdf':'<sdf><model><link name="camera_link"><sensor name="camera" type="camera"/></link></model></sdf>',
        'worlds/default.sdf':'<sdf><world name="default"><model name="ground_plane"><link name="ground"/></model></world></sdf>',
        'server.config':'<server_config><plugins><plugin name="Sensors" filename="sensors" entity_name="*" entity_type="world"/></plugins></server_config>',
        'LICENSE':'Test fixture. Not third-party code.'}
    for name, text in files.items():
        path = root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
    return root


def test_assets_use_one_anchor_map_and_keep_native_bridge_names(native, tmp_path):
    original = {str(p):p.read_bytes() for p in native.rglob('*') if p.is_file()}
    out = tmp_path/'rig'
    result = build(native, CONFIG/'anchors_20260906.json', CONFIG/'gazebo_equipment.json', CONFIG/'gazebo_shadow.json', out)
    assert {str(p):p.read_bytes() for p in native.rglob('*') if p.is_file()} == original
    world = read_xml(out/'worlds'/f'{WORLD}.sdf').find('world')
    trial = json.loads((out/'trial.json').read_text(encoding='utf-8'))
    for i, name in enumerate(('A1', 'A2', 'A3', 'A4')):
        xyz = [float(v) for v in world.find(f"model[@name='uwb_{name}']/pose").text.split()[:3]]
        assert xyz == trial['anchors_xyz_m'][i]
    model = read_xml(out/'models'/MODEL/'model.sdf').find('model')
    assert model.find("link[@name='lidar_sensor_link']/sensor[@name='lidar']") is not None
    assert model.find("link[@name='flow_link']/sensor[@name='optical_flow']") is not None
    assert model.find("link[@name='base_link']/sensor[@name='imu_sensor']/pose").text.startswith('0.14 ')
    assert float(model.find("link[@name='link']/pose").text.split()[0]) == .14
    assert result['simulated_total_mass_kg'] == pytest.approx(2.0)
    assert not result['gazebo_runtime_verified']
    install_assets(out, native)
    with pytest.raises(FileExistsError):
        install_assets(out, native)
    assert all(Path(p).read_bytes() == data for p, data in original.items())


def test_world_link_children_have_unique_names(native, tmp_path):
    """SDFormat checks names across visual/collision types within each link."""
    out = tmp_path/'rig'
    build(native, CONFIG/'anchors_20260906.json', CONFIG/'gazebo_equipment.json', CONFIG/'gazebo_shadow.json', out)
    world = read_xml(out/'worlds'/f'{WORLD}.sdf').find('world')
    for link in world.iter('link'):
        names = [child.attrib['name'] for child in link if 'name' in child.attrib]
        assert len(names) == len(set(names)), f'duplicate child name in link {link.attrib["name"]}: {names}'
    obstacle = world.find("model[@name='test_wall']/link")
    assert obstacle.find('visual/geometry/box/size').text == obstacle.find('collision/geometry/box/size').text
