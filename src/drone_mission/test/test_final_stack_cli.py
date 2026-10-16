"""Fresh private test files; --check-only must never start the field stack."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

pytest.importorskip('rclpy')
from drone_mission.final_stack import aligned_field_configuration


@pytest.mark.parametrize('change, expected', [
    (None, 'Configuration precheck passed'),
    ('old_boot', 'host_reboot_requires_new_alignment_evidence'),
    ('wrong_tag', 'mission_observation_tag_mismatch')])
def test_actual_field_launcher_check_only_starts_no_processes(tmp_path, change, expected):
    if sys.platform != 'linux': pytest.skip('Linux field launcher')
    root = Path(__file__).parents[3]
    config = root/'src/drone_mission/config'
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    mission, alignment = aligned_field_configuration(
        json.loads((config/'final_mission.example.json').read_text()),
        json.loads((config/'final_alignment.example.json').read_text()),
        body_enu_yaw_deg=15., host_boot_id=boot, fc_remote_ns=1)
    # This test isolates the legacy launch contract. LiDAR admission has
    # independent missing/qualified-configuration tests.
    mission['takeoff_lidar_required'] = False
    if change == 'old_boot': mission['bridge_host_boot_id'] = 'synthetic-old-boot'
    if change == 'wrong_tag': mission['drone_id'] = '5'
    files = dict(config=mission, alignment=alignment, **{'site-config':{},'ledger':{}})
    args = [sys.executable, str(root/'src/sangwon_AI/ops/final_field_session.py'), '--check-only']
    args += ['--cpp-ai-binary', sys.executable]
    for name, content in files.items():
        path = tmp_path/(name+'.json'); path.write_text(json.dumps(content), encoding='utf-8')
        args += ['--'+name, str(path)]
    store = tmp_path/'synthetic-unused.sqlite3'; store.touch()
    args += ['--mission-store', str(store), '--state-dir', str(tmp_path/'state')]
    result = subprocess.run(args, env={**os.environ,'ROS_DOMAIN_ID':'99','ROS_LOCALHOST_ONLY':'1'},
        text=True, capture_output=True, timeout=20)
    assert expected in result.stdout+result.stderr
    assert result.returncode == (0 if change is None else 2)
    assert not (tmp_path/'state').exists()


@pytest.mark.parametrize('case,expected', [('missing','takeoff_lidar_config_required'),
    ('unqualified','acquisition_timing_unconfirmed'),('qualified','Configuration precheck passed')])
def test_lidar_launcher_qualification_stays_read_only(tmp_path,case,expected):
    if sys.platform!='linux':pytest.skip('Linux field launcher')
    root=Path(__file__).parents[3];config=root/'src/drone_mission/config'
    mission,alignment=aligned_field_configuration(
        json.loads((config/'final_mission.example.json').read_text()),
        json.loads((config/'final_alignment.example.json').read_text()),
        body_enu_yaw_deg=15.,host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),fc_remote_ns=1)
    mission['takeoff_lidar_fusion_ref']='synthetic-qualified-fusion'
    args=[sys.executable,str(root/'src/sangwon_AI/ops/final_field_session.py'),'--check-only',
        '--cpp-ai-binary',sys.executable,'--cpp-executor-binary',sys.executable]
    for name,content in dict(config=mission,alignment=alignment,**{'site-config':{},'ledger':{}}).items():
        path=tmp_path/(name+'.json');path.write_text(json.dumps(content),encoding='utf-8');args+=['--'+name,str(path)]
    store=tmp_path/'unused.sqlite3';store.touch();args+=['--mission-store',str(store),'--state-dir',str(tmp_path/'state')]
    if case!='missing':
        lidar=dict(settings=dict(enabled=True,tag_id='6',sensor_to_body_flu_m=[0.,0.,.1],sensor_to_body_yaw_rad=0.,
            mount_position_ref='synthetic',mount_yaw_ref='synthetic',scan_plane_parallel_confirmed=True,
            timing_ref='synthetic' if case=='qualified' else '',floor_policy='metric_scan',vertical_structure_ref='synthetic-walls'))
        path=tmp_path/'lidar.json';path.write_text(json.dumps(lidar),encoding='utf-8');args+=['--lidar-config',str(path)]
    result=subprocess.run(args,env={**os.environ,'ROS_DOMAIN_ID':'99','ROS_LOCALHOST_ONLY':'1'},text=True,capture_output=True,timeout=20)
    assert expected in result.stdout+result.stderr
    assert result.returncode==(0 if case=='qualified' else 2)
    assert not (tmp_path/'state').exists()
