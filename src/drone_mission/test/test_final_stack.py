from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from drone_mission.final_stack import (
    aligned_field_configuration, stationary_yaw_mean, validate_field_configuration,
    require_field_dependencies)
from drone_uwb.integration.planar import map_yaw_to_enu


BOOT = 'synthetic-field-boot'


def templates():
    config = Path(__file__).parents[1]/'config'
    mission = json.loads((config/'final_mission.example.json').read_text(encoding='utf-8'))
    alignment = json.loads((config/'final_alignment.example.json').read_text(encoding='utf-8'))
    return mission, alignment


def aligned():
    return aligned_field_configuration(*templates(), body_enu_yaw_deg=-16.309430414773477,
        host_boot_id=BOOT, fc_remote_ns=123456789)


def test_new_final_alignment_template_keeps_sender_jump_protection_on():
    assert templates()[1]['settings']['jump_guard_enabled'] is True


@pytest.mark.parametrize('sign', [-1, 1])
def test_stationary_minus_y_alignment_roundtrips_both_warehouse_charts(sign):
    mission, alignment = templates()
    original = deepcopy((mission, alignment))
    mission['map_y_axis_sign'] = alignment['settings']['map_y_axis_sign'] = sign
    out_mission, out_alignment = aligned_field_configuration(mission, alignment,
        body_enu_yaw_deg=-16.309430414773477, host_boot_id=BOOT, fc_remote_ns=1)
    cfg, obs = validate_field_configuration(out_mission, out_alignment, host_boot_id=BOOT)
    assert map_yaw_to_enu(-90., cfg) == pytest.approx(-16.309430414773477)
    assert cfg.enu_yaw_deg == obs.enu_yaw_deg
    assert mission['enu_yaw_deg'] == original[0]['enu_yaw_deg']
    assert alignment['settings']['host_boot_id'] == original[1]['settings']['host_boot_id']


def test_startup_rejects_old_boot_tag_transform_and_sensor_mismatch():
    mission, alignment = aligned()
    changes = [('bridge_host_boot_id', 'other-boot'), ('drone_id', '5'),
        ('enu_yaw_deg', 0.), ('map_y_axis_sign', 1),
        ('expected_ev_pos_x_m', 0.), ('expected_ev_delay_ms', 100.),
        ('bridge_topic', '/uwb/bridge_status'), ('execute', False)]
    for key, value in changes:
        with pytest.raises(ValueError):
            validate_field_configuration({**mission, key:value}, alignment, host_boot_id=BOOT)
    for remote in (0, -1, True, None, '1'):
        changed = deepcopy(alignment)
        changed['settings']['fc_remote_ns_at_alignment'] = remote
        with pytest.raises(ValueError, match='alignment_evidence_required'):
            validate_field_configuration(mission, changed, host_boot_id=BOOT)


def test_circular_stationary_yaw_accepts_wrap_and_rejects_motion_and_nan():
    mean, span = stationary_yaw_mean([179.9, -179.9]*30)
    assert abs(abs(mean)-180.) < 1e-9 and span == pytest.approx(.2)
    for samples in ([0.]*49, [0.,2.]*30, [0.]*59+[math.nan]):
        with pytest.raises(ValueError): stationary_yaw_mean(samples)


def test_missing_dependency_is_reported_before_starting_processes(monkeypatch):
    imported = []
    def load(name):
        imported.append(name)
        if name == 'pymavlink': raise ImportError('synthetic missing dependency')
    monkeypatch.setattr('drone_mission.final_stack.importlib.import_module', load)
    with pytest.raises(RuntimeError, match='field_runtime_dependency_unavailable:pymavlink'):
        require_field_dependencies()
    assert imported == ['rclpy', 'mavros.mavlink', 'pymavlink']
