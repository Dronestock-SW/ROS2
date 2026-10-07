"""Guards for decisions recorded on 2026-09-20: v1.2 gate values, stored anchor coordinates."""
import json
from pathlib import Path

import pytest
import yaml

from drone_uwb.processing.solvers.observations import Settings
from drone_uwb.processing.settings import PreimuSettings, settings_from_mapping

CONFIG = Path(__file__).resolve().parents[2] / 'config'


def test_preimu_gates_match_handoff_v1_2_section_10a():
    s = PreimuSettings()
    assert (s.range_gate_margin_m, s.range_gate_speed_m_s) == (0.20, 1.20)
    assert (s.range_reacquire_cluster_m, s.range_reacquire_confirm) == (0.15, 3)
    assert (s.position_gate_margin_m, s.position_gate_speed_m_s) == (0.08, 0.80)
    assert (s.position_reacquire_cluster_m, s.position_reacquire_confirm) == (0.12, 3)
    assert s.residual_rms_max_m == 0.10


def test_live_node_gate_defaults_match_handoff_v1_2():
    s = Settings()
    assert (s.range_step_margin_m, s.max_speed_m_s, s.recovery_samples) == (0.20, 1.20, 3)


def test_yaml_gate_values_equal_live_node_defaults():
    node = yaml.safe_load((CONFIG / 'uwb.yaml').read_text(encoding='utf-8'))['uwb_node']['ros__parameters']
    defaults = Settings()
    for name in ('range_step_margin_m', 'max_speed_m_s', 'recovery_samples'):
        assert node[name] == getattr(defaults, name), name


def test_gate_settings_reject_nonpositive_and_unknown_values():
    with pytest.raises(ValueError):
        PreimuSettings(range_gate_margin_m=0.0)
    with pytest.raises(ValueError):
        settings_from_mapping({'range_gate_margin': 0.2})


def test_anchor_coordinates_stay_at_the_stored_jetson_values():
    # Decision 2026-09-20: keep the stored survey values, not the handoff v1.2 table (A2 differs by 0.43 m).
    layout = json.loads((CONFIG / 'anchors_20260906.json').read_text(encoding='utf-8'))
    a2 = layout['anchors_xyz_m'][1]
    assert a2[0] == pytest.approx(5.8102027, abs=1e-6)
    assert a2[1] == pytest.approx(-0.4303270, abs=1e-6)
    assert all(a[2] == 2.2 for a in layout['anchors_xyz_m'])
