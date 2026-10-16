"""Height preparation must preserve measured mounts, source evidence and output gates."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'tools/prepare_anchor_height.py'
SPEC = importlib.util.spec_from_file_location('prepare_anchor_height', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
CONFIG = ROOT.parent / 'drone_uwb/config'


def baseline():
    value = json.loads((CONFIG/'runtime/uwb_btf_tag_b.json').read_text(encoding='utf-8'))
    old = json.loads((CONFIG/'anchors/anchors_20261007_z015.json').read_text(encoding='utf-8'))
    value.update(protocol='single_tag_schema_2', tdma_mode='disabled',
                 layout_id=old['layout_id'], anchors_xyz_m=old['anchors_xyz_m'],
                 ground_antenna_height_m=.15, single_tag_layout_confirmed=True,
                 external_output_allowed=True)
    value['height'].update(tof_to_tag_body_flu_m=[0, 0, .15], mount_confirmed=True,
                           flat_floor_confirmed=True)
    return value


def new_layout():
    return json.loads((CONFIG/'anchors/anchors_20261016_z2p1.json').read_text(encoding='utf-8'))


def test_preserves_sensor_geometry_bias_and_timing_but_resets_output_permission():
    source, layout = baseline(), new_layout()
    untouched = deepcopy(source)
    result = MODULE.prepare_config(source, layout)
    assert source == untouched
    assert result['anchors_xyz_m'] == layout['anchors_xyz_m']
    assert result['layout_id'] == layout['layout_id']
    assert result['height']['tof_to_tag_body_flu_m'] == [0, 0, .15]
    assert result['ground_antenna_height_m'] == .15
    assert result['single_tag_layout_confirmed'] is False
    assert result['external_output_allowed'] is False
    for key in source.keys() - {'layout_id', 'anchors_xyz_m', 'provenance',
                                'single_tag_layout_confirmed', 'external_output_allowed'}:
        assert result[key] == source[key]
    assert result['provenance']['timing'] == source['provenance']['timing']
    assert result['provenance']['tof_to_tag_mount'] == source['provenance']['tof_to_tag_mount']


@pytest.mark.parametrize('fault', ['xy', 'unequal_z', 'old_id', 'nonfinite', 'wrong_protocol'])
def test_rejects_inputs_outside_height_only_change(fault):
    source, layout = baseline(), new_layout()
    if fault == 'xy': layout['anchors_xyz_m'][1][0] += .1
    elif fault == 'unequal_z': layout['anchors_xyz_m'][1][2] += .1
    elif fault == 'old_id': layout['layout_id'] = source['layout_id']
    elif fault == 'nonfinite': layout['anchors_xyz_m'][0][2] = float('nan')
    else: source['protocol'] = 'legacy'
    with pytest.raises(ValueError): MODULE.prepare_config(source, layout)


def test_cli_preserves_source_and_refuses_existing_destination(tmp_path):
    source = tmp_path/'source.json'
    original = (json.dumps(baseline())+'\n').encode('utf-8')
    source.write_bytes(original)
    output = tmp_path/'prepared'
    command = [sys.executable, '-X', 'utf8', str(SCRIPT), '--source-btf', str(source),
               '--anchors', str(CONFIG/'anchors/anchors_20261016_z2p1.json'), '--output', str(output)]
    result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8')
    assert result.returncode == 0, result.stderr
    assert source.read_bytes() == original
    assert (output/'source-field-btf-cpp.json').read_bytes() == original
    prepared = (output/'field-btf-cpp.json').read_bytes()
    assert subprocess.run(command, capture_output=True).returncode != 0
    assert (output/'field-btf-cpp.json').read_bytes() == prepared
