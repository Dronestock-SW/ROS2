import copy
import json
from pathlib import Path

import pytest

from drone_uwb.integration.ros.layout_selection import anchor_path, matched_anchor_path

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('name,height', [('uwb_btf_tag_b.json', .15),
                                       ('uwb_btf_tag_b_z2p2.json', 2.2),
                                       ('uwb_btf_tag_a.json', 2.2)])
def test_packaged_geometry_and_unmeasured_b_mount(name, height):
    btf = json.loads((ROOT / 'config/runtime' / name).read_text())
    anchors = matched_anchor_path(ROOT, btf)
    layout = json.loads(anchors.read_text())
    assert layout['anchors_xyz_m'] == [[0, 0, height], [6.3, 0, height],
                                       [0, 4.6, height], [6.3, 4.6, height]]
    assert layout['independently_verified'] is False
    if btf['tag_id'] == '6':
        assert btf['height']['mount_confirmed'] is False
        assert btf['height']['flat_floor_confirmed'] is False


def test_reject_mixed_height_files():
    btf = json.loads((ROOT / 'config/runtime/uwb_btf_tag_b.json').read_text())
    with pytest.raises(ValueError, match='must match'):
        matched_anchor_path(ROOT, btf, 'config/anchors/anchors_20261004.json')
    wrong = copy.deepcopy(btf)
    wrong['anchors_xyz_m'][0][2] = 2.2
    with pytest.raises(ValueError, match='must match'):
        matched_anchor_path(ROOT, wrong)


def test_relative_path_is_package_relative_and_custom_layout_requires_file():
    assert anchor_path(ROOT, 'config/anchors/anchors_20261004.json').is_file()
    with pytest.raises(ValueError, match='Unknown'):
        matched_anchor_path(ROOT, {'layout_id': 'unregistered'})
