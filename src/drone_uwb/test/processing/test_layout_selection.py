import copy
import json
from pathlib import Path

import pytest

from drone_uwb.integration.ros.layout_selection import anchor_path, matched_anchor_path

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('name,height', [('uwb_btf_tag_b.json', 2.1),
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


def test_prepared_tag_b_does_not_reuse_old_height_or_installation_confirmation():
    btf = json.loads((ROOT/'config/runtime/uwb_btf_tag_b.json').read_text(encoding='utf-8'))
    layout = json.loads(matched_anchor_path(ROOT, btf).read_text(encoding='utf-8'))
    assert layout['installation_confirmed'] is False
    assert btf['external_output_allowed'] is False
    with pytest.raises(ValueError, match='must match'):
        matched_anchor_path(ROOT, btf, 'config/anchors/anchors_20261007_z015.json')


@pytest.mark.parametrize('tag_z', [.85, 1.3, 2.4])
def test_raised_anchor_btf_recovers_xy_and_height_checked_b3(tag_z):
    import numpy as np
    from drone_uwb.acquisition.validation import Cycle
    from drone_uwb.processing.experiments.h80_b import BSettings, H80Window
    from drone_uwb.processing.experiments.tof_subset import ToFTrackedSubsetCandidate

    config = json.loads((ROOT/'config/runtime/uwb_btf_tag_b.json').read_text(encoding='utf-8'))
    anchors = np.asarray(config['anchors_xyz_m'])
    window = H80Window(anchors, config['bias_m'], BSettings(**config['B']))
    candidate = ToFTrackedSubsetCandidate(config, **config['tof_subset'])
    for seq in range(140):
        end = 1_000_000 + seq*20_000
        stamps = [end-12_000+i*4_000 for i in range(4)]
        ranges = np.linalg.norm(np.array([2., 2., tag_z])-anchors, axis=1)
        if seq >= 60: ranges[1] += .5
        cycle = Cycle(seq, stamps[0], end, 15, ranges, stamps, ['ok']*4, list(range(4)))
        row = dict(time_us=end, sample_time_us=stamps, cal_slant_m=ranges.tolist(),
                   height_m=tag_z, sample_height_m=[tag_z]*4,
                   height_source='measured_tof_imu', models={'B': window.process(cycle, seq)})
        result = candidate.process(row)
        if seq == 59:
            assert result['ok'] and result['source'] == 'unchanged_B4'
            assert result['xy_m'] == pytest.approx([2., 2.], abs=.005)
    assert result['ok'] and result['source'] == 'ToF_validated_B3'
    assert result['best_excluded_anchor'] == 2
    assert result['xy_m'] == pytest.approx([2., 2.], abs=.005)
