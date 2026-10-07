"""Resolve packaged layouts and reject receiver/B_TF geometry mismatches."""
import json
from pathlib import Path

LAYOUTS = {
    'warehouse-rectangle-6p3x4p6-z2p2-20261004': 'anchors_20261004.json',
    'warehouse-rectangle-6p3x4p6-z0p15-20261007': 'anchors_20261007_z015.json',
}


def anchor_path(share, value):
    path = Path(value or 'config/anchors/anchors_20261004.json')
    return path if path.is_absolute() else Path(share) / path


def matched_anchor_path(share, btf, explicit=''):
    """A custom B_TF layout requires its matching explicit anchor file."""
    name = LAYOUTS.get(btf['layout_id'])
    if not explicit and name is None:
        raise ValueError('Unknown B_TF layout: supply matching anchor_file')
    path = anchor_path(share, explicit or f'config/anchors/{name}')
    layout = json.loads(path.read_text(encoding='utf-8'))
    if (layout['layout_id'] != btf['layout_id']
            or layout['anchors_xyz_m'] != btf['anchors_xyz_m']):
        raise ValueError('Receiver and B_TF anchor geometry must match')
    return path
