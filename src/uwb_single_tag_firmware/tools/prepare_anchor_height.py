"""Prepare a new single-tag geometry bundle without changing a running service."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path


def prepare_config(config, layout):
    """Change anchor height only; preserve sensor mounts and keep outputs gated."""
    if config.get('protocol') != 'single_tag_schema_2':
        raise ValueError('Expected the deployed single-tag B_TF configuration')
    if layout.get('anchor_order') != ['A1', 'A2', 'A3', 'A4'] or layout.get('units') != 'm':
        raise ValueError('Expected A1..A4 in metres')
    before, after = config.get('anchors_xyz_m'), layout.get('anchors_xyz_m')
    for points in (before, after):
        if (not isinstance(points, list) or len(points) != 4
                or any(not isinstance(p, list) or len(p) != 3 for p in points)
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       for p in points for v in p)):
            raise ValueError('Expected four finite XYZ anchor coordinates')
    if any(a[:2] != b[:2] for a, b in zip(before, after)):
        raise ValueError('Height-only preparation must preserve anchor XY')
    if len({p[2] for p in after}) != 1:
        raise ValueError('This equal-height B_TF setup requires one anchor height')
    if not isinstance(layout.get('layout_id'), str) or not layout['layout_id'].strip():
        raise ValueError('A named layout is required')
    if layout['layout_id'] == config.get('layout_id') and before != after:
        raise ValueError('Changed geometry requires a new layout ID')
    result = deepcopy(config)
    result.update(layout_id=layout['layout_id'], anchors_xyz_m=deepcopy(after),
                  single_tag_layout_confirmed=False, external_output_allowed=False)
    result.setdefault('provenance', {})['anchors_and_downward_sensor'] = (
        layout.get('source', 'Operator-supplied anchor layout')
        + ' Prepared only; physical installation and output enablement remain pending. '
        'Existing ToF mount, ground tag height, timing and range biases were preserved.')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-btf', type=Path, required=True)
    parser.add_argument('--anchors', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, geometry = args.source_btf.read_bytes(), args.anchors.read_bytes()
    config = json.loads(source.decode('utf-8'))
    layout = json.loads(geometry.decode('utf-8'))
    prepared = prepare_config(config, layout)
    # Existing evidence and operating slots must never be overwritten.
    args.output.mkdir(parents=True, exist_ok=False, mode=0o700)
    files = {'field-btf-cpp.json': (json.dumps(prepared, ensure_ascii=False, indent=2)+'\n').encode('utf-8'),
             args.anchors.name: geometry,
             'source-field-btf-cpp.json': source}
    for name, data in files.items():
        (args.output/name).write_bytes(data)
    manifest = dict(source_btf=str(args.source_btf.resolve()),
                    source_layout_id=config['layout_id'], prepared_layout_id=layout['layout_id'],
                    source_sha256=hashlib.sha256(source).hexdigest(),
                    files={name: hashlib.sha256(data).hexdigest() for name, data in files.items()},
                    physical_installation_confirmed=False, service_modified=False,
                    firmware_uploaded=False, output_enabled=False)
    (args.output/'preparation.json').write_bytes(
        (json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == '__main__':
    main()
