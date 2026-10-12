"""Unpack a reviewed wheel set into a private dependency directory; no root."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheels',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if platform.machine() != 'aarch64' or sys.version_info[:2] != (3,10):
        raise RuntimeError('expected_Jetson_aarch64_Python310')
    dest = args.output.resolve()
    dest.mkdir(parents=True,exist_ok=False)
    files = sorted(args.wheels.glob('*.whl'))
    if len(files) != 3 or {p.name.split('-')[0] for p in files} != {'pymavlink','lxml','fastcrc'}:
        raise ValueError('expected_three_reviewed_dependencies')
    manifest = []
    for path in files:
        with zipfile.ZipFile(path) as archive:
            if any(dest not in (dest/name).resolve().parents for name in archive.namelist()):
                raise ValueError('wheel_member_outside_private_directory')
            archive.extractall(dest)
        manifest.append(dict(name=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (dest/'dependency-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    sys.path.insert(0,str(dest))
    from pymavlink.dialects.v20 import common
    print(json.dumps(dict(dependencies=manifest,lidar_estimator_type=common.MAV_ESTIMATOR_TYPE_LIDAR)))


if __name__ == '__main__':
    main()
