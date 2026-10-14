"""Freeze an isolated SIH runtime and hardware source patch; no deployment."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    base = subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if base != 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4':
        raise ValueError('pinned_source_required')
    output.mkdir(parents=True, exist_ok=False)
    build = source/'build/px4_sitl_lidar2d'
    (output/'bin').mkdir()
    for item in (build/'bin').iterdir():
        target = output/'bin'/item.name
        if item.is_symlink():
            if not item.name.startswith('px4-'):
                raise ValueError('unexpected_runtime_link')
            target.symlink_to('px4')
        else:
            shutil.copy2(item,target)
    shutil.copytree(build/'etc',output/'etc',symlinks=False)
    shutil.copy2(source/'boards/px4/sitl/lidar2d.px4board',output/'lidar2d.px4board')
    added = ['msg/LidarVelocity2d.msg','msg/LidarVelocity2dStatus.msg',
             'boards/px4/fmu-v6c/lidar2d.px4board','boards/px4/sitl/lidar2d.px4board',
             'src/modules/ekf2/test/test_EKF_lidar_independent.cpp']
    subprocess.run(['git','-C',str(source),'add','-N',*added],check=True)
    patch = subprocess.check_output(['git','-C',str(source),'diff','--binary','HEAD'])
    (output/'px4_lidar2d.patch').write_bytes(patch)
    hardware = subprocess.check_output(['git','-C',str(source),'diff','--binary','HEAD','--','.',
        ':!boards/px4/sitl',':!src/modules/ekf2/test'])
    (output/'px4_lidar2d_independent_hardware.patch').write_bytes(hardware)
    manifest = dict(base_commit=base,protocol_version=2,physical_deployment=False,
        build_command='make px4_sitl_lidar2d -j3',binary_sha256=sha(output/'bin/px4'),
        patch_sha256=sha(output/'px4_lidar2d.patch'),board_sha256=sha(output/'lidar2d.px4board'),
        hardware_patch_sha256=sha(output/'px4_lidar2d_independent_hardware.patch'),
        observed_axes=['north','east'],vertical_velocity_observed=False,
        controller_changes=False)
    (output/'artifact.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(manifest,indent=2))


if __name__ == '__main__':
    main()
