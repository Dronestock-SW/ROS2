"""Prepare a separate pinned FMUv6C source tree. Does not flash or configure an FC."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

BASE = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'


def prepare(source):
    source = source.resolve()
    if source.name != 'PX4-lidar2d-fmu':
        raise ValueError('dedicated_hardware_source_tree_required')
    git = lambda *args: subprocess.check_output(['git','-C',str(source),*args],text=True).strip()
    if git('rev-parse','HEAD') != BASE or git('status','--porcelain','--untracked-files=no'):
        raise ValueError('clean_pinned_source_required')
    patch = Path(__file__).with_name('px4_lidar2d.patch')
    subprocess.run(['git','-C',str(source),'apply','--exclude=boards/px4/sitl/*',str(patch)],check=True)
    params = source/'src/modules/ekf2/params_aux_velocity.yaml'
    text = params.read_text(encoding='utf-8').replace('      max: 180\n','      max: 200\n')
    text += '''
    EKF2_LID2_VER:
      description:
        short: Dedicated horizontal LiDAR protocol version
        long: Protocol 1 consumes only NE velocity at the IMU origin from component 198. Vertical velocity is unobserved. Use with the matched companion package.
      type: int32
      default: 1
      min: 1
      max: 1
      reboot_required: true
'''
    params.write_text(text,encoding='utf-8')
    ekf = source/'src/modules/ekf2/EKF2.cpp'
    text = ekf.read_text(encoding='utf-8')
    text = text.replace('if (!sample.frame_valid) {',
        'if (sample.source_system != 255 || sample.source_component != 198) {\n'
        '\t\t\treason = 7;\n\t\t} else if (!sample.frame_valid) {',1)
    ekf.write_text(text,encoding='utf-8')
    board = source/'boards/px4/fmu-v6c/lidar2d.px4board'
    board.write_text((source/'boards/px4/fmu-v6c/default.px4board').read_text(encoding='utf-8')
        +'\nCONFIG_EKF2_AUXVEL=y\n',encoding='utf-8')
    subprocess.run(['git','-C',str(source),'add','-N','msg/LidarVelocity2d.msg',
        'msg/LidarVelocity2dStatus.msg','boards/px4/fmu-v6c/lidar2d.px4board'],check=True)
    diff = subprocess.check_output(['git','-C',str(source),'diff','--binary','HEAD'])
    # A physical build must contain no simulation truth or artificial IMU changes.
    files = git('diff','HEAD','--name-only').splitlines()
    if any('simulation' in f or 'HIL_STATE' in f or '/sitl/' in f for f in files):
        raise RuntimeError('simulation_change_in_hardware_tree')
    result = dict(base_commit=BASE, board='px4_fmu-v6c_lidar2d', board_id=56,
        patch_sha256=hashlib.sha256(diff).hexdigest(), files=files,
        firmware_default_lidar_enabled=False, vertical_velocity_observed=False,
        source_system=255, source_component=198, protocol_version=1,
        build_command='make px4_fmu-v6c_lidar2d -j2', physical_deployment=False)
    (source/'physical-build-source.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    Path(__file__).with_name('px4_lidar2d_hardware.patch').write_bytes(diff)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    prepare(parser.parse_args().source)
