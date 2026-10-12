"""Prepare an isolated SIH build configuration; never writes flight settings."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

FIRMWARE_COMMIT = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'


def prepare(root, lockstep=False):
    root = root.resolve()
    commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != FIRMWARE_COMMIT:
        raise ValueError(f'wrong PX4 commit: {commit}')
    if subprocess.check_output(['git', '-C', str(root), 'diff', 'HEAD', '--name-only'], text=True).strip():
        raise ValueError('PX4 tracked source differs from the required commit')
    # Only the new board configuration differs. EKF2 and SIH source stay intact.
    source = (root / 'boards/px4/sitl/default.px4board').read_text(encoding='utf-8')
    omit = ('CONFIG_MODULES_SIMULATION_GZ_', 'CONFIG_MODULES_UXRCE_DDS_CLIENT=',
            'CONFIG_DRIVERS_GPS=', 'CONFIG_DRIVERS_GNSS_SEPTENTRIO=')
    config = '\n'.join(line.split('=')[0] + '=n' if line.startswith(omit) else line
                       for line in source.splitlines())
    config += '\nCONFIG_BOARD_NOLOCKSTEP=' + ('n' if lockstep else 'y') + '\n'
    name = 'evbench' if lockstep else 'lidar'
    destination = root / f'boards/px4/sitl/{name}.px4board'
    if destination.exists() and destination.read_text(encoding='utf-8') != config:
        raise ValueError('refusing to replace an existing, different lidar board')
    destination.write_text(config, encoding='utf-8', newline='\n')
    return dict(commit=commit, board=f'px4_sitl_{name}',
                board_sha256=hashlib.sha256(config.encode()).hexdigest(),
                source_changes='new board only; no EKF2, SIH, or sensor-noise patch',
                build_command=f'make px4_sitl_{name} -j4')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-root', type=Path, required=True)
    parser.add_argument('--lockstep', action='store_true')
    args = parser.parse_args()
    print(json.dumps(prepare(args.px4_root, args.lockstep), ensure_ascii=False, indent=2))
