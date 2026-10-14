"""Create an isolated pinned PX4 checkout using local object stores only.

The donor and its existing build remain unchanged. This does not flash an FC.
"""
import argparse
from pathlib import Path
import shutil
import subprocess

BASE = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def clone_tree(donor, destination, commit):
    if not (destination / '.git').exists():
        subprocess.run(['git', 'clone', '--shared', '--no-checkout', str(donor), str(destination)], check=True)
        subprocess.run(['git', '-C', str(destination), 'checkout', '--detach', commit], check=True)
    elif run('git', '-C', str(destination), 'rev-parse', 'HEAD') != commit:
        raise ValueError('Existing isolated checkout has a different commit')
    entries = subprocess.check_output(['git', '-C', str(destination), 'ls-tree', '-rz', 'HEAD']).split(b'\0')
    for entry in entries:
        if not entry.startswith(b'160000 '):
            continue
        metadata, name = entry.decode().split('\t', 1)
        revision = metadata.split()[2]
        source = donor / name
        if not (source / '.git').exists():
            print('Uninitialized optional donor submodule: ' + str(source), flush=True)
            continue
        clone_tree(source, destination / name, revision)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--donor', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--with-tests', action='store_true',
                        help='Install the isolated SIH board and deterministic EKF tests')
    args = parser.parse_args()
    donor, destination = args.donor.resolve(), args.destination.resolve()
    if (destination.exists() and not args.resume) or donor == destination:
        raise ValueError('A new isolated destination is required')
    if run('git', '-C', str(donor), 'rev-parse', 'HEAD') != BASE:
        raise ValueError('Pinned donor required')
    clone_tree(donor, destination, BASE)
    patch = Path(__file__).with_name('px4_lidar2d_independent_hardware.patch')
    subprocess.run(['git', '-C', str(destination), 'apply', '--check', str(patch)], check=True)
    subprocess.run(['git', '-C', str(destination), 'apply', str(patch)], check=True)
    if args.with_tests:
        board = Path(__file__).with_name('lidar_independent_sitl.px4board').read_text(encoding='utf-8')
        (destination/'boards/px4/sitl/lidar2d.px4board').write_text(board, encoding='utf-8', newline='\n')
        (destination/'boards/px4/sitl/test.px4board').write_text(
            board.replace('CONFIG_BOARD_NOLOCKSTEP=n', 'CONFIG_BOARD_NOLOCKSTEP=y'),
            encoding='utf-8', newline='\n')
        tests = destination/'src/modules/ekf2/test'
        shutil.copy2(Path(__file__).with_name('test_EKF_lidar_independent.cpp'), tests)
        cmake = tests/'CMakeLists.txt'
        entry = 'px4_add_unit_gtest(SRC test_EKF_lidar_independent.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)'
        with cmake.open('a', encoding='utf-8', newline='\n') as stream:
            stream.write(entry+'\n')
    print(destination)


if __name__ == '__main__':
    main()
