"""Package an already-built private experiment with hashes; never deploy."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

BASE = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package(source, output, evidence):
    source, output = source.resolve(), output.resolve()
    if source.name != 'PX4-lidar2d' or output.exists():
        raise ValueError('private source and new destination required')
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != BASE:
        raise ValueError('unexpected base commit')
    build = source/'build/px4_sitl_lidar2d'
    output.mkdir(parents=True)
    # PX4 command aliases are links to the same executable. Preserve them;
    # materializing every alias would duplicate the binary hundreds of times.
    shutil.copytree(build/'bin', output/'bin', symlinks=True)
    shutil.copytree(build/'etc', output/'etc', symlinks=True)
    shutil.copyfile(source/'boards/px4/sitl/lidar2d.px4board', output/'lidar2d.px4board')
    patch = subprocess.check_output(['git', '-C', str(source), 'diff', '--binary', 'HEAD'])
    (output/'px4_lidar2d.patch').write_bytes(patch)
    (output/'source_commit.txt').write_text(BASE+'\n', encoding='utf-8')
    artifact = dict(kind='lidar2d_experiment', base_commit=BASE,
        binary_sha256=sha(output/'bin/px4'), patch_sha256=sha(output/'px4_lidar2d.patch'),
        board_sha256=sha(output/'lidar2d.px4board'), physical_deployment=False,
        auxiliary_velocity_requires_other_horizontal_aiding=True)
    (output/'artifact.json').write_text(json.dumps(artifact, indent=2)+'\n', encoding='utf-8')
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence/'px4_lidar2d_artifact.json').write_text(json.dumps(artifact, indent=2)+'\n', encoding='utf-8')
    (Path(__file__).parent/'px4_lidar2d.patch').write_bytes(patch)
    print(json.dumps(artifact, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    package(args.source, args.output, args.evidence)
