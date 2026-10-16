"""Build an offline EKF call-path probe using the exact SIH library and headers."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
from prepare_lidar_sitl import FIRMWARE_COMMIT

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--px4-root', required=True, type=Path)
p.add_argument('--output', required=True, type=Path)
args = p.parse_args()
root, output = args.px4_root.resolve(), args.output.resolve()
if subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'], text=True).strip() != FIRMWARE_COMMIT:
    raise ValueError('wrong firmware')
if subprocess.check_output(['git','-C',str(root),'diff','HEAD','--name-only'], text=True).strip():
    raise ValueError('PX4 tracked source has local changes')
output.mkdir(parents=True, exist_ok=False)
build = root / 'build/px4_sitl_lidar'
commands = json.loads((build/'compile_commands.json').read_text(encoding='utf-8'))
entry = next(c for c in commands if c['file'].endswith('/EKF/ekf.cpp') and 'ecl_EKF.dir' in c['command'])
tokens = shlex.split(entry['command'])
flags = []
for i,t in enumerate(tokens[1:]):
    if t in ('-o','-c'):
        break
    if not t.startswith('-W'):
        flags.append(t)
flags += ['-I'+str(root/'src/modules/ekf2')]
sources = list((root/'src/modules/ekf2/test/sensor_simulator').glob('*.cpp'))
sources += [Path(__file__).with_name('lidar_px4_reset_probe.cpp')]
objects = []
for source in sources:
    obj = output/(source.stem+'.o')
    subprocess.run([tokens[0],*flags,'-c',str(source),'-o',str(obj)],cwd=entry['directory'],check=True)
    objects.append(str(obj))
libraries = ['src/modules/ekf2/EKF/libecl_EKF.a', 'src/lib/motion_planning/libmotion_planning.a',
             'src/modules/ekf2/EKF/output_predictor/liboutput_predictor.a',
             'src/modules/ekf2/EKF/bias_estimator/libbias_estimator.a',
             'src/modules/ekf2/EKF/yaw_estimator/libyaw_estimator.a',
             'src/lib/lat_lon_alt/liblat_lon_alt.a',
             'src/lib/geo/libgeo.a','src/lib/world_magnetic_model/libworld_magnetic_model.a',
             'src/lib/mathlib/libmathlib.a','src/lib/perf/libperf.a',
             'platforms/common/libpx4_platform.a','platforms/posix/src/px4/common/libpx4_layer.a']
subprocess.run([tokens[0], *objects, '-Wl,--gc-sections', '-Wl,--start-group',
                *[str(build/x) for x in libraries], '-Wl,--end-group', '-lpthread','-lm',
                '-o',str(output/'reset_probe')],check=True)
manifest = dict(firmware_commit=FIRMWARE_COMMIT, scope='direct EKF reset call; not dynamic flight or control-policy trigger test',
                compile_flags=flags, libraries={x: hashlib.sha256((build/x).read_bytes()).hexdigest() for x in libraries},
                probe_sha256=hashlib.sha256((output/'reset_probe').read_bytes()).hexdigest())
(output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
