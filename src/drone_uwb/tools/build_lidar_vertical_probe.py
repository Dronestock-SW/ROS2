"""Link a diagnostic-only EV controller overlay with untouched pinned PX4 libraries.

The overlay is generated in a new output directory. No existing PX4 source,
library, binary, process or parameter is modified. This is an EKF call experiment,
not a PX4 flight/SIH binary or production fix.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

from prepare_lidar_sitl import FIRMWARE_COMMIT


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stock-library',action='store_true',help='Link original libraries without diagnostic overlay')
    a=p.parse_args()
    root=a.px4_root.resolve(); out=a.output.resolve()
    if subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()!=FIRMWARE_COMMIT:
        raise ValueError('wrong firmware')
    if subprocess.check_output(['git','-C',str(root),'diff','HEAD','--name-only'],text=True).strip():
        raise ValueError('tracked PX4 source changed')
    out.mkdir(parents=True,exist_ok=False)
    build=root/'build/px4_sitl_lidar'
    commands=json.loads((build/'compile_commands.json').read_text(encoding='utf-8'))
    entry=next(c for c in commands if c['file'].endswith('/EKF/ekf.cpp') and 'ecl_EKF.dir' in c['command'])
    tokens=shlex.split(entry['command']); flags=[]
    for token in tokens[1:]:
        if token in ('-o','-c'): break
        if not token.startswith('-W'): flags.append(token)
    flags+=['-I'+str(root/'src/modules/ekf2'),'-I'+str(root/'src/modules/ekf2/EKF')]
    source=root/'src/modules/ekf2/EKF/aid_sources/external_vision/ev_vel_control.cpp'
    code=source.read_text(encoding='utf-8')
    code=code.replace('#include "ekf.h"','#include "ekf.h"\n#include <cstdio>\n#include <cstdlib>\n#include <cstring>')
    # Capture the actual state/covariance before/after each body-axis update.
    marker='\t\t\tmeasurementUpdate(Kfusion, H[index], aid_src.observation_variance[index], aid_src.innovation[index]);'
    if code.count(marker)!=1: raise ValueError('axis instrumentation anchor mismatch')
    code=code.replace(marker,'''            const float before_z = _gpos.altitude();
            const float before_vz = _state.vel(2);
            const float gain_z = Kfusion(State::pos.idx+2);
            const float gain_vz = Kfusion(State::vel.idx+2);
'''+marker+'''
            if (_time_delayed_us > 39000000ULL) {
                std::printf("AXIS {\\"us\\":%llu,\\"axis\\":%u,\\"dz\\":%.9g,\\"dvz\\":%.9g,\\"gain_z\\":%.9g,\\"gain_vz\\":%.9g,\\"innovation\\":%.9g}\\n",
                    static_cast<unsigned long long>(_time_delayed_us), unsigned(index),
                    double(before_z-_gpos.altitude()),double(_state.vel(2)-before_vz),
                    double(gain_z),double(gain_vz),double(aid_src.innovation[index]));
            }''')
    marker='\t\t\tev.resetVerticalVelocity();'
    if code.count(marker)!=1: raise ValueError('reset instrumentation anchor mismatch')
    code=code.replace(marker,'''            const float before_vz=_state.vel(2);
            const float before_z=_gpos.altitude();
            const char *mode=std::getenv("LIDAR_PROBE_MODE");
            if (mode && std::strcmp(mode,"no_vreset")==0) {
                // Causal intervention only: leave 3D EV fusion and validity intact.
            } else if (mode && std::strcmp(mode,"rotated_vreset")==0
                       && ev._sample.vel_frame==VelocityFrame::BODY_FRAME_FRD) {
                static_cast<EvVelBodyFrameFrd &>(ev).resetVerticalVelocity();
            } else { ev.resetVerticalVelocity(); }
            std::printf("VRESET {\\"us\\":%llu,\\"before_vz\\":%.9g,\\"after_vz\\":%.9g,\\"dz\\":%.9g,\\"last_hgt_us\\":%llu,\\"last_ver_vel_us\\":%llu}\\n",
                static_cast<unsigned long long>(_time_delayed_us),double(before_vz),double(_state.vel(2)),
                double(before_z-_gpos.altitude()),static_cast<unsigned long long>(_time_last_hgt_fuse),
                static_cast<unsigned long long>(_time_last_ver_vel_fuse));''')
    overlay=out/'ev_vel_control_instrumented.cpp'; overlay.write_text(code,encoding='utf-8')
    velocity_source=root/'src/modules/ekf2/EKF/velocity_fusion.cpp'
    velocity_code=velocity_source.read_text(encoding='utf-8').replace('#include "ekf.h"',
        '#include "ekf.h"\n#include <cstdlib>\n#include <cstring>')
    reset_line='\t_output_predictor.resetVerticalVelocityTo(delta_vert_vel);'
    if velocity_code.count(reset_line)!=1: raise ValueError('output reset anchor mismatch')
    velocity_code=velocity_code.replace(reset_line,'''    const char *diagnostic_mode=std::getenv("LIDAR_PROBE_MODE");
    if (!diagnostic_mode || std::strcmp(diagnostic_mode,"no_output_reset")!=0) {
        _output_predictor.resetVerticalVelocityTo(delta_vert_vel);
    }''')
    velocity_overlay=out/'velocity_fusion_instrumented.cpp'
    velocity_overlay.write_text(velocity_code,encoding='utf-8')
    sources=list((root/'src/modules/ekf2/test/sensor_simulator').glob('*.cpp'))
    probe_source=Path(__file__).with_name('lidar_vertical_probe.cpp')
    sources += [probe_source] if a.stock_library else [probe_source,overlay,velocity_overlay]
    objects=[]
    for src in sources:
        obj=out/(src.stem+'.o')
        subprocess.run([tokens[0],*flags,'-c',str(src),'-o',str(obj)],cwd=entry['directory'],check=True)
        objects.append(str(obj))
    libraries=['src/modules/ekf2/EKF/libecl_EKF.a','src/lib/motion_planning/libmotion_planning.a',
               'src/modules/ekf2/EKF/output_predictor/liboutput_predictor.a',
               'src/modules/ekf2/EKF/bias_estimator/libbias_estimator.a',
               'src/modules/ekf2/EKF/yaw_estimator/libyaw_estimator.a','src/lib/lat_lon_alt/liblat_lon_alt.a',
               'src/lib/geo/libgeo.a','src/lib/world_magnetic_model/libworld_magnetic_model.a',
               'src/lib/mathlib/libmathlib.a','src/lib/perf/libperf.a','platforms/common/libpx4_platform.a',
               'platforms/posix/src/px4/common/libpx4_layer.a']
    # Overlay supplies all symbols from the original ev_vel_control object;
    # the archive linker therefore does not pull that original object in.
    subprocess.run([tokens[0],*objects,'-Wl,--gc-sections','-Wl,--start-group',
                    *[str(build/x) for x in libraries],'-Wl,--end-group','-lpthread','-lm',
                    '-o',str(out/'vertical_probe')],check=True)
    scope=('original EKF library' if a.stock_library else 'instrumented EKF library')
    result=dict(firmware=FIRMWARE_COMMIT,scope=scope+'; synthetic sensors, not SIH/flight',
                diagnostic_overlay_linked=not a.stock_library,
                original_controller_sha256=digest(source),overlay_sha256=digest(overlay),
                probe_source_sha256=digest(probe_source),binary_sha256=digest(out/'vertical_probe'),
                original_velocity_fusion_sha256=digest(velocity_source),
                velocity_overlay_sha256=digest(velocity_overlay),
                libraries={x:digest(build/x) for x in libraries},compile_flags=flags,
                interventions=['stock','no_vreset','rotated_vreset','position_only','aux2d','ned3d','no_output_reset'])
    (out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__=='__main__': main()
