"""Upgrade the dedicated LiDAR input to independent horizontal aiding.

Apply only in the isolated PX4 checkout prepared by prepare_lidar_independent.py.
The resulting Git diff is the reviewable firmware source artifact.
"""
import argparse
from pathlib import Path


def replace(root, name, old, new):
    path = root / name
    source = path.read_text(encoding='utf-8')
    if source.count(old) != 1:
        raise ValueError('Expected exactly one source anchor: ' + name + ': ' + old[:80])
    path.write_text(source.replace(old, new), encoding='utf-8', newline='\n')


def upgrade(root):
    ekf = 'src/modules/ekf2/EKF/'
    replace(root, ekf+'common.h', '\t} flags;\n\tuint64_t value;\n};',
        '\t\tuint64_t aux_vel : 1; ///< 48 - independent horizontal LiDAR velocity aiding\n'
        '\t} flags;\n\tuint64_t value;\n};')
    replace(root, ekf+'estimator_interface.cpp',
        '+ int(_control_status.flags.ev_vel)\n',
        '+ int(_control_status.flags.ev_vel)\n\t       + int(_control_status.flags.aux_vel)\n')
    replace(root, ekf+'ekf.h',
        '\tvoid resetAuxVelData() { if (_auxvel_buffer) { _auxvel_buffer->reset(); } _aid_src_aux_vel.fused = false; }',
        '\tvoid resetAuxVelData() { if (_auxvel_buffer) { _auxvel_buffer->reset(); } stopAuxVelFusion(); }\n'
        '\tvoid setAuxVelIndependent(bool enabled) { if (_auxvel_independent != enabled) { resetAuxVelData(); } _auxvel_independent = enabled; }')
    replace(root, ekf+'ekf.h', '\testimator_aid_source2d_s _aid_src_aux_vel {};',
        '\testimator_aid_source2d_s _aid_src_aux_vel {};\n'
        '\tbool _auxvel_independent{false};\n'
        '\tuint64_t _auxvel_start_time_us{0};\n'
        '\tuint64_t _auxvel_last_sample_us{0};')
    replace(root, ekf+'ekf_helper.cpp',
        '(_control_status.flags.gnss_vel || _control_status.flags.ev_vel)\n',
        '(_control_status.flags.gnss_vel || _control_status.flags.ev_vel || _control_status.flags.aux_vel)\n')
    replace(root, ekf+'ekf_helper.cpp',
        'float Ekf::getHorizontalVelocityInnovationTestRatio() const\n{\n\t// return the largest velocity innovation test ratio\n\tfloat test_ratio = -1.f;',
        'float Ekf::getHorizontalVelocityInnovationTestRatio() const\n{\n\t// return the largest velocity innovation test ratio\n\tfloat test_ratio = -1.f;\n'
        '#if defined(CONFIG_EKF2_AUXVEL)\n'
        '\tif (_control_status.flags.aux_vel) {\n'
        '\t\tfor (const auto ratio : _aid_src_aux_vel.test_ratio_filtered) {\n'
        '\t\t\ttest_ratio = math::max(test_ratio, fabsf(ratio));\n\t\t}\n\t}\n'
        '#endif // CONFIG_EKF2_AUXVEL\n')
    path = root / (ekf+'aid_sources/auxvel/auxvel_fusion.cpp')
    source = path.read_text(encoding='utf-8')
    source = source[:source.index('void Ekf::controlAuxVelFusion')]
    source += '''void Ekf::controlAuxVelFusion(const imuSample &imu_sample)
{
	constexpr uint64_t timeout_us = 500000;
	if (_auxvel_buffer) {
		auxVelSample sample;
		if (_auxvel_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
			if (_auxvel_independent && (!_control_status.flags.tilt_align || !_control_status.flags.yaw_align
			    || !sample.vel.isAllFinite() || !sample.velVar.isAllFinite()
			    || sample.velVar.min() <= 0.f || sample.vel.longerThan(_params.ekf2_vel_lim)
			    || sample.time_us > _time_delayed_us || _time_delayed_us - sample.time_us > timeout_us)) {
				stopAuxVelFusion();
				return;
			}
			updateAidSourceStatus(_aid_src_aux_vel, sample.time_us, sample.vel, sample.velVar,
				Vector2f(_state.vel.xy()) - sample.vel,
				Vector2f(getStateVariance<State::vel>()) + sample.velVar,
				math::max(_params.auxvel_gate, 1.f));
			if (!_auxvel_independent) {
				// Preserve the upstream landing-target auxiliary-aiding behavior.
				if (isHorizontalAidingActive()) { fuseHorizontalVelocity(_aid_src_aux_vel); }
				return;
			}
			if (_auxvel_last_sample_us == 0 || sample.time_us - _auxvel_last_sample_us > 200000) {
				_auxvel_start_time_us = sample.time_us;
			}
			_auxvel_last_sample_us = sample.time_us;
			if (_aid_src_aux_vel.innovation_rejected) {
				_auxvel_start_time_us = 0;
			} else {
				if (_auxvel_start_time_us == 0) { _auxvel_start_time_us = sample.time_us; }
				// Require a continuous, innovation-consistent stream before independent start.
				if (_control_status.flags.aux_vel || sample.time_us - _auxvel_start_time_us >= 200000) {
					if (fuseHorizontalVelocity(_aid_src_aux_vel)) {
						_control_status.flags.aux_vel = true;
					}
				}
			}
		}
	}
	if (_auxvel_independent && _control_status.flags.aux_vel
	    && (isTimedOut(_auxvel_last_sample_us, timeout_us)
	        || isTimedOut(_aid_src_aux_vel.time_last_fuse, timeout_us))) {
		stopAuxVelFusion();
	}
}

void Ekf::stopAuxVelFusion()
{
	_control_status.flags.aux_vel = false;
	_aid_src_aux_vel.fused = false;
	_auxvel_start_time_us = 0;
	_auxvel_last_sample_us = 0;
}
'''
    path.write_text(source, encoding='utf-8', newline='\n')
    replace(root, 'src/modules/ekf2/EKF2.cpp',
        'void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)\n{\n',
        'void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)\n{\n'
        '\t_ekf.setAuxVelIndependent(_param_ekf2_lid2_en.get());\n'
        '\tif (!_param_ekf2_lid2_en.get()) { _lidar2d_primed = false; }\n')
    replace(root, 'src/modules/ekf2/EKF2.cpp',
        '\t\t\tstatus.rejected = _lidar2d_rejected;',
        '\t\t\tstatus.rejected = _lidar2d_rejected;\n'
        '\t\t\tstatus.fusion_active = _ekf.control_status_flags().aux_vel;\n'
        '\t\t\tstatus.fused = _ekf.aid_src_aux_vel().fused;\n'
        '\t\t\tstatus.time_last_fuse = _ekf.aid_src_aux_vel().time_last_fuse;')
    path = root / 'msg/LidarVelocity2dStatus.msg'
    source = path.read_text(encoding='utf-8')
    source += ('\nbool fusion_active # Independent horizontal aiding, control status bit 48\n'
               'bool fused # Most recent EKF aid-source sample was fused\n'
               'uint64 time_last_fuse # EKF delayed fusion time; increases only on successful fusion\n')
    path.write_text(source, encoding='utf-8', newline='\n')
    replace(root, 'src/modules/ekf2/params_aux_velocity.yaml',
        'Requires another active horizontal aid source. No state reset or vertical observation.',
        'Starts independent horizontal velocity aiding after alignment and consistent observations. No position, height or vertical-velocity observation. No state reset.')
    replace(root, 'src/modules/ekf2/params_aux_velocity.yaml',
        'Protocol 1 consumes only NE velocity at the IMU origin from component 198. Vertical velocity is unobserved. Use with the matched companion package.',
        'Protocol 2 adds independent horizontal aiding to the NE-only input from component 198. Vertical velocity is unobserved. Use with the matched companion package.')
    replace(root, 'src/modules/ekf2/params_aux_velocity.yaml',
        '      default: 1\n      min: 1\n      max: 1\n',
        '      default: 2\n      min: 2\n      max: 2\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    upgrade(parser.parse_args().source.resolve())
