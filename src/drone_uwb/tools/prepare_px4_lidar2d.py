"""Prepare the pinned, private PX4 2D LiDAR input experiment; never deploy it.

ODOMETRY estimator_type=LIDAR has a dedicated topic and never enters EV.
Only an already compensated LOCAL_NED horizontal velocity is accepted. The
auxiliary fusion remains dependent on another active horizontal aid source.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

COMMIT='d6f12ad1c4f70ad3230afd7d86e971421e02fef4'

MESSAGE='''# Experimental independent horizontal LiDAR observation at the IMU origin.
# No vertical position, velocity, attitude, or angular rate is observed.
uint64 timestamp
uint64 timestamp_sample
float32[2] velocity_ne
float32[3] covariance_ne # [var_n, cov_ne, var_e], (m/s)^2
uint8 reset_counter
int8 quality
uint8 source_system
uint8 source_component
bool frame_valid
'''

STATUS='''# Admission status. Acceptance is not proof of actual EKF fusion.
uint64 timestamp
uint64 timestamp_sample
uint32 accepted
uint32 rejected
uint8 reason # 0 disabled, 1 priming, 2 admitted, 3 value/covariance, 4 quality, 5 time, 6 duplicate, 7 source, 8 reset, 9 gap, 10 frame
'''

RECEIVER='''
	// Experimental 2D path: independent of the shared external-vision stream.
	if (odom_in.estimator_type == MAV_ESTIMATOR_TYPE_LIDAR) {
		lidar_velocity2d_s lidar{};
		lidar.timestamp = hrt_absolute_time();
		lidar.timestamp_sample = _mavlink_timesync.sync_stamp(odom_in.time_usec);
		lidar.velocity_ne[0] = odom_in.vx;
		lidar.velocity_ne[1] = odom_in.vy;
		lidar.covariance_ne[0] = odom_in.velocity_covariance[0];
		lidar.covariance_ne[1] = odom_in.velocity_covariance[1];
		lidar.covariance_ne[2] = odom_in.velocity_covariance[6];
		lidar.reset_counter = odom_in.reset_counter;
		lidar.quality = odom_in.quality;
		lidar.source_system = msg->sysid;
		lidar.source_component = msg->compid;
		lidar.frame_valid = odom_in.frame_id == MAV_FRAME_LOCAL_NED
			&& odom_in.child_frame_id == MAV_FRAME_LOCAL_NED && std::isnan(odom_in.vz);
		_lidar_velocity2d_pub.publish(lidar);
		return;
	}
'''

ADMISSION='''
	if (_param_ekf2_lid2_en.get()) {
		lidar_velocity2d_s sample{};
		const hrt_abstime now = hrt_absolute_time();
		const uint64_t max_age = static_cast<uint64_t>(_param_ekf2_lid2_age.get() * 1000.f);
		auto report = [&](uint8_t reason, uint64_t stamp) {
			lidar_velocity2d_status_s status{};
			status.timestamp = now;
			status.timestamp_sample = stamp;
			status.reason = reason;
			status.accepted = _lidar2d_accepted;
			status.rejected = _lidar2d_rejected;
			_lidar_velocity2d_status_pub.publish(status);
		};
		if (!_lidar_velocity2d_sub.update(&sample)) {
			if (_lidar2d_primed && now - _lidar2d_last_rx > max_age) {
				_ekf.resetAuxVelData();
				_lidar2d_primed = false;
				report(9, _lidar2d_last_sample);
			}
			return; // Never mix landing-target samples into the same buffer.
		}
		const float nn = sample.covariance_ne[0];
		const float ne = sample.covariance_ne[1];
		const float ee = sample.covariance_ne[2];
		const uint16_t source = (static_cast<uint16_t>(sample.source_system) << 8) | sample.source_component;
		uint8_t reason = 2;
		if (!sample.frame_valid) {
			reason = 10;
		} else if (!PX4_ISFINITE(sample.velocity_ne[0]) || !PX4_ISFINITE(sample.velocity_ne[1])
			   || !PX4_ISFINITE(nn) || !PX4_ISFINITE(ne) || !PX4_ISFINITE(ee)
			   || nn <= 0.f || ee <= 0.f || nn * ee <= ne * ne) {
			reason = 3;
		} else if (sample.quality <= 0 || sample.quality > 100) {
			reason = 4;
		} else if (sample.timestamp_sample == 0 || sample.timestamp_sample > now
			   || now - sample.timestamp_sample > max_age) {
			reason = 5;
		} else if (sample.timestamp_sample <= _lidar2d_last_sample) {
			reason = 6;
		} else if (_lidar2d_source != 0 && _lidar2d_source != source) {
			reason = 7;
		}
		if (reason != 2) {
			_ekf.resetAuxVelData();
			_lidar2d_primed = false;
			++_lidar2d_rejected;
			report(reason, sample.timestamp_sample);
			return;
		}
		const bool reset = _lidar2d_source != 0 && _lidar2d_reset != sample.reset_counter;
		const bool gap = _lidar2d_last_rx != 0 && now - _lidar2d_last_rx > max_age;
		_lidar2d_source = source;
		_lidar2d_reset = sample.reset_counter;
		_lidar2d_last_sample = sample.timestamp_sample;
		_lidar2d_last_rx = now;
		if (!_lidar2d_primed || reset || gap) {
			_ekf.resetAuxVelData(); // Buffer only: never reset a state mean or covariance.
			_lidar2d_primed = true;
			report(reset ? 8 : gap ? 9 : 1, sample.timestamp_sample);
			return;
		}
		// Conservative diagonal majorant D-C is PSD. Dropping C_ne silently is not valid.
		const float cross = fabsf(ne);
		auxVelSample aux{sample.timestamp_sample,
			Vector2f{sample.velocity_ne[0], sample.velocity_ne[1]}, Vector2f{nn + cross, ee + cross}};
		_ekf.setAuxVelData(aux);
		++_lidar2d_accepted;
		report(2, sample.timestamp_sample);
		return;
	}
'''

MEMBERS='''
	uORB::Subscription _lidar_velocity2d_sub{ORB_ID(lidar_velocity2d)};
	uORB::Publication<lidar_velocity2d_status_s> _lidar_velocity2d_status_pub{ORB_ID(lidar_velocity2d_status)};
	hrt_abstime _lidar2d_last_rx{0};
	uint64_t _lidar2d_last_sample{0};
	uint16_t _lidar2d_source{0};
	uint8_t _lidar2d_reset{0};
	bool _lidar2d_primed{false};
	uint32_t _lidar2d_accepted{0};
	uint32_t _lidar2d_rejected{0};
'''

PARAMS='''
    EKF2_LID2_EN:
      description:
        short: Experimental independent horizontal LiDAR input
        long: Replaces landing-target auxiliary velocity input. Requires another active horizontal aid source. No state reset or vertical observation.
      type: boolean
      default: false
      reboot_required: true
    EKF2_LID2_AGE:
      description:
        short: Maximum experimental LiDAR sample age and receive gap
      type: float
      default: 150
      min: 20
      max: 180
      unit: ms
      reboot_required: true
'''


def prepare(root):
    root=root.resolve()
    if root.name!='PX4-lidar2d':raise ValueError('dedicated PX4-lidar2d checkout required')
    def git(*args):return subprocess.check_output(['git','-C',str(root),*args],text=True).strip()
    if git('rev-parse','HEAD')!=COMMIT or git('diff','HEAD','--name-only'):
        raise ValueError('clean pinned firmware checkout required')
    changes={}
    def replace(name,old,new):
        path=root/name;text=path.read_text(encoding='utf-8')
        if text.count(old)!=1:raise ValueError('unexpected source anchor: '+name)
        text=text.replace(old,new);path.write_text(text,encoding='utf-8',newline='\n');changes[name]=hashlib.sha256(text.encode()).hexdigest()
    def create(name,text):
        path=root/name
        if path.exists():raise ValueError('refusing overwrite: '+name)
        path.write_text(text,encoding='utf-8',newline='\n');changes[name]=hashlib.sha256(text.encode()).hexdigest()
    create('msg/LidarVelocity2d.msg',MESSAGE)
    create('msg/LidarVelocity2dStatus.msg',STATUS)
    replace('msg/CMakeLists.txt','\tLandingTargetPose.msg','\tLandingTargetPose.msg\n\tLidarVelocity2d.msg\n\tLidarVelocity2dStatus.msg')
    replace('src/modules/mavlink/mavlink_receiver.h','#include <uORB/topics/vehicle_odometry.h>',
            '#include <uORB/topics/vehicle_odometry.h>\n#include <uORB/topics/lidar_velocity2d.h>')
    replace('src/modules/mavlink/mavlink_receiver.h','\tuORB::Publication<vehicle_odometry_s>\t\t\t_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};',
            '\tuORB::Publication<vehicle_odometry_s>\t\t\t_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};\n\tuORB::Publication<lidar_velocity2d_s> _lidar_velocity2d_pub{ORB_ID(lidar_velocity2d)};')
    replace('src/modules/mavlink/mavlink_receiver.cpp','\tmavlink_msg_odometry_decode(msg, &odom_in);',
            '\tmavlink_msg_odometry_decode(msg, &odom_in);\n'+RECEIVER)
    replace('src/modules/mavlink/mavlink_receiver.cpp','#include "mavlink_receiver.h"','#include "mavlink_receiver.h"\n#include <cmath>')
    replace('src/modules/ekf2/EKF2.hpp','# include <uORB/topics/landing_target_pose.h>',
            '# include <uORB/topics/landing_target_pose.h>\n# include <uORB/topics/lidar_velocity2d.h>\n# include <uORB/topics/lidar_velocity2d_status.h>')
    replace('src/modules/ekf2/EKF2.hpp','\tuORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)};',
            '\tuORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)};\n'+MEMBERS)
    replace('src/modules/ekf2/EKF2.hpp','\t\t(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)',
            '\t\t(ParamBool<px4::params::EKF2_LID2_EN>) _param_ekf2_lid2_en,\n\t\t(ParamFloat<px4::params::EKF2_LID2_AGE>) _param_ekf2_lid2_age,\n\t\t(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)')
    replace('src/modules/ekf2/EKF2.cpp','void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)\n{',
            'void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)\n{\n'+ADMISSION)
    replace('src/modules/ekf2/EKF/ekf.h','\tconst auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }',
            '\tconst auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }\n\tvoid resetAuxVelData() { if (_auxvel_buffer) { _auxvel_buffer->reset(); } _aid_src_aux_vel.fused = false; }')
    yaml=root/'src/modules/ekf2/params_aux_velocity.yaml'
    text=yaml.read_text(encoding='utf-8')+PARAMS;yaml.write_text(text,encoding='utf-8',newline='\n')
    changes[str(yaml.relative_to(root))]=hashlib.sha256(text.encode()).hexdigest()
    replace('src/modules/logger/logged_topics.cpp','\tadd_optional_topic("landing_target_pose", 1000);',
            '\tadd_optional_topic("landing_target_pose", 1000);\n\tadd_topic("lidar_velocity2d");\n\tadd_topic("lidar_velocity2d_status");\n\tadd_topic("estimator_aid_src_aux_vel");')
    source=(root/'boards/px4/sitl/default.px4board').read_text(encoding='utf-8')
    omit=('CONFIG_MODULES_SIMULATION_GZ_','CONFIG_MODULES_UXRCE_DDS_CLIENT=',
          'CONFIG_DRIVERS_GPS=','CONFIG_DRIVERS_GNSS_SEPTENTRIO=')
    board='\n'.join(line.split('=')[0]+'=n' if line.startswith(omit) else line for line in source.splitlines())+'\nCONFIG_BOARD_NOLOCKSTEP=n\n'
    create('boards/px4/sitl/lidar2d.px4board',board)
    subprocess.run(['git','-C',str(root),'add','-N','--','msg/LidarVelocity2d.msg','msg/LidarVelocity2dStatus.msg','boards/px4/sitl/lidar2d.px4board'],check=True)
    return dict(base_commit=COMMIT,files=changes,default_enabled=False,
        contract='Independent 2D NE velocity at IMU origin; conservative XY covariance; no synthetic vz; no EV multiplexing',
        limits=['a separate horizontal aid must remain active','physical timing/mount/covariance not validated','not deployed'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--px4-root',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--patch',type=Path,required=True)
    a=p.parse_args();result=prepare(a.px4_root)
    a.manifest.parent.mkdir(parents=True,exist_ok=True)
    a.manifest.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    a.patch.write_bytes(subprocess.check_output(['git','-C',str(a.px4_root),'diff','--binary','HEAD']))
    print(json.dumps(result,indent=2))
