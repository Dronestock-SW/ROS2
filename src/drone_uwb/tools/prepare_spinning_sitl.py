"""Add precise, timestamped SIMULATION truth transport to the private runtime.

Never install this on a vehicle. Dynamics, sensor noise, estimator and control
are unchanged. ODOMETRY/NAIVE emitted beside HIL_STATE_QUATERNION is a sensor
backend/evaluation interface; it is never forwarded to PX4 as an observation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


BASE = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
STREAM = 'src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp'
MARKER = '// Private spinning-LiDAR sensor backend: truth is never an estimator input.'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source):
    path = source/STREAM
    original = path.read_text(encoding='utf-8')
    if MARKER in original:
        return
    text = original.replace('MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;',
        'MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN + MAVLINK_MSG_ID_ODOMETRY_LEN + 2 * MAVLINK_NUM_NON_PAYLOAD_BYTES;')
    text = text.replace('mavlink_hil_state_quaternion_t msg{};',
        'mavlink_hil_state_quaternion_t msg{};\n\t\t\tmsg.time_usec = lpos.timestamp;')
    needle = '\t\t\tmavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);'
    addition = '''

			// Private spinning-LiDAR sensor backend: truth is never an estimator input.
			// Preserve local float precision and the SIH sample clock. The old
			// HIL latitude/longitude path remains available for baseline UWB.
			mavlink_odometry_t simulation{};
			simulation.time_usec = lpos.timestamp;
			simulation.frame_id = MAV_FRAME_LOCAL_NED;
			simulation.child_frame_id = MAV_FRAME_LOCAL_NED;
			simulation.estimator_type = MAV_ESTIMATOR_TYPE_NAIVE;
			simulation.quality = 100;
			simulation.x = lpos.x;
			simulation.y = lpos.y;
			simulation.z = lpos.z;
			simulation.vx = lpos.vx;
			simulation.vy = lpos.vy;
			simulation.vz = lpos.vz;
			for (unsigned i = 0; i < 4; ++i) { simulation.q[i] = att.q[i]; }
			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &simulation);
'''
    if text.count(needle)!=1 or text==original:
        raise ValueError('unexpected_private_source')
    path.write_text(text.replace(needle, needle+addition), encoding='utf-8')


def package(source, output):
    if output.exists():
        raise ValueError('new_runtime_destination_required')
    build = source/'build/px4_sitl_lidar2d'
    output.mkdir(parents=True)
    shutil.copytree(build/'bin', output/'bin', symlinks=True)
    shutil.copytree(build/'etc', output/'etc', symlinks=True)
    shutil.copyfile(source/'boards/px4/sitl/lidar2d.px4board', output/'lidar2d.px4board')
    patch = subprocess.check_output(['git', '-C', str(source), 'diff', '--binary', 'HEAD'])
    (output/'px4_lidar2d.patch').write_bytes(patch)
    (output/'source_commit.txt').write_text(BASE+'\n', encoding='utf-8')
    incremental = subprocess.check_output(['git', '-C', str(source), 'diff', 'HEAD', '--', STREAM])
    (Path(__file__).parent/'px4_spinning_sensor.patch').write_bytes(incremental)
    artifact = dict(kind='lidar2d_experiment', base_commit=BASE,
        binary_sha256=sha(output/'bin/px4'), patch_sha256=sha(output/'px4_lidar2d.patch'),
        board_sha256=sha(output/'lidar2d.px4board'), physical_deployment=False,
        auxiliary_velocity_requires_other_horizontal_aiding=True,
        simulation_truth_stream='ODOMETRY estimator_type=NAIVE, LOCAL_NED, sample timestamp, float coordinates',
        simulator_dynamics_or_noise_changed=False, truth_for_sensor_backend_and_evaluation_only=True,
        highres_imu_stamp='vehicle_imu.timestamp_sample; upstream bias-corrected gyro')
    (output/'artifact.json').write_text(json.dumps(artifact, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(artifact, indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    if source.name!='PX4-lidar2d' or subprocess.check_output(
            ['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()!=BASE:
        raise ValueError('private_pinned_source_required')
    prepare(source)
    if args.output:
        package(source, args.output.resolve())
