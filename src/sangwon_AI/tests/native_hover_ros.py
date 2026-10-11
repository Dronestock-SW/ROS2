"""Run the real C++/HTTP/IPC path against synthetic MAVROS on localhost domain 173."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

assert os.environ.get('ROS_DOMAIN_ID') == '173'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import EstimatorStatus, ExtendedState, RCIn, State
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState, NavSatFix

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('hover_web', ROOT / 'ops/native_hover_web.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


class FC:
    def __init__(self, node):
        self.node = node
        group = ReentrantCallbackGroup()
        self.services = [node.create_service(kind, path, callback, callback_group=group)
                         for kind, path, callback in (
            (SetMode, '/mavros/set_mode', self.set_mode),
            (CommandBool, '/mavros/cmd/arming', self.arm),
            (ParamPull, '/mavros/param/pull', self.pull),
            (GetParameters, '/mavros/param/get_parameters', self.params))]
        self.pubs = {name: node.create_publisher(kind, topic, qos_profile_sensor_data)
                     for name, kind, topic in (
            ('state', State, '/mavros/state'), ('landed', ExtendedState, '/mavros/extended_state'),
            ('odom', Odometry, '/mavros/local_position/odom'),
            ('estimator', EstimatorStatus, '/mavros/estimator_status'),
            ('rc', RCIn, '/mavros/rc/in'), ('battery', BatteryState, '/mavros/battery'),
            ('global', NavSatFix, '/mavros/global_position/global'))}
        self.reset('nominal')
        self.stop_publishing = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_loop)
        self.publisher_thread.start()

    def publish_loop(self):
        # Delayed service callbacks must not starve synthetic FC telemetry.
        while not self.stop_publishing.is_set():
            self.tick()
            self.stop_publishing.wait(.04)

    def reset(self, scenario):
        self.scenario, self.mode, self.armed, self.z = scenario, 'POSCTL', False, 0.
        self.calls, self.fault, self.fixed_stamp = [], False, None
        self.previous = time.monotonic()

    def set_mode(self, request, response):
        self.calls.append(request.custom_mode)
        if self.scenario == 'takeoff_not_applied' and request.custom_mode == 'AUTO.TAKEOFF':
            response.mode_sent = True
            return response
        if self.scenario == 'land_rejected' and request.custom_mode == 'AUTO.LAND':
            response.mode_sent = False
            return response
        self.mode = request.custom_mode
        response.mode_sent = True
        return response

    def arm(self, request, response):
        self.calls.append('ARM' if request.value else 'DISARM')
        if request.value and self.scenario == 'arm_rejected':
            response.success, response.result = False, 2
            return response
        if request.value and self.scenario in ('delayed_arm_cancel', 'arm_reply_timeout'):
            time.sleep(1.2 if self.scenario == 'delayed_arm_cancel' else 4.)
        self.armed = request.value
        response.success, response.result = True, 0
        return response

    def pull(self, _, response):
        response.success, response.param_received = True, 9
        return response

    def params(self, request, response):
        values = {'MIS_TAKEOFF_ALT': 1.3, 'COM_TAKEOFF_ACT': 0, 'RC_MAP_ROLL': 1,
                  'RC_MAP_PITCH': 2, 'RC_MAP_THROTTLE': 3, 'RC_MAP_YAW': 4,
                  'RC_MAP_FLTMODE': 5, 'RC_MAP_KILL_SW': 7, 'RC_MAP_ARM_SW': 8, 'COM_RC_OVERRIDE': 3}
        response.values = [ParameterValue(type=3, double_value=values[n]) if n == 'MIS_TAKEOFF_ALT'
                           else ParameterValue(type=2, integer_value=values[n]) for n in request.names]
        return response

    def tick(self):
        now = time.monotonic()
        dt, self.previous = min(.1, now-self.previous), now
        if self.armed and self.mode == 'AUTO.TAKEOFF' and self.scenario != 'delayed_arm_cancel':
            self.z = min(1.3, self.z+dt)
            if self.z >= 1.3:
                self.mode = 'AUTO.LOITER'
        elif self.mode == 'AUTO.LAND':
            self.z = max(0., self.z-dt)
            if self.z == 0:
                self.armed = False
        stamp = self.node.get_clock().now().to_msg()
        if self.fault and self.scenario == 'duplicate_stamp':
            if self.fixed_stamp is None:
                self.fixed_stamp = stamp
            stamp = self.fixed_stamp
        state = State(connected=True, armed=self.armed, mode=self.mode)
        landed = ExtendedState(landed_state=1 if self.z == 0 else 2)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.w = 1.
        estimator = EstimatorStatus(attitude_status_flag=True, velocity_horiz_status_flag=True,
            velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True,
            pos_vert_abs_status_flag=True, pred_pos_horiz_rel_status_flag=True,
            pos_horiz_abs_status_flag=self.scenario != 'local_only')
        rc = RCIn(channels=[1500]*8, rssi=255)
        if self.fault and self.scenario == 'rc_loss':
            rc.channels = [0]*8
        if self.fault and self.scenario == 'rc_stick':
            rc.channels[0] = 1900
        if self.fault and self.scenario == 'manual_mode':
            self.mode, state.mode = 'ALTCTL', 'ALTCTL'
        battery = BatteryState(present=True, voltage=24., percentage=.9)
        global_position = NavSatFix(latitude=47.397, longitude=8.546, altitude=488.+self.z)
        global_position.status.status = -1  # No raw GNSS fix: EKF output is independent.
        if self.scenario == 'global_nan':
            global_position.altitude = float('nan')
        for name, message in (('state', state), ('landed', landed), ('odom', odom),
                              ('estimator', estimator), ('rc', rc), ('battery', battery),
                              ('global', global_position)):
            if name == 'global' and (self.scenario == 'global_missing' or
                    self.fault and self.scenario == 'global_loss'):
                continue
            message.header.stamp = stamp
            self.pubs[name].publish(message)


def run(binary, fc, directory, scenario):
    path = directory / scenario
    path.mkdir()
    config = path / 'config.json'
    config.write_text(json.dumps(dict(profile='SITL', ros_domain_id=173, output_enabled=True,
        takeoff_height_m=1.3, state_dir=str(path))), encoding='utf-8')
    fc.reset(scenario)
    log = (path / 'controller.log').open('wb')
    process = subprocess.Popen([binary, '--config', str(config)], stdout=log, stderr=log)
    server = None
    try:
        def wait(predicate, seconds=12):
            end = time.monotonic()+seconds
            last = None
            while time.monotonic() < end:
                assert process.poll() is None, (scenario, 'controller exited', process.returncode)
                try:
                    last = web.exchange(path / 'hover.sock', {'method': 'status'})['status']
                    if predicate(last):
                        return last
                except OSError:
                    pass
                time.sleep(.05)
            raise AssertionError((scenario, 'timeout', last, fc.calls))

        if scenario in ('local_only', 'global_nan', 'global_missing'):
            status = wait(lambda s: s['preflight_checks']['telemetry'] and s['parameter_readback_ok'])
            time.sleep(3.2)
            status = wait(lambda s: s['start_blocker'] == 'PX4_GLOBAL_REFERENCE_NOT_READY')
            rejected = web.exchange(path/'hover.sock', dict(method='start', session=status['session'],
                confirm='TAKEOFF_HOVER_2S_LAND'))
            assert not rejected['ok'] and not fc.calls and not status['preflight_checks']['global_reference']
            return dict(scenario=scenario, calls=fc.calls, status=status, actual_px4=False, physical_flight=False)
        status = wait(lambda s: not s['start_blocker'])
        assert not fc.calls, 'boot emitted a command'
        invalid = web.exchange(path / 'hover.sock', {'method': 'start', 'session': 'old',
                                                    'confirm': 'TAKEOFF_HOVER_2S_LAND'})
        assert not invalid['ok'] and not fc.calls
        # Nominal run exercises HTTP origin/token, then the same production IPC.
        with socket.socket() as free:
            free.bind(('127.0.0.1', 0))
            port = free.getsockname()[1]
        server = subprocess.Popen([sys.executable, str(ROOT / 'ops/native_hover_web.py'),
            '--socket', str(path / 'hover.sock'), '--port', str(port)], stdout=log, stderr=log)
        url = f'http://127.0.0.1:{port}'
        for _ in range(40):
            try:
                transport = json.load(urllib.request.urlopen(url+'/api/status', timeout=1))
                break
            except OSError:
                time.sleep(.05)
        else:
            raise AssertionError('HTTP server unavailable')
        body = json.dumps(dict(session=status['session'], confirm='TAKEOFF_HOVER_2S_LAND')).encode()
        for bad_body, origin, expected_code in ((body, 'http://example.invalid', 403), (b'null', url, 400)):
            invalid_request = urllib.request.Request(url+'/api/start', data=bad_body, headers={
                'Origin': origin, 'X-Bench-Token': transport['csrf'], 'Content-Type': 'application/json'})
            try:
                urllib.request.urlopen(invalid_request, timeout=2)
                raise AssertionError('invalid HTTP command accepted')
            except urllib.error.HTTPError as error:
                assert error.code == expected_code
            assert not fc.calls
        request = urllib.request.Request(url+'/api/start', data=body, headers={
            'Origin': url, 'X-Bench-Token': transport['csrf'], 'Content-Type': 'application/json'})
        started = json.load(urllib.request.urlopen(request, timeout=2))
        assert started['ok']
        cancelled_while_pending = False
        if scenario == 'delayed_arm_cancel':
            wait(lambda s: 'ARM' in fc.calls and s['command_pending'])
            reply = web.exchange(path / 'hover.sock', dict(method='land', session=status['session']))
            assert reply['ok'] and reply['status']['phase'] == 'ABORTING'
            pending = wait(lambda s: s['command_pending'])
            assert not pending['landing_verified']
            cancelled_while_pending = True
        elif scenario in ('duplicate_stamp', 'rc_loss', 'rc_stick', 'manual_mode', 'global_loss'):
            wait(lambda s: s['phase'] == 'CLIMB')
            fc.fault = True
        result = wait(lambda s: s['phase'] in ('COMPLETE', 'FAILED', 'CANCELLED', 'RELEASED'), 14)
        expected = {'nominal': 'COMPLETE', 'arm_rejected': 'FAILED',
                    'takeoff_not_applied': 'FAILED', 'delayed_arm_cancel': 'CANCELLED',
                    'duplicate_stamp': 'RELEASED', 'rc_loss': 'RELEASED', 'rc_stick': 'RELEASED',
                    'manual_mode': 'RELEASED', 'land_rejected': 'RELEASED', 'arm_reply_timeout': 'RELEASED',
                    'global_loss': 'FAILED'}[scenario]
        assert result['phase'] == expected, result
        if scenario == 'nominal':
            assert fc.calls == ['AUTO.TAKEOFF', 'ARM', 'AUTO.LAND'], fc.calls
            assert result['outcome'] == 'PASS' and result['landing_verified']
        if scenario == 'global_loss':
            assert result['reason'] == 'ESTIMATOR_LOST' and 'AUTO.LAND' in fc.calls
        if scenario == 'delayed_arm_cancel':
            assert fc.calls == ['AUTO.TAKEOFF', 'ARM', 'DISARM', 'POSCTL'], fc.calls
            assert result['landing_verified'] and result['outcome'] == 'CANCELLED'
        if scenario == 'takeoff_not_applied':
            assert 'ARM' not in fc.calls
        if scenario == 'rc_loss':
            assert result['reason'] == 'RC_LOST_PX4_FAILSAFE' and 'POSCTL' not in fc.calls
        if scenario == 'rc_stick':
            assert result['reason']=='RC_HANDOFF_UNCONFIRMED' and 'POSCTL' not in fc.calls
        if scenario == 'arm_reply_timeout':
            assert result['reason'] == 'COMMAND_EFFECT_UNKNOWN_OPERATOR_REQUIRED'
            assert not result['landing_verified'] and result['outcome'] == 'UNCONFIRMED'
            time.sleep(1.2)  # Delayed simulated FC effect must settle before the next scenario.
        before = list(fc.calls)
        duplicate = web.exchange(path / 'hover.sock', dict(method='start', session=status['session'],
            confirm='TAKEOFF_HOVER_2S_LAND'))
        assert not duplicate['ok']
        time.sleep(.15)
        assert before == fc.calls
        return dict(scenario=scenario, calls=fc.calls, status=result,
                    cancelled_while_pending=cancelled_while_pending,
                    http_rejection_checks=2, actual_px4=False, physical_flight=False)
    finally:
        if server:
            server.terminate()
            server.wait(timeout=5)
        process.terminate()
        process.wait(timeout=5)
        log.close()
        assert process.returncode == 0, (scenario, process.returncode)


def main():
    directory = Path(os.environ.get('HOVER_TEST_OUTPUT', '/tmp/native-hover-tests'))
    directory.mkdir(parents=True, exist_ok=True)
    directory = directory / str(time.time_ns())
    directory.mkdir()
    rclpy.init()
    node = rclpy.create_node('synthetic_native_hover_mavros')
    fc = FC(node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        results = [run(sys.argv[1], fc, directory, s) for s in (
            'nominal', 'arm_rejected', 'takeoff_not_applied', 'delayed_arm_cancel',
            'duplicate_stamp', 'rc_loss', 'rc_stick', 'manual_mode', 'land_rejected', 'arm_reply_timeout',
            'local_only', 'global_nan', 'global_missing', 'global_loss')]
        (directory / 'summary.json').write_text(json.dumps(results, indent=2)+'\n', encoding='utf-8')
        print(f'PASS {len(results)} C++/ROS/HTTP/IPC scenarios; synthetic FC; {directory}', flush=True)
    finally:
        fc.stop_publishing.set()
        fc.publisher_thread.join(timeout=5)
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
